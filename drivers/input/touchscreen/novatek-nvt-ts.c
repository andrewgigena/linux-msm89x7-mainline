// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for Novatek NT11205 i2c touchscreen controller as found
 * on the Acer Iconia One 7 B1-750 tablet.
 *
 * Copyright (c) 2023 Hans de Goede <hdegoede@redhat.com>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/module.h>

#include <asm/unaligned.h>

#define NVT_TS_TOUCH_START		0x00
#define NVT_TS_TOUCH_SIZE		6


#define NVT_TS_FIRMWARE_INFO_START		    0x78
#define NVT_TS_FIRMWARE_INFO_LEN		    0x0f
/* These are offsets from NVT_TS_FIRMWARE_INFO_START */
#define NVT_TS_PARAMS_FW_VERSION		0x00
#define NVT_TS_PARAMS_FW_VERSION_BAR	0x01
#define NVT_TS_PARAMS_X_AXIS_LINES		0x02
#define NVT_TS_PARAMS_Y_AXIS_LINES		0x03
#define NVT_TS_PARAMS_WIDTH				0x04 // And 0x05 because it's a 16-bit value
#define NVT_TS_PARAMS_HEIGHT			0x06 // And 0x07 because it's a 16-bit value
#define NVT_TS_PARAMS_FW_VERSION_COMMON 0x08
#define NVT_TS_PARAMS_MAX_TOUCH			0x09 
#define NVT_TS_PARAMS_MAX_BUTTONS		0x0a
#define NVT_TS_PARAMS_IRQ_TYPE			0x0b
#define NVT_TS_PARAMS_WAKE_TYPE			0x0c
#define NVT_TS_PARAMS_CHIP_ID			0x0e

#define NVT_TS_MAX_TOUCHES		10
#define NVT_TS_MAX_SIZE			4096

#define NVT_TS_TOUCH_INVALID		0xff
#define NVT_TS_TOUCH_SLOT_SHIFT		3
#define NVT_TS_TOUCH_TYPE_MASK		GENMASK(2, 0)
#define NVT_TS_TOUCH_NEW		1
#define NVT_TS_TOUCH_UPDATE		2
#define NVT_TS_TOUCH_RELEASE		3

static const int nvt_ts_irq_type[4] = {
	IRQF_TRIGGER_RISING,
	IRQF_TRIGGER_FALLING,
	IRQF_TRIGGER_LOW,
	IRQF_TRIGGER_HIGH
};

struct nvt_ts_i2c_chip_data {
	u8 wake_type;
	u8 chip_id;
	u8 init_reset_state;
	bool unlock_firmware_info;
	u8 unlock_firmware_data[NVT_TS_TOUCH_SIZE * NVT_TS_MAX_TOUCHES];
    u8 unlock_firmware_data_size;
};

struct nvt_ts_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data regulators[2];
	struct touchscreen_properties prop;
	const struct nvt_ts_i2c_chip_data *chip_data;
	int max_touches;
	u8 rx_buf[NVT_TS_TOUCH_SIZE * NVT_TS_MAX_TOUCHES];
};

enum nvt_ts_hw_state {
	NVT_TS_HW_STATE_RUNNING,
	NVT_TS_HW_STATE_STOPPED
};

static int nvt_ts_read_data(struct i2c_client *client, u8 tx_buf, u8 *rx_buf,
                            int rx_size)
{
    int ret;

    struct i2c_msg msg[2] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = 1,
            .buf = &tx_buf,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = rx_size,
            .buf = rx_buf,
        }
    };

    ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
    if (ret != ARRAY_SIZE(msg)) {
        dev_err(&client->dev, "Error reading from 0x%02x: %d\n", tx_buf, ret);
        return (ret < 0) ? ret : -EIO;
    }

    dev_dbg(&client->dev, "Read %d bytes from register 0x%02x: %*ph",
             rx_size, tx_buf, rx_size, rx_buf);
    return 0;
}

static int nvt_ts_write_data(struct i2c_client *client, u8 *tx_buf, int tx_size)
{
    int ret;

    struct i2c_msg msg = {
        .addr = client->addr,
        .flags = 0,
        .len = tx_size,
        .buf = tx_buf,
    };

    ret = i2c_transfer(client->adapter, &msg, 1);
    if (ret != 1) {
        dev_err(&client->dev, "Error writing to 0x%02x: %d\n", client->addr, ret);
        return (ret < 0) ? ret : -EIO;
    }

    dev_dbg(&client->dev, "Wrote %d bytes to register 0x%02x: %*ph",
             tx_size, tx_buf[0], tx_size - 1, &tx_buf[1]);
    return 0;
}

static irqreturn_t nvt_ts_irq(int irq, void *dev_id)
{
	struct nvt_ts_data *data = dev_id;
	struct device *dev = &data->client->dev;
	int i, error, slot, x, y;
	bool active;
	u8 *touch;

	error = nvt_ts_read_data(data->client, NVT_TS_TOUCH_START, data->rx_buf,
				 data->max_touches * NVT_TS_TOUCH_SIZE);
	if (error)
		return IRQ_HANDLED;

	for (i = 0; i < data->max_touches; i++) {
		touch = &data->rx_buf[i * NVT_TS_TOUCH_SIZE];

		if (touch[0] == NVT_TS_TOUCH_INVALID)
			continue;

		slot = touch[0] >> NVT_TS_TOUCH_SLOT_SHIFT;
		if (slot < 1 || slot > data->max_touches) {
			dev_warn(dev, "slot %d out of range, ignoring\n", slot);
			continue;
		}

		switch (touch[0] & NVT_TS_TOUCH_TYPE_MASK) {
		case NVT_TS_TOUCH_NEW:
		case NVT_TS_TOUCH_UPDATE:
			active = true;
			break;
		case NVT_TS_TOUCH_RELEASE:
			active = false;
			break;
		default:
			dev_warn(dev, "slot %d unknown state %d\n", slot, touch[0] & 7);
			continue;
		}

		slot--;
		x = (touch[1] << 4) | (touch[3] >> 4);
		y = (touch[2] << 4) | (touch[3] & 0x0f);

		input_mt_slot(data->input, slot);
		input_mt_report_slot_state(data->input, MT_TOOL_FINGER, active);
		touchscreen_report_pos(data->input, &data->prop, x, y, true);
	}

	input_mt_sync_frame(data->input);
	input_sync(data->input);

	return IRQ_HANDLED;
}

static void nvt_ts_hw_reset_to_state(struct nvt_ts_data *data, enum nvt_ts_hw_state end_state)
{
	int gpio_state = gpiod_get_value_cansleep(data->reset_gpio);
	gpiod_set_value_cansleep(data->reset_gpio, gpio_state);
	msleep(30);
	gpiod_set_value_cansleep(data->reset_gpio, !gpio_state);
	msleep(30);
	if (end_state == NVT_TS_HW_STATE_RUNNING) {
		gpiod_set_value_cansleep(data->reset_gpio, gpio_state);
		msleep(30);
	}
}


static int nvt_ts_start(struct input_dev *dev)
{
	struct nvt_ts_data *data = input_get_drvdata(dev);
	int error;

	error = regulator_bulk_enable(ARRAY_SIZE(data->regulators), data->regulators);
	if (error) {
		dev_err(&data->client->dev, "failed to enable regulators\n");
		return error;
	}

	enable_irq(data->client->irq);
	nvt_ts_hw_reset_to_state(data, NVT_TS_HW_STATE_RUNNING);

	return 0;
}


static void nvt_ts_stop(struct input_dev *dev)
{
	struct nvt_ts_data *data = input_get_drvdata(dev);

	disable_irq(data->client->irq);
	nvt_ts_hw_reset_to_state(data, NVT_TS_HW_STATE_STOPPED);
	regulator_bulk_disable(ARRAY_SIZE(data->regulators), data->regulators);
}

static int nvt_ts_suspend(struct device *dev)
{
	struct nvt_ts_data *data = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&data->input->mutex);
	if (input_device_enabled(data->input))
		nvt_ts_stop(data->input);
	mutex_unlock(&data->input->mutex);

	return 0;
}

static int nvt_ts_resume(struct device *dev)
{
	struct nvt_ts_data *data = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&data->input->mutex);
	if (input_device_enabled(data->input))
		nvt_ts_start(data->input);
	mutex_unlock(&data->input->mutex);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(nvt_ts_pm_ops, nvt_ts_suspend, nvt_ts_resume);

static int nvt_ts_probe(struct i2c_client *client)
{
	dev_info(&client->dev, "Novatek Touchscreen probe started\n");
	struct device *dev = &client->dev;
	int error, width, height, irq_type;
	struct nvt_ts_data *data;

	if (!client->irq) {
		dev_err(dev, "Error no irq specified\n");
		return -EINVAL;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		return -ENOMEM;
	}

	data->chip_data = device_get_match_data(dev);
	if (!data->chip_data) {
		return -EINVAL;
	}

	data->client = client;
	i2c_set_clientdata(client, data);

	/*
 	 * VCC is the analog voltage supply
 	 * IOVCC is the digital voltage supply
 	 */
	data->regulators[0].supply = "vcc";
	data->regulators[1].supply = "iovcc";
	error = devm_regulator_bulk_get(dev, ARRAY_SIZE(data->regulators), data->regulators);
	if (error) {
		dev_err(dev, "cannot get regulators: %d\n", error);
		return error;
	}

	error = regulator_bulk_enable(ARRAY_SIZE(data->regulators), data->regulators);
	if (error) {
		dev_err(dev, "failed to enable regulators: %d\n", error);
		return error;
	}

	data->reset_gpio = devm_gpiod_get(dev, "reset", data->chip_data->init_reset_state);
	error = PTR_ERR_OR_ZERO(data->reset_gpio);
	if (error) {
		regulator_bulk_disable(ARRAY_SIZE(data->regulators), data->regulators);
		dev_err(dev, "failed to request reset GPIO: %d\n", error);
		return error;
	}
	
	/* Unlocks reading firmware information */
	if (data->chip_data->unlock_firmware_info) {
		nvt_ts_hw_reset_to_state(data, NVT_TS_HW_STATE_RUNNING);
		nvt_ts_write_data(data->client, data->chip_data->unlock_firmware_data, data->chip_data->unlock_firmware_data_size);
	}

	/* Wait for controller to come out of reset before params read */
	msleep(100);
	error = nvt_ts_read_data(data->client, NVT_TS_FIRMWARE_INFO_START,
		data->rx_buf, NVT_TS_FIRMWARE_INFO_LEN);

	nvt_ts_hw_reset_to_state(data, NVT_TS_HW_STATE_STOPPED);
	regulator_bulk_disable(ARRAY_SIZE(data->regulators), data->regulators);
	if (error)
		return error;
	width  = get_unaligned_be16(&data->rx_buf[NVT_TS_PARAMS_WIDTH]);
	height = get_unaligned_be16(&data->rx_buf[NVT_TS_PARAMS_HEIGHT]);
	data->max_touches = data->rx_buf[NVT_TS_PARAMS_MAX_TOUCH];
	irq_type = data->rx_buf[NVT_TS_PARAMS_IRQ_TYPE];

	dev_info(dev, "Checking touchscreen parameters: width=%d, height=%d, max_touches=%d, irq_type=%d, wake_type=%d, chip_id=%d\n",
		width, height, data->max_touches, irq_type, 
		data->rx_buf[NVT_TS_PARAMS_WAKE_TYPE], data->rx_buf[NVT_TS_PARAMS_CHIP_ID]);

    if (width > NVT_TS_MAX_SIZE || height >= NVT_TS_MAX_SIZE ||
        data->max_touches > NVT_TS_MAX_TOUCHES ||
        irq_type >= ARRAY_SIZE(nvt_ts_irq_type) ||
	    data->rx_buf[NVT_TS_PARAMS_WAKE_TYPE] != data->chip_data->wake_type ||
	    data->rx_buf[NVT_TS_PARAMS_CHIP_ID] != data->chip_data->chip_id) {
		dev_err(dev, "Unsupported touchscreen parameters: %*ph\n",
			NVT_TS_FIRMWARE_INFO_LEN, data->rx_buf);
        return -EIO;
    }

	dev_info(dev, "Detected %dx%d touchscreen with %d max touches\n", width,
		height, data->max_touches);

	if (data->rx_buf[NVT_TS_PARAMS_MAX_BUTTONS])
		dev_warn(dev, "Touchscreen buttons are not supported\n");

	/* Allocate input device */
	data->input = devm_input_allocate_device(dev);
	if (!data->input) {
		dev_err(dev, "Failed to allocate input device\n");
		return -ENOMEM;
	}

	/* Set input device parameters */
	data->input->name = client->name;
	data->input->id.bustype = BUS_I2C;
	data->input->open = nvt_ts_start;
	data->input->close = nvt_ts_stop;
	input_set_abs_params(data->input, ABS_MT_POSITION_X, 0, width - 1, 0, 0);
	input_set_abs_params(data->input, ABS_MT_POSITION_Y, 0, height - 1, 0, 0);
	touchscreen_parse_properties(data->input, true, &data->prop);
	input_mt_init_slots(data->input, data->max_touches, INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (error)
		return error;
	input_set_drvdata(data->input, data);


	/* Set input device callbacks */

	error = devm_request_threaded_irq(dev, client->irq, NULL, nvt_ts_irq,
					  IRQF_ONESHOT | IRQF_NO_AUTOEN | nvt_ts_irq_type[irq_type],
					  client->name, data);
	if (error) {
		dev_err(dev, "failed to request irq: %d\n", error);
		return error;
	}

	/* Register input device */
	error = input_register_device(data->input);
	if (error) {
		dev_err(dev, "failed to register input device: %d\n", error);
		return error;
	}
	
	// Register input device
	dev_info(dev, "Novatek Touchscreen probe completed successfully\n");
	return 0;
}

static const struct nvt_ts_i2c_chip_data nvt_nt11205_ts_data = {
	.wake_type = 0x05,
	.chip_id = 0x05, 
	.init_reset_state = GPIOD_OUT_LOW,
	.unlock_firmware_info = false,
};

static const struct nvt_ts_i2c_chip_data nvt_nt11206_ts_data = {
	.wake_type = 0x01,
	.chip_id = 0x06,
	.init_reset_state = GPIOD_OUT_HIGH,
	.unlock_firmware_info = true,
	.unlock_firmware_data = {0xFF, 0x01, 0x47},
	.unlock_firmware_data_size = 3,
};

static const struct nvt_ts_i2c_chip_data nvt_nt11207_ts_data = {
	.wake_type = 0x01,
	.chip_id = 0x07,
	.init_reset_state = GPIOD_OUT_HIGH,
	.unlock_firmware_info = true, 
	.unlock_firmware_data = {0xFF, 0x01, 0x47},
	.unlock_firmware_data_size = 3,
};

static const struct nvt_ts_i2c_chip_data nvt_nt36672a_ts_data = {
	.wake_type = 0x01,
	.chip_id = 0x08,
	.init_reset_state = GPIOD_OUT_LOW,
	.unlock_firmware_info = false,
};

static const struct of_device_id nvt_ts_of_match[] = {
	{ .compatible = "novatek,nt11205-ts", .data = &nvt_nt11205_ts_data },
	{ .compatible = "novatek,nt11206-ts", .data = &nvt_nt11206_ts_data },
	{ .compatible = "novatek,nt11207-ts", .data = &nvt_nt11207_ts_data },
	{ .compatible = "novatek,nt36672a-ts", .data = &nvt_nt36672a_ts_data },
	{ }
};
MODULE_DEVICE_TABLE(of, nvt_ts_of_match);

static const struct i2c_device_id nvt_ts_i2c_id[] = {
	{ "nt11205-ts", (unsigned long) &nvt_nt11205_ts_data },
	{ "nt11206-ts", (unsigned long) &nvt_nt11206_ts_data },
	{ "nt11207-ts", (unsigned long) &nvt_nt11206_ts_data },
	{ "nt36672a-ts", (unsigned long) &nvt_nt36672a_ts_data },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nvt_ts_i2c_id);

static struct i2c_driver nvt_ts_driver = {
	.driver = {
		.name	= "novatek-nvt-ts",
		.pm	= pm_sleep_ptr(&nvt_ts_pm_ops),
		.of_match_table = nvt_ts_of_match,
	},
	.probe		= nvt_ts_probe,
	.id_table	= nvt_ts_i2c_id,

};

module_i2c_driver(nvt_ts_driver);

MODULE_DESCRIPTION("Novatek NT11205 touchscreen driver");
MODULE_AUTHOR("Hans de Goede <hdegoede@redhat.com>");
MODULE_LICENSE("GPL");
