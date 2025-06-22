/*
 * Solomon Systech SSD253X I2C Touchscreen Driver
 *
 * Version 2: Adapted to use direct GPIO lookup instead of pinctrl.
 * This simplifies the device tree configuration.
 *
 * Key Adaptations:
 * - Probe function now gets the interrupt line via "interrupt-gpios" from DT.
 * - This removes the need for a separate `pinctrl` node in the DTS.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/property.h>

#include "ssd253x.h"

// Forward declaration
static irqreturn_t ssd253x_ts_interrupt(int irq, void *dev_id);

/* ----- Core Driver Logic (largely unchanged) ----- */

static int ssd253x_ts_i2c_read(struct i2c_client *client, u8 addr, u8 *data, int len)
{
    struct i2c_msg msgs[2];
    int ret;

    msgs[0].addr = client->addr;
    msgs[0].flags = 0; // Write
    msgs[0].len = 1;
    msgs[0].buf = &addr;

    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD; // Read
    msgs[1].len = len;
    msgs[1].buf = data;

    ret = i2c_transfer(client->adapter, msgs, 2);
    if (ret < 0)
        dev_err(&client->dev, "i2c read error, addr=0x%x\n", addr);

    return ret;
}

static int ssd253x_ts_i2c_write(struct i2c_client *client, u8 addr, u8 *data, int len)
{
    u8 *buf;
    struct i2c_msg msg;
    int ret;

    buf = kmalloc(len + 1, GFP_KERNEL);
    if (!buf) {
        dev_err(&client->dev, "failed to allocate memory for i2c write\n");
        return -ENOMEM;
    }

    buf[0] = addr;
    memcpy(&buf[1], data, len);

    msg.addr = client->addr;
    msg.flags = 0; // Write
    msg.len = len + 1;
    msg.buf = buf;

    ret = i2c_transfer(client->adapter, &msg, 1);
    if (ret < 0)
        dev_err(&client->dev, "i2c write error, addr=0x%x\n", addr);

    kfree(buf);
    return ret;
}

static void ssd253x_ts_reset(struct ssd253x_ts_data *ts)
{
    if (!ts->reset_gpio)
        return;

    gpiod_set_value_cansleep(ts->reset_gpio, 1);
    msleep(20);
    gpiod_set_value_cansleep(ts->reset_gpio, 0);
    msleep(100); // Wait for the controller to be ready
}

static void ssd253x_ts_report_touch(struct ssd253x_ts_data *ts, u8 *touch_data)
{
    int i;
    int point_num = 0;
    u16 x, y, touch_id;

    point_num = touch_data[0] & 0x0F;

    if (point_num > MAX_POINT)
        point_num = MAX_POINT;

    for (i = 0; i < point_num; i++) {
        touch_id = (touch_data[i * 4 + 1] >> 4) & 0x0F;
        x = ((touch_data[i * 4 + 1] & 0x0F) << 8) | touch_data[i * 4 + 2];
        y = ((touch_data[i * 4 + 3] & 0x0F) << 8) | touch_data[i * 4 + 4];

        input_mt_slot(ts->input_dev, touch_id);
        input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
        input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
        input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
    }

    input_mt_sync_frame(ts->input_dev);
    input_sync(ts->input_dev);
}

static irqreturn_t ssd253x_ts_interrupt(int irq, void *dev_id)
{
    struct ssd253x_ts_data *ts = dev_id;
    u8 data_buf[MAX_POINT * 4 + 1];
    int ret;

    ret = ssd253x_ts_i2c_read(ts->client, SSD253x_READ_DATA_ADDR, data_buf, sizeof(data_buf));
    if (ret < 0) {
        dev_err(&ts->client->dev, "Failed to read touch data\n");
        return IRQ_HANDLED;
    }

    ssd253x_ts_report_touch(ts, data_buf);

    return IRQ_HANDLED;
}


/* ----- Probe and Remove Functions (Adapted for DT) ----- */

static int ssd253x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct ssd253x_ts_data *ts;
    struct input_dev *input_dev;
    struct gpio_desc *irq_gpio;
    int error;
    u32 screen_max_x, screen_max_y;

    dev_info(&client->dev, "probing for SSD253x touchscreen (v2 driver)\n");

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "I2C functionality check failed\n");
        return -EIO;
    }

    ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
    if (!ts)
        return -ENOMEM;

    ts->client = client;
    i2c_set_clientdata(client, ts);

    /* --- GPIO Adaptation --- */
    ts->reset_gpio = devm_gpiod_get_optional(&client->dev, "reset", GPIOD_OUT_LOW);
    if (IS_ERR(ts->reset_gpio)) {
        dev_err(&client->dev, "failed to get reset gpio\n");
        return PTR_ERR(ts->reset_gpio);
    }

    ssd253x_ts_reset(ts);

    /* --- Input Device Setup --- */
    input_dev = devm_input_allocate_device(&client->dev);
    if (!input_dev) {
        dev_err(&client->dev, "failed to allocate input device\n");
        return -ENOMEM;
    }

    ts->input_dev = input_dev;
    input_dev->name = "ssd253x-touchscreen";
    input_dev->id.bustype = BUS_I2C;
    input_dev->dev.parent = &client->dev;

    __set_bit(EV_ABS, input_dev->evbit);
    __set_bit(EV_KEY, input_dev->evbit);
    __set_bit(BTN_TOUCH, input_dev->keybit);

    error = device_property_read_u32(&client->dev, "touchscreen-size-x", &screen_max_x);
    if (error) {
        dev_warn(&client->dev, "touchscreen-size-x not found, using default 800\n");
        screen_max_x = 800;
    }

    error = device_property_read_u32(&client->dev, "touchscreen-size-y", &screen_max_y);
    if (error) {
        dev_warn(&client->dev, "touchscreen-size-y not found, using default 480\n");
        screen_max_y = 480;
    }

    input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, screen_max_x, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, screen_max_y, 0, 0);

    error = input_mt_init_slots(input_dev, MAX_POINT, INPUT_MT_DIRECT);
    if (error) {
        dev_err(&client->dev, "failed to initialize MT slots\n");
        return error;
    }

    /* --- NEW IRQ Setup --- */
    irq_gpio = devm_gpiod_get(&client->dev, "interrupt", GPIOD_IN);
    if (IS_ERR(irq_gpio)) {
        dev_err(&client->dev, "failed to get interrupt gpio\n");
        return PTR_ERR(irq_gpio);
    }
    
    ts->irq = gpiod_to_irq(irq_gpio);
    if (ts->irq < 0) {
        dev_err(&client->dev, "failed to get irq from gpio\n");
        return ts->irq;
    }

    error = devm_request_threaded_irq(&client->dev, ts->irq, NULL,
                                      ssd253x_ts_interrupt,
                                      IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                                      "ssd253x-ts", ts);
    if (error) {
        dev_err(&client->dev, "failed to request irq %d\n", ts->irq);
        return error;
    }
    
    /* Register the input device */
    error = input_register_device(input_dev);
    if (error) {
        dev_err(&client->dev, "failed to register input device\n");
        return error;
    }

    dev_info(&client->dev, "SSD253x touchscreen probed successfully\n");

    return 0;
}

static int ssd253x_ts_remove(struct i2c_client *client)
{
    // devm_ managed resources will be freed automatically.
    dev_info(&client->dev, "removing ssd253x-ts driver\n");
    return 0;
}


/* ----- Driver and DT Matching Structure ----- */

static const struct i2c_device_id ssd253x_ts_id[] = {
    { "ssd2533", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, ssd253x_ts_id);

static const struct of_device_id ssd253x_ts_dt_ids[] = {
    { .compatible = "solomon,ssd2533" },
    { }
};
MODULE_DEVICE_TABLE(of, ssd253x_ts_dt_ids);

static struct i2c_driver ssd253x_ts_driver = {
    .driver = {
        .name   = "ssd253x-ts",
        .owner  = THIS_MODULE,
        .of_match_table = of_match_ptr(ssd253x_ts_dt_ids),
    },
    .probe      = ssd253x_ts_probe,
    .remove     = ssd253x_ts_remove,
    .id_table   = ssd253x_ts_id,
};

module_i2c_driver(ssd253x_ts_driver);

MODULE_AUTHOR("Adapted for standard kernel");
MODULE_DESCRIPTION("Solomon SSD253x I2C Touchscreen Driver (v2)");
MODULE_LICENSE("GPL v2");