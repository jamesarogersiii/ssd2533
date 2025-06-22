/*
 * Solomon Systech SSD253X I2C Touchscreen Driver
 *
 * Version 8: Re-enabled error logging in the polling function.
 * This is the final debugging step to see if the I2C reads are failing
 * after the driver has successfully probed.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/property.h>

#include "ssd253x.h"

#define POLLING_INTERVAL_MS 20 // Poll ~50 times per second

static void ssd253x_ts_work_func(struct work_struct *work);
static int ssd253x_ts_i2c_read(struct i2c_client *client, u8 addr, u8 *data, int len);
static void ssd253x_ts_report_touch(struct ssd253x_ts_data *ts, u8 *touch_data);

static enum hrtimer_restart ssd253x_ts_timer_func(struct hrtimer *timer)
{
    struct ssd253x_ts_data *ts = container_of(timer, struct ssd253x_ts_data, timer);
    schedule_work(&ts->work);
    hrtimer_start(&ts->timer, ktime_set(0, MS_TO_NS(POLLING_INTERVAL_MS)), HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}

static void ssd253x_ts_work_func(struct work_struct *work)
{
    struct ssd253x_ts_data *ts = container_of(work, struct ssd253x_ts_data, work);
    u8 data_buf[MAX_POINT * 4 + 1];
    int point_num;
    int ret;

    ret = ssd253x_ts_i2c_read(ts->client, SSD253x_READ_DATA_ADDR, data_buf, sizeof(data_buf));
    if (ret < 0) {
        /*
         * Re-enabled for debugging. If this message floods the log, the chip
         * is likely returning an error when no finger is present, which may be
         * normal. We only care if it also happens when a finger IS present.
         */
        dev_err(&ts->client->dev, "Polling: i2c read failed with error %d\n", ret);
        return;
    }

    point_num = data_buf[0] & 0x0F;
    if (point_num > 0) {
        ssd253x_ts_report_touch(ts, data_buf);
    }
}

/* ----- Core Driver Logic ----- */

static int ssd253x_ts_i2c_read(struct i2c_client *client, u8 addr, u8 *data, int len)
{
    struct i2c_msg msgs[2];
    int ret;
    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = &addr;
    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = len;
    msgs[1].buf = data;
    ret = i2c_transfer(client->adapter, msgs, 2);
    return ret;
}

static void ssd253x_ts_reset(struct ssd253x_ts_data *ts)
{
    if (ts->reset_gpio) {
        gpiod_set_value_cansleep(ts->reset_gpio, 1);
        msleep(20);
        gpiod_set_value_cansleep(ts->reset_gpio, 0);
        msleep(100);
    }
}

static void ssd253x_ts_report_touch(struct ssd253x_ts_data *ts, u8 *touch_data)
{
    int i, point_num = touch_data[0] & 0x0F;
    u16 x, y, touch_id;

    if (point_num > MAX_POINT) point_num = MAX_POINT;

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

/* ----- Probe and Remove Functions ----- */

static int ssd253x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct ssd253x_ts_data *ts;
    struct input_dev *input_dev;
    int error;
    u32 screen_max_x = 0, screen_max_y = 0;

    dev_info(&client->dev, "probing for SSD253x touchscreen (v8 polling driver with logging)\n");

    ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
    if (!ts) return -ENOMEM;

    ts->client = client;
    i2c_set_clientdata(client, ts);

    ts->reset_gpio = devm_gpiod_get_optional(&client->dev, "reset", GPIOD_OUT_LOW);
    if (IS_ERR(ts->reset_gpio)) {
        dev_err(&client->dev, "Failed to get reset GPIO.\n");
        return PTR_ERR(ts->reset_gpio);
    }

    ssd253x_ts_reset(ts);

    input_dev = devm_input_allocate_device(&client->dev);
    if (!input_dev) return -ENOMEM;

    ts->input_dev = input_dev;
    input_dev->name = "ssd253x-touchscreen-polled";
    input_dev->id.bustype = BUS_I2C;
    input_dev->dev.parent = &client->dev;

    __set_bit(EV_ABS, input_dev->evbit);
    __set_bit(EV_KEY, input_dev->keybit);
    __set_bit(BTN_TOUCH, input_dev->keybit);

    device_property_read_u32(&client->dev, "touchscreen-size-x", &screen_max_x);
    device_property_read_u32(&client->dev, "touchscreen-size-y", &screen_max_y);
    input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, screen_max_x ?: 800, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, screen_max_y ?: 480, 0, 0);
    error = input_mt_init_slots(input_dev, MAX_POINT, INPUT_MT_DIRECT);
    if (error) return error;

    error = input_register_device(input_dev);
    if (error) return error;

    INIT_WORK(&ts->work, ssd253x_ts_work_func);
    hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    ts->timer.function = ssd253x_ts_timer_func;
    hrtimer_start(&ts->timer, ktime_set(0, MS_TO_NS(POLLING_INTERVAL_MS)), HRTIMER_MODE_REL);

    dev_info(&client->dev, "SSD253x polling driver probed successfully\n");
    return 0;
}

static int ssd253x_ts_remove(struct i2c_client *client)
{
    struct ssd253x_ts_data *ts = i2c_get_clientdata(client);
    hrtimer_cancel(&ts->timer);
    cancel_work_sync(&ts->work);
    dev_info(&client->dev, "removing ssd253x-ts-polled driver\n");
    return 0;
}

/* ----- Driver and DT Matching Structure ----- */
static const struct i2c_device_id ssd253x_ts_id[] = { { "ssd2533", 0 }, { } };
MODULE_DEVICE_TABLE(i2c, ssd253x_ts_id);
static const struct of_device_id ssd253x_ts_dt_ids[] = { { .compatible = "solomon,ssd2533" }, { } };
MODULE_DEVICE_TABLE(of, ssd253x_ts_dt_ids);
static struct i2c_driver ssd253x_ts_driver = {
    .driver = { .name = "ssd253x-ts", .owner = THIS_MODULE, .of_match_table = of_match_ptr(ssd253x_ts_dt_ids), },
    .probe = ssd253x_ts_probe,
    .remove = ssd253x_ts_remove,
    .id_table = ssd253x_ts_id,
};
module_i2c_driver(ssd253x_ts_driver);

MODULE_AUTHOR("Adapted for standard kernel");
MODULE_DESCRIPTION("Solomon SSD253x I2C Touchscreen Driver (v8 - Logging)");
MODULE_LICENSE("GPL v2");
