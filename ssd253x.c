/*
 * Solomon Systech SSD253X I2C Touchscreen Driver
 *
 * Version 9: Final Debugging - Chip ID Read Test.
 * The polling function now attempts to read the Chip ID register (0x00)
 * instead of the touch data register (0x10).
 *
 * This will tell us if the chip is still alive on the I2C bus at all,
 * or if it's specifically rejecting the command to read touch data.
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

#define POLLING_INTERVAL_MS 1000 // Poll once per second for this test

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
    u8 data_buf[4]; // Buffer for chip ID
    int ret;

    // --- CHIP ID READ TEST ---
    // Instead of reading touch data, we try to read the chip ID register.
    ret = ssd253x_ts_i2c_read(ts->client, SSD253x_READ_ID_ADDR, data_buf, sizeof(data_buf));
    if (ret < 0) {
        dev_err(&ts->client->dev, "Polling: Chip ID read failed with error %d\n", ret);
        return;
    }

    // If the read succeeds, log the ID we got.
    dev_info(&ts->client->dev, "Polling: Successfully read Chip ID: 0x%02x 0x%02x 0x%02x 0x%02x\n",
             data_buf[0], data_buf[1], data_buf[2], data_buf[3]);

    // We don't report any touch events in this test driver.
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

/* This function is no longer called in v9 */
static void ssd253x_ts_report_touch(struct ssd253x_ts_data *ts, u8 *touch_data) { }

/* ----- Probe and Remove Functions ----- */

static int ssd253x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct ssd253x_ts_data *ts;
    int error;

    dev_info(&client->dev, "probing for SSD253x touchscreen (v9 Chip ID test driver)\n");

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

    // We don't set up an input device for this test driver.
    // We only care about the I2C communication.

    INIT_WORK(&ts->work, ssd253x_ts_work_func);
    hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    ts->timer.function = ssd253x_ts_timer_func;
    hrtimer_start(&ts->timer, ktime_set(0, MS_TO_NS(POLLING_INTERVAL_MS)), HRTIMER_MODE_REL);

    dev_info(&client->dev, "SSD253x Chip ID test driver probed successfully\n");
    return 0;
}

static int ssd253x_ts_remove(struct i2c_client *client)
{
    struct ssd253x_ts_data *ts = i2c_get_clientdata(client);
    hrtimer_cancel(&ts->timer);
    cancel_work_sync(&ts->work);
    dev_info(&client->dev, "removing ssd253x-ts-test driver\n");
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
MODULE_DESCRIPTION("Solomon SSD253x I2C Touchscreen Driver (v9 - Chip ID Test)");
MODULE_LICENSE("GPL v2");
