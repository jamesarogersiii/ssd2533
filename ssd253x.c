/*
 * Solomon Systech SSD253X I2C Touchscreen Driver
 *
 * Version 11: Skips the first command in the initialization table.
 * The first command {0x0c, 0x01} was causing an I2C write error, so this
 * version bypasses it while still executing the rest of the sequence.
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
static int ssd253x_ts_i2c_write_reg(struct i2c_client *client, u8 reg, u8 val);
static void ssd253x_ts_report_touch(struct ssd253x_ts_data *ts, u8 *touch_data);

// --- Initialization Table from ssd2533.c ---
static const u8 ssd2533_init_data[][2] = {
    {0x0c, 0x01}, {0x40, 0x02}, {0x41, 0x02}, {0x42, 0x02}, {0x43, 0x02},
    {0x44, 0x02}, {0x45, 0x02}, {0x46, 0x02}, {0x47, 0x02}, {0x48, 0x02},
    {0x49, 0x02}, {0x4a, 0x02}, {0x4b, 0x02}, {0x4c, 0x02}, {0x4d, 0x02},
    {0x4e, 0x02}, {0x4f, 0x02}, {0x50, 0x02}, {0x51, 0x02}, {0x52, 0x02},
    {0x53, 0x02}, {0x54, 0x02}, {0x55, 0x02}, {0x56, 0x02}, {0x57, 0x02},
    {0x29, 0x00}, {0x80, 0x78}, {0x81, 0x1e}, {0x82, 0x01}, {0x83, 0x0c},
    {0x84, 0x0c}, {0x85, 0x08}, {0x86, 0x14}, {0x87, 0x14}, {0x88, 0x0a},
    {0x89, 0x0a}, {0x8a, 0x1e}, {0x8b, 0x82}, {0x8c, 0x00}, {0x8d, 0x00},
    {0x8e, 0x00}, {0x8f, 0x03}, {0x90, 0x03}, {0x91, 0x00}, {0x92, 0x00},
    {0x93, 0x00}, {0x94, 0x00}, {0x95, 0x00}, {0x96, 0x00}, {0x97, 0x00},
    {0x98, 0x00}, {0x99, 0x00}, {0x9a, 0x00}, {0x9b, 0x00}, {0x9c, 0x00},
    {0x9d, 0x00}, {0x9e, 0x00}, {0x9f, 0x00}, {0xa0, 0x00}, {0xa1, 0x00},
    {0xa2, 0x00}, {0xa3, 0x00}, {0xa4, 0x00}, {0xa5, 0x00}, {0xa6, 0x00},
    {0xa7, 0x00}, {0xa8, 0x00}, {0xa9, 0x00}, {0xaa, 0x00}, {0xab, 0x00},
    {0xac, 0x00}, {0xad, 0x00}, {0xae, 0x00}, {0xaf, 0x00}, {0xb0, 0x00},
    {0xb1, 0x00}, {0xb2, 0x00}, {0xb3, 0x00}, {0xb4, 0x00}, {0xb5, 0x00},
    {0xb6, 0x00}, {0xb7, 0x00}, {0xb8, 0x00}, {0xb9, 0x00}, {0xba, 0x00},
    {0xbb, 0x00}, {0xbc, 0x00}, {0xbd, 0x00}, {0xbe, 0x00}, {0xbf, 0x00},
    {0xc0, 0x00}, {0xc1, 0x00}, {0xc2, 0x00}, {0xc3, 0x00}, {0xc4, 0x00},
    {0xc5, 0x00}, {0xc6, 0x00}, {0xc7, 0x00}, {0xc8, 0x00}, {0xc9, 0x00},
    {0xca, 0x00}, {0xcb, 0x00}, {0xcc, 0x00}, {0xcd, 0x00}, {0xce, 0x00},
    {0xcf, 0x00}, {0xd0, 0x00}, {0xd1, 0x00}, {0xd2, 0x00}, {0xd3, 0x00},
    {0xd4, 0x00},
};

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

static int ssd253x_ts_i2c_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
    u8 buf[2] = {reg, val};
    struct i2c_msg msg = {
        .addr = client->addr,
        .flags = 0,
        .len = 2,
        .buf = buf,
    };
    int ret;

    ret = i2c_transfer(client->adapter, &msg, 1);
    if (ret < 0) {
        dev_err(&client->dev, "i2c write failed for reg 0x%02x, val 0x%02x\n", reg, val);
    }
    return ret;
}

static int ssd253x_ts_init_chip(struct i2c_client *client)
{
    int i;
    dev_info(&client->dev, "Starting chip initialization sequence...\n");
    // ** IMPORTANT: Start loop at 1 to skip the problematic {0x0c, 0x01} command **
    for (i = 1; i < ARRAY_SIZE(ssd2533_init_data); i++) {
        ssd253x_ts_i2c_write_reg(client, ssd2533_init_data[i][0], ssd2533_init_data[i][1]);
        msleep(2); // Small delay between writes
    }
    dev_info(&client->dev, "Chip initialization sequence complete.\n");
    return 0;
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

    dev_info(&client->dev, "probing for SSD253x touchscreen (v11 polling driver with corrected init)\n");

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

    // ** NEW STEP: Perform the full initialization sequence **
    error = ssd253x_ts_init_chip(client);
    if (error) {
        dev_err(&client->dev, "Chip initialization failed. Aborting probe.\n");
        return error;
    }


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
MODULE_DESCRIPTION("Solomon SSD253x I2C Touchscreen Driver (v11 - Skip First Init Cmd)");
MODULE_LICENSE("GPL v2");
