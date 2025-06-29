#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/property.h>
#include <linux/interrupt.h>
#include <linux/unaligned/be_byteshift.h>

/*============================================================================*/
/* MACRO DEFINITIONS                                                          */
/*============================================================================*/

#define FINGERNO 10

#define X_SCALING_REG       0x66
#define Y_SCALING_REG       0x67
#define X_OFFSET_REG        0x68
#define Y_OFFSET_REG        0x69
#define MIN_LEVEL_REG       0x34
#define DRIVE_LEVEL_REG     0xD5
#define INT_GAIN_REG        0x2F
#define DEVICE_ID_REG       0x02
#define VERSION_ID_REG      0x03
#define EVENT_STATUS        0x79
#define FINGER01_REG        0x7C
#define EVENT_FIFO_SCLR     0x87

struct ssd253x_reg_setting {
    unsigned char No;
    unsigned char Reg;
    unsigned char Data1;
    unsigned char Data2;
};

static const struct ssd253x_reg_setting ssd253xcfgTable[] = {
    // --- Using the known-good initialization sequence ---
    {1, 0x04, 0x00, 0x00}, {1, 0xAC, 0x01, 0x00}, {1, 0xAD, 0x03, 0x00},
    {1, 0xAE, 0x0F, 0x00}, {1, 0xAF, 0x30, 0x00}, {1, 0xB0, 0x00, 0x00},
    {1, 0xBB, 0x00, 0x00}, {1, 0xBC, 0x01, 0x00},
    {1, 0x06, 0x13, 0x00}, // 20 Drive Lines
    {1, 0x07, 0x1B, 0x00}, // 28 Sense Lines
    {2, 0x08, 0x01, 0x94}, {2, 0x09, 0x01, 0x93}, {2, 0x0A, 0x01, 0x92},
    {2, 0x0B, 0x01, 0x91}, {2, 0x0C, 0x01, 0x90}, {2, 0x0D, 0x01, 0x8F},
    {2, 0x0E, 0x01, 0x8E}, {2, 0x0F, 0x01, 0x8D}, {2, 0x10, 0x01, 0x8C},
    {2, 0x11, 0x01, 0x8B}, {2, 0x12, 0x00, 0x8A}, {2, 0x13, 0x00, 0x89},
    {2, 0x14, 0x00, 0x88}, {2, 0x15, 0x00, 0x87}, {2, 0x16, 0x00, 0x86},
    {2, 0x17, 0x00, 0x85}, {2, 0x18, 0x00, 0x84}, {2, 0x19, 0x00, 0x83},
    {2, 0x1A, 0x00, 0x82}, {2, 0x1B, 0x00, 0x81},
    {1, DRIVE_LEVEL_REG, 0x07, 0x00}, {1, 0xD8, 0x07, 0x00},
    {1, 0x2A, 0x07, 0x00}, {1, 0x2C, 0x01, 0x00}, {1, 0x2E, 0x0B, 0x00},
    {1, INT_GAIN_REG, 0x03, 0x00}, {1, 0x30, 0x03, 0x00},
    {1, 0x31, 0x07, 0x00}, {1, 0xD7, 0x04, 0x00}, {1, 0xDB, 0x04, 0x00},
    {2, 0x33, 0x00, 0x01}, {2, MIN_LEVEL_REG, 0x00, 0x30},
    {2, 0x35, 0x00, 0x00}, {2, 0x36, 0x00, 0x1F}, {1, 0x37, 0x00, 0x00},
    {1, 0x3D, 0x01, 0x00}, {1, 0x53, 0x16, 0x00}, {1, 0x56, 0x02, 0x00},
    {1, 0x58, 0x00, 0x00}, {1, 0x59, 0x01, 0x00}, {1, 0x5B, 0x20, 0x00},
    {2, 0x7A, 0xFF, 0xFF}, {2, 0x7B, 0xFF, 0xFF}, {1, 0x89, 0x01, 0x00},
    {1, 0x8A, 0x0A, 0x00}, {1, 0x8B, 0x10, 0x00}, {1, 0x8C, 0xB0, 0x00},
    {2, X_SCALING_REG, 0x90, 0x00}, {2, Y_SCALING_REG, 0x6A, 0x50},
    {1, X_OFFSET_REG, 0x00, 0x00}, {1, Y_OFFSET_REG, 0x00, 0x00},
    {1, 0x25, 0x02, 0x00},
};

/*============================================================================*/
/* DRIVER PRIVATE DATA                                                        */
/*============================================================================*/
struct ssl_ts_priv {
    struct i2c_client *client;
    struct input_dev *input;
    struct work_struct ssl_work;
    int irq;
    u32 max_x;
    u32 max_y;
    // Flags for coordinate transformation from device tree
    bool swap_x_y;
    bool invert_x;
    bool invert_y;
};

static struct workqueue_struct *ssd253x_wq;

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/
static void ssd253x_ts_work(struct work_struct *work);
static void WriteRegister(struct i2c_client *client, u8 Reg, u8 Data1, u8 Data2, int ByteNo);
static void SSD253xdeviceInit(struct ssl_ts_priv *p);
static int ReadDataBlock(struct i2c_client *client, u8 reg, u8 *buf, int len);
static irqreturn_t ssd253x_ts_isr(int irq, void *dev_id);

/*============================================================================*/
/* I2C READ/WRITE FUNCTIONS                                                   */
/*============================================================================*/
static void WriteRegister(struct i2c_client *client, u8 Reg, u8 Data1, u8 Data2, int ByteNo)
{
    u8 buf[3] = {Reg, Data1, Data2};
    if (i2c_master_send(client, buf, ByteNo + 1) < 0)
        dev_err(&client->dev, "WriteRegister failed: Reg=0x%02X\n", Reg);
}

static int ReadDataBlock(struct i2c_client *client, u8 reg, u8 *buf, int len)
{
    int ret = i2c_smbus_read_i2c_block_data(client, reg, len, buf);
    if (ret < 0) {
        dev_err(&client->dev, "ReadDataBlock failed: Reg=0x%02X, ret=%d\n", reg, ret);
        return ret;
    }
    return 0;
}

/*============================================================================*/
/* IRQ HANDLER AND WORKQUEUE                                                  */
/*============================================================================*/
static irqreturn_t ssd253x_ts_isr(int irq, void *dev_id)
{
    queue_work(ssd253x_wq, &((struct ssl_ts_priv *)dev_id)->ssl_work);
    return IRQ_HANDLED;
}

static void ssd253x_ts_work(struct work_struct *work)
{
    struct ssl_ts_priv *p = container_of(work, struct ssl_ts_priv, ssl_work);
    int i, status, ret, x, y, pressure, temp, count = 0;
    u8 finger_buf[4]; // Buffer to hold the 4 bytes of finger data

    ret = ReadDataBlock(p->client, EVENT_STATUS, (u8 *)&status, 2);
    if (ret < 0) return;
    
    status = be16_to_cpu(status); // Convert from big-endian
    status >>= 4;

    for (i = 0; i < FINGERNO; i++) {
        if ((status >> i) & 0x1) {
            count++;
            // Read the 4-byte data packet for the active finger
            ret = ReadDataBlock(p->client, FINGER01_REG + i, finger_buf, 4);
            if (ret < 0) continue;

            // ** CORRECTED COORDINATE PARSING FROM RAW BUFFER **
            // Per datasheet page 31 for FINGERxx_REG
            x = finger_buf[0];                  // Byte 1: x-coor[7:0]
            y = finger_buf[1];                  // Byte 2: y-coor[7:0]
            x |= (finger_buf[2] & 0xF0) << 4;   // Byte 3: x-coor[11:8]
            y |= (finger_buf[2] & 0x0F) << 8;   // Byte 3: y-coor[11:8]
            pressure = (finger_buf[3] >> 4) & 0x0F; // Byte 4: pressure

            // Apply coordinate transformations
            if (p->swap_x_y) { temp = x; x = y; y = temp; }
            if (p->invert_x) x = p->max_x - x;
            if (p->invert_y) y = p->max_y - y;

            input_report_abs(p->input, ABS_MT_TRACKING_ID, i);
            input_report_abs(p->input, ABS_MT_POSITION_X, x);
            input_report_abs(p->input, ABS_MT_POSITION_Y, y);
            input_report_abs(p->input, ABS_MT_PRESSURE, pressure);
            input_mt_sync(p->input);
        }
    }

    input_report_key(p->input, BTN_TOUCH, count > 0);
    if (count == 0) input_mt_sync(p->input);
    input_sync(p->input);

    WriteRegister(p->client, EVENT_FIFO_SCLR, 0x01, 0x00, 1);
}

/*============================================================================*/
/* DRIVER INIT AND PROBE                                                      */
/*============================================================================*/
static void SSD253xdeviceInit(struct ssl_ts_priv *p)
{
    int i;
    struct i2c_client *client = p->client;

    for (i = 0; i < ARRAY_SIZE(ssd253xcfgTable); i++) {
        if (ssd253xcfgTable[i].Reg == 0x04) msleep(10);
        WriteRegister(client, ssd253xcfgTable[i].Reg,
                      ssd253xcfgTable[i].Data1,
                      ssd253xcfgTable[i].Data2,
                      ssd253xcfgTable[i].No);
    }
    msleep(300);
}

static int ssd253x_ts_probe(struct i2c_client *client)
{
    struct ssl_ts_priv *ssl_priv;
    struct input_dev *input;
    int error;

    ssl_priv = devm_kzalloc(&client->dev, sizeof(*ssl_priv), GFP_KERNEL);
    if (!ssl_priv) return -ENOMEM;
    ssl_priv->client = client;
    i2c_set_clientdata(client, ssl_priv);

    // Read properties from Device Tree
    ssl_priv->swap_x_y = device_property_read_bool(&client->dev, "touchscreen-swapped-x-y");
    ssl_priv->invert_x = device_property_read_bool(&client->dev, "touchscreen-inverted-x");
    ssl_priv->invert_y = device_property_read_bool(&client->dev, "touchscreen-inverted-y");
    
    device_property_read_u32(&client->dev, "touchscreen-size-x", &ssl_priv->max_x);
    device_property_read_u32(&client->dev, "touchscreen-size-y", &ssl_priv->max_y);

    if (!ssl_priv->max_x || !ssl_priv->max_y) {
        dev_warn(&client->dev, "Touchscreen size not set in DT, using default 4095x4095\n");
        ssl_priv->max_x = 4095;
        ssl_priv->max_y = 4095;
    }
    
    input = devm_input_allocate_device(&client->dev);
    if (!input) return -ENOMEM;
    ssl_priv->input = input;
    input->name = "ssd253x-touchscreen";
    input->id.bustype = BUS_I2C;
    input->dev.parent = &client->dev;

    __set_bit(EV_ABS, input->evbit);
    __set_bit(EV_KEY, input->keybit);
    __set_bit(BTN_TOUCH, input->keybit);

    input_set_abs_params(input, ABS_MT_POSITION_X, 0, ssl_priv->max_x, 0, 0);
    input_set_abs_params(input, ABS_MT_POSITION_Y, 0, ssl_priv->max_y, 0, 0);
    input_set_abs_params(input, ABS_MT_PRESSURE, 0, 15, 0, 0);
    input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, FINGERNO - 1, 0, 0);
    
    SSD253xdeviceInit(ssl_priv);

    ssl_priv->irq = client->irq;
    if (ssl_priv->irq <= 0) return -EINVAL;
    INIT_WORK(&ssl_priv->ssl_work, ssd253x_ts_work);
    error = devm_request_threaded_irq(&client->dev, ssl_priv->irq, NULL,
                                      ssd253x_ts_isr, IRQF_ONESHOT,
                                      "ssd253x-touch", ssl_priv);
    if (error) return error;

    error = input_register_device(input);
    if (error) return error;

    return 0;
}

static int ssd253x_ts_remove(struct i2c_client *client)
{
    struct ssl_ts_priv *ssl_priv = i2c_get_clientdata(client);
    disable_irq(ssl_priv->irq);
    cancel_work_sync(&ssl_priv->ssl_work);
    return 0;
}

static const struct i2c_device_id ssd253x_ts_id[] = {{"ssd2533", 0}, {}};
MODULE_DEVICE_TABLE(i2c, ssd253x_ts_id);
static const struct of_device_id ssd253x_ts_dt_ids[] = {{.compatible = "solomon,ssd2533"}, {}};
MODULE_DEVICE_TABLE(of, ssd253x_ts_dt_ids);

static struct i2c_driver ssd253x_ts_driver = {
    .driver = {.name = "ssd253x-ts", .of_match_table = of_match_ptr(ssd253x_ts_dt_ids)},
    .probe = ssd253x_ts_probe,
    .remove = ssd253x_ts_remove,
    .id_table = ssd253x_ts_id,
};

static int __init ssd253x_ts_init(void)
{
    ssd253x_wq = alloc_ordered_workqueue("ssd253x_wq", 0);
    if (!ssd253x_wq) return -ENOMEM;
    return i2c_add_driver(&ssd253x_ts_driver);
}

static void __exit ssd253x_ts_exit(void)
{
    i2c_del_driver(&ssd253x_ts_driver);
    if (ssd253x_wq) destroy_workqueue(ssd253x_wq);
}

module_init(ssd253x_ts_init);
module_exit(ssd253x_ts_exit);

MODULE_AUTHOR("Your Name Here");
MODULE_DESCRIPTION("Solomon SSD253x I2C Touchscreen Driver (with DT transforms)");
MODULE_LICENSE("GPL v2");
