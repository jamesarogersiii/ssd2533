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
#include <linux/interrupt.h>

/*============================================================================*/
/* MACRO DEFINITIONS                                                          */
/*============================================================================*/

// These values should be in a header file like ssd253x.h
#define FINGERNO 10
#define SSDS53X_SCREEN_MAX_X 4095
#define SSDS53X_SCREEN_MAX_Y 4095

// Register Definitions
#define DEVICE_ID_REG       0x02
#define VERSION_ID_REG      0x03
#define EVENT_STATUS        0x79
#define FINGER01_REG        0x7C // Start register for finger 0 data
#define EVENT_FIFO_SCLR     0x87

// Define a structure for the configuration commands
struct ssd253x_reg_setting {
    unsigned char No;
    unsigned char Reg;
    unsigned char Data1;
    unsigned char Data2;
};

// This table should be defined in a header or another C file.
// Using the latest version from our discussion.
static const struct ssd253x_reg_setting ssd253xcfgTable[] = {
    // --- Phase 1: Wake Up and Basic Setup ---
    {1, 0x04, 0x00, 0x00}, // Exit sleep mode
    // --- Delay 10ms after this command ---
    
    // --- Phase 2: Self-Capacitance and Scan Rate (from Vendor example) ---
    {1, 0xAC, 0x01, 0x00}, {1, 0xAD, 0x03, 0x00}, {1, 0xAE, 0x0F, 0x00},
    {1, 0xAF, 0x30, 0x00}, {1, 0xB0, 0x00, 0x00}, {1, 0xBB, 0x00, 0x00},
    {1, 0xBC, 0x01, 0x00},

    // --- Phase 3: Drive/Sense and Pin Configuration ---
    {1, 0x06, 0x14, 0x00}, // Set Drive Lines to 21
    {1, 0x07, 0x1B, 0x00}, // Set Sense Lines to 28
    
    // --- Drive Line Scanning Order (VERIFY WITH FPC SCHEMATIC) ---
    {2, 0x08, 0x00, 0x00}, {2, 0x09, 0x00, 0x01}, {2, 0x0A, 0x00, 0x02},
    {2, 0x0B, 0x00, 0x03}, {2, 0x0C, 0x00, 0x04}, {2, 0x0D, 0x00, 0x05},
    {2, 0x0E, 0x00, 0x06}, {2, 0x0F, 0x00, 0x07}, {2, 0x10, 0x00, 0x08},
    {2, 0x11, 0x00, 0x09}, {2, 0x12, 0x00, 0x0A}, {2, 0x13, 0x01, 0x0B},
    {2, 0x14, 0x01, 0x0C}, {2, 0x15, 0x01, 0x0D}, {2, 0x16, 0x01, 0x0E},
    {2, 0x17, 0x01, 0x0F}, {2, 0x18, 0x01, 0x10}, {2, 0x19, 0x01, 0x11},
    {2, 0x1A, 0x01, 0x12}, {2, 0x1B, 0x01, 0x13}, {2, 0x1C, 0x01, 0x14},

    // --- Phase 4: Analog, Scanning, and Integration Tuning ---
    {1, 0xD5, 0x03, 0x00}, {1, 0xD8, 0x07, 0x00}, {1, 0x2A, 0x07, 0x00},
    {1, 0x2C, 0x01, 0x00}, {1, 0x2E, 0x0B, 0x00}, {1, 0x2F, 0x01, 0x00},
    {1, 0x30, 0x03, 0x00}, {1, 0x31, 0x07, 0x00}, {1, 0xD7, 0x04, 0x00},
    {1, 0xDB, 0x04, 0x00},

    // --- Phase 5: Finger Detection and Filtering ---
    {2, 0x33, 0x00, 0x01}, {2, 0x34, 0x00, 0x30}, {2, 0x35, 0x00, 0x00},
    {2, 0x36, 0x00, 0x1F}, {1, 0x37, 0x00, 0x00}, {1, 0x3D, 0x01, 0x00},
    {1, 0x53, 0x16, 0x00}, {1, 0x56, 0x02, 0x00}, {1, 0x58, 0x00, 0x00},
    {1, 0x59, 0x01, 0x00}, {1, 0x5B, 0x20, 0x00},

    // --- Phase 6: Interrupt and Performance Tuning (Crucial) ---
    {2, 0x7A, 0xFF, 0xFF}, {2, 0x7B, 0xFF, 0xFF}, {1, 0x89, 0x01, 0x00},
    {1, 0x8A, 0x0A, 0x00}, {1, 0x8B, 0x10, 0x00}, {1, 0x8C, 0xB0, 0x00},

    // --- Phase 7: Start Operation ---
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
    // To track which finger slots are currently active
    bool finger_slots_active[FINGERNO];
};

static struct workqueue_struct *ssd253x_wq;

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/
static void ssd253x_ts_work(struct work_struct *work);
static void WriteRegister(struct i2c_client *client, uint8_t Reg, unsigned char Data1, unsigned char Data2, int ByteNo);
static void SSD253xdeviceInit(struct i2c_client *client);
static int ReadRegister(struct i2c_client *client, uint8_t reg, int ByteNo);
static irqreturn_t ssd253x_ts_isr(int irq, void *dev_id);

/*============================================================================*/
/* I2C READ/WRITE FUNCTIONS                                                   */
/*============================================================================*/
static void WriteRegister(struct i2c_client *client, uint8_t Reg, unsigned char Data1, unsigned char Data2, int ByteNo)
{
    u8 buf[3] = {Reg, Data1, Data2};
    int ret = i2c_master_send(client, buf, ByteNo + 1);
    if (ret < 0) {
        dev_err(&client->dev, "WriteRegister failed: Reg=0x%02X, ret=%d\n", Reg, ret);
    }
}

static int ReadRegister(struct i2c_client *client, uint8_t reg, int ByteNo)
{
    u8 buf[4];
    int ret;
    struct i2c_msg msgs[] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = 1,
            .buf = &reg,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = ByteNo,
            .buf = buf,
        },
    };

    ret = i2c_transfer(client->adapter, msgs, 2);
    if (ret < 0) {
        dev_err(&client->dev, "ReadRegister failed: Reg=0x%02X, ret=%d\n", reg, ret);
        return ret;
    }

    // Combine bytes into a single integer. Assumes chip sends MSB first.
    if (ByteNo == 1) return buf[0];
    if (ByteNo == 2) return (int)get_unaligned_be16(buf);
    if (ByteNo == 4) return (int)get_unaligned_be32(buf);

    return 0;
}

/*============================================================================*/
/* IRQ HANDLER AND WORKQUEUE                                                  */
/*============================================================================*/
static irqreturn_t ssd253x_ts_isr(int irq, void *dev_id)
{
    struct ssl_ts_priv *ssl_priv = dev_id;
    // Use a workqueue to move I2C communication out of interrupt context
    queue_work(ssd253x_wq, &ssl_priv->ssl_work);
    return IRQ_HANDLED;
}

/**
 * ssd253x_ts_work - This is the core logic that runs after an interrupt.
 */
static void ssd253x_ts_work(struct work_struct *work)
{
    struct ssl_ts_priv *ssl_priv = container_of(work, struct ssl_ts_priv, ssl_work);
    struct input_dev *input = ssl_priv->input;
    int i, event_status, finger_info;
    int xpos, ypos, pressure;
    int active_finger_count = 0;
    bool currently_active_fingers[FINGERNO] = {0};

    // 1. Read the LIVE event status register. This tells us which fingers are down.
    event_status = ReadRegister(ssl_priv->client, EVENT_STATUS, 2);
    if (event_status < 0) {
        dev_err(&ssl_priv->client->dev, "Failed to read event status\n");
        return; // Can't proceed
    }

    // The finger bitmap is in the high byte of the 16-bit status word
    event_status >>= 4;

    // 2. Loop through all finger slots to report ACTIVE fingers
    for (i = 0; i < FINGERNO; i++) {
        if ((event_status >> i) & 0x1) {
            currently_active_fingers[i] = true;
            active_finger_count++;

            // This finger is active, read its full 4-byte data
            finger_info = ReadRegister(ssl_priv->client, FINGER01_REG + i, 4);
            if (finger_info < 0) continue; // Skip on read error

            // ** CORRECTED COORDINATE PARSING **
            // Based on datasheet page 20 (FINGER00_REG description)
            xpos = (finger_info >> 24) & 0xFF; // X[7:0]
            xpos |= ((finger_info >> 12) & 0x0F) << 8; // X[11:8]

            ypos = (finger_info >> 16) & 0xFF; // Y[7:0]
            ypos |= ((finger_info >> 8) & 0x0F) << 8; // Y[11:8]

            pressure = (finger_info >> 4) & 0x0F;

            // Report data for this slot
            input_mt_slot(input, i);
            input_report_key(input, BTN_TOOL_FINGER, 1);
            input_report_abs(input, ABS_MT_POSITION_X, xpos);
            input_report_abs(input, ABS_MT_POSITION_Y, ypos);
            input_report_abs(input, ABS_MT_PRESSURE, pressure);
            input_report_abs(input, ABS_MT_TOUCH_MAJOR, pressure);
        }
    }
    
    // 3. Loop through to find fingers that were LIFTED since the last report
    for (i = 0; i < FINGERNO; i++) {
        if (ssl_priv->finger_slots_active[i] && !currently_active_fingers[i]) {
            // This slot was active, but now it's not. Signal a lift-off.
            input_mt_slot(input, i);
            input_report_key(input, BTN_TOOL_FINGER, 0);
        }
    }

    // 4. Update the driver's internal state
    memcpy(ssl_priv->finger_slots_active, currently_active_fingers, sizeof(currently_active_fingers));
    
    // 5. Report overall touch state and commit the event frame
    input_report_key(input, BTN_TOUCH, active_finger_count > 0);
    input_sync(input);

    // 6. Clear the hardware FIFO to re-arm the interrupt
    WriteRegister(ssl_priv->client, EVENT_FIFO_SCLR, 0x01, 0x00, 1);
}

/*============================================================================*/
/* DRIVER INIT AND PROBE                                                      */
/*============================================================================*/
static void SSD253xdeviceInit(struct i2c_client *client)
{
    int i;
    // Per datasheet, delay is required for stabilization after init
    // A single 300ms delay after the loop is better than many small delays.
    for (i = 0; i < ARRAY_SIZE(ssd253xcfgTable); i++) {
        if (ssd253xcfgTable[i].Reg == 0x04) {
             mdelay(10); // Special delay after sleep out command
        }
        WriteRegister(client, ssd253xcfgTable[i].Reg,
                      ssd253xcfgTable[i].Data1,
                      ssd253xcfgTable[i].Data2,
                      ssd253xcfgTable[i].No);
    }
    mdelay(300); // Final stabilization delay
}

static int ssd253x_ts_probe(struct i2c_client *client)
{
    struct ssl_ts_priv *ssl_priv;
    struct input_dev *input;
    struct gpio_desc *reset_gpio;
    int error;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "I2C functionality check failed\n");
        return -EOPNOTSUPP;
    }

    ssl_priv = devm_kzalloc(&client->dev, sizeof(*ssl_priv), GFP_KERNEL);
    if (!ssl_priv) return -ENOMEM;

    ssl_priv->client = client;
    i2c_set_clientdata(client, ssl_priv);

    // --- Reset Hardware ---
    reset_gpio = devm_gpiod_get_optional(&client->dev, "reset", GPIOD_OUT_LOW);
    if (IS_ERR(reset_gpio)) return dev_err_probe(&client->dev, PTR_ERR(reset_gpio), "Failed to get reset GPIO\n");

    if (reset_gpio) {
        gpiod_set_value_cansleep(reset_gpio, 1);
        msleep(5);
        gpiod_set_value_cansleep(reset_gpio, 0);
        msleep(5);
        gpiod_set_value_cansleep(reset_gpio, 1);
        msleep(100); // Give chip time to boot after reset
    }

    // --- Allocate and Configure Input Device ---
    input = devm_input_allocate_device(&client->dev);
    if (!input) return -ENOMEM;
    
    ssl_priv->input = input;
    input->name = "ssd253x-touchscreen";
    input->id.bustype = BUS_I2C;
    input->dev.parent = &client->dev;

    // Set event capabilities
    __set_bit(EV_ABS, input->evbit);
    __set_bit(EV_KEY, input->evbit);
    __set_bit(BTN_TOUCH, input->keybit);
    __set_bit(BTN_TOOL_FINGER, input->keybit);

    // Set absolute parameters for multi-touch (Type A/B protocol)
    input_set_abs_params(input, ABS_MT_POSITION_X, 0, SSDS53X_SCREEN_MAX_X, 0, 0);
    input_set_abs_params(input, ABS_MT_POSITION_Y, 0, SSDS53X_SCREEN_MAX_Y, 0, 0);
    input_set_abs_params(input, ABS_MT_PRESSURE, 0, 15, 0, 0); // Pressure is 4 bits (0-15)
    input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 15, 0, 0);
    
    // Use Type A protocol for simplicity unless slot-based tracking is fully implemented
    // For Type B, you need to manage tracking IDs manually.
    error = input_mt_init_slots(input, FINGERNO, INPUT_MT_DIRECT);
    if (error) return error;

    // --- Initialize Chip ---
    SSD253xdeviceInit(client);

    // --- Setup IRQ ---
    ssl_priv->irq = client->irq;
    if (ssl_priv->irq <= 0) {
        dev_err(&client->dev, "Invalid IRQ number %d\n", ssl_priv->irq);
        return -EINVAL;
    }
    INIT_WORK(&ssl_priv->ssl_work, ssd253x_ts_work);
    error = devm_request_threaded_irq(&client->dev, ssl_priv->irq, NULL,
                                      ssd253x_ts_isr, IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
                                      client->name, ssl_priv);
    if (error) {
        dev_err(&client->dev, "Failed to request IRQ %d\n", ssl_priv->irq);
        return error;
    }

    // --- Register Input Device ---
    error = input_register_device(input);
    if (error) return error;

    dev_info(&client->dev, "SSD253x touchscreen driver probed successfully\n");
    return 0;
}

static int ssd253x_ts_remove(struct i2c_client *client)
{
    struct ssl_ts_priv *ssl_priv = i2c_get_clientdata(client);
    cancel_work_sync(&ssl_priv->ssl_work);
    // devm_free_irq will be called automatically by managed device framework
    return 0;
}

/* ----- Driver and DT Matching Structure ----- */
static const struct i2c_device_id ssd253x_ts_id[] = {{"ssd2533", 0}, {}};
MODULE_DEVICE_TABLE(i2c, ssd253x_ts_id);
static const struct of_device_id ssd253x_ts_dt_ids[] = {{.compatible = "solomon,ssd2533"}, {}};
MODULE_DEVICE_TABLE(of, ssd253x_ts_dt_ids);

static struct i2c_driver ssd253x_ts_driver = {
    .driver = {
        .name = "ssd253x-ts",
        .of_match_table = of_match_ptr(ssd253x_ts_dt_ids),
    },
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


MODULE_DESCRIPTION("Solomon SSD253x I2C Touchscreen Driver (Corrected)");
MODULE_LICENSE("GPL v2");
