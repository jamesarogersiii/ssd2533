#ifndef _SSD253X_H
#define _SSD253X_H

#include <linux/gpio/consumer.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>

/* Register Defines */
#define SSD253x_READ_ID_ADDR        0x00
#define SSD253x_READ_DATA_ADDR      0x10
#define SSD253x_READ_POINT_ADDR     0x11
#define SSD253x_READ_ALL_POINTS     0x12
#define SSD253x_SET_SCAN_MODE       0x0C
#define SSD253x_SET_PARA_MODE       0x0D
#define SSD253x_SET_POWER_MODE      0x0A

/* Constants */
#define MAX_POINT   5
#define MS_TO_NS(x) ((x) * 1000000)

/**
 * struct ssd253x_ts_data - private data for the ssd253x driver
 * @client:      The I2C client device.
 * @input_dev:   The input device structure.
 * @reset_gpio:  GPIO descriptor for the reset pin.
 * @timer:       Kernel timer for polling for touch events.
 * @work:        Work structure to read data off the timer's context.
 */
struct ssd253x_ts_data {
    struct i2c_client      *client;
    struct input_dev       *input_dev;
    struct gpio_desc       *reset_gpio;
    struct hrtimer         timer;
    struct work_struct     work;
};


static const struct ssd2533_reg_setting ssd2533_init_sequence[] = {
    // Phase 1: Wake Up and Basic Setup
    {0, 0x04, 0x00, 0x00}, // Exit sleep mode
    // --- Delay 10ms after this command ---
    {0, 0x06, 0x14, 0x00}, // Set Drive Lines to 21 (0x14 = 20)
    {0, 0x07, 0x1B, 0x00}, // Set Sense Lines to 28 (0x1B = 27)

    // Phase 2: Drive Line Scanning Order (Example - MUST BE VERIFIED)
    // This configuration is dependent on the FPC layout of your device.
    // The following are examples for the first two lines.
    // You must configure all 21 drive lines from register 0x08 to 0x1C.
    {1, 0x08, 0x00, 0x80}, // Map 1st drive line to physical pin DRIVE00
    {1, 0x09, 0x00, 0x81}, // Map 2nd drive line to physical pin DRIVE01
    // ... Add entries for the remaining 19 drive lines ...

    // Phase 3: Analog, Scanning, and Integration Tuning
    {0, 0xD5, 0x03, 0x00}, // Drive Voltage: 7.0V
    {0, 0xD8, 0x07, 0x00}, // Sense Bias Resistance: 15.0kΩ
    {0, 0x2A, 0x07, 0x00}, // Sub-Frames per Scan: 8
    {0, 0x2C, 0x01, 0x00}, // Enable Median Filter
    {0, 0x2F, 0x01, 0x00}, // Integration Gain: 2x
    {0, 0x30, 0x03, 0x00}, // Integration Window Start Time
    {0, 0x31, 0x07, 0x00}, // Integration Window End Time
    {0, 0xD7, 0x04, 0x00}, // ADC Vref Range
    {0, 0xDB, 0x04, 0x00}, // Integrator Capacitor Value

    // Phase 4: Finger Detection and Filtering
    {1, 0x33, 0x00, 0x01}, // Minimum Finger Area
    {1, 0x34, 0x00, 0x30}, // Minimum Finger Level
    {1, 0x35, 0x00, 0x00}, // Minimum Finger Weight
    {1, 0x36, 0x00, 0x1F}, // Maximum Finger Area
    {0, 0x37, 0x00, 0x00}, // Image Segmentation Depth
    {0, 0x3D, 0x01, 0x00}, // 2D Filter: 1-2-1 filter for delta data
    {0, 0x53, 0x16, 0x00}, // Event Move Tolerance
    {0, 0x56, 0x02, 0x00}, // Enable Moving Average Filter (6:2 ratio)

    // Phase 5: Start Operation
    {0, 0x25, 0x02, 0x00}, // Enter Normal Scan Mode
    // --- Delay 300ms for stabilization after this sequence ---
};

#endif /* _SSD253X_H */
