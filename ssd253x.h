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


/**
 * @brief Initialization sequence for the SSD2533QN4A Touchscreen Controller.
 *
 * This table contains the I2C commands to configure the controller upon power-up.
 * It is based on the vendor's example code on page 47 of the datasheet and
 * includes several undocumented but critical registers.
 *
 * The format for each entry is {len, register, data1, data2}, where:
 * - len: 0 for a 1-byte write, 1 for a 2-byte write.
 * - register: The 8-bit register address.
 * - data1: The first data byte.
 * - data2: The second data byte (set to 0x00 for 1-byte writes).
 */

// Define a structure for the configuration commands
struct ssd2533_reg_setting {
    unsigned char len;
    unsigned char reg;
    unsigned char data1;
    unsigned char data2;
};

// Initialization settings array
static const struct ssd2533_reg_setting ssd2533_init_sequence[] = {
    // --- Phase 1: Wake Up and Basic Setup ---
    {0, 0x04, 0x00, 0x00}, // Exit sleep mode
    // --- Delay 10ms after this command ---
    
    // --- Phase 2: Self-Capacitance and Scan Rate (from Vendor example) ---
    // These may be necessary even if the self-cap KEY pins are not used.
    {0, 0xAC, 0x01, 0x00}, // Self-Cap IIR filter
    {0, 0xAD, 0x03, 0x00}, // Scan Rate setting
    {0, 0xAE, 0x0F, 0x00}, // Self-Cap Enable
    {0, 0xAF, 0x30, 0x00}, // Self-Cap Threshold
    {0, 0xB0, 0x00, 0x00}, // Self-Cap R Window
    {0, 0xBB, 0x00, 0x00}, // Self-Cap Window
    {0, 0xBC, 0x01, 0x00}, // Self-Cap Enable

    // --- Phase 3: Drive/Sense and Pin Configuration ---
    {0, 0x06, 0x14, 0x00}, // Set Drive Lines to 21 (0x14 = 20)
    {0, 0x07, 0x1B, 0x00}, // Set Sense Lines to 28 (0x1B = 27)
    
    // --- Drive Line Scanning Order (Corrected based on Register Description) ---
    // This assumes a sequential 1-to-1 mapping. VERIFY THIS WITH YOUR FPC SCHEMATIC.
    // The first data byte sets the "Group" (likely for left/right side pins on the chip).
    // The second data byte sets the physical drive pin to use (DRIVE00, DRIVE01, etc.)
    {1, 0x08, 0x00, 0x00}, // Map logical line 0  to physical pin DRIVE00
    {1, 0x09, 0x00, 0x01}, // Map logical line 1  to physical pin DRIVE01
    {1, 0x0A, 0x00, 0x02}, // Map logical line 2  to physical pin DRIVE02
    {1, 0x0B, 0x00, 0x03}, // Map logical line 3  to physical pin DRIVE03
    {1, 0x0C, 0x00, 0x04}, // Map logical line 4  to physical pin DRIVE04
    {1, 0x0D, 0x00, 0x05}, // Map logical line 5  to physical pin DRIVE05
    {1, 0x0E, 0x00, 0x06}, // Map logical line 6  to physical pin DRIVE06
    {1, 0x0F, 0x00, 0x07}, // Map logical line 7  to physical pin DRIVE07
    {1, 0x10, 0x00, 0x08}, // Map logical line 8  to physical pin DRIVE08
    {1, 0x11, 0x00, 0x09}, // Map logical line 9  to physical pin DRIVE09
    {1, 0x12, 0x00, 0x0A}, // Map logical line 10 to physical pin DRIVE10
    {1, 0x13, 0x01, 0x0B}, // Map logical line 11 to physical pin DRIVE11 (Group 1)
    {1, 0x14, 0x01, 0x0C}, // Map logical line 12 to physical pin DRIVE12 (Group 1)
    {1, 0x15, 0x01, 0x0D}, // Map logical line 13 to physical pin DRIVE13 (Group 1)
    {1, 0x16, 0x01, 0x0E}, // Map logical line 14 to physical pin DRIVE14 (Group 1)
    {1, 0x17, 0x01, 0x0F}, // Map logical line 15 to physical pin DRIVE15 (Group 1)
    {1, 0x18, 0x01, 0x10}, // Map logical line 16 to physical pin DRIVE16 (Group 1)
    {1, 0x19, 0x01, 0x11}, // Map logical line 17 to physical pin DRIVE17 (Group 1)
    {1, 0x1A, 0x01, 0x12}, // Map logical line 18 to physical pin DRIVE18 (Group 1)
    {1, 0x1B, 0x01, 0x13}, // Map logical line 19 to physical pin DRIVE19 (Group 1)
    {1, 0x1C, 0x01, 0x14}, // Map logical line 20 to physical pin DRIVE20 (Group 1)

    // --- Phase 4: Analog, Scanning, and Integration Tuning ---
    {0, 0xD5, 0x03, 0x00}, // Drive Voltage: 7.0V
    {0, 0xD8, 0x07, 0x00}, // Sense Bias Resistance: 15.0kΩ
    {0, 0x2A, 0x07, 0x00}, // Sub-Frames per Scan: 8
    {0, 0x2C, 0x01, 0x00}, // Enable Median Filter
    {0, 0x2E, 0x0B, 0x00}, // Drive Pulse (Undocumented)
    {0, 0x2F, 0x01, 0x00}, // Integration Gain: 2x
    {0, 0x30, 0x03, 0x00}, // Integration Window Start Time
    {0, 0x31, 0x07, 0x00}, // Integration Window End Time
    {0, 0xD7, 0x04, 0x00}, // ADC Vref Range
    {0, 0xDB, 0x04, 0x00}, // Integrator Capacitor Value

    // --- Phase 5: Finger Detection and Filtering ---
    {1, 0x33, 0x00, 0x01}, // Minimum Finger Area
    {1, 0x34, 0x00, 0x30}, // Minimum Finger Level
    {1, 0x35, 0x00, 0x00}, // Minimum Finger Weight
    {1, 0x36, 0x00, 0x1F}, // Maximum Finger Area
    {0, 0x37, 0x00, 0x00}, // Image Segmentation Depth
    {0, 0x3D, 0x01, 0x00}, // 2D Filter: 1-2-1 filter for delta data
    {0, 0x53, 0x16, 0x00}, // Event Move Tolerance
    {0, 0x56, 0x02, 0x00}, // Enable Moving Average Filter (6:2 ratio)
    {0, 0x58, 0x00, 0x00}, // Finger Weight Scaling
    {0, 0x59, 0x01, 0x00}, // Enable Random Walk
    {0, 0x5B, 0x20, 0x00}, // Set Random Walk window

    // --- Phase 6: Interrupt and Performance Tuning (Crucial) ---
    {1, 0x7A, 0xFF, 0xFF}, // Unmask all event types (Touch Down, Move, Lift, etc.)
    {1, 0x7B, 0xFF, 0xFF}, // Unmask all IRQ sources (Finger, FIFO, etc.)
    {0, 0x89, 0x01, 0x00}, // Enable Frame IRQ mode (Undocumented, likely critical)
    {0, 0x8A, 0x0A, 0x00}, // Max Finger (Undocumented)
    {0, 0x8B, 0x10, 0x00}, // 1.5X mode (Undocumented)
    {0, 0x8C, 0xB0, 0x00}, // Edge compensation (Undocumented)

    // --- Phase 7: Start Operation ---
    {0, 0x25, 0x02, 0x00}, // Enter Normal Scan Mode
    // --- Delay 300ms for stabilization after this sequence ---
};


#endif /* _SSD253X_H */
