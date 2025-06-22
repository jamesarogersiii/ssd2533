#ifndef _SSD253X_H
#define _SSD253X_H

#include <linux/gpio/consumer.h>

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

/**
 * struct ssd253x_ts_data - private data for the ssd253x driver
 * @client:      The I2C client device.
 * @input_dev:   The input device structure.
 * @reset_gpio:  GPIO descriptor for the reset pin.
 * @irq:         The interrupt request number for the touchscreen.
 */
struct ssd253x_ts_data {
    struct i2c_client      *client;
    struct input_dev       *input_dev;
    struct gpio_desc       *reset_gpio;
    int                    irq;
};

#endif /* _SSD253X_H */
