```
&i2c2 { /* <-- This is bus 2, as you verified */
    status = "okay";

    /* Solomon SSD2533 Touchscreen Controller */
    touchscreen: touchscreen@48 {
        compatible = "solomon,ssd2533";
        reg = <0x48>; /* The 7-bit I2C slave address you verified */

        /*
         * VERY IMPORTANT: You must find the correct UNUSED local pin numbers
         * on gpio3 (0-31) for both the reset and interrupt lines.
         * The pins 5, 14, 15, and 24 are already used.
         *
         * Change these numbers one by one to test.
         */
        reset-gpios     = <&gpio3 0 GPIO_ACTIVE_LOW>; /* Example: GPIO3_IO00 */
        interrupt-gpios = <&gpio3 2 GPIO_ACTIVE_LOW>; /* Example: GPIO3_IO02 */

        /* Touchscreen Resolution Properties */
        touchscreen-size-x = <800>;  /* Replace with your panel's native width */
        touchscreen-size-y = <480>;  /* Replace with your panel's native height */
    };
};
```