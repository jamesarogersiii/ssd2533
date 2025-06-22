/*

&i2c2 { /* <-- IMPORTANT: Use the correct I2C bus for your board! */
    status = "okay";

    /* Solomon SSD2533 Touchscreen Controller */
    touchscreen: touchscreen@2c {
        compatible = "solomon,ssd2533";
        reg = <0x2c>; /* The 7-bit I2C slave address of the chip */
        pinctrl-names = "default";
        pinctrl-0 = <&pinctrl_touch>;
        
        /*
         * VERY IMPORTANT: You MUST get these GPIO numbers from your
         * custom carrier board's schematics. These are examples.
         * The format is <&gpio_controller port_number flags>.
         */
        reset-gpios = <&gpio1 21 GPIO_ACTIVE_LOW>; /* Example: GPIO1_IO21 */

        /*
         * The IRQ pin should be defined in the pinctrl section and
         * referenced here. The driver will get it from client->irq.
         * Ensure your pinctrl correctly configures the GPIO as an interrupt.
         */
         
        /* Touchscreen Resolution Properties */
        touchscreen-size-x = <800>;  /* Replace with your panel's native width */
        touchscreen-size-y = <480>;  /* Replace with your panel's native height */
    };
};


pinctrl_touch: touchgrp {
    fsl,pins = <
        /*
         * This configures a GPIO3 pin as an interrupt.
         * You MUST find the correct pad definition for your interrupt pin.
         * This example uses EIM_D20 for GPIO3_IO20.
         */
        MX6QDL_PAD_EIM_D20__GPIO3_IO20    0x80000000 
    >;
};

*/