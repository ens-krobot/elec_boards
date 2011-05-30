#include "adc.h"

void sa_adc_init(void) {

    // Initialize the pins in the Analog IN mode
    // (should be the default, but anyway...)
    stm32_gpioPinConfig((struct stm32_gpio *)GPIOC_BASE,
                        BV(0) | BV(1) | BV(2) | BV(3),
                        GPIO_MODE_AIN,
                        GPIO_SPEED_50MHZ);

    stm32_gpioPinConfig((struct stm32_gpio *)GPIOA_BASE,
                        BV(0) | BV(1) | BV(2) | BV(3),
                        GPIO_MODE_AIN,
                        GPIO_SPEED_50MHZ);


    // BeRTOS ADC initialization
    adc_init();

    // Yep, that's all folks...
}

void get_adc_values(adc_values *pkt1, adc_values *pkt2) {
    pkt1->p.val1 = adc_read(10);
    pkt1->p.val2 = adc_read(11);
    pkt1->p.val3 = adc_read(12);
    pkt1->p.val4 = adc_read(13);

    pkt2->p.val1 = adc_read(0);
    pkt2->p.val2 = adc_read(1);
    pkt2->p.val3 = adc_read(2);
    pkt2->p.val4 = adc_read(3);

}
