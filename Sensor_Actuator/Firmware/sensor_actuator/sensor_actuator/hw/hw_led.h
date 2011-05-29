#ifndef HW_LED_H
#define HW_LED_H

#define LEDS_GPIO_BASE          ((struct stm32_gpio *)GPIOD_BASE)

#define LED_PIN BV(2)

#define LED_ON()   do { \
    stm32_gpioPinWrite(LEDS_GPIO_BASE, LED_PIN, 1);\
    } while(0)
#define LED_OFF()   do { \
    stm32_gpioPinWrite(LEDS_GPIO_BASE, LED_PIN, 0);\
    } while(0)

#define LEDS_INIT()                                                     \
    do {                                                    \
    /* Enable clocking on GPIOA and GPIOC */        \
    RCC->APB2ENR |= RCC_APB2_GPIOD;                 \
    /* Configure the LED pins as GPIO */            \
    stm32_gpioPinConfig(LEDS_GPIO_BASE,             \
        LED_PIN, GPIO_MODE_OUT_PP,\
                        GPIO_SPEED_50MHZ);                          \
    } while(0)

#endif /* HW_LED_H */
