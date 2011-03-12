/*
 *  Firmware for Controller Motor STM32
 *
 *  Author : Xavier Lagorce <Xavier.Lagorce@crans.org>
 */
#include <cpu/irq.h>
#include <cfg/debug.h>
#include <drv/timer.h>
#include <drv/gpio_stm32.h>
#include <kern/proc.h>
#include <kern/monitor.h>
#include <math.h>

#include "hw/hw_led.h"
#include "motor.h"

static void init(void)
{
	IRQ_ENABLE;

        // Remapping peripherals
	// Enable clocking on AFIO
	RCC->APB2ENR |= RCC_APB2_AFIO;
        // Remap UART3
        stm32_gpioRemap(GPIO_PARTIALREMAP_USART3, GPIO_REMAP_ENABLE);
        // UART3 TX in ouput AF mode
	stm32_gpioPinConfig((struct stm32_gpio *)GPIOB_BASE, BV(10),
				GPIO_MODE_AF_PP, GPIO_SPEED_50MHZ);

	/* Initialize debugging module (allow kprintf(), etc.) */
	kdbg_init();
	/* Initialize system timer */
	timer_init();

	/*
	 * Kernel initialization: processes (allow to create and dispatch
	 * processes using proc_new()).
	 */
	proc_init();

        // Initialize LED driver
        LEDS_INIT();

        // Initialize MOTOR driver
        motorsInit();
}

static void NORETURN speaktome_process(void)
{
  double compt = 0.856;
  /* Periodically speak to me */
  while (1)
  {
    compt = 2*M_PI*sin(compt);
    kprintf("Coucou ! Tu veux voir mes %f bits ?\n", compt);
    timer_delay(500);
  }
}

static void NORETURN blink_process(void)
{
	/* blinkenlichten ! */
	while (1)
	{
          LED1_ON();
          LED2_OFF();
          LED3_ON();
          LED4_OFF();
          timer_delay(200);
          LED1_OFF();
          LED2_ON();
          LED3_OFF();
          LED4_ON();
          timer_delay(200);
	}
}

static void NORETURN square_process(void)
{
	/* blinkenlichten ! */
	while (1)
	{
          motorSetSpeed(MOTOR3 | MOTOR4, 500);
          timer_delay(2000);
          motorSetSpeed(MOTOR3, -500);
          timer_delay(2000);
          motorSetSpeed(MOTOR3 | MOTOR4, 500);
          timer_delay(2000);
          motorSetSpeed(MOTOR4, -500);
          timer_delay(2000);
          motorSetSpeed(MOTOR3 | MOTOR4, 500);
          timer_delay(2000);
          motorSetSpeed(MOTOR3 | MOTOR4, 1000);
          timer_delay(2000);
          motorSetSpeed(MOTOR3 | MOTOR4, 1500);
          timer_delay(2000);
          motorSetSpeed(MOTOR3 | MOTOR4, 2000);
          timer_delay(2000);
          motorSetSpeed(MOTOR3 | MOTOR4, 2500);
          timer_delay(2000);
          motorSetSpeed(MOTOR3 | MOTOR4, 3000);
          timer_delay(2000);
          motorSetSpeed(MOTOR3 | MOTOR4, 3500);
          timer_delay(2000);
          motorSetSpeed(MOTOR3 | MOTOR4, 0);
          timer_delay(2000);
          break;
	}
}


int main(void)
{
	init();

	/* Create a new child process */
	proc_new(speaktome_process, NULL, KERN_MINSTACKSIZE * 2, NULL);
        proc_new(blink_process, NULL, KERN_MINSTACKSIZE * 2, NULL);
        proc_new(square_process, NULL, KERN_MINSTACKSIZE * 2, NULL);

        enableMotor(MOTOR3 | MOTOR4);

	/*
	 * The main process is kept to periodically report the stack
	 * utilization of all the processes (1 probe per second).
	 */
	while (1)
	{
      		monitor_report();
		timer_delay(2000);
	}

	return 0;
}
