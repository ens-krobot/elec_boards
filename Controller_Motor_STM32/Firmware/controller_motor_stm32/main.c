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
#include "encoder.h"
#include "can_monitor.h"

static void init(void)
{
	IRQ_ENABLE;

	/* Initialize debugging module (allow kprintf(), etc.) */
	kdbg_init();
	/* Initialize system timer */
	timer_init();

	/*
	 * Kernel initialization: processes (allow to create and dispatch
	 * processes using proc_new()).
	 */
	proc_init();

        // Initialize MOTOR driver
        motorsInit();

        // Initialize ENCODER driver
        encodersInit();

        // Initialize CAN_MONITOR
        canMonitorInit();

        // Blink to say we are ready
        for (uint8_t i=0; i < 2; i++) {
          LED1_ON();
          LED2_ON();
          LED3_ON();
          LED4_ON();
          timer_delay(250);
          LED1_OFF();
          LED2_OFF();
          LED3_OFF();
          LED4_OFF();
          timer_delay(250);
        }
}

static void NORETURN square_process(void)
{
        // Let's roll !
	while (1)
	{
          motorSetSpeed(MOTOR3 | MOTOR4, 500);
          timer_delay(1000);
          motorSetSpeed(MOTOR3, -500);
          timer_delay(1000);
          motorSetSpeed(MOTOR3 | MOTOR4, 500);
          timer_delay(1000);
          motorSetSpeed(MOTOR4, -500);
          timer_delay(1000);
          motorSetSpeed(MOTOR3 | MOTOR4, 500);
          timer_delay(1000);
          motorSetSpeed(MOTOR3 | MOTOR4, 1000);
          timer_delay(1000);
          motorSetSpeed(MOTOR3 | MOTOR4, 1500);
          timer_delay(1000);
          //
          disableMotor(MOTOR3 | MOTOR4);
          /*motorSetSpeed(MOTOR3 | MOTOR4, 2000);
          timer_delay(1000);
          motorSetSpeed(MOTOR3 | MOTOR4, 2500);
          timer_delay(1000);
          motorSetSpeed(MOTOR3 | MOTOR4, 3000);
          timer_delay(1000);
          motorSetSpeed(MOTOR3 | MOTOR4, 3500);
          timer_delay(1000);
          motorSetSpeed(MOTOR3 | MOTOR4, 0);
          timer_delay(1000);*/
          break;
	}
}


int main(void)
{
	init();

	/* Create a new child process */
        proc_new(square_process, NULL, KERN_MINSTACKSIZE * 2, NULL);

        enableMotor(MOTOR3 | MOTOR4);
        LED1_ON(); LED2_ON();

	/*
	 * The main process is kept to periodically report the stack
	 * utilization of all the processes (1 probe per second).
	 */
	while (1)
	{
          //monitor_report();
          timer_delay(2000);
	}

	return 0;
}

