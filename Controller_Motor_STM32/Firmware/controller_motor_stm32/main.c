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
#include "motor_controller.h"
#include "can_monitor.h"
#include "command_generator.h"

command_generator_t generator;

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

        // Initialize CONTROL driver (will initialize MOTOR and ENCODER subsystems)
        speedControlInit();

        // Initialize CAN_MONITOR
        canMonitorInit();

        // Start control of drive motors
        new_ramp_generator(&generator, 0., 180.);
        float k[] = {-68.0325, -1.0205};
        float l0[] = {0.0236, 3.9715};
        mc_new_controller(MOTOR3, ENCODER3, -360.0/2000.0/15.0, 0.833, 0.015, 0.005, k, -k[0], l0, &generator);
        mc_new_controller(MOTOR4, ENCODER4, -360.0/2000.0/15.0, 0.833, 0.015, 0.005, k, -k[0], l0, &generator);

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
          motorSetSpeed(MOTOR3 | MOTOR4, 1500);
          timer_delay(1000);
          motorSetSpeed(MOTOR3 | MOTOR4, -1500);
          timer_delay(1000);
          motorSetSpeed(MOTOR3 | MOTOR4, 1500);
          timer_delay(1000);
          motorSetSpeed(MOTOR3 | MOTOR4, -1500);
          timer_delay(1000);
          motorSetSpeed(MOTOR3 | MOTOR4, 1500);
          timer_delay(1000);
          motorSetSpeed(MOTOR3 | MOTOR4, -1500);
          timer_delay(1000);
          disableMotor(MOTOR3 | MOTOR4);
          break;
	}
}

static void NORETURN goto_process(void)
{
  LED1_ON();
  start_generator(&generator);
  timer_delay(2000);
  pause_generator(&generator);
  adjust_speed(&generator, 360.);
  timer_delay(1000);
  start_generator(&generator);
  timer_delay(2000);
  mc_delete_controller(MOTOR4);
  LED1_OFF();
}


int main(void)
{
	init();

	/* Create a new child process */
        proc_new(goto_process, NULL, KERN_MINSTACKSIZE * 8, NULL);

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

