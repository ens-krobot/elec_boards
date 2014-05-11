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
#include "motor_controller.h"
#include "can_monitor.h"
#include "command_generator.h"
#include "lift_controller.h"

PROC_DEFINE_STACK(stack_ind, KERN_MINSTACKSIZE * 2);

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

        // Initialize CAN_MONITOR
        canMonitorInit();

        // Initialize CONTROL driver (will initialize MOTOR and ENCODER subsystems)
        motorControllerInit();
        // Initialize Command generator
        tc_init();
        // Initialize Lift Controller
        //lc_init();

        // Init lifts
        //lc_homing(LC_RIGHT_LIFT);
        //lc_goto_position(LC_RIGHT_LIFT, 0.5);
        //while(tc_is_working(TC_MASK(LC_TC_RIGHT)))
        //  timer_delay(100);

        // Setup Left pump motor
        motorSetMaxPWM(MOTOR1, 1800); // Limit to 12V
        enableMotor(MOTOR1);

        // Setup Right pump motor
        motorSetMaxPWM(MOTOR3, 1800); // Limit to 12V
        enableMotor(MOTOR3);

        // Blink to say we are ready
        for (uint8_t i=0; i < 5; i++) {
          LED1_ON();
          LED2_ON();
          LED3_ON();
          LED4_ON();
          timer_delay(100);
          LED1_OFF();
          LED2_OFF();
          LED3_OFF();
          LED4_OFF();
          timer_delay(100);
          }
}

static void NORETURN ind_process(void)
{
  while(1) {
    LED1_ON();
    lc_goto_position(LC_RIGHT_LIFT, 1.);
    while(tc_is_working(TC_MASK(LC_TC_RIGHT)))
      timer_delay(100);
    timer_delay(1000);
    LED1_OFF();
    lc_goto_position(LC_RIGHT_LIFT, 0);
    while(tc_is_working(TC_MASK(LC_TC_RIGHT)))
      timer_delay(100);
    timer_delay(1000);
  }
}

int main(void)
{
	init();

	/* Create a new child process */
        //proc_new(ind_process, NULL, sizeof(stack_ind), stack_ind);

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

