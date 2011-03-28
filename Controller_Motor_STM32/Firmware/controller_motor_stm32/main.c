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
#include "trajectory_controller.h"

command_generator_t genR, genL;

PROC_DEFINE_STACK(stack_op, KERN_MINSTACKSIZE * 8);

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
        motorControllerInit();

        // Initialize CAN_MONITOR
        canMonitorInit();

        // Start control of drive motors
        init_trajectory_controller();

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

static void NORETURN demo_process(void)
{
  while (1) {
    // Light a LED on an unused motor to indicate we should be doing something
    // with the motors very soon
    LED1_ON();
    timer_delay(1000);

    // Move the robot !
    tc_move(2., 1., 1.);

    // Wait until the movement is finished
    while (!tc_is_finished())
      timer_delay(1000);

    // Stop motor controllers to be able to freely move the robot
    mc_delete_controller(MOTOR3);
    mc_delete_controller(MOTOR4);

    // Doing nothing more, so shutdown the LED
    LED1_OFF();

    // Cleanly stop the demo process
    proc_exit();
  }
}

int main(void)
{
	init();

	/* Create a new child process */
        proc_new(demo_process, NULL, sizeof(stack_op), stack_op);

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

