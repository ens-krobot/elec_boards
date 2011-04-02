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
#include "differential_drive.h"

#define WHEEL_RADIUS 0.049245
#define SHAFT_WIDTH 0.259

PROC_DEFINE_STACK(stack_ind, KERN_MINSTACKSIZE * 2);

static void init(void)
{
        motor_controller_params_t params;

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
        tc_init();
        dd_start(WHEEL_RADIUS, SHAFT_WIDTH);
        // Common parameters
        params.encoder_gain = -2.0*M_PI/2000.0/15.0;
        params.G0 = 0.0145;
        params.tau = 0.015;
        params.k[0] = -3898.0;
        params.k[1] = -58.4696;
        params.l = -params.k[0];
        params.l0[0] = 0.0236;
        params.l0[1] = 3.9715;
        params.T = 0.005;
        // Initialize left motor
        params.motor = MOTOR3;
        params.encoder = ENCODER3;
        mc_new_controller(&params, dd_get_left_wheel_generator());
        // Initialize right motor
        params.motor = MOTOR4;
        params.encoder = ENCODER4;
        mc_new_controller(&params, dd_get_right_wheel_generator());

        // Start odometry
        odometryInit(1e-3, WHEEL_RADIUS, SHAFT_WIDTH, -2.0*M_PI/2000.0/15.0);

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
    if (tc_is_working(MOTOR3 | MOTOR4)) {
      LED1_ON();
    } else {
      LED1_OFF();
    }
    timer_delay(500);
  }
}

int main(void)
{
	init();

	/* Create a new child process */
        proc_new(ind_process, NULL, sizeof(stack_ind), stack_ind);

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

