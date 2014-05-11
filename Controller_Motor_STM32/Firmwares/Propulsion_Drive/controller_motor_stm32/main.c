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

//#define PROP_WHEEL_RADIUS 0.049245
#define PROP_WHEEL_RADIUS_LEFT 0.049207
#define PROP_WHEEL_RADIUS_RIGHT 0.049283
#define PROP_WHEEL_RADIUS ((PROP_WHEEL_RADIUS_RIGHT+PROP_WHEEL_RADIUS_LEFT)/2)
#define PROP_SHAFT_WIDTH 0.22587

#define INDEP_WHEEL_RADIUS 0.0359
#define INDEP_SHAFT_WIDTH 0.266

#define CONTROL_ODOMETRY 0

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
        dd_start(CONTROL_ODOMETRY, // Use odometry CONTROL_ODOMETRY for control
                 PROP_WHEEL_RADIUS_LEFT, PROP_WHEEL_RADIUS_RIGHT, PROP_SHAFT_WIDTH, // Structural parameters
                 8*2*M_PI, // Absolute wheel speed limitation
                 0.5, // Linear velocity limitation
                 3.14,// Rotational speed limitation
                 1.0, // Linear acceleration limitation
                 1.0, // Radial acceleration limitation
                 0.4, 0.7, 1.0, // Controller gains
                 0.005); // Sample period
        // Common parameters
        params.encoder_gain = 2.0*M_PI/2000.0/15;
        params.T = 0.005;
        // Initialize left motor
        params.G0 = 0.011686;
        params.tau = 0.118;
        params.k[0] = -3735.7;
        params.k[1] = -297.5867;
        params.l = -params.k[0];
        params.l0[0] = 0.0561;
        params.l0[1] = 0.0108;
        params.motor = MOTOR3;
        params.encoder = ENCODER3;
        mc_new_controller(&params, dd_get_left_wheel_generator(), CONTROLLER_MODE_NORMAL);
        // Initialize right motor
        params.motor = MOTOR4;
        params.encoder = ENCODER4;
        params.encoder_gain = -2.0*M_PI/2000.0/15; // Left motor is reversed
        mc_new_controller(&params, dd_get_right_wheel_generator(), CONTROLLER_MODE_NORMAL);

        // Start odometrys
        odometryInit(0, 1e-3,
                     PROP_WHEEL_RADIUS_LEFT, PROP_WHEEL_RADIUS_RIGHT,
                     PROP_SHAFT_WIDTH,
                     ENCODER3, ENCODER4,
                     2.0*M_PI/2000.0/15, -2.0*M_PI/2000.0/15);
        odometryInit(1, 1e-3,
                     INDEP_WHEEL_RADIUS, INDEP_WHEEL_RADIUS,
                     INDEP_SHAFT_WIDTH,
                     ENCODER1, ENCODER2,
                     2.0*M_PI/1024.0, -2.0*M_PI/1024.0);

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

        // Enable motor pump
        //enableMotor(MOTOR2);
}

static void NORETURN ind_process(void)
{
  while(1) {
    if (dd_get_ghost_state(NULL, NULL) == DD_GHOST_MOVING) {
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

