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
#include "holonomic_drive.h"

#define PROP_WHEEL_RADIUS 0.02475843
#define PROP_DRIVE_RADIUS 0.15474249

PROC_DEFINE_STACK(stack_ind, KERN_MINSTACKSIZE * 8);

void wait_for_motors(void);

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
        // Invert motor direction
        /*motorInvertDirection(MOTOR2, 1);
        motorInvertDirection(MOTOR3, 1);
        motorInvertDirection(MOTOR4, 1);*/
        //motorSetMaxPWM(MOTOR2, 1800);
        //motorSetMaxPWM(MOTOR3, 1800);
        //motorSetMaxPWM(MOTOR4, 1800);
        hd_start(1,
                 PROP_WHEEL_RADIUS, PROP_DRIVE_RADIUS,
                 251.*2.*M_PI/60., // Absolute wheel speed limitation
                 M_PI/4.,// Maximum target tracking speed
                 2., // proportional gain for target tracking
                 0.005); // Sample period

        // Adjust limits for Omnidirectional Drive
        hd_adjust_limits(0.3, // v_lin_max
                         M_PI/4., // v_rot_max
                         0.5, //a_lin_max
                         M_PI/4.); //a_rot_max

          // Common parameters
        params.encoder_gain = -2.0*M_PI/2797.;
        params.T = 0.005;
        params.G0 = 0.0125/2.;//0.01306;
        params.tau = 0.200;
        /* params.k[0] = 5000.; // Kp */
        /* params.k[1] = 0.; // Ki */
        /* params.l0[0] = 0.; // N/A */
        /* params.l0[1] = 0.; // N/A */
        /* params.l = 1800.; // integral saturation */
        params.k[0] = -5870;
        params.k[1] = -522;
        params.l0[0] = 0.0729;
        params.l0[1] = 0.1174;
        params.l = -params.k[0];
        //params.k[0] = -5616.9;
        //params.k[1] = -499.6;
        //params.l0[0] = 0.0729;
        //params.l0[1] = 0.1174;
        // Initialize front motor
        params.motor = MOTOR4;
        params.encoder = ENCODER4;
        mc_new_controller(&params, hd_get_front_wheel_generator(), CONTROLLER_MODE_NORMAL);
        // Initialize back-left motor
        params.motor = MOTOR2;
        params.encoder = ENCODER2;
        mc_new_controller(&params, hd_get_back_left_wheel_generator(), CONTROLLER_MODE_NORMAL);
        // Initialize back-right motor
        params.motor = MOTOR3;
        params.encoder = ENCODER3;
        mc_new_controller(&params, hd_get_back_right_wheel_generator(), CONTROLLER_MODE_NORMAL);

        // Start odometry
        HolonomicOdometryInit(1e-3,
                              PROP_WHEEL_RADIUS, PROP_DRIVE_RADIUS,
                              ENCODER4, ENCODER2, ENCODER3,
                              params.encoder_gain);

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

void wait_for_motors(void) {
  LED1_ON();
  while (tc_is_working(TC_MASK(HD_LINEAR_SPEED_X_TC)
                       | TC_MASK(HD_LINEAR_SPEED_Y_TC)
                       | TC_MASK(HD_ROTATIONAL_SPEED_TC))) {
    timer_delay(200);
  }
  LED1_OFF();
}

static void NORETURN ind_process(void)
{
  /* timer_delay(1000); */
  /* hd_move_X(0.5, 0.3, 0.5); */
  /* wait_for_motors(); */

  /* timer_delay(1000); */
  /* hd_move_Y(0.5, 0.3, 0.5); */
  /* wait_for_motors(); */

  /* timer_delay(1000); */
  /* hd_move_X(-0.5, 0.3, 0.5); */
  /* hd_move_Y(-0.5, 0.3, 0.5); */
  /* wait_for_motors(); */

  /* timer_delay(1000); */
  /* hd_turn(M_PI/2., 2*M_PI/10., M_PI); */
  /* timer_delay(100); */
  /* wait_for_motors(); */

  while(1) {
    LED1_ON();
    timer_delay(500);

    LED1_OFF();
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

