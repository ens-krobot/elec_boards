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
        tc_new_controller(0);
        tc_new_controller(1);
        motorSetMaxPWM(MOTOR4, 1600);
        // Setup Beacon's motor
        motorSetMaxPWM(MOTOR2, 1080);
        enableMotor(MOTOR2);
        // Common parameters
        params.encoder_gain = 2.0*M_PI/588.0;
        params.G0 = 0.0035;
        params.tau = 0.025;
        params.k[0] = -10216;
        params.k[1] = -255.39;
        params.l = -params.k[0];
        params.l0[0] = 0.0091;
        params.l0[1] = 1.6361;
        params.T = 0.005;
        // Initialize forward lift
        params.motor = MOTOR4;
        params.encoder = ENCODER4;
        //mc_new_controller(&params, tc_get_position_generator(0), CONTROLLER_MODE_NORMAL);
        // Initialize backward lift
        params.motor = MOTOR3;
        params.encoder = ENCODER3;
        //mc_new_controller(&params, tc_get_position_generator(1), CONTROLLER_MODE_NORMAL);

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
    LED4_ON();
    tc_goto(0, 50/23.475, M_PI, 5);
    while(tc_is_working(TC_MASK(0)))
      timer_delay(100);
    timer_delay(500);
    LED4_OFF();
    tc_goto(0, -50/23.475, M_PI, 5);
    while(tc_is_working(TC_MASK(0)))
      timer_delay(100);
    timer_delay(500);

    /*if (dd_get_ghost_state(NULL, NULL) == DD_GHOST_MOVING) {
      LED1_ON();
    } else {
      LED1_OFF();
      }*/
  }
}

int main(void)
{
	init();

	/* Create a new child process */
        //proc_new(ind_process, NULL, sizeof(stack_ind), stack_ind);

        // Tests
        //motorSetSpeed(MOTOR4, 400);
        motorSetSpeed(MOTOR2, 1000);

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

