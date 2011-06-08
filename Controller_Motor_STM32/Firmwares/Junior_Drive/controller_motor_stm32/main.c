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
#include "asservissement.h"
#include "intelligence.h"
#include "odometry.h"
#include "AX12.h"
#include "can_monitor.h"


PROC_DEFINE_STACK(stack_ind, KERN_MINSTACKSIZE * 2);

static void init(void)
{
	IRQ_ENABLE;
        LEDS_INIT();

	/* Initialize debugging module (allow kprintf(), etc.) */
	kdbg_init();
	/* Initialize system timer */
	timer_init();

	/*
	 * Kernel initialization: processes (allow to create and dispatch
	 * processes using proc_new()).
	 */
	proc_init();
	
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
    
    motorsInit();
    encodersInit();
    controlInit();
    intelligenceInit();
    odometryInit();
    canMonitorInit();
    AX12Init();
    avoidInit();

    
}

static void NORETURN ind_process(void)
{
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
	int status=0;
	int coder=0;
	/* Create a new child process */
        proc_new(ind_process, NULL, sizeof(stack_ind), stack_ind);

	/*
	 * The main process is kept to periodically report the stack
	 * utilization of all the processes (1 probe per second).
	 */
	 
	 
	
	
	while (1)
	{
          //monitor_report();
         /* if( getEncoderDirection(ENCODER3) == FORWARD_DIRECTION ) 
          	{
          		if(status==0)
          			{
          				status=1;
          				LED4_ON();
          			}
          	}
          else 
          	{
          		if(status==1)
          			{
          				LED4_OFF();
          				status=0;
          			}
          		LED4_OFF();
          	}
          coder = getEncoderCount(ENCODER2);
          timer_delay(10);
          if ( getEncoderCount(ENCODER2)==65535) LED3_ON();
          //else LED3_OFF();
          timer_delay(10);*/
          timer_delay(1000);
	}

	return 0;
}

