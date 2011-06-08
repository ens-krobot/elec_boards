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

#include "reception.h"


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
	
	receptionInit();
	
	// Blink to say we are ready
        for (uint8_t i=0; i < 5; i++) {
          LED1_ON();
          LED2_ON();
          timer_delay(100);
          LED1_OFF();
          LED2_OFF();
          timer_delay(100);
          }
    

    
}

static void NORETURN ind_process(void)
{
  while(1) {
    /*LED1_ON();
    timer_delay(250);
    LED1_OFF();*/
    timer_delay(250);
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
          timer_delay(1000);
	}

	return 0;
}

