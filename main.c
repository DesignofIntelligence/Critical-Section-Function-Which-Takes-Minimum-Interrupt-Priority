/*
    FreeRTOS V6.1.0 - Copyright (C) 2010 Real Time Engineers Ltd.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    ***NOTE*** The exception to the GPL is included to allow you to distribute
    a combined work that includes FreeRTOS without being obliged to provide the
    source code for proprietary components outside of the FreeRTOS kernel.
    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Demo includes. */
#include "basic_io.h"

/* CMSIS includes. */
#include "ARMCM3.h"

/* The interrupt number to use for the software interrupt generation.  This
could be any unused number.  In this case the first chip level (non system)
interrupt is used, which happens to be the watchdog on the LPC1768. */
#define mainSW_INTERRUPT_ID		( ( IRQn_Type ) 0 )

/* Macro to force an interrupt. */
#define mainTRIGGER_INTERRUPT()	NVIC_SetPendingIRQ( mainSW_INTERRUPT_ID )

/* Macro to clear the same interrupt. */
#define mainCLEAR_INTERRUPT()	NVIC_ClearPendingIRQ( mainSW_INTERRUPT_ID )

/* The priority of the software interrupt.  The interrupt service routine uses
an (interrupt safe) FreeRTOS API function, so the priority of the interrupt must
be equal to or lower than the priority set by
configMAX_SYSCALL_INTERRUPT_PRIORITY - remembering that on the Cortex M3 high
numeric values represent low priority values, which can be confusing as it is
counter intuitive. */
#define mainSOFTWARE_INTERRUPT_PRIORITY 		( 5 )

/* The tasks to be created. */
static void vHandlerTask( void *pvParameters );
static void vPeriodicTask( void *pvParameters );

/* Enable the software interrupt and set its priority. */
static void prvSetupSoftwareInterrupt( void );

/* The service routine for the interrupt.  This is the interrupt that the
task will be synchronized with. */
void vSoftwareInterruptHandler( void );

/*-----------------------------------------------------------*/

/* Declare a variable of type xSemaphoreHandle.  This is used to reference the
semaphore that is used to synchronize a task with an interrupt. */
xSemaphoreHandle xBinarySemaphore;

/*-----------------------------------------------------------*/

int main( void )
{
    vSemaphoreCreateBinary( xBinarySemaphore );
    if( xBinarySemaphore != NULL )
    {
      	prvSetupSoftwareInterrupt();
        xTaskCreate( vHandlerTask, "Handler", 240, NULL, 1, NULL );
        xTaskCreate( vPeriodicTask, "Periodic", 240, NULL, 3, NULL );
        vTaskStartScheduler();
    }
}
static void vHandlerTask( void *pvParameters )
{
    xSemaphoreTake( xBinarySemaphore, 0 );
    for( ;; )
    {
        xSemaphoreTake( xBinarySemaphore, portMAX_DELAY );
        vPrintString( "Handler task - Processing event.\n" );
    }
}
//---------------------------------------------------------------------------------
static unsigned portBASE_TYPE uxCriticalNesting = 0xaaaaaaaa;
	__asm void vPortSetInterrupt( int PRIO )
	{
		PRESERVE8

		push { r0 }
		mov r0, r4
		msr basepri, r0
		pop { r0 }
		bx r14
	}
		__asm void vPortClearInterrupt( void )
	{
		PRESERVE8

		push { r0 }
		mov r0, #0
		msr basepri, r0
		pop { r0 }
		bx r14
	}
void taskENTER_CRITICAL_PRIO( int PRIO )
{
	PRIO = PRIO<<5;
	vPortSetInterrupt(PRIO);
	uxCriticalNesting++;
}
/*-----------------------------------------------------------*/

void taskEXIT_CRITICAL_PRIO( void )
{
	uxCriticalNesting--;

	if( uxCriticalNesting == 0 )
	{
		vPortClearInterrupt();
	}
}
static void vPeriodicTask( void *pvParameters )
{
    for( ;; )
    {
        vTaskDelay( 500 / portTICK_RATE_MS );

			  vPrintString( "Periodic task - About to generate an interrupt.\n" );
				taskENTER_CRITICAL_PRIO(3);
				{
        mainTRIGGER_INTERRUPT();
				vPrintString("HI");
				}
				taskEXIT_CRITICAL_PRIO();
        vPrintString( "Periodic task - Interrupt generated.\n\n" );
    }
}
void vSoftwareInterruptHandler( void )
{
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR( xBinarySemaphore, &xHigherPriorityTaskWoken );
    mainCLEAR_INTERRUPT();
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
/*-----------------------------------------------------------*/
static void prvSetupSoftwareInterrupt( void )
{
	/* The interrupt service routine uses an (interrupt safe) FreeRTOS API
	function so the interrupt priority must be at or below the priority defined
	by configSYSCALL_INTERRUPT_PRIORITY. */
	NVIC_SetPriority( mainSW_INTERRUPT_ID, mainSOFTWARE_INTERRUPT_PRIORITY );

	/* Enable the interrupt. */
	NVIC_EnableIRQ( mainSW_INTERRUPT_ID );
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* This function will only be called if an API call to create a task, queue
	or semaphore fails because there is too little heap RAM remaining - and
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	/* This function will only be called if a task overflows its stack.  Note
	that stack overflow checking does slow down the context switch
	implementation and will only be performed if configCHECK_FOR_STACK_OVERFLOW
	is set to either 1 or 2 in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* This example does not use the idle hook to perform any processing.  The
	idle hook will only be called if configUSE_IDLE_HOOK is set to 1 in 
	FreeRTOSConfig.h. */
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* This example does not use the tick hook to perform any processing.   The
	tick hook will only be called if configUSE_TICK_HOOK is set to 1 in
	FreeRTOSConfig.h. */
}







