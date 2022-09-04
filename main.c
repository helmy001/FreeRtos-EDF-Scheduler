/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>


/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"
#include "queue.h"

/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )
	



typedef struct
{
	 uint16_t msg_Len;
	 signed char *msg;
}Queue_message;


QueueHandle_t xQueue;  // xQueue used to store the messages sended by event tasks to Uart Task



TaskHandle_t Button1_Handle=NULL;
TaskHandle_t Button2_Handle=NULL;
TaskHandle_t Transmitter_Handle=NULL;
TaskHandle_t Uart_Handle=NULL;
TaskHandle_t Load1_Handle=NULL;
TaskHandle_t Load2_Handle=NULL;


/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/

void Button1_Task_Code(void *pvParameters)
{
	TickType_t xLastWakeTime=xTaskGetTickCount();
  const TickType_t xFrequency = 50;
	
	Queue_message Send_string;
	BaseType_t xStatus;
	static uint8_t High_Flag=0;
	static uint8_t Low_Flag=0;

	for(;;)
	{  
		if(GPIO_read(PORT_1,PIN0)==PIN_IS_HIGH &&High_Flag==0)     // If button 1 Pressed Print rising Edge only one time
		{
			signed char Rising_Edge_msg[]="Button 1 Rising Edge\n";
			Send_string.msg=Rising_Edge_msg;
			Send_string.msg_Len=22;
			xStatus = xQueueSendToBack( xQueue,&Send_string, 0 );   //Push the message in the end of the Queue
		  High_Flag=1;
			Low_Flag=0;
		}
		else if(GPIO_read(PORT_1,PIN0)==PIN_IS_LOW &&Low_Flag==0)  // Else if button 2 is released Print falling Edge only one time
		{
			signed char Falling_Edge_msg[] ="Button 1 Falling Edge\n";
			Send_string.msg=Falling_Edge_msg;
			Send_string.msg_Len=23;
			xStatus = xQueueSendToBack( xQueue,&Send_string, 0 );    //Push the message in the end of the Queue
		  Low_Flag=1;
			High_Flag=0;
		}
  	
		GPIO_write(PORT_0,PIN0,PIN_IS_LOW);													//Toggling PIN0 for logic anaylizer
		vTaskDelayUntil( &xLastWakeTime, xFrequency ); 							//Button1 Period 50
		GPIO_write(PORT_0,PIN0,PIN_IS_HIGH);
	}
}


void Button2_Task_Code(void *pvParameters)
{
	TickType_t xLastWakeTime=xTaskGetTickCount();
  const TickType_t xFrequency = 50;
	
	Queue_message Send_string;
	BaseType_t xStatus;
	static uint8_t High_Flag=0;
	static uint8_t Low_Flag=0;
	
	for(;;)
	{ 
		if(GPIO_read(PORT_1,PIN1)==PIN_IS_HIGH &&High_Flag==0)       // If Button 2 Pressed Print rising Edge only one time
		{
			signed char Rising_Edge_msg[]="Button 2 Rising Edge\n";
			Send_string.msg=Rising_Edge_msg;
			Send_string.msg_Len=22;
			xStatus = xQueueSendToBack( xQueue,&Send_string, 0 );			//Push the message in the end of the Queue
		  High_Flag=1;
			Low_Flag=0;
		}
		else if(GPIO_read(PORT_1,PIN1)==PIN_IS_LOW &&Low_Flag==0)  // Else if button 2 is released Print falling Edge only one time
		{
			signed char Falling_Edge_msg[] ="Button 2 Falling Edge\n";
			Send_string.msg=Falling_Edge_msg;
			Send_string.msg_Len=23;
			xStatus = xQueueSendToBack( xQueue,&Send_string, 0 );	  //Push the message in the end of the Queue
		  Low_Flag=1;
			High_Flag=0;
		}
  	
		GPIO_write(PORT_0,PIN1,PIN_IS_LOW); 											//Toggling PIN1 for logic anaylizer
		vTaskDelayUntil( &xLastWakeTime, xFrequency );						//Button2 Period 50
		GPIO_write(PORT_0,PIN1,PIN_IS_HIGH);
	}
} 
void Transmitter_Task_Code(void *pvParameters)
{
	TickType_t xLastWakeTime=xTaskGetTickCount();
  const TickType_t xFrequency = 100;
	
	Queue_message Send_string;
	BaseType_t xStatus;
	for(;;)
	{ 
		
		signed char Rising_Edge_msg[]="Periodic Transmitter Message\n";
		Send_string.msg=Rising_Edge_msg;
		Send_string.msg_Len=30;
		xStatus = xQueueSendToBack( xQueue,&Send_string, 0 );			//Push the message in the end of the Queue
		
		GPIO_write(PORT_0,PIN2,PIN_IS_LOW);
		vTaskDelayUntil( &xLastWakeTime, xFrequency );     				//Period 100
		GPIO_write(PORT_0,PIN2,PIN_IS_HIGH);
	}
}

void Uart_Task_Code(void *pvParameters)
{
	TickType_t xLastWakeTime=xTaskGetTickCount();
  const TickType_t xFrequency = 20;
	
	Queue_message Receive_string;
	BaseType_t xStatus;
	for(;;)
	{ 
		
		 xStatus = xQueueReceive( xQueue, &Receive_string, 0 );
		 if( xStatus == pdPASS )
		 {
			 // Data was successfully received from the queue, print out the received message
				vSerialPutString(Receive_string.msg,Receive_string.msg_Len);
		 }
		 else
		 {
				// Data was not received from the queue 
				//vSerialPutString( "Queue is Empty\r\n",18);
			  
		 }

		GPIO_write(PORT_0,PIN3,PIN_IS_LOW);
		vTaskDelayUntil( &xLastWakeTime, xFrequency );         //Period 20
		GPIO_write(PORT_0,PIN3,PIN_IS_HIGH);
	}
}

void Load1_Task_Code(void *pvParameters)
{
	TickType_t xLastWakeTime=xTaskGetTickCount();
  const TickType_t xFrequency = 10;
		for(;;)
		{
			for(int i=0;i<37220;i++);
			GPIO_write(PORT_0,PIN6,PIN_IS_LOW);
		  vTaskDelayUntil( &xLastWakeTime, xFrequency );
			GPIO_write(PORT_0,PIN6,PIN_IS_HIGH);
		}
}
void Load2_Task_Code(void *pvParameters){
	TickType_t xLastWakeTime=xTaskGetTickCount();
  const TickType_t xFrequency = 100;
		for(;;)
		{
			for(int i=0;i<93050;i++);
			GPIO_write(PORT_0,PIN7,PIN_IS_LOW);
		  vTaskDelayUntil( &xLastWakeTime, xFrequency );
			GPIO_write(PORT_0,PIN7,PIN_IS_HIGH);
		}
}

void vApplicationTickHook(void)
 {
		GPIO_write(PORT_0,PIN4,PIN_IS_HIGH);
		GPIO_write(PORT_0,PIN4,PIN_IS_LOW);
 }

 
 void vApplicationIdleHook(void)
 {
	 GPIO_write(PORT_0,PIN5,PIN_IS_HIGH);
	 GPIO_write(PORT_0,PIN5,PIN_IS_LOW);
 }
 
/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	
  uint8_t uxQueueLength=3;
	xQueue=xQueueCreate(uxQueueLength,sizeof(Queue_message));  // Queue will carry which task is sending the message and the address of the message
	
	if(xQueue!=NULL)    
	{
					/*Tasks here*/
			 
			 /******************** Button 1 Monitior Task *****************/
			 xTaskPeriodicCreate(  Button1_Task_Code,
														"Button_1_Monitor",
														100,
														( void * ) 1,
														1,
														&Button1_Handle,
														50
													);
														
														
			/******************** Button 2 Monitior Task *****************/												
			 xTaskPeriodicCreate(  Button2_Task_Code,
														 "Button_2_Monitor",
														 100,
														 ( void * ) 1,
														 1,
														 &Button2_Handle,
														 50
														);
			
														 
			/******************** Periodic_Transmitter *****************/														
			 xTaskPeriodicCreate(  Transmitter_Task_Code,
														 "Periodic_Transmitter",
														 100,
														 ( void * ) 1,
														 1,
														 &Transmitter_Handle,
														 100
													);
			
														 
			/******************** Uart_Receiver *****************/												 
			 xTaskPeriodicCreate(  Uart_Task_Code,
														 "Uart_Receiver",
														 100,
														 ( void * ) 1,
														 1,
														 &Uart_Handle,
														 20
													);
														 
			/******************** Load_1_Simulation *****************/												 
			 xTaskPeriodicCreate(  Load1_Task_Code,
														 "Load_1_Simulation",
														 100,
														 ( void * ) 1,
														 1,
														 &Load1_Handle,
														 10
													);
														 
			/******************** Load_2_Simulation *****************/												 
			 xTaskPeriodicCreate(  Load2_Task_Code,
														 "Load_2_Simulation",
														 100,
														 ( void * ) 1,
														 1,
														 &Load2_Handle,
														 100
													);						
		 
														 
														 
			/* Now all the tasks have been started - start the scheduler.

			NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
			The processor MUST be in supervisor mode when vTaskStartScheduler is 
			called.  The demo applications included in the FreeRTOS.org download switch
			to supervisor mode prior to main being called.  If you are not using one of
			these demo application projects then ensure Supervisor mode is used here. */
			vTaskStartScheduler();

	}
	else
	{
	   /* The queue could not be created. */
	}
	
	/* Should never reach here!  If you do then there was not enough heap
			available for the idle task to be created. */
	 for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


