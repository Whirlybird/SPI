/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Standard demo includes. */
#include "partest.h"
//#include "xgpio.h"
#include "xparameters.h"

/* Priorities at which the tasks are created. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

/* The rate at which data is sent to the queue.  The 200ms value is converted
to ticks using the portTICK_PERIOD_MS constant. */
#define mainQUEUE_SEND_FREQUENCY_MS			( 1000 / portTICK_PERIOD_MS )

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainQUEUE_LENGTH					( 1 )

/* The LED toggled by the Rx task. */
#define mainTASK_LED						( 1 )

/*-----------------------------------------------------------*/

/*
 * The tasks as described in the comments at the top of this file.
 */
//static void prvQueueReceiveTask( void *pvParameters );
static void prvQueueSendTask( void *pvParameters );

/*-----------------------------------------------------------*/

/* The queue used by both tasks. */
static QueueHandle_t xQueue = NULL;

/*-----------------------------------------------------------*/

void main_blinky( void )
{
	/* Create the queue. */
	xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint32_t ) );

	if( xQueue != NULL )
	{
		xTaskCreate( prvQueueSendTask, "TX", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL );

		/* Start the tasks and timer running. */
		//vTaskStartScheduler();
	}

	/* If all is well, the scheduler will now be running, and the following
	line will never be reached.  If the following line does execute, then
	there was either insufficient FreeRTOS heap memory available for the idle
	and/or timer tasks to be created, or vTaskStartScheduler() was called from
	User mode.  See the memory management section on the FreeRTOS web site for
	more details on the FreeRTOS heap http://www.freertos.org/a00111.html.  The
	mode from which main() is called is set in the C start up code and must be
	a privileged mode (not user mode). */
	//vParTestToggleLED ( mainTASK_LED );
	/*
	for( ;; )
	{
		SPI_SEND();
		vTaskDelay( 100 );
	}
	*/
	u8 readBuf[32];
	int i;

	for (i = 0; i < 32; i++)
	{
		readBuf[i] = 0x00;
	}

	for( ;; )
	{
		/*
		while(1)
		{
			writeTx(readBuf, 1);
		}*/
		while (!dataAvailable())
		{
			//readRx(readBuf, sizeof(unsigned char));
			;
		}
		i = 0;
		while(dataAvailable())
		{

			stopListening();
			readRx(readBuf[i++], sizeof(unsigned char));
			//while(1);
			//readBuf[0] = getRegByte(RPD);
			(void)readBuf;
		}
		startListening();
	}
}
/*-----------------------------------------------------------*/

static void prvQueueSendTask( void *pvParameters )
{
	(void)pvParameters;
	/*
	u8 readBuf[32];
	int i;

	for (i = 0; i < 32; i++)
	{
		readBuf[i] = 0;
	}

	for( ;; )
	{
		if (dataReady())
		{
			readRx(readBuf, sizeof(unsigned long));
		}
		else
		{
			readBuf[0] = 0;
			readBuf[1] = 0;
		}
	}
	*/
}
/*-----------------------------------------------------------*/
