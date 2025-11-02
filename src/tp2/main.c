/*************************************************************************
 * Programa de prueba basado para LM3S6965.
 *
 * Basado en el demo provisto por FreeRTOS, para ejecutar sobre QEMU.
 *
 * Please ensure to read http://www.freertos.org/portlm3sx965.html
 * which provides information on configuring and running this demo for the
 * various Luminary Micro EKs.
 *************************************************************************/

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"

/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* Hardware library includes. */
#include "hw_memmap.h"
#include "hw_types.h"
#include "hw_sysctl.h"
#include "hw_uart.h"
#include "sysctl.h"
#include "gpio.h"
#include "grlib.h"
#include "osram128x64x4.h"
#include "uart.h"
#include "bitmap.h"

/*-----------------------------------------------------------*/
#define SYSTEM_TO_RUN 10
#define MAX_TASKS_PER_SYSTEM 5


/* Dimensions the buffer for text messages. */
#define mainMAX_MSG_LEN                     25

/* Constants used when writing strings to the display. */
#define mainCHARACTER_HEIGHT                ( 9 )
#define mainMAX_ROWS_128                    ( mainCHARACTER_HEIGHT * 14 )
#define mainMAX_ROWS_96                     ( mainCHARACTER_HEIGHT * 10 )
#define mainMAX_ROWS_64                     ( mainCHARACTER_HEIGHT * 7 )
#define mainFULL_SCALE                      ( 15 )
#define ulSSI_FREQUENCY                     ( 3500000UL )

/*-----------------------------------------------------------*/

/*
 * Configure the hardware for the demo.
 */
static void prvSetupHardware( void );

/*
 * Basic polling UART write function.
 */
static void prvPrintString( const char * pcString );

/*
 * Busy wait the specified number of ticks.
 */
static void vBusyWait( TickType_t ticks );

/*
 * Periodic task.
 */
static void prvTask( void* pvParameters );

/*-----------------------------------------------------------*/

/* Functions to access the OLED.  The one used depends on the dev kit
being used. */
void ( *vOLEDInit )( uint32_t ) = NULL;
void ( *vOLEDStringDraw )( const char *, uint32_t, uint32_t, unsigned char ) = NULL;
void ( *vOLEDImageDraw )( const unsigned char *, uint32_t, uint32_t, uint32_t, uint32_t ) = NULL;
void ( *vOLEDClear )( void ) = NULL;

/*-----------------------------------------------------------*/

typedef struct {
    TickType_t wcet;
    TickType_t period;
    TickType_t deadline;
} xTaskParams;

xTaskParams systems[10][MAX_TASKS_PER_SYSTEM] = {
    { {1000,4000,4000}, {1000,5000,5000}, {2000,8000,8000}}, // S1
    { {1000,4000,4000}, {1000,8000,8000}, {2000,9000,9000}}, // S2
    { {1000,4000,4000}, {1000,6000,6000}, {1000,8000,8000}, {3000,13000,13000}}, // S3
    { {1000,4000,4000}, {2000,7000,7000}, {1000,12000,12000}, {2000,14000,14000}}, // S4
    { {1000,5000,5000}, {1000,9000,9000}, {1000,10000,10000}, {2000,15000,15000}, {2000,16000,16000} }, // S5
    { {1000,6000,6000}, {1000,8000,8000}, {2000,11000,11000}, {1000,15000,15000}, {2000,17000,17000} }, // S6
    { {1000,4000,4000}, {1000,7000,7000}, {2000,10000,10000}, {2000,14000,14000}}, // S7
    { {1000,4000,4000}, {1000,8000,8000}, {2000,10000,10000}, {2000,14000,14000}}, // S8
    { {1000,5000,5000}, {1000,8000,8000}, {2000,12000,12000}, {1000,15000,15000}, {1000,16000,16000} }, // S9
    { {1000,5000,5000}, {1000,8000,8000}, {1000,12000,12000}, {1000,13000,13000}, {2000,16000,16000} }  // S10
};

const int system_task_counts[10] = {
    3,
    3,
    4,
    4,
    5,
    5,
    4,
    4,
    5,
    5
};

/*************************************************************************
 * Main
 *************************************************************************/
int main( void )
{
	/* Initialise the trace recorder. */
	vTraceEnable( TRC_INIT );

    prvSetupHardware();

    /* Map the OLED access functions to the driver functions that are appropriate
    for the evaluation kit being used. */
    //configASSERT( ( HWREG( SYSCTL_DID1 ) & SYSCTL_DID1_PRTNO_MASK ) == SYSCTL_DID1_PRTNO_6965 );
    vOLEDInit = OSRAM128x64x4Init;
    vOLEDStringDraw = OSRAM128x64x4StringDraw;
    vOLEDImageDraw = OSRAM128x64x4ImageDraw;
    vOLEDClear = OSRAM128x64x4Clear;

    /* Initialise the OLED and display a startup message. */
    vOLEDInit( ulSSI_FREQUENCY );

    /* Print Hello World! to the OLED display. */
    static char cMessage[ mainMAX_MSG_LEN ];
    sprintf(cMessage, "Hello World!");
    vOLEDStringDraw( cMessage, 0, 0, mainFULL_SCALE );

    /* Print "Start!" to the UART. */
    prvPrintString("Start!\n\r");

    /* Creates the periodic tasks. */
    int tasks_to_create = system_task_counts[SYSTEM_TO_RUN - 1];
    for (int i = 0; i < tasks_to_create; i++) {
            xTaskParams *params = &systems[SYSTEM_TO_RUN - 1][i];

        char name[4];
        sprintf(name, "T%d", i + 1);
        xTaskCreate(
            prvTask,
            name,
            configMINIMAL_STACK_SIZE + 50,
            (void*) params,
            configMAX_PRIORITIES - (i + 1),
            NULL
        );
    }

    vTraceEnable( TRC_START );

    /* Launch the scheduler. */
    vTaskStartScheduler();

    /* Will only get here if there was insufficient memory to create the idle
    task. */
    for( ;; );
}
/*-----------------------------------------------------------*/

void prvSetupHardware( void )
{
    /* If running on Rev A2 silicon, turn the LDO voltage up to 2.75V.  This is
    a workaround to allow the PLL to operate reliably. */
    if( DEVICE_IS_REVA2 )
    {
        SysCtlLDOSet( SYSCTL_LDO_2_75V );
    }

    /* Set the clocking to run from the PLL at 50 MHz */
    SysCtlClockSet( SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ );

    /* Initialise the UART - QEMU usage does not seem to require this
    initialisation. */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_UART0 );
    UARTEnable( UART0_BASE );
}
/*-----------------------------------------------------------*/

static void prvPrintString( const char * pcString )
{
    while( *pcString != 0x00 )
    {
        UARTCharPut( UART0_BASE, *pcString );
        pcString++;
    }
}
/*-----------------------------------------------------------*/

static void vBusyWait( TickType_t ticks )
{
    TickType_t elapsedTicks = 0;
    TickType_t currentTick = 0;
    while ( elapsedTicks < ticks ) {
        currentTick = xTaskGetTickCount();
        while ( currentTick == xTaskGetTickCount() ) {
            asm("nop");
        }
        elapsedTicks++;
    }
}
/*-----------------------------------------------------------*/

void prvTask(void *pvParameters)
{
    char cMessage[ mainMAX_MSG_LEN ];
    unsigned int uxReleaseCount = 0;

    TickType_t pxPreviousWakeTime = 0;

    xTaskParams *task = (xTaskParams*) pvParameters;

    for (;;)
    {
        sprintf(cMessage, "S %s - %u - %lu\n\r",
                pcTaskGetTaskName(NULL),
                uxReleaseCount,
                xTaskGetTickCount());
        prvPrintString(cMessage);

        vBusyWait(task->wcet);

        sprintf(cMessage, "E %s - %u - %lu\n\r",
                pcTaskGetTaskName(NULL),
                uxReleaseCount,
                xTaskGetTickCount());
        prvPrintString(cMessage);

        vTaskDelayUntil(&pxPreviousWakeTime, task->period);

        uxReleaseCount++;
    }
}
/*-----------------------------------------------------------*/

void vAssertCalled( const char *pcFile, uint32_t ulLine )
{
    volatile uint32_t ulSetTo1InDebuggerToExit = 0;
    {
        while( ulSetTo1InDebuggerToExit == 0 )
        {
            /* Nothing to do here.  Set the loop variable to a non zero value in
            the debugger to step out of this function to the point that caused
            the assertion. */
            ( void ) pcFile;
            ( void ) ulLine;
        }
    }
}

char* _sbrk_r (struct _reent *r, int incr)
{
    /* Just to keep the linker quiet. */
    ( void ) r;
    ( void ) incr;

    /* Check this function is never called by forcing an assert() if it is. */
    //configASSERT( incr == -1 );

    return NULL;
}

int __error__(char *pcFilename, unsigned long ulLine) {
    return 0;
}

