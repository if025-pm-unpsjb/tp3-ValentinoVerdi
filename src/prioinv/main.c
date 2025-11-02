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
#include "semphr.h"

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

/* Dimensions the buffer for text messages. */
#define mainMAX_MSG_LEN                     50

/* Constants used when writing strings to the display. */
#define mainFULL_SCALE                      ( 15 )
#define ulSSI_FREQUENCY                     ( 3500000UL )

/* Prioridades de las tareas para el semaforo */
#define PRIO_TAREA_ALTA   ( configMAX_PRIORITIES - 1 )
#define PRIO_TAREA_MEDIA  ( configMAX_PRIORITIES - 2 )
#define PRIO_TAREA_BAJA   ( configMAX_PRIORITIES - 3 )

/* Tiempos de trabajo para el semaforo */
#define WCET_TAREA_ALTA   ( 1000 )
#define WCET_TAREA_MEDIA  ( 5000 ) //Tarea mas larga para bloquear la tarea baja
#define WCET_TAREA_BAJA   ( 3000 )

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
/*
 * Prototipos de las tareas para la demo de inversion de prioridad
 */
static void vTaskHighPriority( void* pvParameters );
static void vTaskMediumPriority( void* pvParameters );
static void vTaskLowPriority( void* pvParameters );

/*-----------------------------------------------------------*/

/* Functions to access the OLED.  The one used depends on the dev kit
being used. */
void ( *vOLEDInit )( uint32_t ) = NULL;
void ( *vOLEDStringDraw )( const char *, uint32_t, uint32_t, unsigned char ) = NULL;
void ( *vOLEDImageDraw )( const unsigned char *, uint32_t, uint32_t, uint32_t, uint32_t ) = NULL;
void ( *vOLEDClear )( void ) = NULL;

/*-----------------------------------------------------------*/

SemaphoreHandle_t xMutex;

/*************************************************************************
 * Main
 *************************************************************************/
int main( void )
{
	vTraceEnable( TRC_INIT );

    prvSetupHardware();

    vOLEDInit = OSRAM128x64x4Init;
    vOLEDStringDraw = OSRAM128x64x4StringDraw;
    vOLEDImageDraw = OSRAM128x64x4ImageDraw;
    vOLEDClear = OSRAM128x64x4Clear;

    vOLEDInit( ulSSI_FREQUENCY );

    static char cMessage[ mainMAX_MSG_LEN ];
    sprintf(cMessage, "Demo PrioInv!");
    vOLEDStringDraw( cMessage, 0, 0, mainFULL_SCALE );

    prvPrintString("--- Demo de Inversion de Prioridad ---\n\r");

    xMutex = xSemaphoreCreateBinary();

    if( xMutex != NULL )
    {
        xSemaphoreGive( xMutex );

        prvPrintString("Semaforo creado exitosamente.\n\r");

        xTaskCreate( vTaskHighPriority,
                     "Task-H",
                     configMINIMAL_STACK_SIZE + 50,
                     NULL, //
                     PRIO_TAREA_ALTA,
                     NULL );

        xTaskCreate( vTaskMediumPriority,
                     "Task-M",
                     configMINIMAL_STACK_SIZE + 50,
                     NULL,
                     PRIO_TAREA_MEDIA,
                     NULL );

        xTaskCreate( vTaskLowPriority,
                     "Task-L",
                     configMINIMAL_STACK_SIZE + 50,
                     NULL,
                     PRIO_TAREA_BAJA,
                     NULL );

        vTraceEnable( TRC_START );

        prvPrintString("Iniciando scheduler...\n\r");
        vTaskStartScheduler();
    }
    else
    {
        prvPrintString("Error: No se pudo crear el semaforo.\n\r");
    }

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

static void vTaskLowPriority( void* pvParameters )
{
    char cMessage[ mainMAX_MSG_LEN ];
    for( ;; )
    {
        prvPrintString("L: Intentando tomar semaforo...\n\r");

        if( xSemaphoreTake( xMutex, portMAX_DELAY ) == pdTRUE )
        {
            sprintf( cMessage, "L: TOMO semaforo (%lu)\n\r", (unsigned long)xTaskGetTickCount() );
            prvPrintString(cMessage);

            prvPrintString("L: Trabajando...\n\r");
            vBusyWait( WCET_TAREA_BAJA );

            sprintf( cMessage, "L: LIBERO semaforo (%lu)\n\r", (unsigned long)xTaskGetTickCount() );
            prvPrintString(cMessage);

            xSemaphoreGive( xMutex );
        }

        vTaskDelay( pdMS_TO_TICKS( 100 ) );
    }
}

static void vTaskMediumPriority( void* pvParameters )
{
    char cMessage[ mainMAX_MSG_LEN ];

    vTaskDelay( pdMS_TO_TICKS( 200 ) );

    for( ;; )
    {
        sprintf( cMessage, "  M: Iniciando trabajo (sin semaforo) (%lu)\n\r", (unsigned long)xTaskGetTickCount() );
        prvPrintString( cMessage );

        vBusyWait( WCET_TAREA_MEDIA );

        sprintf( cMessage, "  M: Fin de trabajo (%lu)\n\r", (unsigned long)xTaskGetTickCount() );
        prvPrintString( cMessage );

        vTaskDelay( pdMS_TO_TICKS( 2000 ) );
    }
}

static void vTaskHighPriority( void* pvParameters )
{
    char cMessage[ mainMAX_MSG_LEN ];

    vTaskDelay( pdMS_TO_TICKS( 100 ) );

    for( ;; )
    {
        sprintf( cMessage, "    H: Intentando tomar semaforo... (%lu)\n\r", (unsigned long)xTaskGetTickCount() );
        prvPrintString( cMessage );

        if( xSemaphoreTake( xMutex, portMAX_DELAY ) == pdTRUE )
        {
            prvPrintString("    H: TOMO semaforo. Trabajando...\n\r");

            vBusyWait( WCET_TAREA_ALTA );

            prvPrintString("    H: LIBERO semaforo.\n\r");

            xSemaphoreGive( xMutex );
        }
    vTaskDelay( pdMS_TO_TICKS( 200 ) );
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

