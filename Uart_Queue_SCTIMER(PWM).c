#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "fsl_sctimer.h"
#include "fsl_gpio.h"

#include "fsl_usart_freertos.h"
#include "fsl_usart.h"

#include "pin_mux.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_USART USART0
#define DEMO_USART_IRQHandler FLEXCOMM0_IRQHandler
#define DEMO_USART_IRQn FLEXCOMM0_IRQn
/* Task priorities. */
#define uart_task_PRIORITY (configMAX_PRIORITIES - 1)
#define USART_NVIC_PRIO 5
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void uart_task(void *pvParameters);
static void vMotorTask(void *pvParameters);
static QueueHandle_t R_Data = NULL;
/*******************************************************************************
 * Code
 ******************************************************************************/
const char *to_send             = "FreeRTOS USART driver example!\r\n";
const char *send_buffer_overrun = "\r\nRing buffer overrun!\r\n";
uint8_t background_buffer[32];
uint8_t recv_buffer[1];

usart_rtos_handle_t handle;
struct _usart_handle t_handle;

struct rtos_usart_config usart_config = {
    .baudrate    = 115200,
    .parity      = kUSART_ParityDisabled,
    .stopbits    = kUSART_OneStopBit,
    .buffer      = background_buffer,
    .buffer_size = sizeof(background_buffer),
};

#define DEMO_FIRST_SCTIMER_OUT kSCTIMER_Out_4
#define DEMO_SECOND_SCTIMER_OUT kSCTIMER_Out_5
#define DEMO_THIRD_SCTIMER_OUT	kSCTIMER_Out_7
#define DEMO_FOURTH_SCTIMER_OUT	kSCTIMER_Out_2
#define SCTIMER_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)
sctimer_config_t sctimerInfo;
 sctimer_pwm_signal_param_t pwmParam;
 uint32_t event1,event2,event3,event4,speed=60;
 uint32_t sctimerClock;


/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */

	 RESET_PeripheralReset( kFC0_RST_SHIFT_RSTn);
	 CLOCK_EnableClock(kCLOCK_Gpio0);
	 CLOCK_EnableClock(kCLOCK_Gpio1);
	 R_Data = xQueueCreate( 1, sizeof( int8_t ) );
     CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    BOARD_InitPins();
    // BOARD_BootClockFROHF48M();a
    BOARD_InitDebugConsole();
     if (xTaskCreate(uart_task, "Uart_task", configMINIMAL_STACK_SIZE + 10, NULL, tskIDLE_PRIORITY + 2 ,NULL) != pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }
    if (xTaskCreate(vMotorTask, "Motor_task", configMINIMAL_STACK_SIZE + 166, NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS)
       {
           PRINTF("Task creation failed!.\r\n");
           while (1)
          ;
       }
    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief Task responsible for loopback.
 */
static void uart_task(void *pvParameters)
{
    int error;
    size_t n            = 0;
    usart_config.srcclk = BOARD_DEBUG_UART_CLK_FREQ;
    usart_config.base   = DEMO_USART;

    NVIC_SetPriority(DEMO_USART_IRQn, USART_NVIC_PRIO);

    if (0 > USART_RTOS_Init(&handle, &t_handle, &usart_config))
    {
        vTaskSuspend(NULL);
    }

    /* Send introduction message. */
    if (0 > USART_RTOS_Send(&handle, (uint8_t *)to_send, strlen(to_send)))
    {
        vTaskSuspend(NULL);
    }

    /* Receive user input and send it back to terminal. */
    for(;;)
    {
      USART_RTOS_Receive(&handle, recv_buffer, sizeof(recv_buffer), &n);
        //if (error == kStatus_USART_RxRingBufferOverrun)
        //{
            /* Notify about hardware buffer overrun */
          //  if (kStatus_Success !=
            //    USART_RTOS_Send(&handle, (uint8_t *)send_buffer_overrun, strlen(send_buffer_overrun)))
            //{
              //  vTaskSuspend(NULL);
            //}

      xQueueSend(R_Data,&recv_buffer,0);
      printf("uart =%s\n\n",recv_buffer);
        }
  
    USART_RTOS_Deinit(&handle);
    vTaskSuspend(NULL);
}

static void vMotorTask( void *pvParameters )
{
	BaseType_t xstatus;
	uint8_t Receive_Data;
	 uint32_t sctimerClock;


	 sctimerClock = SCTIMER_CLK_FREQ;
	    SCTIMER_GetDefaultConfig(&sctimerInfo);
	    /* Initialize SCTimer module */
	    SCTIMER_Init(SCT0, &sctimerInfo);



	    	pwmParam.output = DEMO_FIRST_SCTIMER_OUT;
	    	pwmParam.level = kSCTIMER_HighTrue;
	    	pwmParam.dutyCyclePercent = 1;
	    	if (SCTIMER_SetupPwm(SCT0, &pwmParam, kSCTIMER_EdgeAlignedPwm , 2400U, sctimerClock, &event1) == kStatus_Fail)
	    	{
	           return -1;
	    	}

	    	pwmParam.output = DEMO_SECOND_SCTIMER_OUT;
	    	pwmParam.level = kSCTIMER_HighTrue;
	       	pwmParam.dutyCyclePercent =2 ;
	       	if (SCTIMER_SetupPwm(SCT0, &pwmParam, kSCTIMER_EdgeAlignedPwm , 2400U, sctimerClock, &event2) == kStatus_Fail)
	       	{
	       	        return -1;
	       	}
	       	pwmParam.output = DEMO_THIRD_SCTIMER_OUT;
	        pwmParam.level = kSCTIMER_HighTrue;
	        pwmParam.dutyCyclePercent = 1;
	        if (SCTIMER_SetupPwm(SCT0, &pwmParam, kSCTIMER_EdgeAlignedPwm , 2400U, sctimerClock, &event3) == kStatus_Fail)
	        {
	              return -1;
	        }

	        pwmParam.output = DEMO_FOURTH_SCTIMER_OUT;
	        pwmParam.level = kSCTIMER_HighTrue;
	        pwmParam.dutyCyclePercent =2 ;
	        if (SCTIMER_SetupPwm(SCT0, &pwmParam, kSCTIMER_EdgeAlignedPwm , 2400U, sctimerClock, &event4) == kStatus_Fail)
	        {
	          	        return -1;
	        }



	for(;;)
	{
		xstatus = xQueueReceive(R_Data,&Receive_Data,0);
if(xstatus == pdPASS)
{
		printf("Recived data =%d\n",Receive_Data );
	}
if (Receive_Data == 103)
{

	printf("Rotate on axis\n");
	                	   SCTIMER_UpdatePwmDutycycle(SCT0,DEMO_FIRST_SCTIMER_OUT,1, event1);
	                	   SCTIMER_UpdatePwmDutycycle(SCT0,DEMO_SECOND_SCTIMER_OUT,1, event2);
	                	   SCTIMER_UpdatePwmDutycycle(SCT0,DEMO_THIRD_SCTIMER_OUT, 1, event3);
	                	   SCTIMER_UpdatePwmDutycycle(SCT0,DEMO_FOURTH_SCTIMER_OUT,1, event4);

	printf("f\n");
}
if (Receive_Data == 97)
{
		   SCTIMER_UpdatePwmDutycycle(SCT0,DEMO_FIRST_SCTIMER_OUT, 1, event1);
	    	   SCTIMER_UpdatePwmDutycycle(SCT0,DEMO_SECOND_SCTIMER_OUT, 1, event2);
	    	   SCTIMER_UpdatePwmDutycycle(SCT0,DEMO_THIRD_SCTIMER_OUT, speed, event3);
	    	   SCTIMER_UpdatePwmDutycycle(SCT0,DEMO_FOURTH_SCTIMER_OUT, 1, event4);

	printf("left\n");
	printf("a\n");
}
if (Receive_Data == 98)
{
	printf("right\n");
	        	   SCTIMER_UpdatePwmDutycycle(SCT0,DEMO_FIRST_SCTIMER_OUT, speed, event1);
	        	   SCTIMER_UpdatePwmDutycycle(SCT0,DEMO_SECOND_SCTIMER_OUT,1, event2);
	        	   SCTIMER_UpdatePwmDutycycle(SCT0,DEMO_THIRD_SCTIMER_OUT, 1, event3);
	        	   SCTIMER_UpdatePwmDutycycle(SCT0,DEMO_FOURTH_SCTIMER_OUT,1, event4);

	printf("b\n");
}
if (Receive_Data == 100)
			{
			SCTIMER_UpdatePwmDutycycle(SCT0,DEMO_FIRST_SCTIMER_OUT,speed, event1);
	        	SCTIMER_UpdatePwmDutycycle(SCT0,DEMO_SECOND_SCTIMER_OUT,1, event2);
	        	SCTIMER_UpdatePwmDutycycle(SCT0,DEMO_THIRD_SCTIMER_OUT,speed, event3);
	        	SCTIMER_UpdatePwmDutycycle(SCT0,DEMO_FOURTH_SCTIMER_OUT,1, event4);

printf("go straight\n");
	printf("c\n");
}
if (Receive_Data == 102)
{
	printf("stop");
	        	 SCTIMER_UpdatePwmDutycycle(SCT0,DEMO_FIRST_SCTIMER_OUT, 1, event1);
	        	 SCTIMER_UpdatePwmDutycycle(SCT0,DEMO_SECOND_SCTIMER_OUT, 1, event2);
	        	 SCTIMER_UpdatePwmDutycycle(SCT0,DEMO_THIRD_SCTIMER_OUT, 1, event3);
	        	 SCTIMER_UpdatePwmDutycycle(SCT0,DEMO_FOURTH_SCTIMER_OUT, 1, event4);
printf("d\n");
}
}
}


