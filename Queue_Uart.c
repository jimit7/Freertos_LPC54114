/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

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

/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
	 R_Data = xQueueCreate( 1, sizeof( int32_t ) );
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
    if (xTaskCreate(vMotorTask, "log_task", configMINIMAL_STACK_SIZE + 166, NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS)
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
        }
       // if (n > 0)
        //{
            /* send back the received data */
           // USART_RTOS_Send(&handle, (uint8_t *)recv_buffer, n);
        //}
  //  } //while (kStatus_Success == error);

    USART_RTOS_Deinit(&handle);
    vTaskSuspend(NULL);
}

static void vMotorTask( void *pvParameters )
{

	int8_t Receive_Data ;
	for(;;)
	{
		xQueueReceive(R_Data,&Receive_Data,0);
		printf("Recived data =%D\n",Receive_Data );
	}
}
