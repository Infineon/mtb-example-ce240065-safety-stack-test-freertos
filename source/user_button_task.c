/******************************************************************************
* File Name: user_button_task.c
*
* Description: This file contains the code for handling user button task.
*
* Related Document: README.md
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


/******************************************************************************
* Header files includes
******************************************************************************/
#include "led_task.h"
#include "user_button_task.h"
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "cy_retarget_io.h"
#include "SelfTest.h"


/*******************************************************************************
* Global constants
*******************************************************************************/
#define GPIO_INTERRUPT_PRIORITY (7u)
#define USER_BUTTON_DELAY (1000)

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void user_btn_init(void);
static void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);

/******************************************************************************
* Global variables
******************************************************************************/
volatile bool gpio_intr_flag = false;
cyhal_gpio_callback_data_t gpio_btn_callback_data;
uint8_t count_val = 0;

/*******************************************************************************
* Function Name: user_button_task
********************************************************************************
* Summary:
*  Task that controls the User Button. The task sends user button press count
*  value to the led_task. The task also performs self test for stack overflow
*  and underflow for led_task.
*
* Parameters:
*  void *param : Task parameter defined during task creation (unused)
*
*******************************************************************************/
void user_button_task(void* param)
{
    /*Initialized the USER button.*/
    user_btn_init();

    /* Remove warning for unused parameter */
    (void)param;

   /* Repeatedly running part of the task */
    for(;;)
    {
        printf("USER BTN Task\r\n");

        /* Check the interrupt status */
        if (true == gpio_intr_flag)
        {
            gpio_intr_flag = false;
            count_val++;

            /* Get task informations like stack start address
             * of the led_task */
            vTaskGetInfo(xLedHandle,&xLedTaskDetails,
                    pdTRUE,eInvalid);

            /* Perform self test on the stack of led_task */
            if (SUCCESS == stack_memory_test(xLedTaskDetails))
            {

                /* Block until a count val has been sent to queue */
                xQueueSend(simple_send_data_q, &count_val, portMAX_DELAY);
            }
        }
              vTaskDelay(USER_BUTTON_DELAY);

    }
}

/*******************************************************************************
* Function Name: user_btn_init
********************************************************************************
* Summary:
*   Initialize the gpio peripheral.
*
* Parameters:
*  void *handler_arg (unused)
*
* Return:
* void
*******************************************************************************/
void user_btn_init(void)
{
    cy_rslt_t result;

    /* Initialize the user button */
    result = cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT,
                    CYBSP_USER_BTN_DRIVE, CYBSP_BTN_OFF);
    /* User button init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Configure GPIO interrupt */
    gpio_btn_callback_data.callback = gpio_interrupt_handler;
    cyhal_gpio_register_callback(CYBSP_USER_BTN,
                                 &gpio_btn_callback_data);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL,
                                 GPIO_INTERRUPT_PRIORITY, true);

}

/*******************************************************************************
* Function Name: gpio_interrupt_handler
********************************************************************************
* Summary:
*   GPIO interrupt handler.
*
* Parameters:
*  void *handler_arg (unused)
*  cyhal_gpio_event_t (unused)
*
* Return:
* void
*******************************************************************************/
static void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
    gpio_intr_flag = true;
}

/*****************************************************************************
* Function Name: Memory_Test
******************************************************************************
* Summary:
* The function performs self tests for the stack of the respective task which 
* is passed as a parameter to this function.
*
* Parameters:
*  TaskStatus_t task_details - Handler to store the state of the task
*
* Return:
*  stack_test_error_t - result of memory test
*****************************************************************************/
stack_test_error_t stack_memory_test(TaskStatus_t task_details)
{
    printf("\r\nRunning SelfTest for Task%d - %s \r\n",
            (int)task_details.xTaskNumber,task_details.pcTaskName);
    printf("Stack base address of the %s task is - %p\r\n",
            task_details.pcTaskName,task_details.pxStackBase);

    /* Init Stack SelfTest */
    SelfTests_Init_Stack_Range((uint16_t*)task_details.pxStackBase,
            task_details.usStackHighWaterMark,PATTERN_BLOCK_SIZE);

    /* Run Stack Self Test.      */
    uint8_t ret = SelfTests_Stack_Check_Range((uint16_t*)task_details.pxStackBase,
            task_details.usStackHighWaterMark);
    if ((ERROR_STACK_OVERFLOW & ret))
    {
         /* Process error */
        printf("Stack overflow\r\n");
        return STACK_OVERFLOW;
    }
    else if ((ERROR_STACK_UNDERFLOW & ret))
    {
         /* Process error */
        printf("Stack underflow\r\n\n");
        return STACK_UNDERFLOW;
     }

    else
    {
        printf("Success : Stack memory test  \r\n\n");
        return SUCCESS;
    }

}

/* END OF FILE [] */
