/******************************************************************************
* File Name: led_task.c
*
* Description: This file contains the code for handling led_task.
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


/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "led_task.h"
#include "user_button_task.h"
#include "cybsp.h"
#include "cyhal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cycfg.h"
#include "cy_retarget_io.h"
#include "SelfTest.h"

/*******************************************************************************
 * Global variable
 ******************************************************************************/
/* Queue handle used for sending data */
QueueHandle_t simple_send_data_q;

/*******************************************************************************
* Function Name: led_task
********************************************************************************
* Summary:
*  Task that controls the LED. The task also performs self test for stack overflow
*  and underflow for user_button_task.
*
* Parameters:
*  void *param : Task parameter defined during task creation (unused)
*
*******************************************************************************/
void led_task(void* param)
{
    BaseType_t rtos_api_result;
    uint8_t recv_data = 0;
    cy_rslt_t result;

    /* Initialize the user LED */
    result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT,
                    CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* User LED init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Suppress warning for unused parameter */
    (void)param;

    /* Repeatedly running part of the task */
    for(;;)
    {
        printf("LED Task\r\n");
        /* Block until a command has been received over queue */
        rtos_api_result = xQueueReceive(simple_send_data_q, &recv_data, portMAX_DELAY);
        if(rtos_api_result == pdTRUE)
        {
            if(recv_data % 2)
            {
                /*Odd number received*/
                cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
            }
            else
            {
                /*Even number received*/
                cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
            }
        }

     }

}


/* END OF FILE [] */
