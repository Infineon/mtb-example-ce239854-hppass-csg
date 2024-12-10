/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the HPPASS CSG example for ModusToolbox.
*
* Related Document: See README.md
*
*
********************************************************************************
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
* Header Files
*******************************************************************************/
#include "cybsp.h"
#include "cy_pdl.h"
#include "mtb_hal.h"
#include "cy_retarget_io.h"


/*******************************************************************************
* Macros
*******************************************************************************/
/* The definition of HPPASS AC startup timeout in microseconds.
 * HPPASS startup time contains AREF startup 40us, CSG startup about 15us and 
 * SAR ADC maximum self-calibration time 9ms (HPPASS input clock is 240MHz). To be
 * on the safe side, add to 10ms.
 */
#define HPPASS_AC_STARTUP_TIMEOUT           (10000U)

/* The low threshold of CSG DAC output: 800mV, value = (1024 * 800mV)/3300mV
 * 10-bit DAC, DAC reference voltage is VDDA and VDDA is 3300mV on EVK board
 */
#define USER_CSG_DAC_OUT_LOW_THRESHOLD      (248u)

/* The high threshold of CSG DAC output: 2000mV, value = (1024 * 2000mV)/3300mV */
#define USER_CSG_DAC_OUT_HIGH_THRESHOLD     (621u)

/* User CSG DAC HW start interrupt mask */
#define USER_CSG_DAC_HW_START_INT_MASK      (CY_HPPASS_INTR_CSG_0_DAC_HW_START << USER_CSG_SLICE_IDX)

/* User CSG DAC buffer empty interrupt mask */
#define USER_CSG_DAC_BUF_EMPTY_INT_MASK     (CY_HPPASS_INTR_CSG_0_DAC_BUF_EMPTY << USER_CSG_SLICE_IDX)

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* For the Retarget-IO (Debug UART) usage */
static cy_stc_scb_uart_context_t  DEBUG_UART_context;   /* Debug UART context */
static mtb_hal_uart_t DEBUG_UART_hal_obj;               /* Debug UART HAL object */

/* The started flag of user CSG DAC */
volatile bool user_csg_dac_is_started = false;

/* The buffer empty flag of user CSG DAC */
volatile bool user_csg_dac_buf_is_empty = false;

/* The DAT output value of User CSG */
volatile uint16_t user_csg_dac_value = 0;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
/* User CSG DAC interrupt handler */
void user_csg_dac_intr_handler(void);

/* Check if the user button is pressed */
bool user_button_is_pressed(void);

/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CPU.
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the debug UART */
    result = Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);
    /* UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    Cy_SCB_UART_Enable(DEBUG_UART_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &DEBUG_UART_hal_config, &DEBUG_UART_context, NULL);
    /* HAL UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);
    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize TCPWM using the config structure generated using device configurator*/
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_Counter_Init(USER_TIMER_HW, USER_TIMER_NUM, &USER_TIMER_config))
    {
        CY_ASSERT(0);
    }

    /* Enable the initialized TCPWM */
    Cy_TCPWM_Counter_Enable(USER_TIMER_HW, USER_TIMER_NUM);

    /* The user CSG DAC interrupt configuration structure */
    cy_stc_sysint_t user_csg_dac_intr_config =
    {
        .intrSrc = (IRQn_Type)(pass_interrupt_csg_dac_0_IRQn + USER_CSG_SLICE_IDX),
        .intrPriority = 0U,
    };
    /* Configure HPPASS interrupt */
    Cy_HPPASS_DAC_SetInterruptMask(USER_CSG_DAC_HW_START_INT_MASK | USER_CSG_DAC_BUF_EMPTY_INT_MASK);
    Cy_SysInt_Init(&user_csg_dac_intr_config, user_csg_dac_intr_handler);
    NVIC_EnableIRQ(user_csg_dac_intr_config.intrSrc);

    /* Start user CSG DAC HW mode */
    Cy_HPPASS_DAC_Start(USER_CSG_SLICE_IDX, CY_HPPASS_DAC_HW);
    /* Start the HPPASS autonomous controller (AC) from state 0 */
    if(CY_HPPASS_SUCCESS != Cy_HPPASS_AC_Start(0U, HPPASS_AC_STARTUP_TIMEOUT))
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("********************************************************************************\r\n");
    printf("HPPASS: Comparator and slope generator (CSG) example\r\n");
    printf("********************************************************************************\r\n");
    printf("Press user switch (SW2) to stop or start CSG DAC hardware (HW) mode\r\n");

    /* Enable global interrupts */
    __enable_irq();

    /* Start user timer */
    Cy_TCPWM_TriggerStart_Single(USER_TIMER_HW, USER_TIMER_NUM);

    for (;;)
    {
        /* Check if 'SW2' key was pressed */
        if(user_button_is_pressed())
        {
            /* Check the status of CSG DAC */
            if(!Cy_HPPASS_DAC_IsBusy(USER_CSG_SLICE_IDX))
            {
                user_csg_dac_is_started = false;
                /* Set DAC ouput buffer value */
                user_csg_dac_value = USER_CSG_DAC_OUT_HIGH_THRESHOLD;
                Cy_HPPASS_DAC_SetValue(USER_CSG_SLICE_IDX, user_csg_dac_value);
                /* Start user CSG DAC HW mode */
                printf("\r\nStart CSG DAC HW mode\r\n");
                Cy_HPPASS_DAC_Start(USER_CSG_SLICE_IDX, CY_HPPASS_DAC_HW);
            }
            else
            {
                /* Stop user CSG DAC HW mode */
                printf("Stop CSG DAC HW mode\r\n");
                Cy_HPPASS_DAC_Stop(USER_CSG_SLICE_IDX);
            }
        }

        /* Check if the DAC is started */
        if(user_csg_dac_is_started)
        {
            printf("CSG DAC HW mode is started\r\n");
            user_csg_dac_is_started = false;
        }
        
        /* Check if the DAC is started and buffer is empty */
        if(user_csg_dac_buf_is_empty && Cy_HPPASS_DAC_IsBusy(USER_CSG_SLICE_IDX))
        {
            user_csg_dac_buf_is_empty = false;
            /* Print out the DAC buffer setting and current comparator output status */
            if(USER_CSG_DAC_OUT_HIGH_THRESHOLD <= user_csg_dac_value)
            {
                printf("Set DAC buffer to 2.0V, current comparator status: %d\r\n", \
                        Cy_HPPASS_Comp_GetStatus(USER_CSG_SLICE_IDX));
            }
            else
            {
                printf("Set DAC buffer to 0.8V, current comparator status: %d\r\n", \
                        Cy_HPPASS_Comp_GetStatus(USER_CSG_SLICE_IDX));
            }
        }
    }
}

/*******************************************************************************
* Function Name: user_csg_dac_intr_handler
********************************************************************************
* Summary:
* This function is the user CSG DAC interrupt handler.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void user_csg_dac_intr_handler(void)
{
    uint32_t intrStatus = Cy_HPPASS_DAC_GetInterruptStatusMasked();
    /* Clear interrupt */
    Cy_HPPASS_DAC_ClearInterrupt(intrStatus);

    /* Check CSG DAC HW start interrupt */
    if(USER_CSG_DAC_HW_START_INT_MASK == (intrStatus & USER_CSG_DAC_HW_START_INT_MASK))
    {
        user_csg_dac_is_started = true;
    }

    /* Check CSG DAC buffer empty interrupt */
    if(USER_CSG_DAC_BUF_EMPTY_INT_MASK == (intrStatus & USER_CSG_DAC_BUF_EMPTY_INT_MASK))
    {
        user_csg_dac_buf_is_empty = true;
        /* Update the DAC buffer value */
        if(USER_CSG_DAC_OUT_HIGH_THRESHOLD <= user_csg_dac_value)
        {
            user_csg_dac_value = USER_CSG_DAC_OUT_LOW_THRESHOLD;
        }
        else
        {
            user_csg_dac_value = USER_CSG_DAC_OUT_HIGH_THRESHOLD;
        }
        /* Set DAC output value */
        Cy_HPPASS_DAC_SetValue(USER_CSG_SLICE_IDX, user_csg_dac_value);
    }
}

/*******************************************************************************
* Function Name: user_button_is_pressed
****************************************************************************//**
* Summary:
*  Check if the user button is pressed.
*
* Return:
*  Returns the status of user button.
*
*******************************************************************************/
bool user_button_is_pressed(void)
{
    uint32_t pressCount = 0;

    if(Cy_GPIO_Read(CYBSP_USER_BTN2_PORT, CYBSP_USER_BTN2_NUM) != CYBSP_BTN_PRESSED)
    {
        return false;
    }
    /* Check if User button is pressed */
    while (Cy_GPIO_Read(CYBSP_USER_BTN2_PORT, CYBSP_USER_BTN2_NUM) == CYBSP_BTN_PRESSED)
    {
        /* Wait for 10 ms */
        Cy_SysLib_Delay(10);
        pressCount++;
    }
    /* Add a delay to avoid glitches */
    Cy_SysLib_Delay(10);

    if(10 < pressCount)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/* [] END OF FILE */
