/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel 1 interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

    /* Once DMA transmission is done, all active ADC data will be shifted from the
     * uint16 ADC buffer to the uint8 MSB and LSB fields.
     * Those fields get filled 0->5, not matching what actual channel the data comes from.
     */
    if (LL_DMA_IsActiveFlag_TC1(DMA1)) {
        LL_GPIO_ResetOutputPin(DEBUG_GPIO_Port, DEBUG_Pin);
        uint8_t *dest = (uint8_t*)&(pRegisterMap->ADC_CH0_MSB);

        for (int i = 0; i < ADC_channelCount; i++) {
            uint16_t const adc_val = ADC_buffer[i];
            dest[i * 2] = (uint8_t)(adc_val >> 8);
            dest[(i * 2) + 1] = (uint8_t)(adc_val & 0xFF);
        }

        LL_DMA_ClearFlag_TC1(DMA1);
    }
  /* USER CODE END DMA1_Channel1_IRQn 0 */
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event global interrupt / I2C1 wake-up interrupt through EXTI line 23.
  */
void I2C1_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_IRQn 0 */
    if (LL_I2C_IsActiveFlag_ADDR(I2C1)) {
        LL_I2C_ClearFlag_ADDR(I2C1);

        if (LL_I2C_GetTransferDirection(I2C1) == LL_I2C_DIRECTION_READ) {
            if (I2C_RX_bufferIdx > 0) { // -> a register to read from was written before
                if (I2C_RX_buffer[0] == I2C_REG_CHANNEL_CONFIG) {
                    LL_I2C_TransmitData8(I2C1, pRegisterMap-> ADC_channelConfig);
                } else {
                    LL_I2C_TransmitData8(I2C1, 0xFF); // Master requested invalid register
                }
                I2C_RX_bufferIdx = 0;
            } else { // -> master want's ADC data
                I2C_state = I2C_STATE_SEND_ADC_DATA;
                pCurrentRegister = &(pRegisterMap-> ADC_CH0_MSB);
            }
            LL_I2C_EnableIT_TX(I2C1);
        }
    }

    if (LL_I2C_IsActiveFlag_RXNE(I2C1)) {
        I2C_RX_buffer[I2C_RX_bufferIdx] = LL_I2C_ReceiveData8(I2C1);
        I2C_RX_bufferIdx++;

        if (I2C_RX_bufferIdx == I2C_RX_BUFFER_SIZE) { // -> register and data were written
            if (I2C_RX_buffer[0] == I2C_REG_CHANNEL_CONFIG) {
                //LL_GPIO_SetOutputPin(DEBUG_GPIO_Port, DEBUG_Pin);
                pRegisterMap->ADC_channelConfig = I2C_RX_buffer[1];
                updateChannelConfig = true;
            }
            I2C_RX_bufferIdx = 0;
        }
    }

    if (LL_I2C_IsActiveFlag_TXE(I2C1)) {
        if (I2C_state == I2C_STATE_SEND_ADC_DATA) {
            if (pCurrentRegister >= pADC_maxChannel) { // maxChannel was sent already so quit
                I2C_state = I2C_STATE_DEFAULT;
                LL_I2C_DisableIT_TX(I2C1);
                //LL_I2C_TransmitData8(I2C1, 0x00); // dummy to clear flag
            } else {
                LL_I2C_TransmitData8(I2C1, *pCurrentRegister);
                pCurrentRegister++;
            }
        } else if (I2C_state == I2C_STATE_DEFAULT) {
            LL_I2C_DisableIT_TX(I2C1);
            //LL_I2C_TransmitData8(I2C1, 0x00); // dummy to clear flag
        }
    }

    if (LL_I2C_IsActiveFlag_STOP(I2C1)) {
        LL_I2C_ClearFlag_STOP(I2C1);
    }
  /* USER CODE END I2C1_IRQn 0 */
  /* USER CODE BEGIN I2C1_IRQn 1 */

  /* USER CODE END I2C1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
