/**
  ******************************************************************************
  * File Name          : DFSDM.c
  * Description        : This file provides code for the configuration
  *                      of the DFSDM instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "dfsdm.h"


/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

extern DFSDM_Channel_HandleTypeDef hdfsdm1_channel0;
extern DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;

/* DFSDM1 init function */
void MX_DFSDM1_Init(void)
{

    /* Configure DFSDM Channel */
    hdfsdm1_channel0.Instance = DFSDM1_Channel0;
    hdfsdm1_channel0.Init.OutputClock.Activation = ENABLE;
    hdfsdm1_channel0.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
    hdfsdm1_channel0.Init.OutputClock.Divider = 25;  // Adjust based on desired clock frequency
    hdfsdm1_channel0.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
    hdfsdm1_channel0.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
    hdfsdm1_channel0.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
    hdfsdm1_channel0.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
    hdfsdm1_channel0.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
    hdfsdm1_channel0.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
    hdfsdm1_channel0.Init.Awd.Oversampling = 1;
    hdfsdm1_channel0.Init.Offset = 0;
    hdfsdm1_channel0.Init.RightBitShift = 0; // Adjust to scale PDM data


    if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel0) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    /* Configure DFSDM Filter */
    hdfsdm1_filter0.Instance = DFSDM1_Filter0;
    hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
    hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
    hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE; // DMA for efficient data transfer
    hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
    hdfsdm1_filter0.Init.FilterParam.Oversampling = 64; // Adjust oversampling for audio quality
    hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;


    if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }


}

static uint32_t DFSDM1_Init = 0;

void HAL_DFSDM_ChannelMspDeInit(DFSDM_Channel_HandleTypeDef* dfsdm_channelHandle)
{

  DFSDM1_Init-- ;
  if(DFSDM1_Init == 0)
    {
  /* USER CODE BEGIN DFSDM1_MspDeInit 0 */

  /* USER CODE END DFSDM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DFSDM1_CLK_DISABLE();
  
    /**DFSDM1 GPIO Configuration    
    PC11     ------> DFSDM1_DATIN5
    PD3     ------> DFSDM1_CKOUT
    PC3     ------> DFSDM1_DATIN1 
    */
    HAL_GPIO_DeInit(GPIOC, DFSDM_DATIN5_Pin|DFSDM_DATIN1_Pin);

    HAL_GPIO_DeInit(DFSDM_CKOUT_GPIO_Port, DFSDM_CKOUT_Pin);

  /* USER CODE BEGIN DFSDM1_MspDeInit 1 */

  /* USER CODE END DFSDM1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
