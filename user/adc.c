/**
  ******************************************************************************
  * @file    	../user/adc.c
  * @author  	Bao Son Le - ET02 - K55
  * @version 	V1.0.0
  * @date    	2015/11/30
  * @brief   	ADC basic function define.
  ******************************************************************************
  * @copy
  *
  */
  
/*
*************************************************************************************************************************************
*															INCLUDED FILES															*
*************************************************************************************************************************************
*/
#include <stdio.h>
#include "adc.h"

/** @addtogroup STM32F10x_GEM3M_Examples
  * @{
  */

/** @addtogroup ADC_Potentiometer
  * @{
  */  

  
/*
*************************************************************************************************************************************
*															PRIVATE DEFINE															*
*************************************************************************************************************************************
*/

/*
*************************************************************************************************************************************
*														 	DATA TYPE DEFINE															*
*************************************************************************************************************************************
*/
#ifndef f32
#define	f32		float
#endif

/*
*************************************************************************************************************************************
*													   		PRIVATE VARIABLES														*
*************************************************************************************************************************************
*/ 
GPIO_InitTypeDef GPIO_InitStructure; 
ADC_InitTypeDef  ADC_InitStructure;  
DMA_InitTypeDef  DMA_InitStructure;


u16 adc_raw_val = 0;
u16 potentionmeter_val = 0;

volatile uint16_t ADC1_values[ADC1_ARRAYSIZE];
volatile uint16_t ADC3_values[ADC3_ARRAYSIZE];
volatile uint8_t statusADC1 = 0;  
volatile uint8_t statusADC3 = 0;
/*
*************************************************************************************************************************************
*							  								LOCAL FUNCTIONS															*
*************************************************************************************************************************************
*/

/*
*************************************************************************************************************************************
*															GLOBAL FUNCTIONS														*
*************************************************************************************************************************************
*/

/**
 * @brief  ADC initialize
 *
 * @param  none
 * @return none
 *
 * ADC init command
 */
void ADCInit(void)
{
    uint8_t ADC1numChannels = 15; /* number of channel will be used */
    uint8_t ADC3numChannels = 3; /* number of channel will be used */
  
    //--Enable ADC1 ADC3 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC3  | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB 
                                               | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOF , ENABLE);
    
    GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2| GPIO_Pin_3| GPIO_Pin_4| GPIO_Pin_5| GPIO_Pin_6| GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
  
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
  
    GPIO_StructInit(&GPIO_InitStructure); 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2| GPIO_Pin_3| GPIO_Pin_4| GPIO_Pin_5 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
  
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_7| GPIO_Pin_8| GPIO_Pin_9| GPIO_Pin_10 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
  
    ADC_DeInit(ADC1);
    ADC_StructInit(&ADC_InitStructure);
    
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    //We will convert multiple channels
    ADC_InitStructure.ADC_ScanConvMode =  ADC1numChannels > 1 ? ENABLE : DISABLE;
    //select continuous conversion mode
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//!
    //select no external triggering
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    //right 12-bit data alignment in ADC data register
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    //8 channels conversion
    ADC_InitStructure.ADC_NbrOfChannel =  ADC1numChannels;
    //load structure values to control and status registers
    ADC_Init(ADC1, &ADC_InitStructure);
    //wake up temperature sensor
    //ADC_TempSensorVrefintCmd(ENABLE);
    //configure each channel
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0,   6, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1,  13, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2,   5, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3,  12, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4,   4, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5,  11, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6,   3, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7,  10, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8,   1, ADC_SampleTime_239Cycles5);
  //ADC_RegularChannelConfig(ADC1, ADC_Channel_9,   9, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10,  8, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 15, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12,  7, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 14, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_14,  2, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_15,  9, ADC_SampleTime_239Cycles5);
    
    //Enable ADC1
    ADC_Cmd(ADC1, ENABLE);
    //enable DMA for ADC
    ADC_DMACmd(ADC1, ENABLE);
    //Enable ADC1 reset calibration register
    ADC_ResetCalibration(ADC1);
    //Check the end of ADC1 reset calibration register
    while(ADC_GetResetCalibrationStatus(ADC1));
    //Start ADC1 calibration
    ADC_StartCalibration(ADC1);
    //Check the end of ADC1 calibration
    while(ADC_GetCalibrationStatus(ADC1));
    
    ADC_DeInit(ADC3);
    ADC_StructInit(&ADC_InitStructure);
    
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    //We will convert multiple channels
    ADC_InitStructure.ADC_ScanConvMode = ADC3numChannels > 1 ? ENABLE : DISABLE;
    //select continuous conversion mode
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//!
    //select no external triggering
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    //right 12-bit data alignment in ADC data register
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    //numbers channels conversion
    ADC_InitStructure.ADC_NbrOfChannel = ADC3numChannels;
    //load structure values to control and status registers
    ADC_Init(ADC3, &ADC_InitStructure);
    //wake up temperature sensor
    //ADC_TempSensorVrefintCmd(ENABLE);
    //configure each channel
  //ADC_RegularChannelConfig(ADC3, ADC_Channel_4,   1, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC3, ADC_Channel_5,   2, ADC_SampleTime_239Cycles5);
  //ADC_RegularChannelConfig(ADC3, ADC_Channel_6,   3, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC3, ADC_Channel_7,   1, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC3, ADC_Channel_8,   3, ADC_SampleTime_239Cycles5);

    //Enable ADC3
    ADC_Cmd(ADC3, ENABLE);
    //enable DMA for ADC
    ADC_DMACmd(ADC3, ENABLE);
    //Enable ADC3 reset calibration register
    ADC_ResetCalibration(ADC3);
    //Check the end of ADC3 reset calibration register
    while(ADC_GetResetCalibrationStatus(ADC3));
    //Start ADC3 calibration
    ADC_StartCalibration(ADC3);
    //Check the end of ADC3 calibration
    while(ADC_GetCalibrationStatus(ADC3));    
}

/**
 * @brief  ADC DMA initialize
 *
 * @param  none
 * @return none
 *
 * ADC DMA init command
 */
void DMAInit(void) {
    //enable DMA1 clock
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    //reset DMA1 channe1 to default values;
    DMA_DeInit(DMA1_Channel1);
    //channel will be used for memory to memory transfer
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    //setting normal mode (non circular)
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular ;
    //medium priority
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    //source and destination data size word=32bit
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    //automatic memory destination increment enable.
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    //source address increment disable
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    //Location assigned to peripheral register will be source
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    //chunk of data to be transfered
    DMA_InitStructure.DMA_BufferSize = ADC1_ARRAYSIZE;
    //source and destination start addresses
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ADC1_values;
    //send values to DMA registers
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    // Enable DMA1 Channel Transfer Complete interrupt
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
    DMA_Cmd(DMA1_Channel1, ENABLE); //Enable the DMA1 - Channel1
    
    //enable DMA1 clock
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
    //reset DMA1 channe1 to default values;
    DMA_DeInit(DMA2_Channel5);
    //channel will be used for memory to memory transfer
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    //setting normal mode (non circular)
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular ;
    //medium priority
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    //source and destination data size word=32bit
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    //automatic memory destination increment enable.
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    //source address increment disable
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    //Location assigned to peripheral register will be source
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    //chunk of data to be transfered
    DMA_InitStructure.DMA_BufferSize = ADC3_ARRAYSIZE;
    //source and destination start addresses
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &ADC3->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ADC3_values;
    //send values to DMA registers
    DMA_Init(DMA2_Channel5, &DMA_InitStructure);
    // Enable DMA1 Channel Transfer Complete interrupt
    DMA_ITConfig(DMA2_Channel5, DMA_IT_TC, ENABLE);
    DMA_Cmd(DMA2_Channel5, ENABLE); //Enable the DMA1 - Channel1
}

/**
 * @brief  ADC read manual (doesn't be used DMA)
 *
 * @param  ADC_TypeDef* ADCx
 * @param  uint8_t channel
 * @return none
 *
 * ADC read command
 */
uint16_t ADC_Read(ADC_TypeDef* ADCx, uint8_t channel) {
	uint32_t timeout = 0xFFF;
	
	ADC_RegularChannelConfig(ADCx, channel, 1, ADC_SampleTime_239Cycles5);

	/* Start software conversion */
	ADCx->CR2 |= (uint32_t)ADC_CR2_SWSTART;
	
	/* Wait till done */
	while (!(ADCx->SR & ADC_SR_EOC)) {
		if (timeout-- == 0x00) {
			return 0;
		}
	}
	
	/* Return result */
	return ADCx->DR;
}



/******************* (C) COPYRIGHT 2010 ARMVietNam *****END OF FILE****/
