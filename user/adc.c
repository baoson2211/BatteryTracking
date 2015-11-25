/**
  ******************************************************************************
  * @file    	ADC/Potentiometer/main.c
  * @author  	ARMVN Application Team
  * @version 	V1.0.0
  * @date    	14/03/2010
  * @brief   	Main program body.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 ARMVietNam</center></h2>
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
/* constant for adc resolution is 12 bit = 4096 */
#define ADC_12BIT_FACTOR	4096

/* constant for adc threshold value 3.3V */
#define ADC_VREF			330
uint8_t i;

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

volatile uint16_t ADC_values1[ARRAYSIZE1];
volatile uint16_t ADC_values3[ARRAYSIZE3];
volatile uint32_t status1 = 0;  
volatile uint32_t status3 = 0;
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
  * @brief  	Main program.
  * @param  	None
  * @retval 	None
//  */  
//void ADC_PinConfig(void)
//{
//	
//		/* 
//	  *	kich ADC pins	
//	  */
//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//		GPIO_Init(GPIOB, &GPIO_InitStructure);

//	/* Configure PA.02 (ADC Channel2) as analog input -------------------------*/
//  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
//  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  	GPIO_Init(GPIOA, &GPIO_InitStructure);

//	/* Configure clocks for ADC and GPIO PORT */
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

//	/* ADCx configuration ------------------------------------------------------*/
//  	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
//  	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
//  	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
//  	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
//  	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
//  	ADC_InitStructure.ADC_NbrOfChannel = 1;
//  	ADC_Init(ADC1, &ADC_InitStructure);	
//	  /* ADC1 Regular Channel 2 Configuration */
//	  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_28Cycles5);		
//  	/* Enable ADC1 */
//  	//ADC_Cmd(ADC1, ENABLE);  
//    ADC1->CR2 |= ADC_CR2_ADON;		

//	/* Enable Vrefint channel enable temperature sensor for ADC module */
//	ADC_TempSensorVrefintCmd(ENABLE);

//	/* Enable ADC1 reset calibaration register */   
//    	ADC_ResetCalibration(ADC1);

//    	/* Check the end of ADC1 reset calibration register */
//    	while(ADC_GetResetCalibrationStatus(ADC1));

//    	/* Start ADC1 calibaration */
//    	ADC_StartCalibration(ADC1);
//    	/* Check the end of ADC1 calibration */
//    	while(ADC_GetCalibrationStatus(ADC1));  
//	/* Start ADC1 Software Conversion */ 
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

//	
//	/* start main program */
//	GPIO_WriteBit(GPIOA, GPIO_Pin_1, 0);
//	GPIO_WriteBit(GPIOB, GPIO_Pin_9, 0);
//	while (1)	
//	{
//		for(i=0;i<16;i++)
//		{
//			GPIOA->CRL = 0x00033300;
//			GPIO_WriteBit(GPIOB, GPIO_Pin_9, 1);
//			ADC1->CR2 |= ADC_CR2_ADON;
//			/* get adc raw value */
//			//adc_raw_val = ADC_GetConversionValue(ADC1);
//			while ((ADC1->SR & ADC_SR_EOC) == 0);
//			
//			GPIOA->CRL = 0x00033330;
//			GPIO_WriteBit(GPIOA, GPIO_Pin_1, 0);
//			GPIO_WriteBit(GPIOB, GPIO_Pin_9, 0);
//			
//			adc_raw_val += ADC1->DR;
//		}
//			/* calculate voltage value */
//			//potentionmeter_val = adc_raw_val*ADC_VREF/ADC_12BIT_FACTOR;
//			
//			/* Output a message on Hyperterminal using printf function */
//			//printf("\n\rPoten_Volt = %0.2fV\n\r", (f32)potentionmeter_val/100.0);
//			printf("\n\rADC = %d\n\r", adc_raw_val);
//			/* Insert delay */
//			Delay(30);
//	  
//	}
//}

void ADCInit(void)
{
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
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7| GPIO_Pin_9| GPIO_Pin_10 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
  
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    //We will convert multiple channels
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    //select continuous conversion mode
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//!
    //select no external triggering
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    //right 12-bit data alignment in ADC data register
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    //8 channels conversion
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    //load structure values to control and status registers
    ADC_Init(ADC1, &ADC_InitStructure);
    //wake up temperature sensor
    //ADC_TempSensorVrefintCmd(ENABLE);
    
    //Enable ADC1
    ADC_Cmd(ADC1, ENABLE);
    //enable DMA for ADC
    //ADC_DMACmd(ADC1, ENABLE);
    //Enable ADC1 reset calibration register
    ADC_ResetCalibration(ADC1);
    //Check the end of ADC1 reset calibration register
    while(ADC_GetResetCalibrationStatus(ADC1));
    //Start ADC1 calibration
    ADC_StartCalibration(ADC1);
    //Check the end of ADC1 calibration
    while(ADC_GetCalibrationStatus(ADC1));
    
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    //We will convert multiple channels
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    //select continuous conversion mode
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//!
    //select no external triggering
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    //right 12-bit data alignment in ADC data register
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    //8 channels conversion
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    //load structure values to control and status registers
    ADC_Init(ADC3, &ADC_InitStructure);
    //wake up temperature sensor
    //ADC_TempSensorVrefintCmd(ENABLE);

    //Enable ADC3
    ADC_Cmd(ADC3, ENABLE);
    //enable DMA for ADC
    //ADC_DMACmd(ADC3, ENABLE);
    //Enable ADC3 reset calibration register
    ADC_ResetCalibration(ADC3);
    //Check the end of ADC3 reset calibration register
    while(ADC_GetResetCalibrationStatus(ADC3));
    //Start ADC3 calibration
    ADC_StartCalibration(ADC3);
    //Check the end of ADC3 calibration
    while(ADC_GetCalibrationStatus(ADC3));    
}


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
    DMA_InitStructure.DMA_BufferSize = ARRAYSIZE1;
    //source and destination start addresses
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_values1;
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
    DMA_InitStructure.DMA_BufferSize = ARRAYSIZE3;
    //source and destination start addresses
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_values3;
    //send values to DMA registers
    DMA_Init(DMA2_Channel5, &DMA_InitStructure);
    // Enable DMA1 Channel Transfer Complete interrupt
    DMA_ITConfig(DMA2_Channel5, DMA_IT_TC, ENABLE);
    DMA_Cmd(DMA2_Channel5, ENABLE); //Enable the DMA1 - Channel1
}

uint16_t ADC_Read(ADC_TypeDef* ADCx, uint8_t channel) {
	uint32_t timeout = 0xFFF;
	
	ADC_RegularChannelConfig(ADCx, channel, 1, ADC_SampleTime_28Cycles5);

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
