/**
  ******************************************************************************
  * @file    	RTC/Calendar/main.h
  * @author  	ARMVN Application Team
  * @version 	V1.0.0
  * @date    	01/30/2010
  * @brief   	Header file for main.c module.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, ARMVietNam SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 ARMVietNam</center></h2>
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H
#define __ADC_H

#ifdef __cplusplus
 extern "C" {
#endif 

/*
*************************************************************************************************************************************
*															INCLUDED FILES															*
*************************************************************************************************************************************
*/
#include "stm32f10x.h"
#include "main.h"

/** @addtogroup STM32F10x_GEM3M_Examples
  * @{
  */

/** @addtogroup GPIO_JTAG_Remap
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
#define ADC_VREF			    330

/*
*************************************************************************************************************************************
*															PRIVATE MACRO															*
*************************************************************************************************************************************
*/



/*
*************************************************************************************************************************************
*															PRIVATE TYPE	DEFINE														*
*************************************************************************************************************************************
*/

#define ADC1_ARRAYSIZE 15*10
#define ADC3_ARRAYSIZE  3*10

/*
*************************************************************************************************************************************
*															PRIVATE VARIABLES														*
*************************************************************************************************************************************
*/


/*
*************************************************************************************************************************************
*															PRIVATE FUNCTION PROTOTYPES												*
*************************************************************************************************************************************
*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/*
*************************************************************************************************************************************
*							  						   		GLOBAL FUNCTION PROTOTYPES												*
*************************************************************************************************************************************
*/
void ADCInit(void);
void DMAInit(void);
uint16_t ADC_Read(ADC_TypeDef* ADCx, uint8_t channel);

#ifdef __cplusplus
}
#endif


#endif /* __MAIN_H */

/**
  * @}
  */

/**
  * @}
  */
