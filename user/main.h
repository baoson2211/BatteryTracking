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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif 

/*
*************************************************************************************************************************************
*															INCLUDED FILES															*
*************************************************************************************************************************************
*/
#include "stm32f10x.h"
   
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_usart.h"



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

struct _CELL {
  uint16_t Ch[24];  
  uint16_t Voltage;
} CELL;
               

/*
*************************************************************************************************************************************
*															PRIVATE MACRO															*
*************************************************************************************************************************************
*/

#define CELL01 { CELL.Ch[0]  = ADC_Read(ADC1, ADC_Channel_8) ; }
#define CELL02 { CELL.Ch[1]  = ADC_Read(ADC1, ADC_Channel_14); }
#define CELL03 { CELL.Ch[2]  = ADC_Read(ADC1, ADC_Channel_6) ; }
#define CELL04 { CELL.Ch[3]  = ADC_Read(ADC1, ADC_Channel_4) ; }
#define CELL05 { CELL.Ch[4]  = ADC_Read(ADC1, ADC_Channel_2) ; }
#define CELL06 { CELL.Ch[5]  = ADC_Read(ADC1, ADC_Channel_0) ; }
#define CELL07 { CELL.Ch[6]  = ADC_Read(ADC1, ADC_Channel_12); }
#define CELL08 { CELL.Ch[7]  = ADC_Read(ADC1, ADC_Channel_10); }
#define CELL09 { CELL.Ch[8]  = ADC_Read(ADC3, ADC_Channel_7) ; }
#define CELL10 { CELL.Ch[9]  = ADC_Read(ADC3, ADC_Channel_5) ; }
#define CELL11 { CELL.Ch[10] = ADC_Read(ADC1, ADC_Channel_15); }
#define CELL12 { CELL.Ch[11] = ADC_Read(ADC1, ADC_Channel_7) ; }
#define CELL13 { CELL.Ch[12] = ADC_Read(ADC1, ADC_Channel_5) ; }
#define CELL14 { CELL.Ch[13] = ADC_Read(ADC1, ADC_Channel_3) ; }
#define CELL15 { CELL.Ch[14] = ADC_Read(ADC1, ADC_Channel_1) ; }
#define CELL16 { CELL.Ch[15] = ADC_Read(ADC1, ADC_Channel_13); }
#define CELL17 { CELL.Ch[16] = ADC_Read(ADC1, ADC_Channel_11); }
#define CELL18 { CELL.Ch[17] = ADC_Read(ADC3, ADC_Channel_8) ; }
#define CELL19 { CELL.Ch[18] = ADC_Read(ADC1, ADC_Channel_11); }
#define CELL20 { CELL.Ch[19] = ADC_Read(ADC3, ADC_Channel_8) ; }
#define CELL21 { CELL.Ch[20] = ADC_Read(ADC1, ADC_Channel_11); }
#define CELL22 { CELL.Ch[21] = ADC_Read(ADC3, ADC_Channel_8) ; }
#define CELL23 { CELL.Ch[22] = ADC_Read(ADC1, ADC_Channel_11); }
#define CELL24 { CELL.Ch[23] = ADC_Read(ADC3, ADC_Channel_8) ; }

/*
*************************************************************************************************************************************
*															PRIVATE TYPE	DEFINE														*
*************************************************************************************************************************************
*/


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
void RCC_Configuration(void);
void Delay(__IO uint32_t nCount);

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

void RTC_Configuration(void);
void NVIC_Configuration(void);
uint32_t Time_Regulate(void);
void Time_Adjust(void);
void Time_Show(void);
void Time_Display(uint32_t TimeVar);
void sTime_Display(uint32_t TimeVar, char *timeStr);
void WriteFile(void);
uint8_t USART_Scanf(uint32_t value);
 /*
*************************************************************************************************************************************
*							  						   		GLOBAL FUNCTION PROTOTYPES												*
*************************************************************************************************************************************
*/




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


/******************* (C) COPYRIGHT 2009 ARMVietNam *****END OF FILE****/

