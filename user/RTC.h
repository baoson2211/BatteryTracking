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
#include "stm32f10x_bkp.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_usart.h"
#include <stdint.h>
   
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

#define BKP_DR1_VALUE 0xA5A5

#define FIRSTYEAR   2000                // start year
#define FIRSTDAY    6                   // 0 = Sunday

uint32_t Time_Regulate(void);
void Time_Adjust(void);
void Time_Show(void);
void Time_Display(uint32_t TimeVar);
void sTime_Display(uint32_t TimeVar, char *timeStr);
void Calendar_DateUpdate(void);

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

