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
#ifndef RTC_H_
#define RTC_H_



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
#include <stdbool.h>
   
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

//#define BKP_DR1_VALUE 0xA5A5

//#define FIRSTYEAR   2000                // start year
//#define FIRSTDAY    6                   // 0 = Sunday

//uint32_t Time_Regulate(void);
//void Time_Adjust(void);
//void Time_Show(void);
//void Time_Display(uint32_t TimeVar);
//void sTime_Display(uint32_t TimeVar, char *timeStr);
//void Calendar_DateUpdate(void);

typedef struct {
	uint16_t year;	/* 1..4095 */
	uint8_t  month;	/* 1..12 */
	uint8_t  mday;	/* 1..31 */
	uint8_t  wday;	/* 0..6, Sunday = 0*/
	uint8_t  hour;	/* 0..23 */
	uint8_t  min;	/* 0..59 */
	uint8_t  sec;	/* 0..59 */
	uint8_t  dst;	/* 0 Winter, !=0 Summer */
} RTC_t;

int rtc_init(void);
bool rtc_gettime (RTC_t*);			/* Get time */
bool rtc_settime (const RTC_t*);		/* Set time */
void my_RTC_SetCounter(uint32_t cnt);		/* Set RTC counter */

void RTC_Configuration(void);
void Time_Regulate(RTC_t * RTC_time);

 /*
*************************************************************************************************************************************
*							  						   		GLOBAL FUNCTION PROTOTYPES												*
*************************************************************************************************************************************
*/


#ifdef __cplusplus
}
#endif


#endif /* __RTC_H */

/**
  * @}
  */

/**
  * @}
  */


/******************* (C) COPYRIGHT 2009 ARMVietNam *****END OF FILE****/

