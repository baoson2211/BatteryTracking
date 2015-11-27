/**
  ******************************************************************************
  * @file    	RTC/Calendar/main.c
  * @author  	ARMVN Application Team
  * @version 	V1.0.0
  * @date    	01/30/2010
  * @brief   	Main program body.
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
  
/*
*************************************************************************************************************************************
*															INCLUDED FILES															*
*************************************************************************************************************************************
*/
#include <stdio.h>
#include "rtc.h"


/** @addtogroup STM32F10x_GEM3M_Examples
  * @{
  */

/** @addtogroup RTC_Calendar
  * @{
  */  

  
/*
*************************************************************************************************************************************
*															PRIVATE DEFINE															*
*************************************************************************************************************************************
*/
struct date_t
{
  __IO uint8_t month;
  __IO uint8_t day;
  __IO uint16_t year;
};
struct date_t date_s;


typedef struct {
        uint16_t year;  /* 1..4095 */
        uint8_t  month; /* 1..12 */
        uint8_t  mday;  /* 1.. 31 */
        uint8_t  wday;  /* 0..6, Sunday = 0*/
        uint8_t  hour;  /* 0..23 */
        uint8_t  min;   /* 0..59 */
        uint8_t  sec;   /* 0..59 */
        uint8_t  dst;   /* 0 Winter, !=0 Summer */
} RTC_t;

/*
*************************************************************************************************************************************
*														 	DATA TYPE DEFINE															*
*************************************************************************************************************************************
*/


/*
*************************************************************************************************************************************
*													   		PRIVATE VARIABLES														*
*************************************************************************************************************************************
*/ 
__IO uint32_t TimeDisplay = 0;
//__IO uint32_t index = 0;
//__IO uint32_t index1 = 0;
//__IO uint32_t index2=0;
/*
*************************************************************************************************************************************
*							  								LOCAL FUNCTIONS															*
*************************************************************************************************************************************
*/
/**
  * @brief  	Configures the different system clocks.
  * @param  	None
  * @retval 	None
  */
  extern u8 USART_Scanf(u32 value);
/*
*************************************************************************************************************************************
*															GLOBAL FUNCTIONS														*
*************************************************************************************************************************************
*/
/**
  * @brief  Returns the time entered by user, using Hyperterminal.
  * @param  None
  * @retval Current time RTC counter value
  */
  
uint32_t Time_Regulate(void)
{
	uint32_t Tmp_HH = 0xFF, Tmp_MM = 0xFF, Tmp_SS = 0xFF;
  date_s.day = 0xFF, date_s.month = 0xFF, date_s.year = 0xFF;

	printf("\r\n==============Time Settings=====================================");
	printf("\r\n  Please Set Hours");

	while (Tmp_HH == 0xFF)
	{
		Tmp_HH = USART_Scanf(23);
	}
	printf(":  %d", Tmp_HH);
	printf("\r\n  Please Set Minutes");
	while (Tmp_MM == 0xFF)
	{
		Tmp_MM = USART_Scanf(59);
	}
	printf(":  %d", Tmp_MM);
	printf("\r\n  Please Set Seconds");
	while (Tmp_SS == 0xFF)
	{
		Tmp_SS = USART_Scanf(59);
	}
	printf(":  %d", Tmp_SS);
  
	printf("\r\n  Please Set Day");
	while (date_s.day == 0xFF)
	{
		 date_s.day = USART_Scanf(31);
	}
	printf(":  %d", date_s.day);
  
  printf("\r\n  Please Set Month");
	while (date_s.month == 0xFF)
	{
		date_s.month= USART_Scanf(12);
	}
	printf(":  %d", date_s.month);
  
  printf("\r\n  Please Set Year");
	while (date_s.year == 0xFF)
	{
		date_s.year = USART_Scanf(99) + 2000;
	}
	printf(":  %d", date_s.year);
    
	/* Return the value to store in RTC counter register */
	return((Tmp_HH*3600 + Tmp_MM*60 + Tmp_SS));
}

/**
  * @brief  Adjusts time.
  * @param  None
  * @retval None
  */
void Time_Adjust(void)
{
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();
	/* Change the current time */
	RTC_SetCounter(Time_Regulate());                // set thoi gian ban dau 
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();
}

/**
  * @brief  Displays the current time.
  * @param  TimeVar: RTC counter value.
  * @retval None
  */
void Time_Display(uint32_t TimeVar)
{
	uint32_t THH = 0, TMM = 0, TSS = 0;
  //uint32_t year_h = 0,year_l = 0;
	//year_h =(date_s.year /100);
	//year_l = (date_s.year %100) ;
    
	/* Compute  hours */
	THH = TimeVar / 3600;
	/* Compute minutes */
	TMM = (TimeVar % 3600) / 60;
	/* Compute seconds */
	TSS = (TimeVar % 3600) % 60;
  
  if(THH==24) THH=00;
  //date_s.day++;}

	printf("Time: %0.2d:%0.2d:%0.2d - ", THH, TMM, TSS);
  //printf("%0.2d/%0.2d/%0.4d\r\n", date_s.day, date_s.month,date_s.year) ;
}

void sTime_Display(uint32_t TimeVar, char *timeStr) {
{
	uint32_t THH = 0, TMM = 0, TSS = 0;
  //uint32_t year_h = 0,year_l = 0;
	//year_h =(date_s.year /100);
	//year_l = (date_s.year %100) ;
    
	/* Compute  hours */
	THH = TimeVar / 3600;
	/* Compute minutes */
	TMM = (TimeVar % 3600) / 60;
	/* Compute seconds */
	TSS = (TimeVar % 3600) % 60;
  
  if(THH==24) THH=00;
  //date_s.day++;}

	sprintf(timeStr, "\r\nTime: %3.2d:%0.2d:%0.2d", THH, TMM, TSS);
  //printf("%0.2d/%0.2d/%0.4d\r\n", date_s.day, date_s.month,date_s.year) ;
}
}

/**
  * @brief  Shows the current time (HH:MM:SS) on the Hyperterminal.
  * @param  None
  * @retval None
  */   
void Time_Show(void)
{
	//printf("\n\r");

	/* Infinite loop */
	//while (1)
	{
		/* If 1s has paased */
		if (TimeDisplay == 1)
		{
			/* Display current time */
			Time_Display(RTC_GetCounter());
			TimeDisplay = 0;
		}
	}
}

void Calendar_DateUpdate(void)
{
  if (date_s.month == 1 || date_s.month == 3 || date_s.month == 5 || date_s.month == 7 ||
      date_s.month == 8 || date_s.month == 10 || date_s.month == 12)
  {
    if (date_s.day < 31)
    {
      date_s.day++;
    }
    /* Date structure member: date_s.day = 31 */
    else
    {
      if (date_s.month != 12)
      {
        date_s.month++;
        date_s.day = 1;
      }
      /* Date structure member: date_s.day = 31 & date_s.month =12 */
      else
      {
        date_s.month = 1;
        date_s.day = 1;
        date_s.year++;
      }
    }
  }
  else if (date_s.month == 4 || date_s.month == 6 || date_s.month == 9 ||
           date_s.month == 11)
  {
    if (date_s.day < 30)
    {
      date_s.day++;
    }
    /* Date structure member: date_s.day = 30 */
    else
    {
      date_s.month++;
      date_s.day = 1;
    }
  }
  else if (date_s.month == 2)
  {
    if (date_s.day < 28)
    {
      date_s.day++;
    }
    else if (date_s.day == 28)
    {
      /* Leap year */
      if (((date_s.year)%4)==0)
      {
        date_s.day++;
      }
      else
      {
        date_s.month++;
        date_s.day = 1;
      }
    }
    else if (date_s.day == 29)
    {
      date_s.month++;
      date_s.day = 1;
    }
  }
}

/******************* (C) COPYRIGHT 2009 ARMVietNam *****END OF FILE****/

