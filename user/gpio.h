/**
  ******************************************************************************
  * @file    	../user/gpio.h
  * @author  	Bao Son Le - ET02 - K55
  * @version 	V1.0.0
  * @date    	2015/11/30
  * @brief   	Header file for gpio.c.
  ******************************************************************************
  * @copy
  *
  */ 

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"

/*
*************************************************************************************************************************************
*															PRIVATE DEFINE															*
*************************************************************************************************************************************
*/

#define SET_LED01     { GPIO_SetBits(GPIOB, GPIO_Pin_15); }//1
#define SET_LED13     { GPIO_SetBits(GPIOD, GPIO_Pin_9 ); }//13
#define SET_LED02     { GPIO_SetBits(GPIOD, GPIO_Pin_11); }//2
#define SET_LED14     { GPIO_SetBits(GPIOD, GPIO_Pin_13); }//14
#define SET_LED03     { GPIO_SetBits(GPIOD, GPIO_Pin_15); }//3
#define SET_LED15     { GPIO_SetBits(GPIOG, GPIO_Pin_3 ); }//15
#define SET_LED04     { GPIO_SetBits(GPIOG, GPIO_Pin_5 ); }//4
#define SET_LED16     { GPIO_SetBits(GPIOG, GPIO_Pin_7 ); }//16
#define SET_LED05     { GPIO_SetBits(GPIOC, GPIO_Pin_6 ); }//5
#define SET_LED17     { GPIO_SetBits(GPIOG, GPIO_Pin_2 ); }//17 *
#define SET_LED06     { GPIO_SetBits(GPIOA, GPIO_Pin_8 ); }//6
#define SET_LED18     { GPIO_SetBits(GPIOA, GPIO_Pin_10); }//18
#define SET_LED07     { GPIO_SetBits(GPIOA, GPIO_Pin_12); }//7
#define SET_LED24     { GPIO_SetBits(GPIOG, GPIO_Pin_4 ); }//19 *
#define SET_LED08     { GPIO_SetBits(GPIOG, GPIO_Pin_6 ); }//8  *
#define SET_LED20     { GPIO_SetBits(GPIOG, GPIO_Pin_8 ); }//20 **
#define SET_LED09     { GPIO_SetBits(GPIOD, GPIO_Pin_1 ); }//9
#define SET_LED21     { GPIO_SetBits(GPIOD, GPIO_Pin_3 ); }//21
#define SET_LED10     { GPIO_SetBits(GPIOD, GPIO_Pin_5 ); }//10
#define SET_LED22     { GPIO_SetBits(GPIOD, GPIO_Pin_7 ); }//22
#define SET_LED11     { GPIO_SetBits(GPIOG, GPIO_Pin_10); }//11
#define SET_LED23     { GPIO_SetBits(GPIOG, GPIO_Pin_12); }//23
#define SET_LED12     { GPIO_SetBits(GPIOG, GPIO_Pin_14); }//12
#define SET_LED19     { GPIO_SetBits(GPIOG, GPIO_Pin_15); }//24 *

#define RESET_LED01   { GPIO_ResetBits(GPIOB, GPIO_Pin_15); }//1
#define RESET_LED13   { GPIO_ResetBits(GPIOD, GPIO_Pin_9 ); }//2
#define RESET_LED02   { GPIO_ResetBits(GPIOD, GPIO_Pin_11); }//3
#define RESET_LED14   { GPIO_ResetBits(GPIOD, GPIO_Pin_13); }//4
#define RESET_LED03   { GPIO_ResetBits(GPIOD, GPIO_Pin_15); }//5
#define RESET_LED15   { GPIO_ResetBits(GPIOG, GPIO_Pin_3 ); }//6
#define RESET_LED04   { GPIO_ResetBits(GPIOG, GPIO_Pin_5 ); }//7
#define RESET_LED16   { GPIO_ResetBits(GPIOG, GPIO_Pin_7 ); }//8
#define RESET_LED05   { GPIO_ResetBits(GPIOC, GPIO_Pin_6 ); }//9
#define RESET_LED17   { GPIO_ResetBits(GPIOG, GPIO_Pin_2 ); }//10
#define RESET_LED06   { GPIO_ResetBits(GPIOA, GPIO_Pin_8 ); }//11
#define RESET_LED18   { GPIO_ResetBits(GPIOA, GPIO_Pin_10); }//12
#define RESET_LED07   { GPIO_ResetBits(GPIOA, GPIO_Pin_12); }//13
#define RESET_LED24   { GPIO_ResetBits(GPIOG, GPIO_Pin_4 ); }//14
#define RESET_LED08   { GPIO_ResetBits(GPIOG, GPIO_Pin_6 ); }//15
#define RESET_LED20   { GPIO_ResetBits(GPIOG, GPIO_Pin_8 ); }//16
#define RESET_LED09   { GPIO_ResetBits(GPIOD, GPIO_Pin_1 ); }//17
#define RESET_LED21   { GPIO_ResetBits(GPIOD, GPIO_Pin_3 ); }//18
#define RESET_LED10   { GPIO_ResetBits(GPIOD, GPIO_Pin_5 ); }//19
#define RESET_LED22   { GPIO_ResetBits(GPIOD, GPIO_Pin_7 ); }//20
#define RESET_LED11   { GPIO_ResetBits(GPIOG, GPIO_Pin_10); }//21
#define RESET_LED23   { GPIO_ResetBits(GPIOG, GPIO_Pin_12); }//22
#define RESET_LED12   { GPIO_ResetBits(GPIOG, GPIO_Pin_14); }//23
#define RESET_LED19   { GPIO_ResetBits(GPIOG, GPIO_Pin_15); }//24

#define MUX00         { GPIO_ResetBits(GPIOB, GPIO_Pin_7 ); GPIO_ResetBits(GPIOB, GPIO_Pin_5 ); }
#define MUX01         { GPIO_ResetBits(GPIOB, GPIO_Pin_7 ); GPIO_SetBits  (GPIOB, GPIO_Pin_5 ); }
#define MUX10         { GPIO_SetBits  (GPIOB, GPIO_Pin_7 ); GPIO_ResetBits(GPIOB, GPIO_Pin_5 ); }
#define MUX11         { GPIO_SetBits  (GPIOB, GPIO_Pin_7 ); GPIO_SetBits  (GPIOB, GPIO_Pin_5 ); }

#define RELAY_FLIP    { GPIO_SetBits  (GPIOD, GPIO_Pin_8 ) ;}
#define RELAY_FLOP    { GPIO_ResetBits(GPIOD, GPIO_Pin_8 ) ;}

/*
*************************************************************************************************************************************
*													   		PRIVATE VARIABLES														*
*************************************************************************************************************************************
*/ 

/*
*************************************************************************************************************************************
*							  								GLOBAL FUNCTIONS														*
*************************************************************************************************************************************
*/
void GPIOInit(void);
void Set_IO  (uint8_t ch);
void Reset_IO(uint8_t ch);
