#include "stm32f10x.h"
#include "stm32f10x_gpio.h"

/*
*************************************************************************************************************************************
*															PRIVATE DEFINE															*
*************************************************************************************************************************************
*/

#define SET_LED01     { GPIO_SetBits(GPIOB, GPIO_Pin_15); }
#define SET_LED02     { GPIO_SetBits(GPIOD, GPIO_Pin_9 ); }
#define SET_LED03     { GPIO_SetBits(GPIOD, GPIO_Pin_11); }
#define SET_LED04     { GPIO_SetBits(GPIOD, GPIO_Pin_13); }
#define SET_LED05     { GPIO_SetBits(GPIOD, GPIO_Pin_15); }
#define SET_LED06     { GPIO_SetBits(GPIOG, GPIO_Pin_3 ); }
#define SET_LED07     { GPIO_SetBits(GPIOG, GPIO_Pin_5 ); }
#define SET_LED08     { GPIO_SetBits(GPIOG, GPIO_Pin_7 ); }
#define SET_LED09     { GPIO_SetBits(GPIOC, GPIO_Pin_6 ); }
#define SET_LED10     { GPIO_SetBits(GPIOC, GPIO_Pin_8 ); }
#define SET_LED11     { GPIO_SetBits(GPIOA, GPIO_Pin_8 ); }
#define SET_LED12     { GPIO_SetBits(GPIOA, GPIO_Pin_10); }
#define SET_LED13     { GPIO_SetBits(GPIOA, GPIO_Pin_12); }
#define SET_LED14     { GPIO_SetBits(GPIOA, GPIO_Pin_14); }
#define SET_LED15     { GPIO_SetBits(GPIOC, GPIO_Pin_10); }
#define SET_LED16     { GPIO_SetBits(GPIOC, GPIO_Pin_12); }
#define SET_LED17     { GPIO_SetBits(GPIOD, GPIO_Pin_1 ); }
#define SET_LED18     { GPIO_SetBits(GPIOD, GPIO_Pin_3 ); }
#define SET_LED19     { GPIO_SetBits(GPIOD, GPIO_Pin_5 ); }
#define SET_LED20     { GPIO_SetBits(GPIOD, GPIO_Pin_7 ); }
#define SET_LED21     { GPIO_SetBits(GPIOG, GPIO_Pin_10); }
#define SET_LED22     { GPIO_SetBits(GPIOG, GPIO_Pin_12); }
#define SET_LED23     { GPIO_SetBits(GPIOG, GPIO_Pin_14); }
#define SET_LED24     { GPIO_SetBits(GPIOB, GPIO_Pin_3 ); }

#define RESET_LED01   { GPIO_ResetBits(GPIOB, GPIO_Pin_15); }
#define RESET_LED02   { GPIO_ResetBits(GPIOD, GPIO_Pin_9 ); }
#define RESET_LED03   { GPIO_ResetBits(GPIOD, GPIO_Pin_11); }
#define RESET_LED04   { GPIO_ResetBits(GPIOD, GPIO_Pin_13); }
#define RESET_LED05   { GPIO_ResetBits(GPIOD, GPIO_Pin_15); }
#define RESET_LED06   { GPIO_ResetBits(GPIOG, GPIO_Pin_3 ); }
#define RESET_LED07   { GPIO_ResetBits(GPIOG, GPIO_Pin_5 ); }
#define RESET_LED08   { GPIO_ResetBits(GPIOG, GPIO_Pin_7 ); }
#define RESET_LED09   { GPIO_ResetBits(GPIOC, GPIO_Pin_6 ); }
#define RESET_LED10   { GPIO_ResetBits(GPIOC, GPIO_Pin_8 ); }
#define RESET_LED11   { GPIO_ResetBits(GPIOA, GPIO_Pin_8 ); }
#define RESET_LED12   { GPIO_ResetBits(GPIOA, GPIO_Pin_10); }
#define RESET_LED13   { GPIO_ResetBits(GPIOA, GPIO_Pin_12); }
#define RESET_LED14   { GPIO_ResetBits(GPIOA, GPIO_Pin_14); }
#define RESET_LED15   { GPIO_ResetBits(GPIOC, GPIO_Pin_10); }
#define RESET_LED16   { GPIO_ResetBits(GPIOC, GPIO_Pin_12); }
#define RESET_LED17   { GPIO_ResetBits(GPIOD, GPIO_Pin_1 ); }
#define RESET_LED18   { GPIO_ResetBits(GPIOD, GPIO_Pin_3 ); }
#define RESET_LED19   { GPIO_ResetBits(GPIOD, GPIO_Pin_5 ); }
#define RESET_LED20   { GPIO_ResetBits(GPIOD, GPIO_Pin_7 ); }
#define RESET_LED21   { GPIO_ResetBits(GPIOG, GPIO_Pin_10); }
#define RESET_LED22   { GPIO_ResetBits(GPIOG, GPIO_Pin_12); }
#define RESET_LED23   { GPIO_ResetBits(GPIOG, GPIO_Pin_14); }
#define RESET_LED24   { GPIO_ResetBits(GPIOB, GPIO_Pin_3 ); }

#define MUX00         { GPIO_ResetBits(GPIOB, GPIO_Pin_7 ); GPIO_ResetBits(GPIOB, GPIO_Pin_5 ); }
#define MUX01         { GPIO_ResetBits(GPIOB, GPIO_Pin_7 ); GPIO_SetBits  (GPIOB, GPIO_Pin_5 ); }
#define MUX10         { GPIO_SetBits  (GPIOB, GPIO_Pin_7 ); GPIO_ResetBits(GPIOB, GPIO_Pin_5 ); }
#define MUX11         { GPIO_SetBits  (GPIOB, GPIO_Pin_7 ); GPIO_SetBits  (GPIOB, GPIO_Pin_5 ); }

#define RELAY_FLIP    { GPIO_SetBits  (GPIOB, GPIO_Pin_7 ) ;}
#define RELAY_FLOP    { GPIO_ResetBits(GPIOB, GPIO_Pin_7 ) ;}

/*
*************************************************************************************************************************************
*													   		PRIVATE VARIABLES														*
*************************************************************************************************************************************
*/ 

/*
*************************************************************************************************************************************
*							  								LOCAL FUNCTIONS															*
*************************************************************************************************************************************
*/
void GPIOInit(void);
