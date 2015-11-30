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
#include "gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"

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

/*
*************************************************************************************************************************************
*															GLOBAL FUNCTIONS														*
*************************************************************************************************************************************
*/

void GPIOInit(void) {
  GPIO_InitTypeDef GPIO_InitStructure; 
  EXTI_InitTypeDef  EXTI_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | 
                         RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG, ENABLE);
  
  GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_10 | GPIO_Pin_12 | GPIO_Pin_14 ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_9 | GPIO_Pin_14 | GPIO_Pin_15 ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_10 | GPIO_Pin_12 ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_10 | GPIO_Pin_12 | GPIO_Pin_14 ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  
  GPIO_SetBits  ( GPIOB, GPIO_Pin_14);
  GPIO_ResetBits( GPIOB, GPIO_Pin_9 );
  GPIO_ResetBits( GPIOD, GPIO_Pin_8 );
  
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

void Set_IO(uint8_t ch){
  switch (ch) {
    case 0:  SET_LED01  ; break;
    case 1:  SET_LED02  ; break;
    case 2:  SET_LED03  ; break;
    case 3:  SET_LED04  ; break;
    case 4:  SET_LED05  ; break;
    case 5:  SET_LED06  ; break;
    case 6:  SET_LED07  ; break;
    case 7:  SET_LED08  ; break;
    case 8:  SET_LED09  ; break;
    case 9:  SET_LED10  ; break;
    case 10: SET_LED11  ; break;
    case 11: SET_LED12  ; break;
    case 12: SET_LED13  ; break;
    case 13: SET_LED14  ; break;
    case 14: SET_LED15  ; break;
    case 15: SET_LED16  ; break;
    case 16: SET_LED17  ; break;
    case 17: SET_LED18  ; break;
    case 18: SET_LED19  ; break;
    case 19: SET_LED20  ; break;
    case 20: SET_LED21  ; break;
    case 21: SET_LED22  ; break;
    case 22: SET_LED23  ; break; 
    case 23: SET_LED24  ; break;
    default:              break;
  }
}

void Reset_IO(uint8_t ch){
  switch (ch) {
    case 0:  RESET_LED01  ; break;
    case 1:  RESET_LED02  ; break;
    case 2:  RESET_LED03  ; break;
    case 3:  RESET_LED04  ; break;
    case 4:  RESET_LED05  ; break;
    case 5:  RESET_LED06  ; break;
    case 6:  RESET_LED07  ; break;
    case 7:  RESET_LED08  ; break;
    case 8:  RESET_LED09  ; break;
    case 9:  RESET_LED10  ; break;
    case 10: RESET_LED11  ; break;
    case 11: RESET_LED12  ; break;
    case 12: RESET_LED13  ; break;
    case 13: RESET_LED14  ; break;
    case 14: RESET_LED15  ; break;
    case 15: RESET_LED16  ; break;
    case 16: RESET_LED17  ; break;
    case 17: RESET_LED18  ; break;
    case 18: RESET_LED19  ; break;
    case 19: RESET_LED20  ; break;
    case 20: RESET_LED21  ; break;
    case 21: RESET_LED22  ; break;
    case 22: RESET_LED23  ; break; 
    case 23: RESET_LED24  ; break;
    default:              break;
  }
}
