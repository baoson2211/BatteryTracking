/**
  ******************************************************************************
  * @file    	../user/main.c
  * @author  	Bao Son Le - ET02 - K55
  * @version 	V1.0.0
  * @date    	2015/11/30
  * @brief   	Main program body.
  ******************************************************************************
  * @copy
  *
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "sdcard.h"
#include "stdio.h"
#include "adc.h"
#include "rtc.h"
#include "gpio.h"
#include "stm32f10x_usart.h"

#include <stdlib.h>
#include <string.h>

#include "integer.h"
#include "ff.h"
#include "diskio.h"

/** @addtogroup StdPeriph_Examples
  * @{
  */

/** @addtogroup SDIO_Example
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private define ------------------------------------------------------------*/
#define TxBufferSize1   (countof(TxBuffer1) - 1)
#define RxBufferSize1   (countof(TxBuffer1) - 1)

/* Private macro -------------------------------------------------------------*/
#define countof(a)   (sizeof(a) / sizeof(*(a)))

/* Private variables ---------------------------------------------------------*/
USART_InitTypeDef USART_InitStructure;
uint8_t TxBuffer1[] = "USART Interrupt Example: This is USART1 DEMO";
uint8_t RxBuffer1[RxBufferSize1],rec_f;
__IO uint8_t TxCounter1 = 0x00;
__IO uint8_t RxCounter1 = 0x00; 
uint8_t NbrOfDataToTransfer1 = TxBufferSize1;
uint8_t NbrOfDataToRead1 = RxBufferSize1;

__IO uint32_t TimingDelay;

extern __IO uint32_t TimeDisplay;
extern volatile uint16_t ADC1_values[ADC1_ARRAYSIZE]; //ADC1 Value
extern volatile uint16_t ADC3_values[ADC3_ARRAYSIZE]; //ADC3 Value
extern volatile uint8_t  statusADC1, statusADC3;

/* Private define ------------------------------------------------------------*/
#define BlockSize            512 /* Block Size in Bytes */
//#define BlockSize          128 /* Block Size in Bytes */
#define BufferWordsSize      (BlockSize >> 2)

#define NumberOfBlocks       2  /* For Multi Blocks operation (Read/Write) */
#define MultiBufferWordsSize ((BlockSize * NumberOfBlocks) >> 2)

#define TEST                 1
#define RELEASE              1
#define TESTLED              1

/* Array resistance voltage divider:    R1/1 |  R2/10 | scale * 10           */
uint32_t RESISTANCE[24][3]    =    { {   22  ,  1000  ,    112   },  /* CH01 */
                                     {   47  ,   100  ,    108   },  /* CH02 */
                                     {  100  ,   100  ,    109   },  /* CH03 */
                                     {  180  ,   100  ,    108   },  /* CH04 */
                                     {  270  ,   100  ,    108   },  /* CH05 */
                                     {  330  ,   100  ,    108   },  /* CH06 */
                                     {  390  ,   100  ,    109   },  /* CH07 */
                                     {  470  ,   100  ,    108   },  /* CH08 */
                                     {  820  ,   150  ,    109   },  /* CH09 */
                                     {  560  ,   100  ,    109   },  /* CH10 */
                                     {  680  ,   100  ,    111   },  /* CH11 */
                                     {  100  ,   120  ,     22   },  /* CH12 */
                                     {  820  ,   100  ,    109   },  /* CH13 */
                                     {  560  ,    68  ,    108   },  /* CH14 */
                                     {  470  ,    47  ,    107   },  /* CH15 */
                                     { 1000  ,   100  ,    106   },  /* CH16 */
                                     {  470  ,    47  ,    106   },  /* CH17 */
                                     { 1200  ,   100  ,    107   },  /* CH18 */
                                     { 1200  ,   100  ,    106   },  /* CH19 */
                                     {  910  ,    51  ,    109   },  /* CH20 */
                                     {  680  ,    47  ,    108   },  /* CH21 */
                                     {  470  ,    33  ,    108   },  /* CH22 */
                                     {  820  ,    51  ,    107   },  /* CH23 */
                                     {  820  ,    47  ,    107   }   /* CH24 */ };

/* Minimum threshold voltage be accepted */
const int16_t THRESHOLD[24] = { 175 , 175 , 175 , 175 , 175 , 175 , 175 , 175 , 175 , 175 , 175 , 175 ,
                                175 , 175 , 175 , 175 , 175 , 175 , 175 , 175 , 175 , 175 , 175 , 175 };

/* Warning statement                WARN | STATE */
static __IO uint8_t WARNING[24][2]={ { 0 , 0 },  /* CH01 */
                                     { 0 , 0 },  /* CH02 */
                                     { 0 , 0 },  /* CH03 */
                                     { 0 , 0 },  /* CH04 */
                                     { 0 , 0 },  /* CH05 */
                                     { 0 , 0 },  /* CH06 */ 
                                     { 0 , 0 },  /* CH07 */
                                     { 0 , 0 },  /* CH08 */
                                     { 0 , 0 },  /* CH09 */
                                     { 0 , 0 },  /* CH10 */
                                     { 0 , 0 },  /* CH11 */
                                     { 0 , 0 },  /* CH12 */
                                     { 0 , 0 },  /* CH13 */
                                     { 0 , 0 },  /* CH14 */
                                     { 0 , 0 },  /* CH15 */
                                     { 0 , 0 },  /* CH16 */
                                     { 0 , 0 },  /* CH17 */ 
                                     { 0 , 0 },  /* CH18 */
                                     { 0 , 0 },  /* CH19 */
                                     { 0 , 0 },  /* CH20 */
                                     { 0 , 0 },  /* CH21 */
                                     { 0 , 0 },  /* CH22 */ 
                                     { 0 , 0 },  /* CH23 */
                                     { 0 , 0 }   /* CH24 */      };
                           
enum CLI{
      Create_file = 0x31,
      Write_file  = 0x32,
      Read_file   = 0x33
    } cmd;

/**
  * @brief  Cell structure
  * 
  * @var    uint16_t Ch[24]     - Channels input voltage 
  * @var    int16_t  Value[24]  - Cells voltage 
  * @var    uint16_t Voltage    - Battery voltage 
  * 
  */
struct _CELL {
      uint16_t Ch[24];  
       int16_t Value[24];
      uint16_t Voltage;
    } CELL;

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
SD_CardInfo SDCardInfo;
uint32_t Buffer_Block_Tx[BufferWordsSize], Buffer_Block_Rx[BufferWordsSize];
uint32_t Buffer_MultiBlock_Tx[MultiBufferWordsSize], Buffer_MultiBlock_Rx[MultiBufferWordsSize];
volatile TestStatus EraseStatus = FAILED, TransferStatus1 = FAILED, TransferStatus2 = FAILED;
SD_Error Status = SD_OK;

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void NVIC_Configuration(void);
void Fill_Buffer(uint32_t *pBuffer, uint16_t BufferLenght, uint32_t Offset);
TestStatus Buffercmp(uint32_t* pBuffer1, uint32_t* pBuffer2, uint16_t BufferLength);
TestStatus eBuffercmp(uint32_t* pBuffer, uint16_t BufferLength);


void USART1_Config(void);
void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,uint16_t Len);
int GetKey (void) ;
int SendChar (int ch) ;
void fstring (char *str);
void Delay(__IO uint32_t nCount);
void Serial_Init(void);	
void RTC_Configuration(void);
		
/* Private functions ---------------------------------------------------------*/
    FATFS fs;            // Work area (file system object) for logical drive
    FIL fdst;            // file objects
    BYTE buffer[512];    // file copy buffer
    FRESULT res;         // FatFs function common result code
    UINT br, bw;         // File R/W count
    
    
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval : None
  */

/**
  * @brief  Delay nTime milliseconds with systick timer
  * @param  __IO uint32_t nTime
  * @retval none
  */
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}

/**
  * @brief  RCC Configuration
  * @param  none
  * @retval none
  */
void RCC_Configuration(void)
{
  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,
     initialize the PLL and update the SystemFrequency variable. */
  SystemInit();
}

/**
  * @brief  Remapping: mapping cell channels to ADC channel inputs, Analog to Digital converter and
  *         cell channel seleted by CD4052
  * @param  none
  * @retval none
  */
void Remapping (void){

/* CD4052 pin pack BA = 00 */
  MUX00;
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  Delay(10);
  ADC_SoftwareStartConvCmd(ADC1, DISABLE);
   
  CELL.Ch[16] = (( ADC1_values[14+(15*0)] + ADC1_values[14+(15*1)] + ADC1_values[14+(15*2)] + ADC1_values[14+(15*3)] + 
                   ADC1_values[14+(15*4)] + ADC1_values[14+(15*5)] + ADC1_values[14+(15*6)] + ADC1_values[14+(15*7)] +
                   ADC1_values[14+(15*8)] + ADC1_values[14+(15*9)] ) / 10 );
  
  ADC_SoftwareStartConvCmd(ADC3, ENABLE);
  Delay(10);
  ADC_SoftwareStartConvCmd(ADC3, DISABLE);
  
  CELL.Ch[17] = (( ADC3_values[2+(3*0)] + ADC3_values[2+(3*1)] + ADC3_values[2+(3*2)] + ADC3_values[2+(3*3)] + 
                   ADC3_values[2+(3*4)] + ADC3_values[2+(3*5)] + ADC3_values[2+(3*6)] + ADC3_values[2+(3*7)] +
                   ADC3_values[2+(3*8)] + ADC3_values[2+(3*9)] ) / 10 );
  
/* CD4052 pin pack BA = 01 */
  MUX01;
  
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  Delay(10);
  ADC_SoftwareStartConvCmd(ADC1, DISABLE);

  CELL.Ch[18] = (( ADC1_values[14+(15*0)] + ADC1_values[14+(15*1)] + ADC1_values[14+(15*2)] + ADC1_values[14+(15*3)] + 
                   ADC1_values[14+(15*4)] + ADC1_values[14+(15*5)] + ADC1_values[14+(15*6)] + ADC1_values[14+(15*7)] +
                   ADC1_values[14+(15*8)] + ADC1_values[14+(15*9)] ) / 10 );
  
  ADC_SoftwareStartConvCmd(ADC3, ENABLE);
  Delay(10);
  ADC_SoftwareStartConvCmd(ADC3, DISABLE);
    
  CELL.Ch[19] = (( ADC3_values[2+(3*0)] + ADC3_values[2+(3*1)] + ADC3_values[2+(3*2)] + ADC3_values[2+(3*3)] + 
                   ADC3_values[2+(3*4)] + ADC3_values[2+(3*5)] + ADC3_values[2+(3*6)] + ADC3_values[2+(3*7)] +
                   ADC3_values[2+(3*8)] + ADC3_values[2+(3*9)] ) / 10 );

/* CD4052 pin pack BA = 10 */
  MUX10;
  
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  Delay(10);
  ADC_SoftwareStartConvCmd(ADC1, DISABLE);

  CELL.Ch[20] = (( ADC1_values[14+(15*0)] + ADC1_values[14+(15*1)] + ADC1_values[14+(15*2)] + ADC1_values[14+(15*3)] + 
                   ADC1_values[14+(15*4)] + ADC1_values[14+(15*5)] + ADC1_values[14+(15*6)] + ADC1_values[14+(15*7)] +
                   ADC1_values[14+(15*8)] + ADC1_values[14+(15*9)] ) / 10 );
  
  ADC_SoftwareStartConvCmd(ADC3, ENABLE);
  Delay(10);
  ADC_SoftwareStartConvCmd(ADC3, DISABLE);
    
  CELL.Ch[21] = (( ADC3_values[2+(3*0)] + ADC3_values[2+(3*1)] + ADC3_values[2+(3*2)] + ADC3_values[2+(3*3)] + 
                   ADC3_values[2+(3*4)] + ADC3_values[2+(3*5)] + ADC3_values[2+(3*6)] + ADC3_values[2+(3*7)] +
                   ADC3_values[2+(3*8)] + ADC3_values[2+(3*9)] ) / 10 );

/* CD4052 pin pack BA = 11 */
  MUX11;
  
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  Delay(10);
  ADC_SoftwareStartConvCmd(ADC1, DISABLE);
  
  CELL.Ch[0]  = (( ADC1_values[ 0+(15*0)] + ADC1_values[ 0+(15*1)] + ADC1_values[ 0+(15*2)] + ADC1_values[ 0+(15*3)] + 
                   ADC1_values[ 0+(15*4)] + ADC1_values[ 0+(15*5)] + ADC1_values[ 0+(15*6)] + ADC1_values[ 0+(15*7)] +
                   ADC1_values[ 0+(15*8)] + ADC1_values[ 0+(15*9)] ) / 10 );;
  CELL.Ch[1]  = (( ADC1_values[ 1+(15*0)] + ADC1_values[ 1+(15*1)] + ADC1_values[ 1+(15*2)] + ADC1_values[ 1+(15*3)] + 
                   ADC1_values[ 1+(15*4)] + ADC1_values[ 1+(15*5)] + ADC1_values[ 1+(15*6)] + ADC1_values[ 1+(15*7)] +
                   ADC1_values[ 1+(15*8)] + ADC1_values[ 1+(15*9)] ) / 10 );
  CELL.Ch[2]  = (( ADC1_values[ 2+(15*0)] + ADC1_values[ 2+(15*1)] + ADC1_values[ 2+(15*2)] + ADC1_values[ 2+(15*3)] + 
                   ADC1_values[ 2+(15*4)] + ADC1_values[ 2+(15*5)] + ADC1_values[ 2+(15*6)] + ADC1_values[ 2+(15*7)] +
                   ADC1_values[ 2+(15*8)] + ADC1_values[ 2+(15*9)] ) / 10 );
  CELL.Ch[3]  = (( ADC1_values[ 3+(15*0)] + ADC1_values[ 3+(15*1)] + ADC1_values[ 3+(15*2)] + ADC1_values[ 3+(15*3)] + 
                   ADC1_values[ 3+(15*4)] + ADC1_values[ 3+(15*5)] + ADC1_values[ 3+(15*6)] + ADC1_values[ 3+(15*7)] +
                   ADC1_values[ 3+(15*8)] + ADC1_values[ 3+(15*9)] ) / 10 );
  CELL.Ch[4]  = (( ADC1_values[ 4+(15*0)] + ADC1_values[ 4+(15*1)] + ADC1_values[ 4+(15*2)] + ADC1_values[ 4+(15*3)] + 
                   ADC1_values[ 4+(15*4)] + ADC1_values[ 4+(15*5)] + ADC1_values[ 4+(15*6)] + ADC1_values[ 4+(15*7)] +
                   ADC1_values[ 4+(15*8)] + ADC1_values[ 4+(15*9)] ) / 10 );
  CELL.Ch[5]  = (( ADC1_values[ 5+(15*0)] + ADC1_values[ 5+(15*1)] + ADC1_values[ 5+(15*2)] + ADC1_values[ 5+(15*3)] + 
                   ADC1_values[ 5+(15*4)] + ADC1_values[ 5+(15*5)] + ADC1_values[ 5+(15*6)] + ADC1_values[ 5+(15*7)] +
                   ADC1_values[ 5+(15*8)] + ADC1_values[ 5+(15*9)] ) / 10 );
  CELL.Ch[6]  = (( ADC1_values[ 6+(15*0)] + ADC1_values[ 6+(15*1)] + ADC1_values[ 6+(15*2)] + ADC1_values[ 6+(15*3)] + 
                   ADC1_values[ 6+(15*4)] + ADC1_values[ 6+(15*5)] + ADC1_values[ 6+(15*6)] + ADC1_values[ 6+(15*7)] +
                   ADC1_values[ 6+(15*8)] + ADC1_values[ 6+(15*9)] ) / 10 );
  CELL.Ch[7]  = (( ADC1_values[ 7+(15*0)] + ADC1_values[ 7+(15*1)] + ADC1_values[ 7+(15*2)] + ADC1_values[ 7+(15*3)] + 
                   ADC1_values[ 7+(15*4)] + ADC1_values[ 7+(15*5)] + ADC1_values[ 7+(15*6)] + ADC1_values[ 7+(15*7)] +
                   ADC1_values[ 7+(15*8)] + ADC1_values[ 7+(15*9)] ) / 10 );
  
  CELL.Ch[10] = (( ADC1_values[ 8+(15*0)] + ADC1_values[ 8+(15*1)] + ADC1_values[ 8+(15*2)] + ADC1_values[ 8+(15*3)] + 
                   ADC1_values[ 8+(15*4)] + ADC1_values[ 8+(15*5)] + ADC1_values[ 8+(15*6)] + ADC1_values[ 8+(15*7)] +
                   ADC1_values[ 8+(15*8)] + ADC1_values[ 8+(15*9)] ) / 10 );
  CELL.Ch[11] = (( ADC1_values[ 9+(15*0)] + ADC1_values[ 9+(15*1)] + ADC1_values[ 9+(15*2)] + ADC1_values[ 9+(15*3)] + 
                   ADC1_values[ 9+(15*4)] + ADC1_values[ 9+(15*5)] + ADC1_values[ 9+(15*6)] + ADC1_values[ 9+(15*7)] +
                   ADC1_values[ 9+(15*8)] + ADC1_values[ 9+(15*9)] ) / 10 );
  CELL.Ch[12] = (( ADC1_values[10+(15*0)] + ADC1_values[10+(15*1)] + ADC1_values[10+(15*2)] + ADC1_values[10+(15*3)] + 
                   ADC1_values[10+(15*4)] + ADC1_values[10+(15*5)] + ADC1_values[10+(15*6)] + ADC1_values[10+(15*7)] +
                   ADC1_values[10+(15*8)] + ADC1_values[10+(15*9)] ) / 10 );
  CELL.Ch[13] = (( ADC1_values[11+(15*0)] + ADC1_values[11+(15*1)] + ADC1_values[11+(15*2)] + ADC1_values[11+(15*3)] + 
                   ADC1_values[11+(15*4)] + ADC1_values[11+(15*5)] + ADC1_values[11+(15*6)] + ADC1_values[11+(15*7)] +
                   ADC1_values[11+(15*8)] + ADC1_values[11+(15*9)] ) / 10 );
  CELL.Ch[14] = (( ADC1_values[12+(15*0)] + ADC1_values[12+(15*1)] + ADC1_values[12+(15*2)] + ADC1_values[12+(15*3)] + 
                   ADC1_values[12+(15*4)] + ADC1_values[12+(15*5)] + ADC1_values[12+(15*6)] + ADC1_values[12+(15*7)] +
                   ADC1_values[12+(15*8)] + ADC1_values[12+(15*9)] ) / 10 );
  CELL.Ch[15] = (( ADC1_values[13+(15*0)] + ADC1_values[13+(15*1)] + ADC1_values[13+(15*2)] + ADC1_values[13+(15*3)] + 
                   ADC1_values[13+(15*4)] + ADC1_values[13+(15*5)] + ADC1_values[13+(15*6)] + ADC1_values[13+(15*7)] +
                   ADC1_values[13+(15*8)] + ADC1_values[13+(15*9)] ) / 10 );
  
  CELL.Ch[22] = (( ADC1_values[14+(15*0)] + ADC1_values[14+(15*1)] + ADC1_values[14+(15*2)] + ADC1_values[14+(15*3)] + 
                   ADC1_values[14+(15*4)] + ADC1_values[14+(15*5)] + ADC1_values[14+(15*6)] + ADC1_values[14+(15*7)] +
                   ADC1_values[14+(15*8)] + ADC1_values[14+(15*9)] ) / 10 );
  
  ADC_SoftwareStartConvCmd(ADC3, ENABLE);
  Delay(10);
  ADC_SoftwareStartConvCmd(ADC3, DISABLE);
  
  CELL.Ch[8]  = (( ADC3_values[0+(3*0)] + ADC3_values[0+(3*1)] + ADC3_values[0+(3*0)] + ADC3_values[0+(3*3)] + 
                   ADC3_values[0+(3*4)] + ADC3_values[0+(3*5)] + ADC3_values[0+(3*6)] + ADC3_values[0+(3*7)] +
                   ADC3_values[0+(3*8)] + ADC3_values[0+(3*9)] ) / 10 );
  CELL.Ch[9]  = (( ADC3_values[1+(3*0)] + ADC3_values[1+(3*1)] + ADC3_values[1+(3*1)] + ADC3_values[1+(3*3)] + 
                   ADC3_values[1+(3*4)] + ADC3_values[1+(3*5)] + ADC3_values[1+(3*6)] + ADC3_values[1+(3*7)] +
                   ADC3_values[1+(3*8)] + ADC3_values[1+(3*9)] ) / 10 );
  
  CELL.Ch[23] = (( ADC3_values[2+(3*0)] + ADC3_values[2+(3*1)] + ADC3_values[2+(3*2)] + ADC3_values[2+(3*3)] + 
                   ADC3_values[2+(3*4)] + ADC3_values[2+(3*5)] + ADC3_values[2+(3*6)] + ADC3_values[2+(3*7)] +
                   ADC3_values[2+(3*8)] + ADC3_values[2+(3*9)] ) / 10 );
}

/**
  * @brief  Converting ADC value to channel inputs voltage
  * @param  none
  * @retval none
  */
void Converting(void) {
  uint8_t ch;
  for ( ch = 0; ch < 24 ; ch++ ){
    CELL.Ch[ch] = (CELL.Ch[ch] * ADC_VREF * (RESISTANCE[ch][0] + RESISTANCE[ch][1])) / (ADC_12BIT_FACTOR * RESISTANCE[ch][1]);
    CELL.Ch[ch] = (int16_t)  (CELL.Ch[ch] * 100 / RESISTANCE[ch][2] );
  }
}

/**
  * @brief  Calculate cells voltage
  * @param  none
  * @retval none
  */
void ValueCell(void) {
  CELL.Value[0]  = (int16_t)   CELL.Ch[0];
  CELL.Value[1]  = (int16_t) ( CELL.Ch[1]  - CELL.Ch[0]);
  CELL.Value[2]  = (int16_t) ( CELL.Ch[2]  - CELL.Ch[1]);
  CELL.Value[3]  = (int16_t) ( CELL.Ch[3]  - CELL.Ch[2]);
  CELL.Value[4]  = (int16_t) ( CELL.Ch[4]  - CELL.Ch[3]);
  CELL.Value[5]  = (int16_t) ( CELL.Ch[5]  - CELL.Ch[4]);
  CELL.Value[6]  = (int16_t) ( CELL.Ch[6]  - CELL.Ch[5]);
  CELL.Value[7]  = (int16_t) ( CELL.Ch[7]  - CELL.Ch[6]);
  CELL.Value[8]  = (int16_t) ( CELL.Ch[8]  - CELL.Ch[7]);
  CELL.Value[9]  = (int16_t) ( CELL.Ch[9]  - CELL.Ch[8]);
  CELL.Value[10] = (int16_t) ( CELL.Ch[10] - CELL.Ch[9]);
  CELL.Value[11] = (int16_t) ( CELL.Ch[11] - CELL.Ch[10]);
  CELL.Value[12] = (int16_t) ( CELL.Ch[12] - CELL.Ch[11]);
  CELL.Value[13] = (int16_t) ( CELL.Ch[13] - CELL.Ch[12]);
  CELL.Value[14] = (int16_t) ( CELL.Ch[14] - CELL.Ch[13]);
  CELL.Value[15] = (int16_t) ( CELL.Ch[15] - CELL.Ch[14]);
  CELL.Value[16] = (int16_t) ( CELL.Ch[16] - CELL.Ch[15]);
  CELL.Value[17] = (int16_t) ( CELL.Ch[17] - CELL.Ch[16]);
  CELL.Value[18] = (int16_t) ( CELL.Ch[18] - CELL.Ch[17]);
  CELL.Value[19] = (int16_t) ( CELL.Ch[19] - CELL.Ch[18]);
  CELL.Value[20] = (int16_t) ( CELL.Ch[20] - CELL.Ch[19]);
  CELL.Value[21] = (int16_t) ( CELL.Ch[21] - CELL.Ch[20]);
  CELL.Value[22] = (int16_t) ( CELL.Ch[22] - CELL.Ch[21]);
  CELL.Value[23] = (int16_t) ( CELL.Ch[23] - CELL.Ch[22]);
}

/**
  * @brief  Checking cells voltage, if cells voltage less threshold of channel
  *         If this true, will be warning
  * @param  none
  * @retval none
  */
void Checking(uint8_t ch) {  
  if ( ( CELL.Value[ch]) < THRESHOLD[ch] ) { 
    RELAY_FLIP;
    WARNING[ch][0] = 1;
  }
  else {
    WARNING[ch][0] = 0;
  }
}

/**
  * @brief  Turn on/off led if will be warning
  * @param  uint8_t ch (GPIO would be chosen)
  * @retval none
  */
void Light(uint8_t ch) {
  if (WARNING[ch][0])
    Set_IO(ch);
  else
    Reset_IO(ch);
}

/**
  * @brief  Indicator when every sec had gone
  * @param  RTC_t * CurrentTime 
  * @param  RTC_t * OldTime 
  * @param  bool    choose        (allow / denied function)
  * @retval none
  */
bool Tick (RTC_t * CurrentTime, RTC_t * OldTime, bool choose) {
  char heading[201];
  //uint8_t ch;
  rtc_gettime(CurrentTime);
  
  if (OldTime->mday != CurrentTime->mday) {
    if(f_open(&fdst, "log.txt", FA_OPEN_EXISTING | FA_WRITE)==FR_OK ) {
      bw=1;
      sprintf(heading,"\r\n\r\n         TIME\\CELL          NO.01  NO.02  NO.03  NO.04  NO.05  NO.06  NO.07  NO.08  NO.09  NO.10  NO.11  NO.12  NO.13  NO.14  NO.15  NO.16  NO.17  NO.18  NO.19  NO.20  NO.21  NO.22  NO.23  NO.24\r\n");
    
      res = f_lseek(&fdst, fdst.fsize);
      res = f_write(&fdst, heading, sizeof(heading), &bw);
      res = f_close(&fdst);
    }
  }
  
  if (choose) {    
    if ((CurrentTime->sec) != (OldTime->sec)) {
      OldTime->hour  = CurrentTime->hour  ;
      OldTime->min   = CurrentTime->min   ;
      OldTime->sec   = CurrentTime->sec   ;
      OldTime->year  = CurrentTime->year  ;
      OldTime->month = CurrentTime->month ;
      OldTime->mday  = CurrentTime->mday  ;
      return true;
    }
    return false;
  }
  return false;
}

/* Back up */
//bool MinuteInterval (RTC_t * CurrentTime, RTC_t * OldTime, bool choose) {
//  char heading[201];
//  //uint8_t ch;
//  rtc_gettime(CurrentTime);
//  
//  if (OldTime->mday != CurrentTime->mday) {
//    if(f_open(&fdst, "log.txt", FA_OPEN_EXISTING | FA_WRITE)==FR_OK ) {
//      bw=1;
//      sprintf(heading,"\r\n\r\n         TIME\\CELL          NO.01  NO.02  NO.03  NO.04  NO.05  NO.06  NO.07  NO.08  NO.09  NO.10  NO.11  NO.12  NO.13  NO.14  NO.15  NO.16  NO.17  NO.18  NO.19  NO.20  NO.21  NO.22  NO.23  NO.24\r\n");
//    
//      res = f_lseek(&fdst, fdst.fsize);
//      res = f_write(&fdst, heading, sizeof(heading), &bw);
//      res = f_close(&fdst);
//    }
//  }
//  
//  if (choose) {    
//    if (((CurrentTime->min) - (OldTime->min)) >= 2 ) {
//      OldTime->hour  = CurrentTime->hour  ;
//      OldTime->min   = CurrentTime->min   ;
//      OldTime->sec   = CurrentTime->sec   ;
//      OldTime->year  = CurrentTime->year  ;
//      OldTime->month = CurrentTime->month ;
//      OldTime->mday  = CurrentTime->mday  ;
//      return true;
//    }
//    else return false;
//  }
//  else return true;
//}

/**
  * @brief  scan file in sd card
  * @param  char* path           ( path of directory )
  * @retval none
  */
static
FRESULT scan_files (char* path)
{
  DWORD acc_size;				/* Work register for fs command */
  WORD acc_files, acc_dirs;
  FILINFO finfo;
  DIR dirs;
  FRESULT res;
  BYTE i;


	if ((res = f_opendir(&dirs, path)) == FR_OK) {
		i = strlen(path);
		while (((res = f_readdir(&dirs, &finfo)) == FR_OK) && finfo.fname[0]) {
			if (finfo.fattrib & AM_DIR) {
				acc_dirs++;
				*(path+i) = '/'; strcpy(path+i+1, &finfo.fname[0]);
				res = scan_files(path);
				*(path+i) = '\0';
				if (res != FR_OK) break;
			} else {
				acc_files++;
				acc_size += finfo.fsize;
			}
		}
	}

	return res;
}

/**
  * @brief  created new file
  * @param  none
  * @retval none
  */
void CreateFile(void) {
  FRESULT res;
  FILINFO finfo;
  DIR dirs;
  
  RTC_t RTC_Time;
  char time[36];
  char heading[30];
  char banner[200];
    
  disk_initialize(0);  
  
  f_mount(0, &fs);
  
  NVIC_DisableIRQ( DMA1_Channel1_IRQn );
  NVIC_DisableIRQ( DMA2_Channel4_5_IRQn );
  NVIC_DisableIRQ( ADC1_2_IRQn );
  NVIC_DisableIRQ( ADC1_2_IRQn );
  NVIC_DisableIRQ( EXTI1_IRQn );
 
  if(f_open(&fdst, "log.txt", FA_CREATE_NEW | FA_WRITE)==FR_OK ) {
    bw=1;
    rtc_gettime(&RTC_Time);
    
    sprintf(heading,"BATTERY MONITORING LOG FILE \r\n");
    
    //res = f_lseek(&fdst, fdst.fsize);
    res = f_write(&fdst, heading, sizeof(heading), &bw);
    res = f_sync(&fdst);
        
    sprintf(heading,"MADE BY:                    \r\n");
    
    res = f_lseek(&fdst, fdst.fsize);
    res = f_write(&fdst, heading, sizeof(heading), &bw);
    res = f_sync(&fdst);
    
    sprintf(time,"CREATED TIME: %4d-%2d-%2d %0.2d:%0.2d:%0.2d \n\r",RTC_Time.year,RTC_Time.month,RTC_Time.mday,
                                                                    RTC_Time.hour,RTC_Time.min,RTC_Time.sec);
    
    res = f_lseek(&fdst, fdst.fsize); //fdst.fsize
    res = f_write(&fdst, time, sizeof(time), &bw); 
    res = f_sync(&fdst);    
    
    sprintf(banner,"\r\n         TIME\\CELL         NO.01  NO.02  NO.03  NO.04  NO.05  NO.06  NO.07  NO.08  NO.09  NO.10  NO.11  NO.12  NO.13  NO.14  NO.15  NO.16  NO.17  NO.18  NO.19  NO.20  NO.21  NO.22  NO.23  NO.24\r\n");
    res = f_lseek(&fdst, fdst.fsize);
    res = f_write(&fdst, banner, sizeof(banner), &bw);
    res = f_sync(&fdst); 
    res = f_close(&fdst); 
    
    printf("\n\rFile created !\n\r");
  }
  else printf("\n\rFile existed !\n\r"); 
    
  NVIC_EnableIRQ( DMA1_Channel1_IRQn );
  NVIC_EnableIRQ( DMA2_Channel4_5_IRQn );
  NVIC_EnableIRQ( ADC1_2_IRQn );
  NVIC_EnableIRQ( ADC1_2_IRQn );
  NVIC_EnableIRQ( EXTI1_IRQn );
}

/**
  * @brief  write into log file
  * @param  none
  * @retval none
  */
void WriteFile(void)
{ 
  FRESULT res;
  FILINFO finfo;
  DIR dirs;
  
  RTC_t RTC_Time;
  uint8_t ch;
  char time[27];
  char string[7];
  
  disk_initialize(0);
    
  f_mount(0, &fs);
  
  NVIC_DisableIRQ( DMA1_Channel1_IRQn );
  NVIC_DisableIRQ( DMA2_Channel4_5_IRQn );
  NVIC_DisableIRQ( ADC1_2_IRQn );
  NVIC_DisableIRQ( ADC1_2_IRQn );
  NVIC_DisableIRQ( EXTI1_IRQn );
      
  if(f_open(&fdst, "log.txt", FA_OPEN_ALWAYS | FA_WRITE)==FR_OK ) {
    bw=1;
    
    /* Time - Voltage */
    rtc_gettime(&RTC_Time);
    
    sprintf(time,"\r\nTime: %4d-%2d-%2d %0.2d:%0.2d:%0.2d ",RTC_Time.year,RTC_Time.month,RTC_Time.mday,
                                                            RTC_Time.hour,RTC_Time.min,RTC_Time.sec);
    
    res = f_lseek(&fdst, fdst.fsize); //fdst.fsize
    res = f_write(&fdst, time, sizeof(time), &bw); 
    res = f_sync(&fdst); 
      
    //f_lseek(&fdst, fdst.fsize);
    for(ch = 0; ch < 24; ch++) { 
      res = f_lseek(&fdst, fdst.fsize); //fdst.fsize 
      sprintf(string,"  %0.2f", (float) (CELL.Value[ch] / 100.00));  
      res = f_write(&fdst, string, sizeof(string), &bw); 
      res = f_sync(&fdst);
    }
    
    /* Warning State */
    sprintf(time,"\r\nWARNING STATE:           ");
    
    res = f_lseek(&fdst, fdst.fsize); //fdst.fsize
    res = f_write(&fdst, time, sizeof(time), &bw); 
    res = f_sync(&fdst);
    
    for(ch = 0; ch < 24; ch++) {      
      res = f_lseek(&fdst, fdst.fsize); //fdst.fsize
      if (WARNING[ch][0])
        sprintf(string,"  WARN!");
      else
        sprintf(string,"       ");  
      res = f_write(&fdst, string, sizeof(string), &bw); 
      res = f_sync(&fdst);
    }
    f_close(&fdst);
  }
  NVIC_EnableIRQ( DMA1_Channel1_IRQn );
  NVIC_EnableIRQ( DMA2_Channel4_5_IRQn );
  NVIC_EnableIRQ( ADC1_2_IRQn );
  NVIC_EnableIRQ( ADC1_2_IRQn );
  NVIC_EnableIRQ( EXTI1_IRQn );
}

/* Dont be used */
//void ReadFile(void)
//{ 
//  unsigned int a;
//  FRESULT res;
//  FILINFO finfo;
//  DIR dirs;
//  //int i;
//  //char *fn;
//  
//  char path[50]={""};  
//  //char name[]={"WVO.TXT"};
//  
//  disk_initialize(0);
//    
//  f_mount(0, &fs);
//  
//  //printf("\n\rPlease fill name's file you want to read(max. 13 character): ");
//  //fstring(finfo.fname);
//  strcpy(finfo.fname,"log.txt");
//  
//  if (f_opendir(&dirs, path) == FR_OK) 
//  {
//    while (f_readdir(&dirs, &finfo) == FR_OK)  
//    {
//      if (finfo.fattrib & AM_ARC) 
//      {
//        if(!finfo.fname[0])	
//          break;         
//        printf("\n\r file name is:\r\n\r   %s\r\n",finfo.fname);
//        res = f_open(&fdst, finfo.fname, FA_OPEN_EXISTING | FA_READ);
//		    bw=1;
//	    	a=0;
//		    for (;;) {
//			  for(a=0; a<512; a++) buffer[a]=0; 
//    	    res = f_read(&fdst, buffer, sizeof(buffer), &br);
//			  printf("%s\r\n\r",buffer);	
//			    //printp("\r\n\r@@@@@res=%2d  br=%6d  bw=%6d",res,br,bw);
//    	    if (res || br == 0) break;   // error or eof
//       	  //if (res || bw < br) break;   // error or disk full	
//        }
//		  f_close(&fdst);                     
//      }
//    } 
//  }

//  //while(1);
//}

/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{
  static uint16_t tick = INTERVAL_TIME;
  static RTC_t CurrentTime, OldTime;
//char time[16];
  uint8_t ch = 0;
  uint8_t initTime;
  bool enable_checkinterval = false;
  
#ifdef DEBUG
  debug();
#endif
  
  // Clock Config: HSE 72 MHz
  RCC_Configuration();
  SysTick_Config(SystemFrequency_SysClk/1000);
  
  // Interrupt Config
  NVIC_Configuration();
  
  // USART Config : 115200,8,n,1
  Serial_Init();
  
  if (BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5)
	{
		/* Backup data register value is not correct or not yet programmed (when
		the first time the program is executed) */

		printf("\r\n\n RTC not yet configured....");

		/* RTC Configuration */
		RTC_Configuration();

		printf("\r\n RTC configured....");

		/* Adjust time by values entred by the user on the hyperterminal */
		//RTC_WaitForLastTask();
    
    Time_Regulate(&CurrentTime); 
    
    rtc_settime(&CurrentTime);
    /* Wait until last write operation on RTC registers has finished */
    //RTC_WaitForLastTask();
    
		BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);
	}
	else
	{
		/* Check if the Power On Reset flag is set */
		if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
		{
			printf("\r\n\n Power On Reset occurred....");
		}
		/* Check if the Pin Reset flag is set */
		else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
		{
			printf("\r\n\n External Reset occurred....");
		}

		printf("\r\n No need to configure RTC....");
		/* Wait for RTC registers synchronization */
		RTC_WaitForSynchro();

		/* Enable the RTC Second */
		RTC_ITConfig(RTC_IT_SEC, ENABLE);
		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();
    
    printf("\r\n");
	}
  
  /* Initialize GPIO & ADC /w DMA */
  GPIOInit();
  ADCInit();
  DMAInit();
  
  /////////////////////////////////////////////////////////////////////
  //////// SDCARD Initialisation //////////////////////////////////////
  /////////////////Section adapted from ST example/////////////////////
  
  /*-------------------------- SD Init ----------------------------- */
  Status = SD_Init();

  if (Status == SD_OK)
  {
    /*----------------- Read CSD/CID MSD registers ------------------*/
    Status = SD_GetCardInfo(&SDCardInfo);
  }
  
  if (Status == SD_OK)
  {
    /*----------------- Select Card --------------------------------*/
    Status = SD_SelectDeselect((u32) (SDCardInfo.RCA << 16));
  }
  
  if (Status == SD_OK)
  {
    //Status = SD_EnableWideBusOperation(SDIO_BusWide_4b);
  }
  
  /* Set Device Transfer Mode to DMA */
  if (Status == SD_OK)
  {  
  //Status = SD_SetDeviceMode(SD_DMA_MODE);//oet
  //Status = SD_SetDeviceMode(SD_POLLING_MODE);
    Status = SD_SetDeviceMode(SD_INTERRUPT_MODE);
	  printf("\r\n\rTEST OK!\r\n\r");
  }

	/* Clear reset flags */
	RCC_ClearFlag();
    
  /* Create log file */
  CreateFile(); 
  
  //Enable DMA1 Channel transfer
  DMA_Cmd(DMA1_Channel1, ENABLE);
    
  //Enable DMA2 Channel transfer
  DMA_Cmd(DMA2_Channel5, ENABLE);
  
  /* Initialize values */
  rtc_gettime(&CurrentTime);
  rtc_gettime(&OldTime);
  
  printf("Initialize, Please wait: ");
  Delay(999);
  
  for (initTime=10; initTime>0; initTime--) {
    printf("%d.. ",initTime);
    Remapping();
    Converting();
    ValueCell();
    Delay(999);
  }

  printf("\r\n");
  printf("%02d-%02d-%02d\n\r",CurrentTime.year,CurrentTime.month,CurrentTime.mday);
  printf("\r\n         TIME\\CELL          NO.01  NO.02  NO.03  NO.04  NO.05  NO.06  NO.07  NO.08  NO.09  NO.10  NO.11  NO.12  NO.13  NO.14  NO.15  NO.16  NO.17  NO.18  NO.19  NO.20  NO.21  NO.22  NO.23  NO.24\r\n");
    
  while (1)
  {
    if (Tick( &CurrentTime , &OldTime , enable_checkinterval)) {
      tick++;
    }
    if ((tick == 30)||(tick == 60)||(tick == 90)||(tick ==120)||(tick ==150)||
        (tick ==180)||(tick ==210)||(tick ==240)||(tick ==270)||(tick ==INTERVAL_TIME)) {
          
      //printf(".");
      Remapping();
      Converting();
      ValueCell();
      for ( ch = 0 ; ch<24 ; ch++ ) {
        Checking(ch);      
        Light(ch);
      }
    }
    if (tick == INTERVAL_TIME) {
#ifdef RELEASE
      WriteFile();
      //printf(":");
#endif    
#ifdef TEST 
      //sTime_Display(RTC_GetCounter(), time);
      //rtc_gettime(&CurrentTime);
      printf("\r\nTime: %4d-%2d-%2d %0.2d:%0.2d:%0.2d ",CurrentTime.year,CurrentTime.month,CurrentTime.mday,
                                                        CurrentTime.hour,CurrentTime.min,CurrentTime.sec);
      
      for(ch = 0; ch < 24; ch++) {
        //printf("  %5d", ( int16_t) CELL.Value[ch]);
        printf("  %5d", (uint16_t) CELL.Ch[ch]);
      }
      
      printf("\r\nTime: %4d-%2d-%2d %0.2d:%0.2d:%0.2d ",CurrentTime.year,CurrentTime.month,CurrentTime.mday,
                                                        CurrentTime.hour,CurrentTime.min,CurrentTime.sec);
      for(ch = 0; ch < 24; ch++) {
        if (WARNING[ch][0])
          printf("  WARN!");
        else
          printf("       "); 
      }
#endif
      tick = 0;
    }
    enable_checkinterval = true;
    
#ifndef TESTLED
    SET_LED01; SET_LED02; SET_LED03; SET_LED04; SET_LED05; SET_LED06; SET_LED07; SET_LED08;
    SET_LED09; SET_LED10; SET_LED11; SET_LED12; SET_LED13; SET_LED14; SET_LED15; SET_LED16;
    SET_LED17; SET_LED18; SET_LED19; SET_LED20; SET_LED21; SET_LED22; SET_LED23; SET_LED24;
    Delay(10000);
    
    RESET_LED01; RESET_LED02; RESET_LED03; RESET_LED04; RESET_LED05; RESET_LED06; RESET_LED07; RESET_LED08;
    RESET_LED09; RESET_LED10; RESET_LED11; RESET_LED12; RESET_LED13; RESET_LED14; RESET_LED15; RESET_LED16;
    RESET_LED17; RESET_LED18; RESET_LED19; RESET_LED20; RESET_LED21; RESET_LED22; RESET_LED23; RESET_LED24;
    Delay(10000);
#endif    
  }
}

/*************************************************************************
 * Function Name: Serial_Init
 * Description: Init USARTs
 *************************************************************************/
void Serial_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
  RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, DISABLE);

  /* Configure USART1 Tx (PA.09) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART1 Rx (PA.10) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
/* USART1 configuration ------------------------------------------------------*/
  // USART1 configured as follow:
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Configure the USART1 */
  USART_Init(USART1, &USART_InitStructure);

  /* Enable the USART Transmoit interrupt: this interrupt is generated when the 
     USART1 transmit data register is empty */  
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

  /* Enable the USART Receive interrupt: this interrupt is generated when the 
     USART1 receive data register is not empty */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  /* Enable USART1 */
  USART_Cmd(USART1, ENABLE);
}



/*******************************************************************************
* Function Name  : NVIC_Config
* Description    : Configures SDIO IRQ channel.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

	/* Enable the RTC Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the DMA Interrupt */  
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel4_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/* Implementation of putchar (also used by printf function to output data)    */
int SendChar (int ch)  {                /* Write character to Serial Port     */

  USART_SendData(USART1, (unsigned char) ch);
  while (!(USART1->SR & USART_FLAG_TXE));
  return (ch);
}


int GetKey (void)  {                    /* Read character from Serial Port    */

  while (!(USART1->SR & USART_FLAG_RXNE));
  return (USART_ReceiveData(USART1));
}

void fstring (char *str) {
  uint16_t i;
  for (i = 0; i<sizeof(str) ; i++)
  {
    while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);      
    str[i] = USART_ReceiveData(USART1);
  }
}

/*******************************************************************************
* Function Name  : fputc
* Description    : Retargets the C library printf function to the USART.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int fputc(int ch, FILE *f)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (u8) ch);

  /* Loop until the end of transmission */
  while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {
  }

  return ch;
}


/*******************************************************************************
* Function Name  : USART_Scanf
* Description    : Gets numeric values from the hyperterminal.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 USART_Scanf(u32 value)
{
  u32 index = 0;
  u32 tmp[2] = {0, 0};

  while (index < 2)
  {
    /* Loop until RXNE = 1 */
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
    {}
    tmp[index++] = (USART_ReceiveData(USART1));
    if ((tmp[index - 1] < 0x30) || (tmp[index - 1] > 0x39))
    {
      printf("\n\r\rPlease enter valid number between 0 and 9");
      index--;
    }
  }
  /* Calculate the Corresponding value */
  index = (tmp[1] - 0x30) + ((tmp[0] - 0x30) * 10);
  /* Checks */
  if (index > value)
  {
    printf("\n\r\rPlease enter valid number between 0 and %d", value);
    return 0xFF;
  }
  return index;
}


#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n\r", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n\r", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
