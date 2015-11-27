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
extern volatile uint16_t ADC1_values[ADC1_ARRAYSIZE];
extern volatile uint16_t ADC3_values[ADC3_ARRAYSIZE];
extern volatile uint8_t  statusADC1, statusADC3;

/* Private define ------------------------------------------------------------*/
#define BlockSize            512 /* Block Size in Bytes */
//#define BlockSize          128 /* Block Size in Bytes */
#define BufferWordsSize      (BlockSize >> 2)

#define NumberOfBlocks       2  /* For Multi Blocks operation (Read/Write) */
#define MultiBufferWordsSize ((BlockSize * NumberOfBlocks) >> 2)

#define TEST                 1
#define RELEASE              1

/* Array resistance voltage divider:    R1/1 |  R2/10 | scale * 10
 * 
 */
uint32_t RESISTANCE[24][3]    =    { {   22  ,  1000  ,    10   },  /* CH01 */
                                     {   47  ,   100  ,    10   },  /* CH02 */
                                     {  100  ,   100  ,    10   },  /* CH03 */
                                     {  180  ,   100  ,    10   },  /* CH04 */
                                     {  270  ,   100  ,    10   },  /* CH05 */
                                     {  330  ,   100  ,    10   },  /* CH06 */
                                     {  390  ,   100  ,    10   },  /* CH07 */
                                     {  470  ,   100  ,    10   },  /* CH08 */
                                     {  820  ,   150  ,    10   },  /* CH09 */
                                     {  560  ,   100  ,    10   },  /* CH10 */
                                     {  680  ,   100  ,    10   },  /* CH11 */
                                     {  100  ,   120  ,    10   },  /* CH12 */
                                     {  820  ,   100  ,    10   },  /* CH13 */
                                     {  560  ,    68  ,    10   },  /* CH14 */
                                     {  470  ,    47  ,    10   },  /* CH15 */
                                     { 1000  ,   100  ,    10   },  /* CH16 */
                                     {  470  ,    47  ,    10   },  /* CH17 */
                                     { 1200  ,   100  ,    10   },  /* CH18 */
                                     { 1200  ,   100  ,    10   },  /* CH19 */
                                     {  910  ,    51  ,    10   },  /* CH20 */
                                     {  680  ,    47  ,    10   },  /* CH21 */
                                     {  470  ,    33  ,    10   },  /* CH22 */
                                     {  820  ,    51  ,    10   },  /* CH23 */
                                     {  820  ,    47  ,    10   }   /* CH24 */ };

enum CLI{
      Create_file = 0x31,
      Write_file  = 0x32,
      Read_file   = 0x33
    } cmd;

struct _CELL {
      uint16_t Ch[24];  
      uint16_t Value[24];
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

void RCC_Configuration(void)
{
  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,
     initialize the PLL and update the SystemFrequency variable. */
  SystemInit();
}
    
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

void Remapping (void){

/* CD4052 pin pack BA = 00 */
  MUX00;
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  Delay(10);
  ADC_SoftwareStartConvCmd(ADC1, DISABLE);
   
  CELL.Ch[16] = ADC1_values[14];
  
  ADC_SoftwareStartConvCmd(ADC3, ENABLE);
  Delay(10);
  ADC_SoftwareStartConvCmd(ADC3, DISABLE);
  
  CELL.Ch[17] = ADC3_values[2];
  
/* CD4052 pin pack BA = 01 */
  MUX01;
  
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  Delay(10);
  ADC_SoftwareStartConvCmd(ADC1, DISABLE);

  CELL.Ch[18] = ADC1_values[14];
  
  ADC_SoftwareStartConvCmd(ADC3, ENABLE);
  Delay(10);
  ADC_SoftwareStartConvCmd(ADC3, DISABLE);
    
  CELL.Ch[19] = ADC3_values[2];

/* CD4052 pin pack BA = 10 */
  MUX10;
  
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  Delay(10);
  ADC_SoftwareStartConvCmd(ADC1, DISABLE);

  CELL.Ch[20] = ADC1_values[14];
  
  ADC_SoftwareStartConvCmd(ADC3, ENABLE);
  Delay(10);
  ADC_SoftwareStartConvCmd(ADC3, DISABLE);
    
  CELL.Ch[21] = ADC3_values[2];

/* CD4052 pin pack BA = 11 */
  MUX11;
  
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  Delay(10);
  ADC_SoftwareStartConvCmd(ADC1, DISABLE);
  
  CELL.Ch[0]  = ADC1_values[0];
  CELL.Ch[1]  = ADC1_values[1];
  CELL.Ch[2]  = ADC1_values[2];
  CELL.Ch[3]  = ADC1_values[3];
  CELL.Ch[4]  = ADC1_values[4];
  CELL.Ch[5]  = ADC1_values[5];
  CELL.Ch[6]  = ADC1_values[6];
  CELL.Ch[7]  = ADC1_values[7];
  
  CELL.Ch[10] = ADC1_values[8];
  CELL.Ch[11] = ADC1_values[9];
  CELL.Ch[12] = ADC1_values[10];
  CELL.Ch[13] = ADC1_values[11];
  CELL.Ch[14] = ADC1_values[12];
  CELL.Ch[15] = ADC1_values[13];
  
  CELL.Ch[22] = ADC1_values[14];
  
  ADC_SoftwareStartConvCmd(ADC3, ENABLE);
  Delay(10);
  ADC_SoftwareStartConvCmd(ADC3, DISABLE);
  
  CELL.Ch[8]  = ADC3_values[0];
  CELL.Ch[9]  = ADC3_values[1];
  
  CELL.Ch[23] = ADC3_values[2];
}

void Converting(void) {
  uint8_t ch;
  for ( ch = 0; ch < 24 ; ch++ ){
    CELL.Ch[ch] = (CELL.Ch[ch] * ADC_VREF * (RESISTANCE[ch][0]+ RESISTANCE[ch][1])) / (ADC_12BIT_FACTOR * RESISTANCE[ch][1]);
  }
}

void ValueCell(void) {
  uint8_t ch;
  CELL.Value[0] = CELL.Ch[0];
  for ( ch = 1; ch < 24; ch++ ) {
    CELL.Value[ch] = CELL.Ch[ch] - CELL.Ch[ch-1];
  }
}

void CreateFile() {
  FRESULT res;
  FILINFO finfo;
  DIR dirs;
  
  char heading[162];
    
  disk_initialize(0);  
  
  f_mount(0, &fs);
 
  if(f_open(&fdst, "log.txt", FA_CREATE_NEW | FA_WRITE)==FR_OK ) {
    bw=1;
    sprintf(heading,"      TIME       CH01  CH02  CH03  CH04  CH05  CH06  CH07  CH08  CH09  CH10  CH11  CH12  CH13  CH14  CH15  CH16  CH17  CH18  CH19  CH20  CH21  CH22  CH23  CH24\r\n");
    f_write(&fdst, heading, sizeof(heading), &bw);
    f_close(&fdst);
    printf("\n\rFile created !\n\r");
  }
  else printf("\n\rFile existed !\n\r"); 
}

void WriteFile(void)
{ 
  FRESULT res;
  FILINFO finfo;
  DIR dirs;
  
  uint8_t ch;
  char time[17];
  char string[6];
  
  disk_initialize(0);
    
  f_mount(0, &fs);
      
  if(f_open(&fdst, "log.txt", FA_OPEN_EXISTING | FA_WRITE)==FR_OK ) {
    bw=1;
     
    //print averages
    f_lseek(&fdst, fdst.fsize); 
    //sTime_Display(RTC_GetCounter(), time); 
    f_write(&fdst, time, sizeof(time), &bw); 
    f_sync(&fdst); 
      
    //f_lseek(&fdst, fdst.fsize);
    for(ch = 0; ch < 24; ch++){ 
      f_lseek(&fdst, fdst.fsize); 
      sprintf(string,"  %4d", CELL.Ch[ch]);  
      f_write(&fdst, string, sizeof(string), &bw); 
      f_sync(&fdst); 
    }
    f_close(&fdst);
  }
}

void ReadFile(void)
{ 
  unsigned int a;
  FRESULT res;
  FILINFO finfo;
  DIR dirs;
  //int i;
  //char *fn;
  
  char path[50]={""};  
  //char name[]={"WVO.TXT"};
  
  disk_initialize(0);
    
  f_mount(0, &fs);
  
  //printf("\n\rPlease fill name's file you want to read(max. 13 character): ");
  //fstring(finfo.fname);
  strcpy(finfo.fname,"log.txt");
  
  if (f_opendir(&dirs, path) == FR_OK) 
  {
    while (f_readdir(&dirs, &finfo) == FR_OK)  
    {
      if (finfo.fattrib & AM_ARC) 
      {
        if(!finfo.fname[0])	
          break;         
        printf("\n\r file name is:\r\n\r   %s\r\n",finfo.fname);
        res = f_open(&fdst, finfo.fname, FA_OPEN_EXISTING | FA_READ);
		    bw=1;
	    	a=0;
		    for (;;) {
			  for(a=0; a<512; a++) buffer[a]=0; 
    	    res = f_read(&fdst, buffer, sizeof(buffer), &br);
			  printf("%s\r\n\r",buffer);	
			    //printp("\r\n\r@@@@@res=%2d  br=%6d  bw=%6d",res,br,bw);
    	    if (res || br == 0) break;   // error or eof
       	  //if (res || bw < br) break;   // error or disk full	
        }
		  f_close(&fdst);                     
      }
    } 
  }

  //while(1);
}

/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{
  RTC_t RTC_Time;
  char time[16];
  uint8_t ch = 0;
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
//    Status = SD_SetDeviceMode(SD_DMA_MODE);//oet
 //   Status = SD_SetDeviceMode(SD_POLLING_MODE);
    Status = SD_SetDeviceMode(SD_INTERRUPT_MODE);
	  printf("\r\n\rTEST OK!\r\n\r");
  }
  
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
    
    Time_Regulate(&RTC_Time); 
    
    rtc_settime(&RTC_Time);                // set thoi gian ban dau 
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

	/* Clear reset flags */
	RCC_ClearFlag();
  
  CreateFile(); 
  
  //Enable DMA1 Channel transfer
  DMA_Cmd(DMA1_Channel1, ENABLE);
    
  //Enable DMA2 Channel transfer
  DMA_Cmd(DMA2_Channel5, ENABLE);
  
  rtc_gettime(&RTC_Time);
  printf("%02d-%02d-%02d\n\r",RTC_Time.year,RTC_Time.month,RTC_Time.mday);
  printf("      TIME       CH01  CH02  CH03  CH04  CH05  CH06  CH07  CH08  CH09  CH10  CH11  CH12  CH13  CH14  CH15  CH16  CH17  CH18  CH19  CH20  CH21  CH22  CH23  CH24\r\n");
  
  while (1)
  {
    Remapping();
    Converting();
    ValueCell();
    //printf("running \r\n");
#ifndef RELEASE
    WriteFile();
    //printf(".");
#endif    
#ifdef TEST 
    //sTime_Display(RTC_GetCounter(), time);
    rtc_gettime(&RTC_Time);
    printf("\r\nTime: %3.2d:%0.2d:%0.2d",RTC_Time.hour,RTC_Time.min,RTC_Time.sec);
    
    for(ch = 0; ch < 24; ch++) {
      //printf("  %4d", (uint16_t) CELL.Value[ch]);
      printf("  %4d", (uint16_t) CELL.Ch[ch]);
    }
/*  -- to be changed --
    //Start ADC1 Software Conversion
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    //wait for DMA complete
    Delay(10);
    ADC_SoftwareStartConvCmd(ADC1, DISABLE);
    for(ch = 0; ch < 15; ch++) {
      printf("  %4d", (uint16_t) ADC1_values[ch]);
    }
    
    //Start ADC3 Software Conversion
    ADC_SoftwareStartConvCmd(ADC3, ENABLE);
    //wait for DMA complete
    Delay(10);
    ADC_SoftwareStartConvCmd(ADC3, DISABLE);
    for(ch = 0; ch < 3; ch++) {
      printf("  %4d", (uint16_t) ADC3_values[ch]);
    }
*/
#endif    
    Delay(10000);
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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the DMA Interrupt */  
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel4_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
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
