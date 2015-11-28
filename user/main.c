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
uint32_t RESISTANCE[24][3]    =    { {   22  ,  1000  ,    100   },  /* CH01 */
                                     {   47  ,   100  ,    100   },  /* CH02 */
                                     {  100  ,   100  ,    100   },  /* CH03 */
                                     {  180  ,   100  ,    100   },  /* CH04 */
                                     {  270  ,   100  ,    100   },  /* CH05 */
                                     {  330  ,   100  ,    100   },  /* CH06 */
                                     {  390  ,   100  ,    100   },  /* CH07 */
                                     {  470  ,   100  ,    100   },  /* CH08 */
                                     {  820  ,   150  ,    100   },  /* CH09 */
                                     {  560  ,   100  ,    100   },  /* CH10 */
                                     {  680  ,   100  ,    100   },  /* CH11 */
                                     {  100  ,   120  ,    100   },  /* CH12 */
                                     {  820  ,   100  ,    100   },  /* CH13 */
                                     {  560  ,    68  ,    100   },  /* CH14 */
                                     {  470  ,    47  ,    100   },  /* CH15 */
                                     { 1000  ,   100  ,    100   },  /* CH16 */
                                     {  470  ,    47  ,    100   },  /* CH17 */
                                     { 1200  ,   100  ,    100   },  /* CH18 */
                                     { 1200  ,   100  ,    100   },  /* CH19 */
                                     {  910  ,    51  ,    100   },  /* CH20 */
                                     {  680  ,    47  ,    100   },  /* CH21 */
                                     {  470  ,    33  ,    100   },  /* CH22 */
                                     {  820  ,    51  ,    100   },  /* CH23 */
                                     {  820  ,    47  ,    100   }   /* CH24 */ };

uint16_t THRESHOLD[24] = { 110 , 110 , 110 , 110 , 110 , 110 , 110 , 110 , 110 , 110 , 110 , 110 ,
                           110 , 110 , 110 , 110 , 110 , 110 , 110 , 110 , 110 , 110 , 110 , 110 };
                                     
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

bool Check10min (RTC_t * CurrentTime, RTC_t * OldTime, bool choose) {
  char heading[201];
  rtc_gettime(CurrentTime);
  if (choose) {
    if (((CurrentTime->min) - (OldTime->min)) >= 2 ) {
      if (OldTime->mday != CurrentTime->mday) {
        if(f_open(&fdst, "log.txt", FA_OPEN_EXISTING | FA_WRITE)==FR_OK ) {
          bw=1;
          sprintf(heading,"\n\r\n\r         TIME\\CELL          NO.01  NO.02  NO.03  NO.04  NO.05  NO.06  NO.07  NO.08  NO.09  NO.10  NO.11  NO.12  NO.13  NO.14  NO.15  NO.16  NO.17  NO.18  NO.19  NO.20  NO.21  NO.22  NO.23  NO.24\r\n");
    
          res = f_lseek(&fdst, fdst.fsize);
          res = f_write(&fdst, heading, sizeof(heading), &bw);
          res = f_close(&fdst);
        }       
      }
      OldTime->hour  = CurrentTime->hour  ;
      OldTime->min   = CurrentTime->min   ;
      OldTime->sec   = CurrentTime->sec   ;
      OldTime->year  = CurrentTime->year  ;
      OldTime->month = CurrentTime->month ;
      OldTime->mday  = CurrentTime->mday  ;
      return true;
    }
    else return false;
  }
  else return true;
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
    CELL.Ch[ch] = (CELL.Ch[ch] * ADC_VREF * (RESISTANCE[ch][0] + RESISTANCE[ch][1])) / (ADC_12BIT_FACTOR * RESISTANCE[ch][1]);
    CELL.Ch[ch] =  (uint16_t)(CELL.Ch[ch] *  RESISTANCE[ch][2] / 100);
  }
}

void ValueCell(void) {
  uint8_t ch;
  CELL.Value[0] = CELL.Ch[0];
  for ( ch = 1; ch < 24; ch++ ) {
    CELL.Value[ch] = CELL.Ch[ch] - CELL.Ch[ch-1];
  }
}

void Checking(uint8_t ch) {
  switch (ch) {
    case 0:
    {
      if (CELL.Value[ch] <= THRESHOLD[ch]) {
        RESET_LED01; 
        RELAY_FLIP;
      }
      else SET_LED01;
      break;
    }
    case 1:
    {
      if (CELL.Value[ch] <= THRESHOLD[ch]) {
        RESET_LED02; 
        RELAY_FLIP;
      }
      else SET_LED02;
      break;
    }
    case 2:
    {
      if (CELL.Value[ch] <= THRESHOLD[ch]) {
        RESET_LED03; 
        RELAY_FLIP;
      }
      else SET_LED03;
      break;
    }
    case 3:
    {
      if (CELL.Value[ch] <= THRESHOLD[ch]) {
        RESET_LED04; 
        RELAY_FLIP;
      }
      else SET_LED04;
      break;
    }
    case 4:
    {
      if (CELL.Value[ch] <= THRESHOLD[ch]) {
        RESET_LED05; 
        RELAY_FLIP;
      }
      else SET_LED05;
      break;
    }
    case 5:
    {
      if (CELL.Value[ch] <= THRESHOLD[ch]) {
        RESET_LED06; 
        RELAY_FLIP;
      }
      else SET_LED06;
      break;
    }
    case 6:
    {
      if (CELL.Value[ch] <= THRESHOLD[ch]) {
        RESET_LED07; 
        RELAY_FLIP;
      }
      else SET_LED07;
      break;
    }
    case 7:
    {
      if (CELL.Value[ch] <= THRESHOLD[ch]) {
        RESET_LED08; 
        RELAY_FLIP;
      }
      else SET_LED08;
      break;
    }
    case 8:
    {
      if (CELL.Value[ch] <= THRESHOLD[ch]) {
        RESET_LED09; 
        RELAY_FLIP;
      }
      else SET_LED09;
      break;
    }
    case 9:
    {
      if (CELL.Value[ch] <= THRESHOLD[ch]) {
        RESET_LED10; 
        RELAY_FLIP;
      }
      else SET_LED10;
      break;
    }
    case 10:
    {
      if (CELL.Value[ch] <= THRESHOLD[ch]) {
        RESET_LED11; 
        RELAY_FLIP;
      }
      else SET_LED11;
      break;
    }
    case 11:
    {
      if (CELL.Value[ch] <= THRESHOLD[ch]) {
        RESET_LED12; 
        RELAY_FLIP;
      }
      else SET_LED12;
      break;
    }
    case 12:
    {
      if (CELL.Value[ch] <= THRESHOLD[ch]) {
        RESET_LED13; 
        RELAY_FLIP;
      }
      else SET_LED13;
      break;
    }
    case 13:
    {
      if (CELL.Value[ch] <= THRESHOLD[ch]) {
        RESET_LED14; 
        RELAY_FLIP;
      }
      else SET_LED14;
      break;
    }
    case 14:
    {
      if (CELL.Value[ch] <= THRESHOLD[ch]) {
        RESET_LED15; 
        RELAY_FLIP;
      }
      else SET_LED15;
      break;
    }
    case 15:
    {
      if (CELL.Value[ch] <= THRESHOLD[ch]) {
        RESET_LED16; 
        RELAY_FLIP;
      }
      else SET_LED16;
      break;
    }
    case 16:
    {
      if (CELL.Value[ch] <= THRESHOLD[ch]) {
        RESET_LED17; 
        RELAY_FLIP;
      }
      else SET_LED17;
      break;
    }
    case 17:
    {
      if (CELL.Value[ch] <= THRESHOLD[ch]) {
        RESET_LED18; 
        RELAY_FLIP;
      }
      else SET_LED18;
      break;
    }
    case 18:
    {
      if (CELL.Value[ch] <= THRESHOLD[ch]) {
        RESET_LED19; 
        RELAY_FLIP;
      }
      else SET_LED19;
      break;
    }
    case 19:
    {
      if (CELL.Value[ch] <= THRESHOLD[ch]) {
        RESET_LED20; 
        RELAY_FLIP;
      }
      else SET_LED20;
      break;
    }
    case 20:
    {
      if (CELL.Value[ch] <= THRESHOLD[ch]) {
        RESET_LED21; 
        RELAY_FLIP;
      }
      else SET_LED21;
      break;
    }
    case 21:
    {
      if (CELL.Value[ch] <= THRESHOLD[ch]) {
        RESET_LED22; 
        RELAY_FLIP;
      }
      else SET_LED22;
      break;
    }
    case 22:
    {
      if (CELL.Value[ch] <= THRESHOLD[ch]) {
        RESET_LED23; 
        RELAY_FLIP;
      }
      else SET_LED23;
      break;
    }
    case 23:
    {
      if (CELL.Value[ch] <= THRESHOLD[ch]) {
        RESET_LED24; 
        RELAY_FLIP;
      }
      else SET_LED24;
      break;
    }
    default: break;                          
  } 
}

void CreateFile() {
  FRESULT res;
  FILINFO finfo;
  DIR dirs;
  RTC_t ModTime;
  
  char heading[199];
    
  disk_initialize(0);  
  
  f_mount(0, &fs);
  
  NVIC_DisableIRQ( DMA1_Channel1_IRQn );
  NVIC_DisableIRQ( DMA2_Channel4_5_IRQn );
  NVIC_DisableIRQ( ADC1_2_IRQn );
  NVIC_DisableIRQ( ADC1_2_IRQn );
 
  rtc_gettime(&ModTime);
  
  if(f_open(&fdst, "log.txt", FA_CREATE_NEW | FA_WRITE)==FR_OK ) {
    bw=1;
    sprintf(heading,"\n\r         TIME\\CELL          NO.01  NO.02  NO.03  NO.04  NO.05  NO.06  NO.07  NO.08  NO.09  NO.10  NO.11  NO.12  NO.13  NO.14  NO.15  NO.16  NO.17  NO.18  NO.19  NO.20  NO.21  NO.22  NO.23  NO.24\r\n");
    
    //res = f_lseek(&fdst, fdst.fsize);
    res = f_write(&fdst, heading, sizeof(heading), &bw);
    finfo.fdate = (ModTime.year<<9)+((ModTime.month&0x000F)<<5)+((ModTime.month&0x001F));
    finfo.fdate = ((ModTime.hour&0x001F)<<11)+((ModTime.min&0x003F)<<5)+((ModTime.sec&0x003E)<<1);
    res = f_close(&fdst);
    printf("\n\rFile created !\n\r");
  }
  else printf("\n\rFile existed !\n\r"); 
    
  NVIC_EnableIRQ( DMA1_Channel1_IRQn );
  NVIC_EnableIRQ( DMA2_Channel4_5_IRQn );
  NVIC_EnableIRQ( ADC1_2_IRQn );
  NVIC_EnableIRQ( ADC1_2_IRQn );
}

void WriteFile(void)
{ 
  FRESULT res;
  FILINFO finfo;
  DIR dirs;
  
  RTC_t RTC_Time,ModTime;
  uint8_t ch;
  char time[27];
  char string[7];
  
  disk_initialize(0);
    
  f_mount(0, &fs);
  
  NVIC_DisableIRQ( DMA1_Channel1_IRQn );
  NVIC_DisableIRQ( DMA2_Channel4_5_IRQn );
  NVIC_DisableIRQ( ADC1_2_IRQn );
  NVIC_DisableIRQ( ADC1_2_IRQn );
      
  if(f_open(&fdst, "log.txt", FA_OPEN_EXISTING | FA_WRITE)==FR_OK ) {
    bw=1;
     
    rtc_gettime(&RTC_Time);
    sprintf(time,"\r\nTime: %4d-%2d-%2d %0.2d:%0.2d:%0.2d ",RTC_Time.year,RTC_Time.month,RTC_Time.mday,
                                                            RTC_Time.hour,RTC_Time.min,RTC_Time.sec);
    
    res = f_lseek(&fdst, fdst.fsize); //fdst.fsize
    res = f_write(&fdst, time, sizeof(time), &bw); 
    res = f_sync(&fdst); 
      
    //f_lseek(&fdst, fdst.fsize);
    for(ch = 0; ch < 24; ch++){ 
      res = f_lseek(&fdst, fdst.fsize); //fdst.fsize
      sprintf(string,"  %5d", CELL.Ch[ch]);  
      res = f_write(&fdst, string, sizeof(string), &bw); 
      res = f_sync(&fdst); 
    }
    rtc_gettime(&ModTime);
    finfo.fdate = (ModTime.year<<9)+((ModTime.month&0x000F)<<5)+((ModTime.month&0x001F));
    finfo.fdate = ((ModTime.hour&0x001F)<<11)+((ModTime.min&0x003F)<<5)+((ModTime.sec&0x003E)<<1);
    f_close(&fdst);
  }
  NVIC_EnableIRQ( DMA1_Channel1_IRQn );
  NVIC_EnableIRQ( DMA2_Channel4_5_IRQn );
  NVIC_EnableIRQ( ADC1_2_IRQn );
  NVIC_EnableIRQ( ADC1_2_IRQn );
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
  RTC_t CurrentTime, OldTime;
//char time[16];
  uint8_t ch = 0;
  bool enable_check10min = false;
  
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
    
    rtc_settime(&CurrentTime);                // set thoi gian ban dau 
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
    
  rtc_gettime(&CurrentTime);
  rtc_gettime(&OldTime);
  
  printf("%02d-%02d-%02d\n\r",CurrentTime.year,CurrentTime.month,CurrentTime.mday);
  printf("\n\r         TIME\\CELL          NO.01  NO.02  NO.03  NO.04  NO.05  NO.06  NO.07  NO.08  NO.09  NO.10  NO.11  NO.12  NO.13  NO.14  NO.15  NO.16  NO.17  NO.18  NO.19  NO.20  NO.21  NO.22  NO.23  NO.24\r\n");
  
  while (1)
  {
    Remapping();
    Converting();
    ValueCell();
    for (ch = 0 ; ch < 24 ; ch ++) {  Checking(ch); }
    //if (Check10min( &CurrentTime , &OldTime , enable_check10min)) {
#ifndef RELEASE
      WriteFile();
      printf(".");
#endif    
#ifdef TEST 
      //sTime_Display(RTC_GetCounter(), time);
      //rtc_gettime(&CurrentTime);
      printf("\r\nTime: %4d-%2d-%2d %0.2d:%0.2d:%0.2d ",CurrentTime.year,CurrentTime.month,CurrentTime.mday,
                                                        CurrentTime.hour,CurrentTime.min,CurrentTime.sec);
      
      for(ch = 0; ch < 24; ch++) {
      //printf("  %5d", (uint16_t) CELL.Value[ch]);
        printf("  %5d", (uint16_t) CELL.Ch[ch]);
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
    //}
    //enable_check10min = true;
    Delay(5000);
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
