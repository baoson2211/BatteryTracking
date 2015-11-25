/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "sdcard.h"
#include "stdio.h"
#include "adc.h"
#include "rtc.h"
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

static __IO uint8_t index;
__IO uint32_t TimingDelay;

extern __IO uint32_t TimeDisplay;
extern volatile uint16_t ADC_values1[ARRAYSIZE1];
extern volatile uint16_t ADC_values3[ARRAYSIZE3];
extern volatile uint32_t status1, status3;

/* Private define ------------------------------------------------------------*/
#define BlockSize            512 /* Block Size in Bytes */
//#define BlockSize            128 /* Block Size in Bytes */
#define BufferWordsSize      (BlockSize >> 2)

#define NumberOfBlocks       2  /* For Multi Blocks operation (Read/Write) */
#define MultiBufferWordsSize ((BlockSize * NumberOfBlocks) >> 2)

enum CLI{
      Create_file = 0x31,
      Write_file  = 0x32,
      Read_file   = 0x33
    } cmd;

struct _CELL {
      uint16_t Ch[24];  
      uint16_t Tmp;
      uint16_t Voltage;
    } CELL;

/* Private macro -------------------------------------------------------------*/

#define CELL01 { CELL.Ch[0]  = ADC_Read(ADC1, ADC_Channel_8) ; }
#define CELL02 { CELL.Ch[1]  = ADC_Read(ADC1, ADC_Channel_14); }
#define CELL03 { CELL.Ch[2]  = ADC_Read(ADC1, ADC_Channel_6) ; }
#define CELL04 { CELL.Ch[3]  = ADC_Read(ADC1, ADC_Channel_4) ; }
#define CELL05 { CELL.Ch[4]  = ADC_Read(ADC1, ADC_Channel_2) ; }
#define CELL06 { CELL.Ch[5]  = ADC_Read(ADC1, ADC_Channel_0) ; }
#define CELL07 { CELL.Ch[6]  = ADC_Read(ADC1, ADC_Channel_12); }
#define CELL08 { CELL.Ch[7]  = ADC_Read(ADC1, ADC_Channel_10); }
#define CELL09 { CELL.Ch[8]  = ADC_Read(ADC3, ADC_Channel_5) ; }
#define CELL10 { CELL.Ch[9]  = ADC_Read(ADC3, ADC_Channel_7) ; }
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

uint16_t ReadCell(uint8_t i) {
  switch (i) {
    case 0:
      return ADC_Read(ADC1, ADC_Channel_8) ; break;
    case 1:
      return ADC_Read(ADC1, ADC_Channel_14); break;
    case 2:
      return ADC_Read(ADC1, ADC_Channel_6) ; break;
    case 3:  
      return ADC_Read(ADC1, ADC_Channel_4) ; break;
    case 4:
      return ADC_Read(ADC1, ADC_Channel_2) ; break;
    case 5:
      return ADC_Read(ADC1, ADC_Channel_0) ; break;
    case 6:
      return ADC_Read(ADC1, ADC_Channel_12); break;
    case 7:  
      return ADC_Read(ADC1, ADC_Channel_10); break;
    case 8:  
      return ADC_Read(ADC3, ADC_Channel_5) ; break;
    case 9:
      return ADC_Read(ADC3, ADC_Channel_7) ; break;
    case 10:
      return ADC_Read(ADC1, ADC_Channel_15); break;
    case 11:
      return ADC_Read(ADC1, ADC_Channel_7) ; break;
    case 12:
      return ADC_Read(ADC1, ADC_Channel_5) ; break;
    case 13:
      return ADC_Read(ADC1, ADC_Channel_3) ; break;
    case 14:
      return ADC_Read(ADC1, ADC_Channel_1) ; break;
    case 15:
      return ADC_Read(ADC1, ADC_Channel_13); break;
    default:                                 break; 
  }
}

void CreateFile() {
  FRESULT res;
  FILINFO finfo;
  DIR dirs;
  
  char heading[205];
    
  disk_initialize(0);  
  
  f_mount(0, &fs);
 
  if(f_open(&fdst, "log.txt", FA_CREATE_NEW | FA_WRITE)==FR_OK ) {
    bw=1;
    sprintf(heading,"      TIME         CH01     CH02     CH03     CH04     CH05     CH06     CH07     CH08     CH09     CH10     CH11     CH12     CH13     CH14     CH15     CH16     CH17     CH18     CH19     CH20     CH21\r\n");
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
  
  char time[16];
  char string[9];
  
  disk_initialize(0);
    
  f_mount(0, &fs);
      
  if(f_open(&fdst, "log.txt", FA_OPEN_EXISTING | FA_WRITE)==FR_OK ) {
    bw=1;
     
    //print averages
    f_lseek(&fdst, fdst.fsize);
    sTime_Display(RTC_GetCounter(), time);
    f_write(&fdst, time, sizeof(time), &bw);
    f_sync(&fdst);
      
    //f_lseek(&fdst, fdst.fsize);
    for(index = 0; index < 16; index++){ 
      sprintf(string,"     %4d", ReadCell(index)); 
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
  //DMAInit();
  
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
		Time_Adjust();

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
  //DMA_Cmd(DMA1_Channel1, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  
  //Enable DMA2 Channel transfer
  
  //DMA_Cmd(DMA2_Channel5, ENABLE);
  
  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);
  
  //printf("CH01     CH02     CH03     CH04     CH05     CH06     CH07     CH08     CH09     CH10     CH11     CH12     CH13     CH14     CH15     CH16     CH17     CH18     CH19     CH20     CH21\r\n");
  
  while (1)
  {
    WriteFile();
    printf("running \r\n");        
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

  
  //VS1003 PE12, PE13, PE14   CS,SI,CLK
  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;
  
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13;    //PD13 VS1003 RST   
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;    //PC6 VS1003 XDCS   
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  	
  //PENIRQ, SO	
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;	 //VS1003 DOUT
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;	 //VS1003 DREQ
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  //GPIO_Init(GPIOB, &GPIO_InitStructure);
  

  GPIO_SetBits(GPIOC, GPIO_Pin_7);			//vs1003 DREQ
  

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
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

	/* Enable the RTC Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the DMA Interrupt */  
  NVIC_InitStructure.NVIC_IRQChannel = (DMA1_Channel1_IRQn | DMA2_Channel4_5_IRQn);
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

/**
  * @brief  Configures the RTC.
  * @param  None
  * @retval None
  */
void RTC_Configuration(void)
{
	/* Enable PWR and BKP clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

	/* Allow access to BKP Domain */
	PWR_BackupAccessCmd(ENABLE);

	/* Reset Backup Domain */
	BKP_DeInit();

	/* Enable LSE */
	RCC_LSEConfig(RCC_LSE_ON);
	/* Wait till LSE is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
	{}

	/* Select LSE as RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

	/* Enable RTC Clock */
	RCC_RTCCLKCmd(ENABLE);

	/* Wait for RTC registers synchronization */
	RTC_WaitForSynchro();

	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();

	/* Enable the RTC Second */
	RTC_ITConfig(RTC_IT_SEC, ENABLE);

	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();

	/* Set RTC prescaler: set RTC period to 1sec */
	RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */

	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();
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
