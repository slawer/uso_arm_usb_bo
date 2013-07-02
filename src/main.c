/**
  ******************************************************************************
  * @file    Audio_playback_and_record/src/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    28-October-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"

u8 in=0, poz=0;
		u32 tmp=0;
		u8 kol=0;
		float fl_tmp=0;
		
//u32  zad_spi=10000,zad_spi2=100000;
u32  zad_spi=10000,zad_spi2=100000;

/** @addtogroup STM32F4-Discovery_Audio_Player_Recorder
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#if defined MEDIA_USB_KEY
 USB_OTG_CORE_HANDLE          USB_OTG_Core;
 USBH_HOST                    USB_Host;
#endif

RCC_ClocksTypeDef RCC_Clocks;
__IO uint8_t RepeatState = 0;
__IO uint16_t CCR_Val = 16826;
extern __IO uint8_t LED_Toggle;

/* Private function prototypes -----------------------------------------------*/
static void TIM_LED_Config(void);
/* Private functions ---------------------------------------------------------*/


#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)
__IO uint16_t ADC3ConvertedValue = 0;



#include "my.h"
#include "my_def_ext.h"


// rcc
#include "rtc.h"
//#include "gpio.h"


	
	uint16_t zad,ms,kolkor,tekkor;
	
	uint16_t flper,tmo,pertmo;


void USART2_IRQHandler(void)
{ 

    	
				if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)		  // приняли
					{		u8 tmp=0;
						USART_ClearITPendingBit(USART2, USART_IT_RXNE);
				
						tmp=USART_ReceiveData (USART2);
						if (tmp==0x3A)
								tekpr=0;
	/*
						if (tekpr==1)
								if (tmp!=address)
								{
									tekpr=0;
									rxsize=0;
								}
		*/				
						if (tmp==0x0D)
						{
							rxsize=tekpr;
				//			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
							new_komand=1;
						}
						RxBuffer[tekpr]=tmp;
						tekpr++;
				//		GPIOD->ODR ^= tx_pin_en;
				//		GPIOD->ODR ^= rx_pin_en;
						

					}
//Transmission complete interrupt								 // чтото передали
        if(USART_GetITStatus(USART2, USART_IT_TC) != RESET)
        {
		
					USART_ClearITPendingBit(USART2, USART_IT_TC);//очищаем признак прерывания
					
					if (txsize>tekper)
						USART_SendData(USART2,TxBuffer[tekper]);
					else
					{
						if (txsize==tekper)
								USART_SendData(USART2, 0x0D);
						else
						{
							GPIO_WriteBit(GPIOD, rx_pin_en, Bit_RESET); 
							USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
						}
						//
//						GPIOD->ODR ^= tx_pin_en;
		//					GPIOD->ODR ^= rx_pin_en;
					}
					if (tekper>TxBufferSize-1)
						tekper=0;
					tekper++;
					
			 
					//					GPIOA->BSRR=GPIO_BSRR_BR11|GPIO_BSRR_BR12; // настроиться на прием
        }



}



void ADC3_CH12_DMA_Config(void)
{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  // Enable ADC3, DMA2 and GPIO clocks **************************************
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

  // DMA2 Stream0 channel0 configuration ************************************
  DMA_InitStructure.DMA_Channel = DMA_Channel_2;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream0, ENABLE);

 // Configure ADC3 Channel12 pin as analog input ****************************
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // ADC Common Init *********************************************************
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

 // ADC3 Init ***************************************************************
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC3, &ADC_InitStructure);

  // ADC3 regular channel12 configuration ***********************************
  ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_480Cycles); //ADC_SampleTime_3Cycles);

 // Enable DMA request after last transfer (Single-ADC mode)
  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);

  // Enable ADC3 DMA 
  ADC_DMACmd(ADC3, ENABLE);

  // Enable ADC3 
  ADC_Cmd(ADC3, ENABLE);
}


void UART2Init(void)
{

   GPIO_InitTypeDef GPIO_InitStructure;
   USART_InitTypeDef USART_InitStructure;

   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);    	// 1.
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); 			// 2.

   GPIO_PinAFConfig  ( GPIOD, GPIO_PinSource5 , GPIO_AF_USART2) ;
   GPIO_PinAFConfig  ( GPIOD, GPIO_PinSource6 , GPIO_AF_USART2) ;   
   // 
   //     //  Tx
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // alternate function!
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP ;
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
     GPIO_Init(GPIOD, &GPIO_InitStructure);
   ////

   //     // Rx
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //GPIO_Mode_IN;
     GPIO_Init(GPIOD, &GPIO_InitStructure);
   //
     USART_InitStructure.USART_BaudRate = 9600;
     USART_InitStructure.USART_WordLength = USART_WordLength_8b;
     USART_InitStructure.USART_StopBits = USART_StopBits_1;
     USART_InitStructure.USART_Parity = USART_Parity_No;
     USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
     USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

   USART_Init(USART2, &USART_InitStructure);
	 
	 // enable interrupt on sended data
	//	USART_ITConfig(USART2, USART_IT_TC, ENABLE);
		// enable interrupt on received data
//	  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//?????????? ?????????? ?? ?????

//RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;   //???????????? GPIO
//RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;   //???????????? ?????????????? ??????? GPIO
//RCC->APB2ENR |= RCC_APB2ENR_USART1EN; //???????????? ?????? USART1

   USART_Cmd(USART2, ENABLE); // enable USART2

}

/*
void SendStringUSART2(const char *str)
{
   while(*str != '\0')
   {
      //  while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        USART_SendData(USART2, *str++);
   } 
}
*/


void settime(void)
{

		RTC->ISR |= RTC_ISR_INIT;
    while(!(RTC->ISR & RTC_ISR_INITF)) {}
			
	//	rtc_SetDate(2, 6, 13, 7);
		{
    uint32_t Tens, Units;
    uint32_t TempReg = 0;
		uint8_t	Day=2,  Month=6,  Year=13,  DayOfWeek=7;
    

    TempReg = 0;
    {
        Tens  = (Year / 10) & 0x0f;          // ??????? ???
        Units = (Year - (Tens * 10)) & 0x0f; // ??????? ???
        
        TempReg |= (Tens  << 20); // YT, 20
        TempReg |= (Units << 16); // YU, 16
    }

    {
        Tens  = (Month / 10) & 0x01;          // ??????? ???????
        Units = (Month - (Tens * 10)) & 0x0f; // ??????? ???????
        
        TempReg |= (Tens  << 12); // MT, 12
        TempReg |= (Units << 8);  // MU, 8
    }

    {
        Tens  = (Day / 10) & 0x03;          // ??????? ????
        Units = (Day - (Tens * 10)) & 0x0f; // ??????? ????
        
        TempReg |= (Tens  << 4); // DT, 4
        TempReg |= (Units << 0);  // DU, 0
    }

    {
        TempReg |= ((DayOfWeek & 0x07) << 13); // WDU, 13
    }
    RTC->DR = TempReg;
		}
			
			
//    rtc_SetTime(0, 0, 00);
		
		{
			uint32_t Tens, Units;
			uint32_t TempReg = 0;
			uint8_t Hours=0,  Minutes=0,  Seconds=0;
    
    // ??????? ???? ????
    TempReg = 0;
    
    // ??????? ????
    {
        Tens  = (Hours / 10) & 0x03;          // ??????? ?????
        Units = (Hours - (Tens * 10)) & 0x0f; // ??????? ?????
        
        TempReg |= (Tens  << 20); // HT, 20
        TempReg |= (Units << 16); // HU, 16
    }
    // ??????? ??????
    {
        Tens  = (Minutes / 10) & 0x07;          // ??????? ?????
        Units = (Minutes - (Tens * 10)) & 0x0f; // ??????? ?????
        
        TempReg |= (Tens  << 12); // MNT, 12
        TempReg |= (Units << 8);  // MNU, 8
    }
    // ??????? ???????
    {
        Tens  = (Seconds / 10) & 0x07;          // ??????? ??????
        Units = (Seconds - (Tens * 10)) & 0x0f; // ??????? ??????
        
        TempReg |= (Tens  << 4); // ST, 4
        TempReg |= (Units << 0);  // SU, 0
    }
    
    // ?????????? ???? ??? ?????
    RTC->TR = TempReg;
		
		}
        
    RTC->CR |= RTC_CR_FMT;
    RTC->ISR &= ~RTC_ISR_INIT;
		RTC->WPR = 0xFF;
	
}

void sohr_backup(u16 kol_byte,uint8_t* buf)
{

}

void spi_init(){

		GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitTypeDef gpio;
	  SPI_InitTypeDef spi1;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);  // ???????????? ?????
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);  // ???????????? SPI1

    GPIO_StructInit(&gpio);
/*
     gpio.GPIO_Pin   = GPIO_Pin_4;   // NSS
     gpio.GPIO_Mode  = GPIO_Mode_OUT;
     gpio.GPIO_Speed = GPIO_Speed_50MHz;
     gpio.GPIO_OType = GPIO_OType_PP;
     gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
     GPIO_Init(GPIOA, &GPIO_InitStructure);
*/
    gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA,&gpio);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);   
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);
    SPI_I2S_DeInit(SPI1);

    SPI_StructInit(&spi1);
    spi1.SPI_Mode = SPI_Mode_Master;
    spi1.SPI_DataSize = SPI_DataSize_16b;
    spi1.SPI_NSS = SPI_NSS_Soft;
    SPI_Init(SPI1,&spi1);
    SPI_Cmd(SPI1,ENABLE);
}

void spi1_init() {
	SPI_InitTypeDef spi1;
	 GPIO_InitTypeDef gpio;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);  // ???????????? ?????
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);  // ???????????? SPI1 
   
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA,&gpio);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);
    SPI_I2S_DeInit(SPI1);
    
    SPI_StructInit(&spi1);
    spi1.SPI_Mode = SPI_Mode_Master;
    spi1.SPI_DataSize = SPI_DataSize_8b; //SPI_DataSize_16b;		
    spi1.SPI_NSS = SPI_NSS_Soft;

  spi1.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  spi1.SPI_FirstBit = SPI_FirstBit_MSB;
	
    SPI_Init(SPI1,&spi1);
    SPI_Cmd(SPI1,ENABLE);
}


void spi2_init() {
	SPI_InitTypeDef spi2;
	 GPIO_InitTypeDef gpio;
	
 //   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
   
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB,&gpio);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);
    SPI_I2S_DeInit(SPI2);
    
    SPI_StructInit(&spi2);
    spi2.SPI_Mode = SPI_Mode_Slave;
    spi2.SPI_DataSize = SPI_DataSize_16b;
    spi2.SPI_NSS = SPI_NSS_Soft;
    SPI_Init(SPI2,&spi2);
    SPI_Cmd(SPI2,ENABLE);
}


void spi_send(uint16_t data) {
	SPI_I2S_SendData(SPI1,data);
    while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == RESET);  // ???? ???? ?????? ?????
}
uint16_t spi_receve() {
		uint16_t received;
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);  // ???? ???? ?????? ???????
    received = SPI_I2S_ReceiveData(SPI2);
    return received;
}


void delay_spi(u32 kol)
{
	u32 i=0;


	for (i=0;i<kol;i++)
				__ASM volatile ("nop");
}



void test_ind_all(u8 rez)
{
	
				GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET); 	delay_spi(zad_spi);
				spi_send(0x0f); delay_spi(zad_spi);				
				spi_send(rez);  delay_spi(zad_spi);
				GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);		delay_spi(zad_spi2);

				GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET); 	delay_spi(zad_spi);
				spi_send(0x0f); delay_spi(zad_spi);				
				spi_send(rez);  delay_spi(zad_spi);
				GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);		delay_spi(zad_spi2);

				GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_RESET); 	delay_spi(zad_spi);
				spi_send(0x0f); delay_spi(zad_spi);				
				spi_send(rez);  delay_spi(zad_spi);
				GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET);		delay_spi(zad_spi2);

				GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_RESET); 	delay_spi(zad_spi);
				spi_send(0x0f); delay_spi(zad_spi);				
				spi_send(rez);  delay_spi(zad_spi);
				GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_SET);		delay_spi(zad_spi2);	
	
				
}

uint16_t pin_ind(u8 numb_ind)
{
	uint16_t  pin=0;

			switch (numb_ind-1)
			{
				case 0x00:  // CS0   PA0
					pin=GPIO_Pin_0;
					break;

				case 0x01:  // CS1	PA1
					pin=GPIO_Pin_1;
					break;

				case 0x02:  // CS2	PA2
					pin=GPIO_Pin_2;
					break;

				case 0x03:  // CS3	PA3
					pin=GPIO_Pin_3;
					break;
				/*
				case 0x04:  // CS4
					pin=GPIO_Pin_4;
					break;	
				*/
				default:
					pin=0;
					break;
			}
			
			return pin;

}
void indicate_lin(u8 numb_ind,u16 zn, u16 maks, u16 max_kol_st)
{
		uint16_t  pin=0;
		const u8 tabl[9]={0,1,3,7,0xf,0x1f,0x3f,0x7f,0xff};
		u8 viv=0;
		
//		u16  zad_spi=1000,zad_spi2=10000;

	 tmp=(u32) (max_kol_st*zn);
	fl_tmp=(float) ((float)zn/(float)maks*(float)max_kol_st);
	fl_tmp+=0.5;
	 kol=(u8) (fl_tmp);

	
/*
		  switch (numb_ind)
			{
				case 0x00:  // CS0
					pin=GPIO_Pin_0;
					break;

				case 0x01:  // CS1
					pin=GPIO_Pin_1;
					break;

				case 0x02:  // CS2
					pin=GPIO_Pin_2;
					break;

				case 0x03:  // CS3
					pin=GPIO_Pin_3;
					break;

				case 0x04:  // CS4
					pin=GPIO_Pin_4;
					break;								
			}
	*/
	  pin=pin_ind(numb_ind);
		if (pin==0)
				return ;
		if ((kol>max_kol_st)|(zn>=maks))
		{
			if (max_kol_st==28)
			{
				GPIO_WriteBit(GPIOA, pin, Bit_RESET); 	delay_spi(zad_spi);
				spi_send((u8)0x01);											delay_spi(zad_spi);
				spi_send((u8) 0); 											delay_spi(zad_spi);	
				GPIO_WriteBit(GPIOA, pin, Bit_SET);			delay_spi(zad_spi2);

				GPIO_WriteBit(GPIOA, pin, Bit_RESET); 	delay_spi(zad_spi);
				spi_send((u8)0x02);											delay_spi(zad_spi);
				spi_send((u8) 0); 											delay_spi(zad_spi);	
				GPIO_WriteBit(GPIOA, pin, Bit_SET);			delay_spi(zad_spi2);

				GPIO_WriteBit(GPIOA, pin, Bit_RESET); 	delay_spi(zad_spi);
				spi_send((u8)0x03);											delay_spi(zad_spi);
				spi_send((u8) 0); 											delay_spi(zad_spi);	
				GPIO_WriteBit(GPIOA, pin, Bit_SET);			delay_spi(zad_spi2);				
				
				GPIO_WriteBit(GPIOA, pin, Bit_RESET); 	delay_spi(zad_spi);
				spi_send((u8)0x04);											delay_spi(zad_spi);
				spi_send((u8) 0x08); 										delay_spi(zad_spi);	
				GPIO_WriteBit(GPIOA, pin, Bit_SET);			delay_spi(zad_spi2);
			 }
			return ;
		}
		if (kol>7)
			viv=tabl[8];
		else
			viv=tabl[kol%8];
		
		GPIO_WriteBit(GPIOA, pin, Bit_RESET); 	delay_spi(zad_spi);
		spi_send((u8)0x01);											delay_spi(zad_spi);
		spi_send((u8) viv); 										delay_spi(zad_spi);	
		GPIO_WriteBit(GPIOA, pin, Bit_SET);			delay_spi(zad_spi2);
		
		if (kol<8)
			viv=0;
		else
		{
			kol=kol-8;
			if (kol>7)
				viv=tabl[8];
			else
				viv=tabl[kol%8];
		}
		
		GPIO_WriteBit(GPIOA, pin, Bit_RESET); 	delay_spi(zad_spi);
		spi_send((u8)0x02);											delay_spi(zad_spi);
		spi_send((u8)viv); 											delay_spi(zad_spi);	
		GPIO_WriteBit(GPIOA, pin, Bit_SET);			delay_spi(zad_spi2);
	
		if (kol<8)
			viv=0;
		else
		{
			kol=kol-8;
			if (kol>7)
				viv=tabl[8];
			else
				viv=tabl[kol%8];
		}
		
		GPIO_WriteBit(GPIOA, pin, Bit_RESET); 	delay_spi(zad_spi);
		spi_send((u8)0x03);delay_spi(zad_spi);
		spi_send((u8)viv); delay_spi(zad_spi);	
		GPIO_WriteBit(GPIOA, pin, Bit_SET);			delay_spi(zad_spi2);

		if (kol<8)
			viv=0;
		else
		{
			kol=kol-8;
			if (kol>7)
				viv=tabl[8];
			else
				viv=tabl[kol%8];
		}
		
		GPIO_WriteBit(GPIOA, pin, Bit_RESET); 	delay_spi(zad_spi);
		spi_send((u8)0x04);delay_spi(zad_spi);
		spi_send((u8)viv); delay_spi(zad_spi);	
		GPIO_WriteBit(GPIOA, pin, Bit_SET);			delay_spi(zad_spi2);
	//	kol=kol%8;			

	
}
/*
void test_lin2(void)
{
		u8 i=0, z=0;	
		uint16_t  pin=GPIO_Pin_0;
	
		GPIO_WriteBit(GPIOA, pin, Bit_RESET); 	delay_spi(100);
		spi_send((u8)0x0F);delay_spi(100);
		spi_send((u8)1); delay_spi(100);	
		GPIO_WriteBit(GPIOA, pin, Bit_SET);			delay_spi(1000);
	
			GPIO_WriteBit(GPIOA, pin, Bit_RESET); 	delay_spi(100);
		spi_send((u8)0x0F);delay_spi(100);
		spi_send((u8)0); delay_spi(100);	
		GPIO_WriteBit(GPIOA, pin, Bit_SET);			delay_spi(1000);
	
		GPIO_WriteBit(GPIOA, pin, Bit_RESET); 	delay_spi(100);
		spi_send((u8)0x0C);delay_spi(100);
		spi_send((u8)1); delay_spi(100);	
		GPIO_WriteBit(GPIOA, pin, Bit_SET);			delay_spi(1000);

		GPIO_WriteBit(GPIOA, pin, Bit_RESET); 	delay_spi(100);
		spi_send((u8)0x0B);delay_spi(100);
		spi_send((u8)7); delay_spi(100);	
		GPIO_WriteBit(GPIOA, pin, Bit_SET);			delay_spi(1000);	

		GPIO_WriteBit(GPIOA, pin, Bit_RESET); 	delay_spi(100);
		spi_send((u8)0x0A);delay_spi(100);
		spi_send((u8)0xFF); delay_spi(100);	
		GPIO_WriteBit(GPIOA, pin, Bit_SET);			delay_spi(1000);	
	
		GPIO_WriteBit(GPIOA, pin, Bit_RESET); 	delay_spi(100);
		spi_send((u8)0x09);delay_spi(100);
		spi_send((u8)0); delay_spi(100);	
		GPIO_WriteBit(GPIOA, pin, Bit_SET);			delay_spi(1000);	
	
	
	in=1;
	poz=1;
		while (1)
		{
		GPIO_WriteBit(GPIOA, pin, Bit_RESET); 	delay_spi(100);
		spi_send((u8)0x01);delay_spi(100);
		spi_send((u8)0); delay_spi(100);	
		GPIO_WriteBit(GPIOA, pin, Bit_SET);			delay_spi(1000);
	
		
		for (i=1;i<200;i++)
			delay_spi(100000);
		
		GPIO_WriteBit(GPIOA, pin, Bit_RESET); 	delay_spi(100);
		spi_send((u8)in);delay_spi(100);
		spi_send((u8)poz); delay_spi(100);	
		GPIO_WriteBit(GPIOA, pin, Bit_SET);			delay_spi(1000);	
		

		if (poz>=128)
		{
		  in++;
			if (in>8)
					in=1;
			poz=1;
		}
		poz=poz*2;	
		for (i=1;i<200;i++)
			delay_spi(10000);
		}
}

*/

/*
void test_lin(void)
{
		u8 i=0, z=0;
	
	uint16_t  pin=GPIO_Pin_0;
	uint16_t  pin_c=GPIO_Pin_1;	
	// test lineika
	
		GPIO_WriteBit(GPIOA, pin, Bit_RESET);
		GPIO_WriteBit(GPIOA, pin_c, Bit_RESET); 
				delay_spi(100);
				spi_send(0x01);delay_spi(100);
				spi_send(0); delay_spi(100);	
		GPIO_WriteBit(GPIOA, pin_c, Bit_SET);
		GPIO_WriteBit(GPIOA, pin, Bit_SET);	
	
			delay_spi(1000);
		GPIO_WriteBit(GPIOA, pin, Bit_RESET);
		GPIO_WriteBit(GPIOA, pin_c, Bit_RESET); 
				delay_spi(100);
				spi_send(0x02);delay_spi(100);
				spi_send(0); delay_spi(100);	
		GPIO_WriteBit(GPIOA, pin_c, Bit_SET);
		GPIO_WriteBit(GPIOA, pin, Bit_SET);	
		
			delay_spi(1000);
		GPIO_WriteBit(GPIOA, pin, Bit_RESET);			
					GPIO_WriteBit(GPIOA, pin_c, Bit_RESET); 
				delay_spi(100);
				spi_send(0x03);delay_spi(100);
				spi_send(0); delay_spi(100);	
		GPIO_WriteBit(GPIOA, pin_c, Bit_SET);
		GPIO_WriteBit(GPIOA, pin, Bit_SET);	
		
			delay_spi(1000);
		GPIO_WriteBit(GPIOA, pin, Bit_RESET);			
					GPIO_WriteBit(GPIOA, pin_c, Bit_RESET); 
				delay_spi(100);
				spi_send(0x04);delay_spi(100);
				spi_send(0); delay_spi(100);	
		GPIO_WriteBit(GPIOA, pin_c, Bit_SET);
		GPIO_WriteBit(GPIOA, pin, Bit_SET);	
		
			delay_spi(1000);
		GPIO_WriteBit(GPIOA, pin, Bit_RESET);			
					GPIO_WriteBit(GPIOA, pin_c, Bit_RESET); 
				delay_spi(100);
				spi_send(0x05);delay_spi(100);
				spi_send(0); delay_spi(100);	
		GPIO_WriteBit(GPIOA, pin_c, Bit_SET);
		GPIO_WriteBit(GPIOA, pin, Bit_SET);	
		
			delay_spi(1000);
		GPIO_WriteBit(GPIOA, pin, Bit_RESET);			
					GPIO_WriteBit(GPIOA, pin_c, Bit_RESET); 
				delay_spi(100);
				spi_send(0x06);delay_spi(100);
				spi_send(0); delay_spi(100);	
		GPIO_WriteBit(GPIOA, pin_c, Bit_SET);
		GPIO_WriteBit(GPIOA, pin, Bit_SET);	
		
			delay_spi(1000);
		GPIO_WriteBit(GPIOA, pin, Bit_RESET);			
					GPIO_WriteBit(GPIOA, pin_c, Bit_RESET); 
				delay_spi(100);
				spi_send(0x07);delay_spi(100);
				spi_send(0); delay_spi(100);	
		GPIO_WriteBit(GPIOA, pin_c, Bit_SET);
		GPIO_WriteBit(GPIOA, pin, Bit_SET);		
			delay_spi(1000);
					GPIO_WriteBit(GPIOA, pin, Bit_RESET);
					GPIO_WriteBit(GPIOA, pin_c, Bit_RESET); 
				delay_spi(100);
				spi_send(0x08);delay_spi(100);
				spi_send(0); delay_spi(100);	
		GPIO_WriteBit(GPIOA, pin_c, Bit_SET);
		GPIO_WriteBit(GPIOA, pin, Bit_SET);	
			delay_spi(1000);
	
			
			// scan limit
			GPIO_WriteBit(GPIOA, pin_c, Bit_RESET); 
			GPIO_WriteBit(GPIOA, pin, Bit_RESET);      //   GPIOA.0			
			delay_spi(100);
			spi_send(0x0B);
			delay_spi(100);
			spi_send(1);
			delay_spi(100);			
			GPIO_WriteBit(GPIOA, pin_c, Bit_SET);  
			GPIO_WriteBit(GPIOA, pin, Bit_SET);  
			
			delay_spi(1000);
		
			// registr intensivnost

				GPIO_WriteBit(GPIOA, pin, Bit_RESET);  
			GPIO_WriteBit(GPIOA, pin_c, Bit_RESET);      //   GPIOA.0			
			delay_spi(100);
			spi_send(0x0A);	
			delay_spi(100);
			spi_send(0x01);																	
			delay_spi(100);			
			GPIO_WriteBit(GPIOA, pin_c, Bit_SET);  
			GPIO_WriteBit(GPIOA, pin, Bit_SET);  	
			// code B

				GPIO_WriteBit(GPIOA, pin, Bit_RESET);  
			GPIO_WriteBit(GPIOA, pin_c, Bit_RESET);      //   GPIOA.0			
			delay_spi(100);
			spi_send(0x09);	
			delay_spi(100);
			spi_send(0x00);																	
			delay_spi(100);			
			GPIO_WriteBit(GPIOA, pin_c, Bit_SET);  	
			GPIO_WriteBit(GPIOA, pin, Bit_SET);  

				GPIO_WriteBit(GPIOA, pin, Bit_RESET);  
				GPIO_WriteBit(GPIOA, pin_c, Bit_RESET);      //   CS  GPIOA.0
			delay_spi(100);
			spi_send(0x0C);
			delay_spi(100);
			spi_send(0x01);
			delay_spi(100);			
			GPIO_WriteBit(GPIOA, pin_c, Bit_SET);      //   GPIOA.0
			GPIO_WriteBit(GPIOA, pin, Bit_SET);  
			delay_spi(1000);

	
	while (1)
	{
	
		 GPIO_WriteBit(GPIOA, pin, Bit_RESET); 
		GPIO_WriteBit(GPIOA, pin_c, Bit_RESET); 
				delay_spi(100);
				spi_send(0x0f);delay_spi(100);
				spi_send(0); delay_spi(100);					
			GPIO_WriteBit(GPIOA, pin, Bit_SET); 
		GPIO_WriteBit(GPIOA, pin_c, Bit_SET);
		delay_spi(1000);


		 GPIO_WriteBit(GPIOA, pin, Bit_RESET); 
		GPIO_WriteBit(GPIOA, pin_c, Bit_RESET); 
				delay_spi(100);
				spi_send(0x01);delay_spi(100);
				spi_send((u8)z); delay_spi(100);	
		
		  switch (z)
			{
				case 0x00:  
					z=1;
					break;

				case 1:  
					z=2;
					break;

				case 2:  
					z=4;
					break;

				case 4:  
					z=8;
					break;

				case 8:  
					z=16;
					break;

				case 16:  
					z=32;
					break;

				case 32:  
					z=64;
					break;

				case 64:  
					z=128;
					break;

				case 128:  
					z=0;
					break;

				default:
						z=0;
					break;
			}
			
		 GPIO_WriteBit(GPIOA, pin_c, Bit_SET);
		 GPIO_WriteBit(GPIOA, pin, Bit_RESET); 		

		for (i=1;i<200;i++)
			delay_spi(100000);
		
		for (i=1;i<200;i++)
			delay_spi(9000);	

	}
}
*/

void indicate_err(u8 numb_ind)
{
			uint16_t  pin=0;
			u8 i=0;
	
			pin=pin_ind(numb_ind);
			if (pin==0)
				return;
			
			for (i=1;i<9;i++)
			{
				// error
				GPIO_WriteBit(GPIOA, pin, Bit_RESET);      delay_spi(zad_spi);
				spi_send((u8) i);					delay_spi(zad_spi);
				spi_send(0x01);					  delay_spi(zad_spi);			
				GPIO_WriteBit(GPIOA, pin, Bit_SET);      		delay_spi(zad_spi2);
			}	
}

void ind_blank_all(u8 numb_ind)
{
		u8 i=0,ind=0;
		uint16_t  pin=0;
	
		pin=pin_ind(numb_ind);
		if (pin==0)
				return;

			for (i=1;i<9;i++)
			{
				// blank all
				GPIO_WriteBit(GPIOA, pin, Bit_RESET);      delay_spi(zad_spi);
				spi_send((u8) i);					delay_spi(zad_spi);
				spi_send(0x00);					  delay_spi(zad_spi);			
				GPIO_WriteBit(GPIOA, pin, Bit_SET);      		delay_spi(zad_spi2);
			}
		
}
void init_ind(u8 numb_ind, u8 kol_ind, u8 type_ind)
{
			uint16_t  pin=0;
			u8 i=0;
//			u16  zad_spi=1000,zad_spi2=10000;

			/*
		  switch (numb_ind)
			{
				case 0x00:  // CS0
					pin=GPIO_Pin_0;
					break;

				case 0x01:  // CS1
					pin=GPIO_Pin_1;
					break;

				case 0x02:  // CS2
					pin=GPIO_Pin_2;
					break;

				case 0x03:  // CS3
					pin=GPIO_Pin_3;
					break;

				case 0x04:  // CS4
					pin=GPIO_Pin_4;
					break;								
			}
			*/
			pin=pin_ind(numb_ind);
			if (pin==0)
				return;
			
			// shutdown off
			GPIO_WriteBit(GPIOA, pin, Bit_RESET);  delay_spi(zad_spi);		
			spi_send(0x0C);			delay_spi(zad_spi);
			spi_send(0x01);			delay_spi(zad_spi);			
			GPIO_WriteBit(GPIOA, pin, Bit_SET);      		delay_spi(zad_spi2);

		for (i=1;i<9;i++)
		{
			// blank all
			GPIO_WriteBit(GPIOA, pin, Bit_RESET);      delay_spi(zad_spi);
			spi_send((u8) i);					delay_spi(zad_spi);
			spi_send(0x00);					  delay_spi(zad_spi);			
			GPIO_WriteBit(GPIOA, pin, Bit_SET);      		delay_spi(zad_spi2);
		}
		
			// scan limit
			GPIO_WriteBit(GPIOA, pin, Bit_RESET);   			delay_spi(zad_spi);
			spi_send(0x0B);				delay_spi(zad_spi);
			spi_send(kol_ind-1);	delay_spi(zad_spi);			
			GPIO_WriteBit(GPIOA, pin, Bit_SET);      			delay_spi(zad_spi2);
			
			// registr intensivnost
			GPIO_WriteBit(GPIOA, pin, Bit_RESET);      delay_spi(zad_spi);
			spi_send(0x0A);					delay_spi(zad_spi);
			spi_send(0x0F);					delay_spi(zad_spi);			
			GPIO_WriteBit(GPIOA, pin, Bit_SET);     delay_spi(zad_spi2);

/*	
		// test
			GPIO_WriteBit(GPIOA, pin, Bit_RESET);      delay_spi(zad_spi);
			spi_send((u8) 0x0f);					delay_spi(zad_spi);
			spi_send(0x01);					delay_spi(zad_spi);			
			GPIO_WriteBit(GPIOA, pin, Bit_SET);      delay_spi(zad_spi2);

		  delay_spi(zad_spi2*50);	
		
			GPIO_WriteBit(GPIOA, pin, Bit_RESET);      delay_spi(zad_spi);
			spi_send((u8) 0x0f);					delay_spi(zad_spi);
			spi_send(0x00);					delay_spi(zad_spi);			
			GPIO_WriteBit(GPIOA, pin, Bit_SET);     delay_spi(zad_spi2);			
*/			
	//		test_ind_all(1);
	//	test_lin();
}

void test_ind(u8 numb_ind)
{
	uint16_t  pin=0;
	 /*
			switch (numb_ind)
			{
				case 0x00:  // CS0
					pin=GPIO_Pin_0;
					break;

				case 0x01:  // CS1
					pin=GPIO_Pin_1;
					break;

				case 0x02:  // CS2
					pin=GPIO_Pin_2;
					break;

				case 0x03:  // CS3
					pin=GPIO_Pin_3;
					break;

				case 0x04:  // CS4
					pin=GPIO_Pin_4;
					break;								
			}
			*/
			
			pin=pin_ind(numb_ind);
		if (pin==0)
				return ;			

				GPIO_WriteBit(GPIOA, pin, Bit_RESET);      //   GPIOB.2
				delay_spi(100);
				spi_send(0x0f);delay_spi(100);
				spi_send(0x01); delay_spi(100);
				GPIO_WriteBit(GPIOA, pin, Bit_SET);      //   GPIOB.2
				delay_spi(1000);
}

/*
void norm_ind(u8 numb_ind)
{
	uint16_t  pin=0;
	
			switch (numb_ind)
			{
				case 0x00:  // CS0
					pin=GPIO_Pin_0;
					break;

				case 0x01:  // CS1
					pin=GPIO_Pin_1;
					break;

				case 0x02:  // CS2
					pin=GPIO_Pin_2;
					break;

				case 0x03:  // CS3
					pin=GPIO_Pin_3;
					break;

				case 0x04:  // CS4
					pin=GPIO_Pin_4;
					break;								
			}
			

				GPIO_WriteBit(GPIOA, pin, Bit_RESET);      //   GPIOB.2
				delay_spi(100);
				spi_send(0x0f);delay_spi(100);
				spi_send(0x00); delay_spi(100);
				GPIO_WriteBit(GPIOA, pin, Bit_SET);      //   GPIOB.2
				delay_spi(1000);
}
*/

/*
void indicate_test(u8 numb_ind, u8 test_numb)
{
	uint16_t  pin=0;
	u8 i=0;

				pin=pin_ind(numb_ind);
			
				GPIO_WriteBit(GPIOA, pin, Bit_RESET);      //   GPIOB.2
				delay_spi(1000);
				spi_send(0x0f);delay_spi(1000);
				spi_send(0x00); delay_spi(1000);
				GPIO_WriteBit(GPIOA, pin, Bit_SET);      //   GPIOB.2
				delay_spi(10000);
			
  if (test_numb==8)
	{
				
				GPIO_WriteBit(GPIOA, pin, Bit_RESET);      
				delay_spi(1000);
				spi_send((u8) 0);delay_spi(1000);
				spi_send((u8) symb_code[0]); delay_spi(1000);		
				GPIO_WriteBit(GPIOA, pin, Bit_SET);      
			  delay_spi(10000);			

				GPIO_WriteBit(GPIOA, pin, Bit_RESET);      
				delay_spi(1000);
				spi_send((u8) 1);delay_spi(1000);
				spi_send((u8) symb_code[1]); delay_spi(1000);		
				GPIO_WriteBit(GPIOA, pin, Bit_SET);      
			  delay_spi(10000);	

				GPIO_WriteBit(GPIOA, pin, Bit_RESET);      
				delay_spi(1000);
				spi_send((u8) 2);delay_spi(1000);
				spi_send((u8) symb_code[2]); delay_spi(1000);		
				GPIO_WriteBit(GPIOA, pin, Bit_SET);      
			  delay_spi(10000);	
				
				GPIO_WriteBit(GPIOA, pin, Bit_RESET);      
				delay_spi(1000);
				spi_send((u8) 3);delay_spi(1000);
				spi_send((u8) symb_code[3]); delay_spi(1000);		
				GPIO_WriteBit(GPIOA, pin, Bit_SET);      
			  delay_spi(10000);	
				
				GPIO_WriteBit(GPIOA, pin, Bit_RESET);      
				delay_spi(1000);
				spi_send((u8) 4);delay_spi(1000);
				spi_send((u8) symb_code[4]); delay_spi(1000);		
				GPIO_WriteBit(GPIOA, pin, Bit_SET);      
			  delay_spi(10000);	

				GPIO_WriteBit(GPIOA, pin, Bit_RESET);      
				delay_spi(1000);
				spi_send((u8) 5);delay_spi(1000);
				spi_send((u8) symb_code[5]); delay_spi(1000);		
				GPIO_WriteBit(GPIOA, pin, Bit_SET);      
			  delay_spi(10000);	
				
				GPIO_WriteBit(GPIOA, pin, Bit_RESET);      
				delay_spi(1000);
				spi_send((u8) 6);delay_spi(1000);
				spi_send((u8) symb_code[6]); delay_spi(1000);		
				GPIO_WriteBit(GPIOA, pin, Bit_SET);      
			  delay_spi(10000);	
				
				GPIO_WriteBit(GPIOA, pin, Bit_RESET);      
				delay_spi(1000);
				spi_send((u8) 7);delay_spi(1000);
				spi_send((u8) symb_code[7]); delay_spi(1000);		
				GPIO_WriteBit(GPIOA, pin, Bit_SET);      
			  delay_spi(10000);	
				
				GPIO_WriteBit(GPIOA, pin, Bit_RESET);      
				delay_spi(1000);
				spi_send((u8) 8);delay_spi(1000);
				spi_send((u8) symb_code[8]); delay_spi(1000);		
				GPIO_WriteBit(GPIOA, pin, Bit_SET);      
			  delay_spi(10000);
			}

			if (1)
			{
				for (i=1;i<8;i++)
				{	
						GPIO_WriteBit(GPIOA, pin, Bit_RESET);      
						delay_spi(1000);
						spi_send((u8) i); delay_spi(1000);
						spi_send((u8) symb_code[1]); delay_spi(1000);		
						GPIO_WriteBit(GPIOA, pin, Bit_SET);      
						delay_spi(10000);		
				}
			}
			
			if (9)
			{
				for (i=1;i<8;i++)
				{	
						GPIO_WriteBit(GPIOA, pin, Bit_RESET);      
						delay_spi(1000);
						spi_send((u8) i); delay_spi(1000);
						spi_send((u8) 0); delay_spi(1000);		
						GPIO_WriteBit(GPIOA, pin, Bit_SET);      
						delay_spi(10000);		
				}
			}
}
*/

void indicate(u8 numb_ind,u16 chislo_new, u8 kol_cifr)
{
		 	uint16_t  pin=0;
			u16 chislo=chislo_new; //indicators[numb_ind].chislo;
			u8 i=0, zn[6], null=1, simb=0;
			u32   maximum=0;
			

			/*
			switch (numb_ind)
			{
				case 0x00:  // CS0
					pin=GPIO_Pin_0;
					break;

				case 0x01:  // CS1
					pin=GPIO_Pin_1;
					break;

				case 0x02:  // CS2
					pin=GPIO_Pin_2;
					break;

				case 0x03:  // CS3
					pin=GPIO_Pin_3;
					break;

				case 0x04:  // CS4
					pin=GPIO_Pin_4;
					break;								
			}
			*/
			pin=pin_ind(numb_ind);
		if (pin==0)
				return ;
		
	//		switch (conf.indicators[numb_ind-1].kol_cifr)
			switch (kol_cifr)
			{
				case 0x00:  // CS0
					return;
					break;
				case 0x01:  // CS0
					maximum=10;
					break;			
				case 0x02:  // CS0
					maximum=100;
					break;
				case 0x03:  // CS0
					maximum=1000;
					break;
				case 0x04:  // CS0
					maximum=10000;
					break;
				case 0x05:  // CS0
					maximum=100000;
					break;	
				case 0x06:  // CS0
					maximum=1000000;
					break;	
				case 0x07:  // CS0
					maximum=10000000;
					break;	
				case 0x08:  // CS0
					maximum=100000000;
					break;					
				default:  // CS0
					maximum=100000000;
					break;					
			}
	
			if (chislo>=maximum)   // reflow
			{			
				GPIO_WriteBit(GPIOA, pin, Bit_RESET);      
				delay_spi(zad_spi);
				spi_send(1);delay_spi(zad_spi);
				spi_send(0x4F); delay_spi(zad_spi);		
				GPIO_WriteBit(GPIOA, pin, Bit_SET);      
			  delay_spi(zad_spi2);	
				
		//		for (i=2;i<conf.indicators[numb_ind-1].kol_cifr+1;i++)  {
				for (i=2;i<kol_cifr+1;i++)  {
				GPIO_WriteBit(GPIOA, pin, Bit_RESET);      
				delay_spi(zad_spi);
				spi_send((u8) i);delay_spi(zad_spi);
				spi_send(0); delay_spi(zad_spi);		
				GPIO_WriteBit(GPIOA, pin, Bit_SET);      
			  delay_spi(zad_spi2);			}					
				return ;
			}	

			
	//		for (i=conf.indicators[numb_ind-1].kol_cifr;i>0;i--)
			for (i=kol_cifr;i>0;i--)
			{	
					zn[i]=(u8) (chislo%10);
					chislo=chislo/10;
			}
			/*
			zn[0]=0;
			zn[1]=1;
			zn[2]=2;
			zn[3]=3;
			zn[4]=4;
			zn[5]=5;
*/

	//		for (i=1;i<conf.indicators[numb_ind-1].kol_cifr+1;i++)
			for (i=1;i<kol_cifr+1;i++)
			{	
						simb=symb_code[zn[i]];
				
		//				if ((conf.indicators[numb_ind-1].kol_cifr-i)==conf.indicators[numb_ind-1].pol_zap)
				if ((kol_cifr-i)==1)
							simb+=0x80;
				
						if ((simb==0x7E)&(null))
							simb=0;
						else
							null=0;
						
	//					if (i==conf.indicators[numb_ind-1].kol_cifr)
						if (i==kol_cifr)
							simb&=0x7F;
	
						GPIO_WriteBit(GPIOA, pin, Bit_RESET);     	delay_spi(zad_spi);
						spi_send((u8) i);delay_spi(zad_spi);
						spi_send(simb); delay_spi(zad_spi);			
				//		spi_send(symb_code[i]); delay_spi(zad_spi);	
								//  indicators
								//   12.3
						GPIO_WriteBit(GPIOA, pin, Bit_SET);    			delay_spi(zad_spi2);				
			}
}


void indicate_time(u8 numb_ind, u8 hh, u8 mm, u8 en)
{
		 	uint16_t  pin=0;
			u8 i=0;

/*
			switch (numb_ind)
			{
				case 0x00:  // CS0
					pin=GPIO_Pin_0;
					break;

				case 0x01:  // CS1
					pin=GPIO_Pin_1;
					break;

				case 0x02:  // CS2
					pin=GPIO_Pin_2;
					break;

				case 0x03:  // CS3
					pin=GPIO_Pin_3;
					break;

				case 0x04:  // CS4
					pin=GPIO_Pin_4;
					break;						
			}
	*/
	
			  pin=pin_ind(numb_ind);
		if (pin==0)
				return ;
				GPIO_WriteBit(GPIOA, pin, Bit_RESET);      //   
				delay_spi(100);
				spi_send(0x0f);delay_spi(100);
				spi_send(0x00); delay_spi(100);
				GPIO_WriteBit(GPIOA, pin, Bit_SET);      //  
				delay_spi(1000);
	/*	
			for (i=1;i<8;i++)
			{		
			GPIO_WriteBit(GPIOA, pin, Bit_RESET);   delay_spi(zad_spi);
			spi_send(i);														delay_spi(zad_spi);
			spi_send(symb_code[i]); 														delay_spi(zad_spi);
			GPIO_WriteBit(GPIOA, pin, Bit_SET);     delay_spi(zad_spi2);						
			}		
	*/	

			GPIO_WriteBit(GPIOA, pin, Bit_RESET);     delay_spi(zad_spi);
			spi_send(1);															delay_spi(zad_spi);
			if (hh/10==0)
				spi_send(0); 
			else
				spi_send(symb_code[hh/10]); 
			delay_spi(zad_spi);
			GPIO_WriteBit(GPIOA, pin, Bit_SET);       delay_spi(zad_spi2);
	
			GPIO_WriteBit(GPIOA, pin, Bit_RESET);     delay_spi(zad_spi);
			spi_send(2);															delay_spi(zad_spi);
			spi_send(0x80*en+symb_code[hh%10]); 			delay_spi(zad_spi);	
			GPIO_WriteBit(GPIOA, pin, Bit_SET);     	delay_spi(zad_spi2);
			
			GPIO_WriteBit(GPIOA, pin, Bit_RESET);     delay_spi(zad_spi);
			spi_send(3);															delay_spi(zad_spi);
			spi_send(1*en+symb_code_min[mm/10]); 			delay_spi(zad_spi);
			GPIO_WriteBit(GPIOA, pin, Bit_SET);    	  delay_spi(zad_spi2);

			GPIO_WriteBit(GPIOA, pin, Bit_RESET);     delay_spi(zad_spi);
			spi_send(4);															delay_spi(zad_spi);
			spi_send(symb_code_min[mm%10]); 					delay_spi(zad_spi);
			GPIO_WriteBit(GPIOA, pin, Bit_SET);       delay_spi(zad_spi2);
}


/**
  * @brief  Main program.
  * @param  None
  * @retval None
*/
int main(void)
{ 
	u8 i=0;
	GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  RCC_ClocksTypeDef RCC_ClockFreq;
	
	uint16_t data[32];
	
	TDateTime DT;


  /* Initialize LEDS */
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDInit(LED5);
  STM_EVAL_LEDInit(LED6);
 
  /* Green Led On: start of application */
	STM_EVAL_LEDOn(LED3);
	STM_EVAL_LEDOn(LED4);
	STM_EVAL_LEDOn(LED5);
	STM_EVAL_LEDOn(LED6);	
	 
  /* This function fills the RCC_ClockFreq structure with the current
  frequencies of different on chip clocks (for debug purpose) **************/
  RCC_GetClocksFreq(&RCC_ClockFreq);
  
  /* Enable Clock Security System(CSS): this will generate an NMI exception
  when HSE clock fails *****************************************************/
  RCC_ClockSecuritySystemCmd(ENABLE);
  
  /* Enable and configure RCC global IRQ channel, will be used to manage HSE ready 
     and PLL ready interrupts. 
     These interrupts are enabled in stm32f4xx_it.c file **********************/
  NVIC_InitStructure.NVIC_IRQChannel = RCC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);


 // RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	delay_spi(10000);
	/*
	RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//	port A
	RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//	port B
	RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//	port C		
	RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//	port E
  RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//	port E
*/
  

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//	port A
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//	port B
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//	port C		
//	RCC_AHB2PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//	port E
 // RCC_AHB2PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//	port E
	
  /* Output clock on MCO2 pin(PC9) ****************************************/ 
  /* Enable the GPIOC peripheral */ 
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	

  /* Configure MCO2 pin(PC9) in alternate function */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_Init(GPIOC, &GPIO_InitStructure);
    
  /* System clock selected to output on MCO2 pin(PC9)*/
  RCC_MCO2Config(RCC_MCO2Source_SYSCLK, RCC_MCO2Div_2);



    // ???????? RTC
    rtc_Reset();
    rtc_Init();
			
		init_dc();

		
//	ADC_InitTypeDef  ADC_InitStructure;
	
       
  
  /* Configure TIM4 Peripheral to manage LEDs lighting */
  TIM_LED_Config();
  
  /* Initialize the repeat status */
  RepeatState = 0;
  LED_Toggle = 7;
  
	// PC pin2
  ADC3_CH12_DMA_Config();
  // Start ADC3 Software Conversion 
  ADC_SoftwareStartConv(ADC3);
	
	
  
#if defined MEDIA_USB_KEY
  
  /* Initialize User Button */
//  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
   
  /* Init Host Library */
  USBH_Init(&USB_OTG_Core, USB_OTG_FS_CORE_ID, &USB_Host, &USBH_MSC_cb, &USR_Callbacks);
 


/*
SPI 1:

 nss - pa4
 
 sck - pa5
 mosi - pa7
 
 miso - pa6
SPI 2:

 miso - pb14
 mosi - pb15
 sck - pb13
 nss - pb12
 
*/
		spi1_init();
    spi2_init();
 
		
	
//		RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//port A
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;  //  vivod for CS 
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;     			// rezim vivoda
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;          //  PP GPIO_OType_PP
		// gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     //speed
		GPIO_Init(GPIOA, &GPIO_InitStructure); 
		

		
		/*
	//	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;  //  vivod for RELE and svet AVARIYA 

		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;  //  vivod for RELE and svet AVARIYA 
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;     			// rezim vivoda
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //GPIO_OType_OD;          //  PP GPIO_OType_PP
		// gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     //speed
		GPIO_Init(GPIOA, &GPIO_InitStructure); 
*/

		GPIO_InitStructure.GPIO_Pin   = PIN_RELE;  							//  vivod for RELE and svet AVARIYA 
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;     				// 	rezim vivoda
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 						//	GPIO_OType_OD;          //  PP GPIO_OType_PP
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     			//	speed
		GPIO_Init(PORT_RELE, &GPIO_InitStructure); 
		
		
		GPIO_InitStructure.GPIO_Pin   = PIN_PER_NIZ;  							//  vivod for RELE and svet AVARIYA 
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;     				// 	rezim vivoda
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 						//	GPIO_OType_OD;          //  PP GPIO_OType_PP
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     			//	speed
		GPIO_Init(PORT_PER_NIZ, &GPIO_InitStructure); 		

		GPIO_InitStructure.GPIO_Pin   = PIN_PER_VERH;  							//  vivod for RELE and svet AVARIYA 
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;     				// 	rezim vivoda
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 						//	GPIO_OType_OD;          //  PP GPIO_OType_PP
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     			//	speed
		GPIO_Init(PORT_PER_VERH, &GPIO_InitStructure); 
		
		GPIO_InitStructure.GPIO_Pin   = PIN_ZAP_EN;  							//  vivod for RELE and svet AVARIYA 
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;     				// 	rezim vivoda
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 						//	GPIO_OType_OD;          //  PP GPIO_OType_PP
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     			//	speed
		GPIO_Init(PORT_ZAP_EN, &GPIO_InitStructure); 
	
		GPIO_InitStructure.GPIO_Pin   = PIN_ZAP_DIS;  							//  vivod for RELE and svet AVARIYA 
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;     				// 	rezim vivoda
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 						//	GPIO_OType_OD;          //  PP GPIO_OType_PP
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     			//	speed
		GPIO_Init(PORT_ZAP_DIS, &GPIO_InitStructure);	

		GPIO_InitStructure.GPIO_Pin   = PIN_AVARIYA;  							//  vivod for RELE and svet AVARIYA 
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;     				// 	rezim vivoda
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 						//	GPIO_OType_OD;          //  PP GPIO_OType_PP
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     			//	speed
		GPIO_Init(PORT_AVARIYA, &GPIO_InitStructure);


		GPIO_InitStructure.GPIO_Pin   = PIN_PRIBL;  						// 	vvod for datchika pribl
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN; 					//	rezim vvoda
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     	//	speed
		GPIO_Init(PORT_PRIBL, &GPIO_InitStructure); 	
	
	GPIO_InitStructure.GPIO_Pin   = PIN_L1;      		//  vivod svetodiod knopka 1
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;    // rezim vivoda
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //speed
	GPIO_Init(PORT_L1, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin   = PIN_L2;      		  //  vivod svetodiod knopka 2
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;    // rezim vivoda
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //speed
	GPIO_Init(PORT_L2, &GPIO_InitStructure); 
		
/*
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;  						// 	vvod for knopka
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN; 					//GPIO_Mode_OUT; //GPIO_Mode_IN;     			// 	rezim vivoda
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     		//	speed
		GPIO_Init(GPIOC, &GPIO_InitStructure); 	
*/
/*
		RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//	port C
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;  						// 	vvod for knopka
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN; //GPIO_Mode_IN;     			// 	rezim vivoda
	//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; //GPIO_OType_PP; 					//	GPIO_OType_OD;          //  PP GPIO_OType_PP
		// gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
//		GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL; //GPIO_PuPd_UP; //GPIO_PuPd_DOWN;  //GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     		//	speed
		GPIO_Init(GPIOC, &GPIO_InitStructure); 
	*/	
		/*
	  GPIO_PuPd_NOPULL = 0x00,
  GPIO_PuPd_UP     = 0x01,
  GPIO_PuPd_DOWN   = 0x02
*/

//		PIN_PRIBL
//		Port_PRIBL
		
/*
		RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//port A
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;  //  vivod and RELE
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;     			// rezim vivoda
	//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;          //  PP GPIO_OType_PP
		GPIO_InitStructure.GPIO_OType = GPIO_PuPd_DOWN;          //  PP GPIO_OType_PP
		
		// gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     //speed
		GPIO_Init(GPIOA, &GPIO_InitStructure); 
*/
/*
	conf.indicators[0].numb=1;
	conf.indicators[0].kol_cifr=8;
	conf.indicators[0].type_ind=0;
	conf.indicators[0].yark=0x0f;
	conf.indicators[0].rez_viv=0;   //   
	conf.indicators[0].chislo=0;
	conf.indicators[0].pol_zap=0;
	conf.indicators[0].porog=0xffff;	

// lineika
	conf.indicators[1].numb=2;
	conf.indicators[1].kol_cifr=4;
	conf.indicators[1].type_ind=0;
	conf.indicators[1].yark=0x0f;
	conf.indicators[1].rez_viv=0;   //   
	conf.indicators[1].chislo=0;
	conf.indicators[1].pol_zap=0;
	conf.indicators[1].porog=0xffff;	

	conf.indicators[2].numb=3;
	conf.indicators[2].kol_cifr=8;
	conf.indicators[2].type_ind=0;
	conf.indicators[2].yark=0x0f;
	conf.indicators[2].rez_viv=0;   //   
	conf.indicators[2].chislo=0;
	conf.indicators[2].pol_zap=0;
	conf.indicators[2].porog=0xffff;	

// time
	conf.indicators[3].numb=4;
	conf.indicators[3].kol_cifr=4;
	conf.indicators[3].type_ind=0;
	conf.indicators[3].yark=0x0f;
	conf.indicators[3].rez_viv=0;   //   
	conf.indicators[3].chislo=0;
	conf.indicators[3].pol_zap=0;
	conf.indicators[3].porog=0xffff;	
//test_lin2();
 
//	init_ind(indicators[0].numb, indicators[0].kol_cifr, indicators[0].type_ind);
//  init_ind(indicators[1].numb, indicators[1].kol_cifr, indicators[1].type_ind);

	init_ind(conf.indicators[0].numb, conf.indicators[0].kol_cifr, conf.indicators[0].type_ind);
  init_ind(conf.indicators[1].numb, conf.indicators[1].kol_cifr, conf.indicators[1].type_ind);
	init_ind(conf.indicators[2].numb, conf.indicators[2].kol_cifr, conf.indicators[2].type_ind);
  init_ind(conf.indicators[3].numb, conf.indicators[3].kol_cifr, conf.indicators[3].type_ind);
	*/
		
	test_ind_all(1);
	
	MCO(1);
	delay_spi(100);
	MDO(1);
	
	for (i = 0; i < 100; i ++)
	{ //	delay_spi(500000);
		MCO(0);
		delay_spi(100000);
		MCO(1);		
	}
		
	test_ind_all(0);
	init_ind(1, 4, 0);		// lineika
  init_ind(2, 8, 0);   
	init_ind(3, 8, 0);
  init_ind(4, 4, 0);   // time
	
//	init_I2C1(); // initialize I2C peripheral
	
//	write_dat_clock();	
	


// nastroika gpio

//	GPIO_InitTypeDef  GPIO_InitStructure;                 
//	RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); //port D
	GPIO_InitStructure.GPIO_Pin   = tx_pin_en|rx_pin_en;      //  vivod for control mod-rs485
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;     // rezim vivoda
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //speed
	GPIO_Init(GPIOD, &GPIO_InitStructure);              

	UART2Init();
	
	/*
	// nastroika na pereda4y
	GPIO_WriteBit(GPIOD, tx_pin_en, Bit_SET);      // GPIOD 4
	GPIO_WriteBit(GPIOD, rx_pin_en, Bit_SET);    //   GPIOD 3
	GPIOD->ODR ^= tx_pin_en;
*/
	// nastroika na priem
	GPIO_WriteBit(GPIOD, tx_pin_en, Bit_SET);      // GPIOD 4
	GPIO_WriteBit(GPIOD, rx_pin_en, Bit_SET);    //   GPIOD 3
	GPIOD->ODR ^= rx_pin_en;

							GPIO_WriteBit(GPIOD, rx_pin_en, Bit_RESET); 
			//				USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; //?????
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //?????????
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//????????? ?????????
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //???????? ?????
NVIC_Init(&NVIC_InitStructure); //??????????????


// pd5 rxd  in wleif 3 contakt
// pd6 txd           4
// pd3 enabl         10
USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);  //???????? ?????? ?????????? ??? ??? ?????
USART_ITConfig(USART2, USART_IT_TC, ENABLE);  //???????? ?????? ?????????? ??? ??? ?????


	NVIC_EnableIRQ (USART2_IRQn); // ????????? ?????????? ?? USART1
	/*
  NVIC_EnableIRQ (ADC1_IRQn); // ????????? ?????????? ?? ???
  NVIC_DisableIRQ (USART1_IRQn); // ????????? ???????
  NVIC_DisableIRQ (ADC1_IRQn); // ??? ??? ????????? ?????????? ????
*/
//	SendStringUSART2("start ver 1.0 /0"); 
/*
	txsize=10;
	tekper=0;
	for (i = 0; i < txsize ; i++)
  {
		TxBuffer[i]=0x30+i;
	}
	USART_SendData(USART2, 0x3A);
*/	

	STM_EVAL_LEDOff(LED3);
	STM_EVAL_LEDOff(LED4);
	STM_EVAL_LEDOff(LED5);
	STM_EVAL_LEDOff(LED6);
	
  /* SysTick end of count event each 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);
	

	write_dat_clock();
	

	
			
		
	while (1)
	{
		MCO(1);
		sleep(10000);
		MCO(0);
		sleep(10000);
	}		
			
  while (1)
  {
    /* Host Task handler */
    USBH_Process(&USB_OTG_Core, &USB_Host);
  }
  
#endif
	

  
}

/**
  * @brief  Configures the TIM Peripheral for Led toggling.
  * @param  None
  * @retval None
  */
static void TIM_LED_Config(void)
{
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  uint16_t prescalervalue = 0;
  
  /* TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* Enable the TIM3 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Initialize Leds mounted on STM324F4-EVAL board */
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDInit(LED6);
  
  /* Compute the prescaler value */
  prescalervalue = (uint16_t) ((SystemCoreClock ) / 550000) - 1;
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = prescalervalue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  /* Enable TIM4 Preload register on ARR */
  TIM_ARRPreloadConfig(TIM4, ENABLE);
  
  /* TIM PWM1 Mode configuration: Channel */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  
  /* Output Compare PWM1 Mode configuration: Channel2 */
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Disable);
    
  /* TIM Interrupts enable */
  TIM_ITConfig(TIM4, TIM_IT_CC1 , ENABLE);
  
  /* TIM4 enable counter */
  TIM_Cmd(TIM4, ENABLE);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

  
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
