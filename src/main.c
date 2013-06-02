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

    	
				if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)		  // �������
					{		u8 tmp=0;
						USART_ClearITPendingBit(USART2, USART_IT_RXNE);
				
						tmp=USART_ReceiveData (USART2);
						if (tmp==0x3A)
								tekpr=0;
	
						if (tekpr==1)
								if (tmp!=address)
								{
									tekpr=0;
									rxsize=0;
								}
						
						if (tmp==0x0D)
						{
							rxsize=tekpr;
							USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
							new_komand=1;
						}
						RxBuffer[tekpr]=tmp;
						tekpr++;
				//		GPIOD->ODR ^= tx_pin_en;
				//		GPIOD->ODR ^= rx_pin_en;
						

					}
//Transmission complete interrupt								 // ����� ��������
        if(USART_GetITStatus(USART2, USART_IT_TC) != RESET)
        {
		
					USART_ClearITPendingBit(USART2, USART_IT_TC);//������� ������� ����������
					
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
					
			 
					//					GPIOA->BSRR=GPIO_BSRR_BR11|GPIO_BSRR_BR12; // ����������� �� �����
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
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
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
  ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_3Cycles);

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
	
	TDateTime DT;
    
	 
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

  /* Output clock on MCO2 pin(PC9) ****************************************/ 
  /* Enable the GPIOC peripheral */ 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  
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
 		
	
//	ADC_InitTypeDef  ADC_InitStructure;
	
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
       
  /* SysTick end of count event each 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);
  
  /* Configure TIM4 Peripheral to manage LEDs lighting */
  TIM_LED_Config();
  
  /* Initialize the repeat status */
  RepeatState = 0;
  LED_Toggle = 7;
  
	
  ADC3_CH12_DMA_Config();
  // Start ADC3 Software Conversion 
  ADC_SoftwareStartConv(ADC3);
	
	
  
#if defined MEDIA_USB_KEY
  
  /* Initialize User Button */
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
   
  /* Init Host Library */
  USBH_Init(&USB_OTG_Core, USB_OTG_FS_CORE_ID, &USB_Host, &USBH_MSC_cb, &USR_Callbacks);
  
	STM_EVAL_LEDOff(LED3);
	STM_EVAL_LEDOff(LED4);
	STM_EVAL_LEDOff(LED5);
	STM_EVAL_LEDOff(LED6);


// nastroika gpio

//	GPIO_InitTypeDef  GPIO_InitStructure;                 
	RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); //port D
	GPIO_InitStructure.GPIO_Pin   = tx_pin_en|rx_pin_en;      //  vivod for control mod-rs485
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;     // rezim vivoda
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //speed
	GPIO_Init(GPIOD, &GPIO_InitStructure);              

	UART2Init();
	
	GPIO_WriteBit(GPIOD, tx_pin_en, Bit_SET);      // GPIOD 4
	GPIO_WriteBit(GPIOD, rx_pin_en, Bit_SET);    //   GPIOD 3
	GPIOD->ODR ^= tx_pin_en;

	

NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; //?????
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //?????????
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//????????? ?????????
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //???????? ?????
NVIC_Init(&NVIC_InitStructure); //??????????????
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
