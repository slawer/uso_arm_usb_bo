


#include "rtc.h"


// my add for usb write
#define tx_pin_en	GPIO_Pin_4
#define rx_pin_en	GPIO_Pin_3

	#define RxBufferSize        ((u8)255)  // USART2 global Interrupt 
	#define TxBufferSize        ((u8)255)  // USART2 global Interrupt 
	
	u8 txsize, rxsize;
	
	u8  RxBuffer[RxBufferSize];
	u8  TxBuffer[RxBufferSize];
	u8 tekper,tekpr;

u32 tick=0;
#define vrem_tm 100
zap kor[numbkor];

TDateTime DT_zap; 
BOOL number_buff=0;

uint16_t Buf_adc_zap1 [1000];
uint16_t Buf_adc_zap2 [1000];
uint8_t Buf_zap [6000];
uint16_t time_label=0;
u16 por=0;


uint8_t counter=0;
uint16_t bytesWritten;
	
u8 tmp1=0, tmp2=0,tmp3=0,tmp4=0, sm=0;
u16 cnt=0, kol_zap=0;

u8 file_cr=0;

u8 buffering=0;

u16 del=0;

u16 minute=0;

u16 pred_minute=0;

u16 average[10],summa[10], fz[10], fz_average[10], max[10];

u8 kol_average=0;

u8 new_komand=0;

u8 address=1;



