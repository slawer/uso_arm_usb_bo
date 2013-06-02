
#include "rtc.h"

// my add for usb write
#define numbkor 10
#define vrem_tm 100

#define	kpl_simb_in_stroka	7


#define tx_pin_en	GPIO_Pin_4
#define rx_pin_en	GPIO_Pin_3

	#define RxBufferSize        ((u8)255)  // USART2 global Interrupt 
	#define TxBufferSize        ((u8)255)  // USART2 global Interrupt 
		
	
extern	u8 txsize, rxsize;
	
extern		u8  RxBuffer[RxBufferSize];
extern		u8  TxBuffer[RxBufferSize];
extern		u8 tekper,tekpr;

typedef struct
{
  uint8_t adr;      
  uint8_t func; 
  uint16_t reg;       
  uint16_t kol;  
  uint16_t crc;             
} zap;

extern u32 tick;

extern zap kor[numbkor];


extern TDateTime DT_zap; 
extern BOOL number_buff;
extern uint16_t Buf_adc_zap1 [1000];
extern uint16_t Buf_adc_zap2 [1000];
extern uint8_t Buf_zap [6000];
extern uint16_t time_label;
extern u16 por;


extern 	 uint8_t counter;
extern 	uint16_t bytesWritten;
	
extern 	u8 tmp1, tmp2,tmp3,tmp4, sm;
extern 	u16 cnt, kol_zap;

extern u8 file_cr;

extern u8 buffering;
extern u16 del;
extern u16 minute;
extern u16 pred_minute;

extern u16 minute;

extern u16 pred_minute;

extern u16 average[10],summa[10], fz[10], fz_average[10], max[10];

extern u8 kol_average;


typedef struct 
{
u16 kod[10];
float fz[10];
} st_tab_kal;

extern st_tab_kal tab_kal;


extern u8 address;

extern u8 new_komand;