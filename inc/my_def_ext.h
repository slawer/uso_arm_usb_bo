


#include "rtc.h"


// my add for usb write
#define tx_pin_en	GPIO_Pin_4
#define rx_pin_en	GPIO_Pin_3

	#define RxBufferSize        ((u16)500)  // USART2 global Interrupt 
	#define TxBufferSize        ((u16)500)  // USART2 global Interrupt 
	
	u16 txsize, rxsize;
	
	u8  RxBuffer[RxBufferSize];
	u8  TxBuffer[TxBufferSize];
	u16 tekper,tekpr;

u32 tick=0;
#define vrem_tm 100
//zap kor[numbkor];

TDateTime DT_zap; 
bool number_buff=0;

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
u8 sost_pribl=0;

u8 kol_average=0;

u8 new_komand=0;

u8 address=1;

uint16_t SPI1_Buffer_Tx[32] = {0x0102, 0x0304, 0x0506, 0x0708, 0x090A, 0x0B0C,
                                  0x0D0E, 0x0F10, 0x1112, 0x1314, 0x1516, 0x1718,
                                  0x191A, 0x1B1C, 0x1D1E, 0x1F20, 0x2122, 0x2324,
                                  0x2526, 0x2728, 0x292A, 0x2B2C, 0x2D2E, 0x2F30,
                                  0x3132, 0x3334, 0x3536, 0x3738, 0x393A, 0x3B3C,
                                  0x3D3E, 0x3F40};

			u8 smes=0;
			u32 pr_tick=0;


u8 symb_code[13]={0x7E,0x30,0x6D,0x79,0x33,0x5B,0x5F,0x70,0x7F,0x7B,0x4F, 0x79,0x80};
//  . 0x80					0 1 2 3 4 5 6 7 8 9 E ~E .

u8 symb_code_min[13]={0x7E,0x30,0xEC,0xf8,0xB2,0xDA,0xDE,0x70,0xFE,0xFA,0xCE, 0xf8,0x01};


st_conf conf;

u16 tek_kol=0;
u16 kol_usr=0;
u32 buf_sum=0;


#define PIN_PRIBL                         GPIO_Pin_0
#define PORT_PRIBL                    		GPIOC

#define PIN_RELE                         	GPIO_Pin_4
#define PORT_RELE                    			GPIOA

#define PIN_K1                         		GPIO_Pin_7
#define PORT_K1                 					GPIOC
#define PIN_L1                         		GPIO_Pin_6
#define PORT_L1                 					GPIOC

#define PIN_K2                         		GPIO_Pin_9
#define PORT_K2                 					GPIOC
#define PIN_L2                         		GPIO_Pin_8
#define PORT_L2                 					GPIOC

u16 kol_pribl_vikl=0;
u16 kol_pribl_vkl=0;

u8 avariya=0;
u8 sost_flesh=0;













