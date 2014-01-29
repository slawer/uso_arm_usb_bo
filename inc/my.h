
#include "rtc.h"

// my add for usb write
#define numbkor 10
#define vrem_tm 100

#define	kol_simb_in_stroka	8 // 7 - without point

#define	kol_simb_in_stroka_new	29 // 7 - without point



#define tx_pin_en	GPIO_Pin_4
#define rx_pin_en	GPIO_Pin_3

	#define RxBufferSize        ((u16)500)  // USART2 global Interrupt 
	#define TxBufferSize        ((u16)500)  // USART2 global Interrupt 

#define PIN_Conrtol		GPIO_Pin_4
#define PORT_Conrtol	GPIOD
	
extern	u16 txsize, rxsize;
	
extern		u8  RxBuffer[RxBufferSize];
extern		u8  TxBuffer[RxBufferSize];
extern		u16 tekper,tekpr;

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


extern TDateTime DT_zap, DT_zap_pr; 
extern bool number_buff;

extern uint16_t Buf_adc_zap1 [1000];
extern uint16_t Buf_adc_zap2 [1000];

extern uint16_t Buf_adc_zap1_dmk [700];
extern uint16_t Buf_adc_zap2_dmk [700];

extern uint16_t Buf_max_zap1 [700];
extern uint16_t Buf_max_zap2 [700];

extern uint16_t Buf_max_zap1_dmk [700];
extern uint16_t Buf_max_zap2_dmk [700];


extern uint8_t Buf_zap [20000];
extern uint16_t time_label;
extern u16 por;


extern 	uint8_t counter;
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

extern u8 sost_pribl;


typedef struct 
{
  u16 kod[10];
//float fz[10];
	u16 fz[10];
} st_tab_kal;

extern st_tab_kal tab_kal;


extern u8 address;

extern u8 new_komand, new_group;




typedef struct 
{
		u8 numb;
		u8	kol_cifr;
		u8	type_ind;
		u8	yark;
		u8	rez_viv;   // 0 -blank   1 - norm  2 - migaet
		u8	pol_zap;
		u8 r1;
	  u8 r2;
		u16	chislo;
		u16	porog;
	
} st_indikators;

//st_indikators indicators[10];

typedef struct 
{
		st_tab_kal tabl1;
		st_tab_kal tabl2;	
}  gr_kal;

typedef struct 
{
		u8 kol_st;
		u8 rez;
		u16 max1;	
		u16 max2;
		u16 max3;
		u16 max4;
}  st_lin;

typedef struct 
{
		u8 address;
		u8 ver_po_st;
		u8 ver_po_ml;
	  u8 tek_gr_kal; 
	  u8 tm_antidreb;
		u8 revers_group_select;
		u8 revers_peredacha_select;
		u8 rez8;
	
		u16	per_usr;
		u16	time_max;
		u16 por_rele;
		u16 tm_rele_on;
		u16 tm_rele_off;
		u16 rez16;
	
//	  st_indikators indicators[4];	
		st_lin lin;
	
		gr_kal gr_kal1;
		gr_kal gr_kal2;
	
	/*
		u16	per_usr_dmk;
		u16	time_max_dmk;
		u16 por_rele_dmk;
		u16 tm_rele_on_dmk;
		u16 tm_rele_off_dmk;
		
		u8 revers_switch;
		u8 revers_menu;
		u16 lin_max_dmk;
		
		st_tab_kal kal_dmk;
		*/

} st_conf;


typedef struct 
{
	  u8 tm_antidreb;
		u8 rez8;
	
		u16	per_usr;
		u16	time_max;
		u16 por_rele;
		u16 tm_rele_on;
		u16 tm_rele_off;
		u16 rez16;

		
		u8 revers_switch;
		u8 revers_menu;
		u16 lin_max;
		
		st_tab_kal kal;

} st_conf_dmk;


extern u16 tek_kol;
extern u16 kol_usr;
extern u32 buf_sum;

extern u16 tek_kol_dmk;
extern u16 kol_usr_dmk;
extern u32 buf_sum_dmk;

#define PIN_PRIBL                         GPIO_Pin_5
#define PORT_PRIBL                    		GPIOC

#define PIN_RELE                         	GPIO_Pin_4
#define PORT_RELE                    			GPIOA

#define PIN_RELE_DMK                      GPIO_Pin_6
#define PORT_RELE_DMK                    	GPIOE

#define PIN_K1                         		GPIO_Pin_7		// BTN1	rezim 1
#define PORT_K1                 					GPIOC

#define PIN_L1                         		GPIO_Pin_6		// LED1   VD1    rezim 1
#define PORT_L1                 					GPIOC

#define PIN_K2                         		GPIO_Pin_9		//BTN2   rezim 2
#define PORT_K2                 					GPIOC

#define PIN_L2                         		GPIO_Pin_8		// LED2    VD2		rezim 2
#define PORT_L2                 					GPIOC

#define PIN_PER_NIZ                       GPIO_Pin_3		//  L3
#define PORT_PER_NIZ                			GPIOE

#define PIN_PER_VERH                      GPIO_Pin_8		//	L4
#define PORT_PER_VERH                			GPIOA

#define PIN_ZAP_EN                      	GPIO_Pin_4		//	L5
#define PORT_ZAP_EN              				  GPIOE

#define PIN_ZAP_DIS                      	GPIO_Pin_5  	//	L6
#define PORT_ZAP_DIS              				GPIOE

#define PIN_AVARIYA                      	GPIO_Pin_15		//	L7
#define PORT_AVARIYA                			GPIOA	
 
#define PIN_BUTTON_MENU                  	GPIO_Pin_7		//	BTN3  menyu
#define PORT_BUTTON_MENU             			GPIOE	

#define PIN_SW_KEY        	             	GPIO_Pin_11		//	SW1
#define PORT_SW_KEY         	       			GPIOC	

extern u16 kol_pribl_vikl;
extern u16 kol_pribl_vkl;

extern u16 kol_gr1_vkl;
extern u16 kol_gr2_vkl;

extern u8 avariya;
extern u8 avariya_dmk;
extern u8 avariya_both;
extern u8 sost_flesh;

extern		u8 tek,lin;


