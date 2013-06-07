
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
extern bool number_buff;
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
//float fz[10];
	u16 fz[10];
} st_tab_kal;

extern st_tab_kal tab_kal;


extern u8 address;

extern u8 new_komand;




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
	
	  st_indikators indicators[4];	
	
		gr_kal gr_kal1;
		gr_kal gr_kal2;

} st_conf;