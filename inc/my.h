

// my add for usb write
#define numbkor 10
#define vrem_tm 100

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


extern uint16_t Buf_adc_zap [100];
extern uint8_t Buf_zap [400];
extern uint16_t time_label;
extern u8 por;


extern 	 uint8_t counter;
extern 	uint16_t bytesWritten;
	
extern 	u8 tmp1, tmp2,tmp3,tmp4, sm, kol_zap;
extern 	u16 cnt;

extern u8 file_cr;

extern u8 buffering;
extern u16 del;
extern u16 minute;
extern u16 pred_minute;

extern u16 minute;

extern u16 pred_minute;

extern u16 average[10],summa[10], fz[10], fz_average[10], max[10];

extern u8 kol_average;
