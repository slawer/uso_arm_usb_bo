

// my add for usb write
#define numbkor 10

u32 tick=0;
#define vrem_tm 100
zap kor[numbkor];


uint16_t Buf_adc_zap [100];
uint8_t Buf_zap [400];
uint16_t time_label=0;
u8 por=0;


uint8_t counter=0;
uint16_t bytesWritten;
	
u8 tmp1=0, tmp2=0,tmp3=0,tmp4=0, sm=0, kol_zap=0;
u16 cnt=0;

u8 file_cr=0;

u8 buffering=1;

u16 del=0;

u16 minute=0;

u16 pred_minute=0;

u16 average[10],summa[10], fz[10], fz_average[10], max[10];

u8 kol_average=0;


