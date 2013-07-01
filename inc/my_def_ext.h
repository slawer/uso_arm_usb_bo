


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


#define PIN_PRIBL                         GPIO_Pin_1
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

u16 kol_pribl_vikl=0;
u16 kol_pribl_vkl=0;

u16 kol_gr1_vkl=0;
u16 kol_gr2_vkl=0;

u8 avariya=0;
u8 sost_flesh=0;





/*------------------------------------------------------------------
ћакрос   FUM_RD_DAT_CLOCK прочитать значение RAM часов 
len-3;adcbuf[12]
------------------------------------------------------------------*/
/*
I2CM=1;
MCO=1; 
MDE=1;
MDO=1; //мастер I2c

scl		pb8
sda  pb7

I2CM=1;
MCO=1; 
MDE=1;
MDO=1; //мастер I2c

*/

#define ds_delay 100000
#define ds_wait  1000

#define PIN_DS_SCL                      	GPIO_Pin_8		//	scl		pb8
#define PORT_DS_SCL               			  GPIOB	

#define PIN_DS_SDA                      	GPIO_Pin_7		//	sda  pb7
#define PORT_DS_SDA                				GPIOB	

void init_dc()
{
	GPIO_InitTypeDef      GPIO_InitStructure;
	/*
		GPIO_InitStructure.GPIO_Pin   = PIN_RELE;  							//  vivod for RELE and svet AVARIYA 
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;     				// 	rezim vivoda
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 						//	GPIO_OType_OD;          //  PP GPIO_OType_PP
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     			//	speed
		GPIO_Init(PORT_RELE, &GPIO_InitStructure); 
	
		GPIO_InitStructure.GPIO_Pin   = PIN_RELE;  							//  vivod for RELE and svet AVARIYA 
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;     				// 	rezim vivoda
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 						//	GPIO_OType_OD;          //  PP GPIO_OType_PP
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     			//	speed
		GPIO_Init(PORT_RELE, &GPIO_InitStructure); 

	  GPIO_PuPd_NOPULL = 0x00,
  GPIO_PuPd_UP     = 0x01,
  GPIO_PuPd_DOWN   = 0x02
*/	
		GPIO_InitStructure.GPIO_Pin   = PIN_DS_SCL;  							//  vivod for RELE and svet AVARIYA 
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT; //GPIO_Mode_IN; // GPIO_Mode_OUT;     				// 	rezim vivoda
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //GPIO_OType_OD; //GPIO_OType_PP; 						//	GPIO_OType_OD;          //  PP GPIO_OType_PP
//		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;  
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     			//	speed
		GPIO_Init(PORT_DS_SCL, &GPIO_InitStructure); 
	
		GPIO_InitStructure.GPIO_Pin   = PIN_DS_SDA;  							//  vivod for RELE and svet AVARIYA 
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT; //GPIO_Mode_IN; //GPIO_Mode_OUT;     				// 	rezim vivoda
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //GPIO_OType_OD; //GPIO_OType_PP; 						//	GPIO_OType_OD;          //  PP GPIO_OType_PP
//		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;  
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     			//	speed
		GPIO_Init(PORT_DS_SDA, &GPIO_InitStructure); 
	
}

void sleep(u32 dlit)
{
	u32 i=0;
	for (i=0;i<dlit;i++)
				__ASM volatile ("nop");
}

u8 r_MDI(void)
{
		if ((PORT_DS_SDA->IDR & PIN_DS_SDA)==0)
			return 0;
		else
			return 1;
}

u8 r_MCO(void)
{
		if ((PORT_DS_SCL->IDR & PIN_DS_SCL)==0)
			return 0;
		else
			return 1;
}

void MDO(u8 sost)
{	u32 i=0, kol_wait=0;;
	if (sost==1)
	{
	    PORT_DS_SDA->BSRRL = PIN_DS_SDA;
			sleep(ds_delay);
			while ((kol_wait<ds_wait) &(r_MDI()==1))
				kol_wait++;
	}
	else
	{
    PORT_DS_SDA->BSRRH = PIN_DS_SDA ;	
		sleep(ds_delay);
		while ((kol_wait<ds_wait) &(r_MDI()==1))
			kol_wait++;
	}
}

void MCO(u8 sost)
{	u32 i=0, kol_wait=0;
	if (sost==1)
	{
		PORT_DS_SCL->BSRRL = PIN_DS_SCL;
		sleep(ds_delay);
		while ((kol_wait<ds_wait) &(r_MCO()==1))
			kol_wait++;
	}
	else
	{
		PORT_DS_SCL->BSRRH = PIN_DS_SCL;
		sleep(ds_delay);		
		while ((kol_wait<ds_wait) &(r_MCO()==0))
			kol_wait++;
	}
}


u8 b_err_cl=0, bufout[100], zbuf[100];


void start_ds(void)
{
		MDO(0); 					 
		MCO(0); 
}

void stop_ds(void)
{	
		MDO(0);									  
		MCO(1);	
		MDO(1); 
}

void write_bait_ds(u8 bait)
{		u8 i1=0;                     	 
		for(i1=0;i1<8;i1++) 												  
		{ 
			if((bait&0x80)==0)	
				MDO(0);
			else
				MDO(1); 					  					   
			MCO(1);
			MCO(0);	                                                 
			bait= bait<<1; 
		} 		
}

u8 read_bait_ds()
{
			u8 tmp=0;
				MDO(1); 
				MCO(1);    
				if (r_MDI() ==0) 																									    
				{tmp = tmp<<1; tmp = tmp&0xFE;} 															    
				else 																											    
				{tmp = tmp<<1; tmp = tmp|0x01;}                          					 
				MCO(0); 
		return tmp;
}

u8 ack_ds()
{		u8 ret=0;
		MDO(1);
		MCO(1);                
		ret=r_MDI();			   
    MCO(0);
	return ret;
}

void read_dat_clock(void)
{		u8 tmp_rw=0, DL=0, j=0,i1=0;
		
		for(i1=0;i1<100;i1++) 
			zbuf[i1]=0;

		for(i1=0;i1<100;i1++) 
			bufout[i1]=0;
	
		start_ds();
		write_bait_ds(0xD1);
		ack_ds();

		for(j=0;j<50;j++)																									 
		{   							  
			bufout[j]=read_bait_ds();			
			if (j!=(50-1)) 
				ack_ds();				
		}
		stop_ds();
	}					
	
	
void write_dat_clock(void)
{
		u8 i1=0, j=0, DL=0;

		for(i1=0;i1<100;i1++) 
			zbuf[i1]=i1; 
				
	
		zbuf[7]=0xD0;
		zbuf[8]=0x14;

		MDO(0); 	sleep(ds_delay);
		MCO(0); 	sleep(ds_delay);
		for(j=7;j<17;j++) 
		{   
			for(i1=0;i1<8;i1++) 
			{ 
				if((zbuf[j]&0x80)==0) 				
					 MDO(0);  
				else  
  				 MDO(1);  
				sleep(ds_delay); 
				MCO(1);	sleep(ds_delay);
				MCO(0);	sleep(ds_delay);                    
				if (i1!=7) { zbuf[j] = zbuf[j]<<1;} 
				MDO(0); 	sleep(ds_delay);
			}                 
			 if (j!=16)
			 {
			 sleep(ds_delay);     
			 MDO(1); 	sleep(ds_delay);
			 MCO(1); 	sleep(ds_delay);
			 if (r_MCO()==1) b_err_cl++;                       
			 MCO(0);  sleep(ds_delay);
			 }
		 }                          
			sleep(ds_delay);
			MCO(1);		sleep(ds_delay);
			MDO(1);   sleep(ds_delay); 		
}









