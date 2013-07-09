


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

u8 new_komand=0, new_group=0;

u8 address=1;

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

u8 error_ds=0;


#define PIN_Conrtol		GPIO_Pin_4
#define PORT_Conrtol	GPIOD

void init_control(void)
{
		GPIO_InitTypeDef      GPIO_InitStructure;
	
		GPIO_InitStructure.GPIO_Pin   = PIN_Conrtol;  							//  vivod for RELE and svet AVARIYA 
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT; //GPIO_Mode_IN; // GPIO_Mode_OUT;     				// 	rezim vivoda
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //GPIO_OType_OD; //GPIO_OType_PP; 						//	GPIO_OType_OD;          //  PP GPIO_OType_PP
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  //GPIO_PuPd_UP; // GPIO_PuPd_DOWN  GPIO_PuPd_NOPULL
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     			//	speed
		GPIO_Init(PORT_Conrtol, &GPIO_InitStructure); 	
}

/*
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;         // set output to open drain --> the line has to be only pulled low, not driven high
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; 
*/


/*------------------------------------------------------------------
Макрос   FUM_RD_DAT_CLOCK прочитать значение RAM часов 
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

#define ds_delay 3
#define ds_wait  2

#define PIN_DS_SCL                      	GPIO_Pin_8		//	scl		pb8
#define PORT_DS_SCL               			  GPIOB	

#define PIN_DS_SDA                      	GPIO_Pin_7		//	sda  pb7
#define PORT_DS_SDA                				GPIOB	


#define mode	GPIO_Mode_OUT
#define type	GPIO_OType_OD
#define pull	GPIO_PuPd_NOPULL





//	GPIO_Mode_IN; 		GPIO_Mode_OUT;   GPIO_Mode_AN
// 	GPIO_OType_OD; 		GPIO_OType_PP;
//  GPIO_PuPd_NOPULL	GPIO_PuPd_UP   	 GPIO_PuPd_DOWN   
/*
#define mode	GPIO_Mode_IN
#define type	GPIO_OType_OD
#define pull	GPIO_PuPd_NOPULL
flwaus ff

#define mode	GPIO_Mode_IN
#define type	GPIO_OType_OD
#define pull	GPIO_PuPd_UP
alwaus ff

#define mode	GPIO_Mode_IN
#define type	GPIO_OType_OD
#define pull	GPIO_PuPd_DOWN

alwaus ff



#define mode	GPIO_Mode_IN
#define type	GPIO_OType_PP
#define pull	GPIO_PuPd_NOPULL



#define mode	GPIO_Mode_IN
#define type	GPIO_OType_PP
#define pull	GPIO_PuPd_UP
alwaus ff
4 V


#define mode	GPIO_Mode_IN
#define type	GPIO_OType_PP
#define pull	GPIO_PuPd_DOWN
alwaus ff


#define mode	GPIO_Mode_OUT
#define type	GPIO_OType_OD
#define pull	GPIO_PuPd_NOPULL
read alwaus differrent data 
with errore
5v


#define mode	GPIO_Mode_OUT
#define type	GPIO_OType_OD
#define pull	GPIO_PuPd_UP
4v
read alwaus differrent data 
with errore


#define mode	GPIO_Mode_OUT
#define type	GPIO_OType_OD
#define pull	GPIO_PuPd_DOWN
read alwaus differrent data 
with errore 
4v



#define mode	GPIO_Mode_OUT
#define type	GPIO_OType_PP
#define pull	GPIO_PuPd_NOPULL
read alwaus differrent data      ??? only ff!!!
with errore 
but scl and data есть
3v 



#define mode	GPIO_Mode_OUT
#define type	GPIO_OType_PP
#define pull	GPIO_PuPd_UP
read alwaus differrent data   ?????  вроде нет!!!
with errore 
but scl and data есть
3v


#define mode	GPIO_Mode_OUT
#define type	GPIO_OType_PP
#define pull	GPIO_PuPd_DOWN
read alwaus differrent data 
with errore 
but scl and data есть
3v sda



#define mode	GPIO_Mode_AN
#define type	GPIO_OType_OD
#define pull	GPIO_PuPd_NOPULL
no error
no read ff but only 00
no data on sda only ff



#define mode	GPIO_Mode_AN
#define type	GPIO_OType_OD
#define pull	GPIO_PuPd_UP
no error
no read ff but only 00
no data on sda only ff
5V on line



#define mode	GPIO_Mode_AN
#define type	GPIO_OType_OD
#define pull	GPIO_PuPd_DOWN
no error
no read ff but only 00
no data on sda only ff
5V on line






#define mode	GPIO_Mode_AN
#define type	GPIO_OType_PP
#define pull	GPIO_PuPd_NOPULL
no error
no read ff but only 00
no data on sda only ff
5V on line



#define mode	GPIO_Mode_AN
#define type	GPIO_OType_PP
#define pull	GPIO_PuPd_UP
no error
no read ff but only 00
no data on sda only ff
5V on line




#define mode	GPIO_Mode_AN
#define type	GPIO_OType_PP
#define pull	GPIO_PuPd_DOWN
no error
no read ff but only 00
no data on sda only ff
5V on line
*/


/*
*/



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
	
	/*
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
*/
		GPIO_InitStructure.GPIO_Pin   = PIN_DS_SCL;  							//  vivod for RELE and svet AVARIYA 
		GPIO_InitStructure.GPIO_Mode  = mode; //GPIO_Mode_IN; // GPIO_Mode_OUT;     				// 	rezim vivoda
		GPIO_InitStructure.GPIO_OType = type; //GPIO_OType_OD; //GPIO_OType_PP; 						//	GPIO_OType_OD;          //  PP GPIO_OType_PP
		GPIO_InitStructure.GPIO_PuPd = pull;  
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     			//	speed
		GPIO_Init(PORT_DS_SCL, &GPIO_InitStructure); 
	
		GPIO_InitStructure.GPIO_Pin   = PIN_DS_SDA;  							//  vivod for RELE and svet AVARIYA 
		GPIO_InitStructure.GPIO_Mode  = mode; //GPIO_Mode_IN; //GPIO_Mode_OUT;     				// 	rezim vivoda
		GPIO_InitStructure.GPIO_OType = type; //GPIO_OType_OD; //GPIO_OType_PP; 						//	GPIO_OType_OD;          //  PP GPIO_OType_PP
		GPIO_InitStructure.GPIO_PuPd = pull;  
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     			//	speed
		GPIO_Init(PORT_DS_SDA, &GPIO_InitStructure); 
	
	
		/*
		  GPIO_PuPd_NOPULL = 0x00,
  GPIO_PuPd_UP     = 0x01,
  GPIO_PuPd_DOWN   = 0x02
		
		*/
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


u8 b_err_cl, bufout[20], zbuf[20];


void start_ds(void)
{
		MDO(0); 					 
		MCO(0); 
}

void stop_ds(void)
{	u8 j=0;
		MDO(0);			// from bksd						  
		MCO(1);	
		for(j=0;j<3;j++)																									 
		{  	
			sleep(ds_delay);
		}
		MDO(1); 
}

u8 ack_ds(void)
{		u8 ret=0;
	
		MDO(1);
		MCO(1);                
		ret=r_MDI();	
		if (ret==1)
		{
			b_err_cl++;
			if (error_ds==0)
					error_ds=1;
		}
    MCO(0);
	return ret;
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
	//		MDO(0);			  // from bksd
			bait= bait<<1; 
		} 	
		ack_ds();		
}

u8 read_bait_ds()
{
		u8 tmp=0, j=0;
		 
		for(j=0;j<8;j++)																									 
		{  	
				MDO(1);   // from bksd 
				MCO(1);    
				if (r_MDI() ==0) 																									    
				{tmp = tmp<<1; tmp = tmp&0xFE;} 															    
				else 																											    
				{tmp = tmp<<1; tmp = tmp|0x01;}                          					 
				MCO(0);
		}
		return tmp;
}


u8 wr_ack_ds(u8 ac)
{		
		if (ac==1)
			MDO(0);
		else
			MDO(1);
    MCO(1); 
		MCO(0);
}
/*
void read_dat_clock(void)
{		u8 tmp_rw=0, DL=0, j=0,i1=0;
		u8 kol_bait=1;

		error_ds=0;

		start_ds();
		write_bait_ds(0xD1);
		ack_ds();

		for(j=0;j<kol_bait;j++)																									 
		{   							  
			bufout[j]=read_bait_ds();	

// my 
	//		if (j!=(kol_bait-1)) 
				ack_ds();							  			
			tmp_rw=r_MDI();	
			if (tmp_rw==1)
			{
				b_err_cl++;
				if (error_ds==0)
						error_ds=1;
			}							
			MCO(0);	
// end my

	
// end from bksd			
			
		}
		stop_ds();	
}		

*/

void read_ds(void)
{		u8 i=0;
			
		start_ds();
		write_bait_ds(0xD0);
		write_bait_ds(0x00);
		stop_ds();
			
		sleep(100);
			
		start_ds();
		write_bait_ds(0xD1);		

		for (i = 0; i < 10; i ++)
		{
			zbuf[i]=read_bait_ds();	
			if (i!=9)
				wr_ack_ds(1);				
		}
		wr_ack_ds(0);		
		stop_ds();			
}
/*
void write_dat_clock(void)
{
		u8 i1=0, j=0, DL=0;

		error_ds=0;
		start_ds();
		write_bait_ds(0xD0);
		ack_ds();
		write_bait_ds(0x00);
		ack_ds();
  
		write_bait_ds(0);
		ack_ds();	
                         
		stop_ds();
}
*/
	
/*	

void write_dat_clock(void)
{
		u8 i1=0, j=0, DL=0;

		error_ds=0;
		start_ds();
		write_bait_ds(0xD0);
		ack_ds();
		write_bait_ds(0x00);
		ack_ds();

		for(j=0;j<10;j++) 
		{   
			write_bait_ds(0);
			ack_ds();	
		}                          
		stop_ds();
}


void write_dat_clock1(void)
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


void init_I2C1(void)
{
      GPIO_InitTypeDef      GPIO_InitStruct;
			I2C_InitTypeDef I2C_InitStruct;
 
    // enable APB1 peripheral clock for I2C1
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    // enable clock for SCL and SDA pins
  //  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // we are going to use PB6 and PB7
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;           // set pins to alternate function
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;      // set GPIO speed
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;         // set output to open drain --> the line has to be only pulled low, not driven high
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;           // enable pull up resistors
    GPIO_Init(GPIOB, &GPIO_InitStruct);                 // init GPIOB
 
    // Connect I2C1 pins to AF
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1); // SCL
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA
 
    // configure I2C1
    I2C_InitStruct.I2C_ClockSpeed = 100000;         // 100kHz
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;         // I2C mode
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2; // 50% duty cycle --> standard
    I2C_InitStruct.I2C_OwnAddress1 = 0x00;          // own address, not relevant in master mode
    I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;       // disable acknowledge when reading (can be changed later on)
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
    I2C_Init(I2C1, &I2C_InitStruct);                // init I2C1
 
    // enable I2C1
    I2C_Cmd(I2C1, ENABLE);
}

void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction)
{
    // wait until I2C1 is not busy anymore
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
 
    // Send I2C1 START condition
    I2C_GenerateSTART(I2Cx, ENABLE);
 
    // wait for I2C1 EV5 --> Slave has acknowledged start condition
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
 
    // Send slave Address for write
    I2C_Send7bitAddress(I2Cx, address, direction);
 

    if(direction == I2C_Direction_Transmitter){
        while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    }
    else if(direction == I2C_Direction_Receiver){
        while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    }
}
 

 
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
    I2C_SendData(I2Cx, data);
    // wait for I2C1 EV8_2 --> byte has been transmitted
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}
 

uint8_t I2C_read_ack(I2C_TypeDef* I2Cx)
{
    uint8_t data;
    // enable acknowledge of recieved data
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
    // wait until one byte has been received
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
    // read data from I2C data register and return data byte
    data = I2C_ReceiveData(I2Cx);
    return data;
}

uint8_t I2C_read_nack(I2C_TypeDef* I2Cx)
{
    uint8_t data;
    // disabe acknowledge of received data
    I2C_AcknowledgeConfig(I2Cx, DISABLE);
    // wait until one byte has been received
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
    // read data from I2C data register and return data byte
    data = I2C_ReceiveData(I2Cx);
    return data;
}

void I2C_stop(I2C_TypeDef* I2Cx)
{
    // Send I2C1 STOP Condition
    I2C_GenerateSTOP(I2Cx, ENABLE);
}

*/






