#line 1 "Libraries\\STM32_USB_OTG_Driver\\usb_hcd.c"



















 

 
#line 1 "Libraries\\STM32_USB_OTG_Driver\\usb_core.h"



















 

 



 
#line 1 ".\\inc\\usb_conf.h"



















 

 



 
#line 1 ".\\src\\stm32f4xx.h"




































  



 



 
    






  


 
  


 







 





#line 82 ".\\src\\stm32f4xx.h"







            








 










 
#line 118 ".\\src\\stm32f4xx.h"
                                             


 



 



 









 
 



 
typedef enum IRQn
{
 
  NonMaskableInt_IRQn         = -14,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      
 
  WWDG_IRQn                   = 0,       
  PVD_IRQn                    = 1,       
  TAMP_STAMP_IRQn             = 2,       
  RTC_WKUP_IRQn               = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Stream0_IRQn           = 11,      
  DMA1_Stream1_IRQn           = 12,      
  DMA1_Stream2_IRQn           = 13,      
  DMA1_Stream3_IRQn           = 14,      
  DMA1_Stream4_IRQn           = 15,      
  DMA1_Stream5_IRQn           = 16,      
  DMA1_Stream6_IRQn           = 17,      
  ADC_IRQn                    = 18,      
  CAN1_TX_IRQn                = 19,      
  CAN1_RX0_IRQn               = 20,      
  CAN1_RX1_IRQn               = 21,      
  CAN1_SCE_IRQn               = 22,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_TIM9_IRQn          = 24,      
  TIM1_UP_TIM10_IRQn          = 25,      
  TIM1_TRG_COM_TIM11_IRQn     = 26,      
  TIM1_CC_IRQn                = 27,      
  TIM2_IRQn                   = 28,      
  TIM3_IRQn                   = 29,      
  TIM4_IRQn                   = 30,      
  I2C1_EV_IRQn                = 31,      
  I2C1_ER_IRQn                = 32,      
  I2C2_EV_IRQn                = 33,      
  I2C2_ER_IRQn                = 34,        
  SPI1_IRQn                   = 35,      
  SPI2_IRQn                   = 36,      
  USART1_IRQn                 = 37,      
  USART2_IRQn                 = 38,      
  USART3_IRQn                 = 39,      
  EXTI15_10_IRQn              = 40,      
  RTC_Alarm_IRQn              = 41,      
  OTG_FS_WKUP_IRQn            = 42,          
  TIM8_BRK_TIM12_IRQn         = 43,      
  TIM8_UP_TIM13_IRQn          = 44,      
  TIM8_TRG_COM_TIM14_IRQn     = 45,      
  TIM8_CC_IRQn                = 46,      
  DMA1_Stream7_IRQn           = 47,      
  FSMC_IRQn                   = 48,      
  SDIO_IRQn                   = 49,      
  TIM5_IRQn                   = 50,      
  SPI3_IRQn                   = 51,      
  UART4_IRQn                  = 52,      
  UART5_IRQn                  = 53,      
  TIM6_DAC_IRQn               = 54,      
  TIM7_IRQn                   = 55,      
  DMA2_Stream0_IRQn           = 56,      
  DMA2_Stream1_IRQn           = 57,      
  DMA2_Stream2_IRQn           = 58,      
  DMA2_Stream3_IRQn           = 59,      
  DMA2_Stream4_IRQn           = 60,      
  ETH_IRQn                    = 61,      
  ETH_WKUP_IRQn               = 62,      
  CAN2_TX_IRQn                = 63,      
  CAN2_RX0_IRQn               = 64,      
  CAN2_RX1_IRQn               = 65,      
  CAN2_SCE_IRQn               = 66,      
  OTG_FS_IRQn                 = 67,      
  DMA2_Stream5_IRQn           = 68,      
  DMA2_Stream6_IRQn           = 69,      
  DMA2_Stream7_IRQn           = 70,      
  USART6_IRQn                 = 71,       
  I2C3_EV_IRQn                = 72,      
  I2C3_ER_IRQn                = 73,      
  OTG_HS_EP1_OUT_IRQn         = 74,      
  OTG_HS_EP1_IN_IRQn          = 75,      
  OTG_HS_WKUP_IRQn            = 76,      
  OTG_HS_IRQn                 = 77,      
  DCMI_IRQn                   = 78,      
  CRYP_IRQn                   = 79,      
  HASH_RNG_IRQn               = 80,       
  FPU_IRQn                    = 81       
} IRQn_Type;



 

#line 1 ".\\Libraries\\CMSIS\\core_cm4.h"
 




















 
























 













 




 






 

 











#line 100 ".\\Libraries\\CMSIS\\core_cm4.h"

 
#line 113 ".\\Libraries\\CMSIS\\core_cm4.h"

#line 142 ".\\Libraries\\CMSIS\\core_cm4.h"

#line 1 "C:\\Keil4\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"
 
 





 










#line 26 "C:\\Keil4\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"







 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     
typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;

     
typedef   signed       __int64 intmax_t;
typedef unsigned       __int64 uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     


     


     


     

     


     


     


     

     



     



     


     
    
 



#line 197 "C:\\Keil4\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"

     







     










     











#line 261 "C:\\Keil4\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"



 



#line 144 ".\\Libraries\\CMSIS\\core_cm4.h"
#line 1 ".\\Libraries\\CMSIS\\core_cmInstr.h"
 




















 





 



 


 









 







 







 






 








 







 







 









 









 
static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 
static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}










 









 









 









 











 











 











 







 










 










 









 






#line 582 ".\\Libraries\\CMSIS\\core_cmInstr.h"

   

#line 145 ".\\Libraries\\CMSIS\\core_cm4.h"
#line 1 ".\\Libraries\\CMSIS\\core_cmFunc.h"
 




















 





 



 


 





 
 






 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}







 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}







 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}







 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}







 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}







 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}







 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}







 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}
 







 







 








 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}







 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xff);
}
 






 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}







 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & (uint32_t)1);
}











 
static __inline uint32_t __get_FPSCR(void)
{




   return(0);

}







 
static __inline void __set_FPSCR(uint32_t fpscr)
{




}




#line 605 ".\\Libraries\\CMSIS\\core_cmFunc.h"

 


#line 146 ".\\Libraries\\CMSIS\\core_cm4.h"
#line 1 ".\\Libraries\\CMSIS\\core_cm4_simd.h"
 




















 











 


 



 


 

 
#line 106 ".\\Libraries\\CMSIS\\core_cm4_simd.h"








 



#line 693 ".\\Libraries\\CMSIS\\core_cm4_simd.h"

 




#line 147 ".\\Libraries\\CMSIS\\core_cm4.h"








 
#line 182 ".\\Libraries\\CMSIS\\core_cm4.h"

 
#line 191 ".\\Libraries\\CMSIS\\core_cm4.h"

 





 









 





 


 
typedef union
{
  struct
  {



    uint32_t _reserved0:16;               
    uint32_t GE:4;                        
    uint32_t _reserved1:7;                

    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       



    uint32_t _reserved0:7;                
    uint32_t GE:4;                        
    uint32_t _reserved1:4;                

    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 


 
typedef struct
{
  volatile uint32_t ISER[8];                  
       uint32_t RESERVED0[24];
  volatile uint32_t ICER[8];                  
       uint32_t RSERVED1[24];
  volatile uint32_t ISPR[8];                  
       uint32_t RESERVED2[24];
  volatile uint32_t ICPR[8];                  
       uint32_t RESERVED3[24];
  volatile uint32_t IABR[8];                  
       uint32_t RESERVED4[56];
  volatile uint8_t  IP[240];                  
       uint32_t RESERVED5[644];
  volatile  uint32_t STIR;                     
}  NVIC_Type;

 



 






 


 
typedef struct
{
  volatile const  uint32_t CPUID;                    
  volatile uint32_t ICSR;                     
  volatile uint32_t VTOR;                     
  volatile uint32_t AIRCR;                    
  volatile uint32_t SCR;                      
  volatile uint32_t CCR;                      
  volatile uint8_t  SHP[12];                  
  volatile uint32_t SHCSR;                    
  volatile uint32_t CFSR;                     
  volatile uint32_t HFSR;                     
  volatile uint32_t DFSR;                     
  volatile uint32_t MMFAR;                    
  volatile uint32_t BFAR;                     
  volatile uint32_t AFSR;                     
  volatile const  uint32_t PFR[2];                   
  volatile const  uint32_t DFR;                      
  volatile const  uint32_t ADR;                      
  volatile const  uint32_t MMFR[4];                  
  volatile const  uint32_t ISAR[5];                  
       uint32_t RESERVED0[5];
  volatile uint32_t CPACR;                    
} SCB_Type;

 















 






























 



 





















 









 


















 










































 









 









 















 






 


 
typedef struct
{
       uint32_t RESERVED0[1];
  volatile const  uint32_t ICTR;                     
  volatile uint32_t ACTLR;                    
} SCnSCB_Type;

 



 















 






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t LOAD;                     
  volatile uint32_t VAL;                      
  volatile const  uint32_t CALIB;                    
} SysTick_Type;

 












 



 



 









 






 


 
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                   
    volatile  uint16_t   u16;                  
    volatile  uint32_t   u32;                  
  }  PORT [32];                           
       uint32_t RESERVED0[864];
  volatile uint32_t TER;                      
       uint32_t RESERVED1[15];
  volatile uint32_t TPR;                      
       uint32_t RESERVED2[15];
  volatile uint32_t TCR;                      
} ITM_Type;

 



 



























   







 


 
typedef struct
{
  volatile const  uint32_t TYPE;                     
  volatile uint32_t CTRL;                     
  volatile uint32_t RNR;                      
  volatile uint32_t RBAR;                     
  volatile uint32_t RASR;                     
  volatile uint32_t RBAR_A1;                  
  volatile uint32_t RASR_A1;                  
  volatile uint32_t RBAR_A2;                  
  volatile uint32_t RASR_A2;                  
  volatile uint32_t RBAR_A3;                  
  volatile uint32_t RASR_A3;                  
} MPU_Type;

 









 









 



 









 












 



#line 870 ".\\Libraries\\CMSIS\\core_cm4.h"






 


 
typedef struct
{
  volatile uint32_t DHCSR;                    
  volatile  uint32_t DCRSR;                    
  volatile uint32_t DCRDR;                    
  volatile uint32_t DEMCR;                    
} CoreDebug_Type;

 




































 






 







































 




 

 
#line 986 ".\\Libraries\\CMSIS\\core_cm4.h"

#line 993 ".\\Libraries\\CMSIS\\core_cm4.h"











 





 






 



 



 










 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07);                

  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;                                                    
  reg_value &= ~((0xFFFFUL << 16) | (7UL << 8));              
  reg_value  =  (reg_value                                 |
                ((uint32_t)0x5FA << 16) |
                (PriorityGroupTmp << 8));                                      
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR =  reg_value;
}








 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8)) >> 8);    
}








 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
 
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(uint32_t)((int32_t)IRQn) >> 5] = (uint32_t)(1 << ((uint32_t)((int32_t)IRQn) & (uint32_t)0x1F));  
}








 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}










 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}








 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}












 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[((uint32_t)(IRQn) & 0xF)-4] = ((priority << (8 - 4)) & 0xff); }  
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[(uint32_t)(IRQn)] = ((priority << (8 - 4)) & 0xff);    }         
}













 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[((uint32_t)(IRQn) & 0xF)-4] >> (8 - 4)));  }  
  else {
    return((uint32_t)(((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[(uint32_t)(IRQn)]           >> (8 - 4)));  }  
}















 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 4) ? 4 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 4) < 7) ? 0 : PriorityGroupTmp - 7 + 4;

  return (
           ((PreemptPriority & ((1 << (PreemptPriorityBits)) - 1)) << SubPriorityBits) |
           ((SubPriority     & ((1 << (SubPriorityBits    )) - 1)))
         );
}















 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 4) ? 4 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 4) < 7) ? 0 : PriorityGroupTmp - 7 + 4;

  *pPreemptPriority = (Priority >> SubPriorityBits) & ((1 << (PreemptPriorityBits)) - 1);
  *pSubPriority     = (Priority                   ) & ((1 << (SubPriorityBits    )) - 1);
}





 
static __inline void NVIC_SystemReset(void)
{
  __dsb(0xF);                                                     
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FA << 16)      |
                 (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8)) |
                 (1UL << 2));                    
  __dsb(0xF);                                                      
  while(1);                                                     
}

 



 



 











 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if (ticks > (0xFFFFFFUL << 0))  return (1);             

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (ticks & (0xFFFFFFUL << 0)) - 1;       
  NVIC_SetPriority (SysTick_IRQn, (1<<4) - 1);   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2) |
                   (1UL << 1)   |
                   (1UL << 0);                     
  return (0);                                                   
}



 



 



 

extern volatile int32_t ITM_RxBuffer;                     











 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if ((((CoreDebug_Type *) (0xE000EDF0UL))->DEMCR & (1UL << 24))  &&       
      (((ITM_Type *) (0xE0000000UL) )->TCR & (1UL << 0))                  &&       
      (((ITM_Type *) (0xE0000000UL) )->TER & (1UL << 0)        )                    )      
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0].u32 == 0);
    ((ITM_Type *) (0xE0000000UL) )->PORT[0].u8 = (uint8_t) ch;
  }
  return (ch);
}










 
static __inline int32_t ITM_ReceiveChar (void) {
  int32_t ch = -1;                            

  if (ITM_RxBuffer != 0x5AA55AA5) {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5;        
  }

  return (ch);
}









 
static __inline int32_t ITM_CheckChar (void) {

  if (ITM_RxBuffer == 0x5AA55AA5) {
    return (0);                                  
  } else {
    return (1);                                  
  }
}

 





#line 246 ".\\src\\stm32f4xx.h"
#line 1 ".\\src\\system_stm32f4xx.h"



















  



 



   
  


 









 



 




 

extern uint32_t SystemCoreClock;           




 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
#line 247 ".\\src\\stm32f4xx.h"
#line 248 ".\\src\\stm32f4xx.h"



   
 
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;   
typedef const int16_t sc16;   
typedef const int8_t sc8;    

typedef volatile int32_t  vs32;
typedef volatile int16_t  vs16;
typedef volatile int8_t   vs8;

typedef volatile const int32_t vsc32;   
typedef volatile const int16_t vsc16;   
typedef volatile const int8_t vsc8;    

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;   
typedef const uint16_t uc16;   
typedef const uint8_t uc8;    

typedef volatile uint32_t  vu32;
typedef volatile uint16_t vu16;
typedef volatile uint8_t  vu8;

typedef volatile const uint32_t vuc32;   
typedef volatile const uint16_t vuc16;   
typedef volatile const uint8_t vuc8;    

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;



 



    



 

typedef struct
{
  volatile uint32_t SR;      
  volatile uint32_t CR1;           
  volatile uint32_t CR2;     
  volatile uint32_t SMPR1;   
  volatile uint32_t SMPR2;   
  volatile uint32_t JOFR1;   
  volatile uint32_t JOFR2;   
  volatile uint32_t JOFR3;   
  volatile uint32_t JOFR4;   
  volatile uint32_t HTR;     
  volatile uint32_t LTR;     
  volatile uint32_t SQR1;    
  volatile uint32_t SQR2;    
  volatile uint32_t SQR3;    
  volatile uint32_t JSQR;    
  volatile uint32_t JDR1;    
  volatile uint32_t JDR2;    
  volatile uint32_t JDR3;    
  volatile uint32_t JDR4;    
  volatile uint32_t DR;      
} ADC_TypeDef;

typedef struct
{
  volatile uint32_t CSR;     
  volatile uint32_t CCR;     
  volatile uint32_t CDR;    
 
} ADC_Common_TypeDef;




 

typedef struct
{
  volatile uint32_t TIR;   
  volatile uint32_t TDTR;  
  volatile uint32_t TDLR;  
  volatile uint32_t TDHR;  
} CAN_TxMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t RIR;   
  volatile uint32_t RDTR;  
  volatile uint32_t RDLR;  
  volatile uint32_t RDHR;  
} CAN_FIFOMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t FR1;  
  volatile uint32_t FR2;  
} CAN_FilterRegister_TypeDef;



 
  
typedef struct
{
  volatile uint32_t              MCR;                  
  volatile uint32_t              MSR;                  
  volatile uint32_t              TSR;                  
  volatile uint32_t              RF0R;                 
  volatile uint32_t              RF1R;                 
  volatile uint32_t              IER;                  
  volatile uint32_t              ESR;                  
  volatile uint32_t              BTR;                  
  uint32_t                   RESERVED0[88];        
  CAN_TxMailBox_TypeDef      sTxMailBox[3];        
  CAN_FIFOMailBox_TypeDef    sFIFOMailBox[2];      
  uint32_t                   RESERVED1[12];        
  volatile uint32_t              FMR;                  
  volatile uint32_t              FM1R;                 
  uint32_t                   RESERVED2;            
  volatile uint32_t              FS1R;                 
  uint32_t                   RESERVED3;            
  volatile uint32_t              FFA1R;                
  uint32_t                   RESERVED4;            
  volatile uint32_t              FA1R;                 
  uint32_t                   RESERVED5[8];          
  CAN_FilterRegister_TypeDef sFilterRegister[28];  
} CAN_TypeDef;



 

typedef struct
{
  volatile uint32_t DR;          
  volatile uint8_t  IDR;         
  uint8_t       RESERVED0;   
  uint16_t      RESERVED1;   
  volatile uint32_t CR;          
} CRC_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;        
  volatile uint32_t SWTRIGR;   
  volatile uint32_t DHR12R1;   
  volatile uint32_t DHR12L1;   
  volatile uint32_t DHR8R1;    
  volatile uint32_t DHR12R2;   
  volatile uint32_t DHR12L2;   
  volatile uint32_t DHR8R2;    
  volatile uint32_t DHR12RD;   
  volatile uint32_t DHR12LD;   
  volatile uint32_t DHR8RD;    
  volatile uint32_t DOR1;      
  volatile uint32_t DOR2;      
  volatile uint32_t SR;        
} DAC_TypeDef;



 

typedef struct
{
  volatile uint32_t IDCODE;   
  volatile uint32_t CR;       
  volatile uint32_t APB1FZ;   
  volatile uint32_t APB2FZ;   
}DBGMCU_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;        
  volatile uint32_t SR;        
  volatile uint32_t RISR;      
  volatile uint32_t IER;       
  volatile uint32_t MISR;      
  volatile uint32_t ICR;       
  volatile uint32_t ESCR;      
  volatile uint32_t ESUR;      
  volatile uint32_t CWSTRTR;   
  volatile uint32_t CWSIZER;   
  volatile uint32_t DR;        
} DCMI_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;      
  volatile uint32_t NDTR;    
  volatile uint32_t PAR;     
  volatile uint32_t M0AR;    
  volatile uint32_t M1AR;    
  volatile uint32_t FCR;     
} DMA_Stream_TypeDef;

typedef struct
{
  volatile uint32_t LISR;    
  volatile uint32_t HISR;    
  volatile uint32_t LIFCR;   
  volatile uint32_t HIFCR;   
} DMA_TypeDef;



 

typedef struct
{
  volatile uint32_t MACCR;
  volatile uint32_t MACFFR;
  volatile uint32_t MACHTHR;
  volatile uint32_t MACHTLR;
  volatile uint32_t MACMIIAR;
  volatile uint32_t MACMIIDR;
  volatile uint32_t MACFCR;
  volatile uint32_t MACVLANTR;              
  uint32_t      RESERVED0[2];
  volatile uint32_t MACRWUFFR;              
  volatile uint32_t MACPMTCSR;
  uint32_t      RESERVED1[2];
  volatile uint32_t MACSR;                  
  volatile uint32_t MACIMR;
  volatile uint32_t MACA0HR;
  volatile uint32_t MACA0LR;
  volatile uint32_t MACA1HR;
  volatile uint32_t MACA1LR;
  volatile uint32_t MACA2HR;
  volatile uint32_t MACA2LR;
  volatile uint32_t MACA3HR;
  volatile uint32_t MACA3LR;                
  uint32_t      RESERVED2[40];
  volatile uint32_t MMCCR;                  
  volatile uint32_t MMCRIR;
  volatile uint32_t MMCTIR;
  volatile uint32_t MMCRIMR;
  volatile uint32_t MMCTIMR;                
  uint32_t      RESERVED3[14];
  volatile uint32_t MMCTGFSCCR;             
  volatile uint32_t MMCTGFMSCCR;
  uint32_t      RESERVED4[5];
  volatile uint32_t MMCTGFCR;
  uint32_t      RESERVED5[10];
  volatile uint32_t MMCRFCECR;
  volatile uint32_t MMCRFAECR;
  uint32_t      RESERVED6[10];
  volatile uint32_t MMCRGUFCR;
  uint32_t      RESERVED7[334];
  volatile uint32_t PTPTSCR;
  volatile uint32_t PTPSSIR;
  volatile uint32_t PTPTSHR;
  volatile uint32_t PTPTSLR;
  volatile uint32_t PTPTSHUR;
  volatile uint32_t PTPTSLUR;
  volatile uint32_t PTPTSAR;
  volatile uint32_t PTPTTHR;
  volatile uint32_t PTPTTLR;
  volatile uint32_t RESERVED8;
  volatile uint32_t PTPTSSR;
  uint32_t      RESERVED9[565];
  volatile uint32_t DMABMR;
  volatile uint32_t DMATPDR;
  volatile uint32_t DMARPDR;
  volatile uint32_t DMARDLAR;
  volatile uint32_t DMATDLAR;
  volatile uint32_t DMASR;
  volatile uint32_t DMAOMR;
  volatile uint32_t DMAIER;
  volatile uint32_t DMAMFBOCR;
  volatile uint32_t DMARSWTR;
  uint32_t      RESERVED10[8];
  volatile uint32_t DMACHTDR;
  volatile uint32_t DMACHRDR;
  volatile uint32_t DMACHTBAR;
  volatile uint32_t DMACHRBAR;
} ETH_TypeDef;



 

typedef struct
{
  volatile uint32_t IMR;     
  volatile uint32_t EMR;     
  volatile uint32_t RTSR;    
  volatile uint32_t FTSR;    
  volatile uint32_t SWIER;   
  volatile uint32_t PR;      
} EXTI_TypeDef;



 

typedef struct
{
  volatile uint32_t ACR;       
  volatile uint32_t KEYR;      
  volatile uint32_t OPTKEYR;   
  volatile uint32_t SR;        
  volatile uint32_t CR;        
  volatile uint32_t OPTCR;     
} FLASH_TypeDef;



 

typedef struct
{
  volatile uint32_t BTCR[8];        
} FSMC_Bank1_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t BWTR[7];     
} FSMC_Bank1E_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR2;        
  volatile uint32_t SR2;         
  volatile uint32_t PMEM2;       
  volatile uint32_t PATT2;       
  uint32_t      RESERVED0;   
  volatile uint32_t ECCR2;       
} FSMC_Bank2_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR3;        
  volatile uint32_t SR3;         
  volatile uint32_t PMEM3;       
  volatile uint32_t PATT3;       
  uint32_t      RESERVED0;   
  volatile uint32_t ECCR3;       
} FSMC_Bank3_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR4;        
  volatile uint32_t SR4;         
  volatile uint32_t PMEM4;       
  volatile uint32_t PATT4;       
  volatile uint32_t PIO4;        
} FSMC_Bank4_TypeDef; 



 

typedef struct
{
  volatile uint32_t MODER;     
  volatile uint32_t OTYPER;    
  volatile uint32_t OSPEEDR;   
  volatile uint32_t PUPDR;     
  volatile uint32_t IDR;       
  volatile uint32_t ODR;       
  volatile uint16_t BSRRL;     
  volatile uint16_t BSRRH;     
  volatile uint32_t LCKR;      
  volatile uint32_t AFR[2];    
} GPIO_TypeDef;



 
  
typedef struct
{
  volatile uint32_t MEMRMP;        
  volatile uint32_t PMC;           
  volatile uint32_t EXTICR[4];     
  uint32_t      RESERVED[2];    
  volatile uint32_t CMPCR;         
} SYSCFG_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;         
  uint16_t      RESERVED0;   
  volatile uint16_t CR2;         
  uint16_t      RESERVED1;   
  volatile uint16_t OAR1;        
  uint16_t      RESERVED2;   
  volatile uint16_t OAR2;        
  uint16_t      RESERVED3;   
  volatile uint16_t DR;          
  uint16_t      RESERVED4;   
  volatile uint16_t SR1;         
  uint16_t      RESERVED5;   
  volatile uint16_t SR2;         
  uint16_t      RESERVED6;   
  volatile uint16_t CCR;         
  uint16_t      RESERVED7;   
  volatile uint16_t TRISE;       
  uint16_t      RESERVED8;   
} I2C_TypeDef;



 

typedef struct
{
  volatile uint32_t KR;    
  volatile uint32_t PR;    
  volatile uint32_t RLR;   
  volatile uint32_t SR;    
} IWDG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;    
  volatile uint32_t CSR;   
} PWR_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;             
  volatile uint32_t PLLCFGR;        
  volatile uint32_t CFGR;           
  volatile uint32_t CIR;            
  volatile uint32_t AHB1RSTR;       
  volatile uint32_t AHB2RSTR;       
  volatile uint32_t AHB3RSTR;       
  uint32_t      RESERVED0;      
  volatile uint32_t APB1RSTR;       
  volatile uint32_t APB2RSTR;       
  uint32_t      RESERVED1[2];   
  volatile uint32_t AHB1ENR;        
  volatile uint32_t AHB2ENR;        
  volatile uint32_t AHB3ENR;        
  uint32_t      RESERVED2;      
  volatile uint32_t APB1ENR;        
  volatile uint32_t APB2ENR;        
  uint32_t      RESERVED3[2];   
  volatile uint32_t AHB1LPENR;      
  volatile uint32_t AHB2LPENR;      
  volatile uint32_t AHB3LPENR;      
  uint32_t      RESERVED4;      
  volatile uint32_t APB1LPENR;      
  volatile uint32_t APB2LPENR;      
  uint32_t      RESERVED5[2];   
  volatile uint32_t BDCR;           
  volatile uint32_t CSR;            
  uint32_t      RESERVED6[2];   
  volatile uint32_t SSCGR;          
  volatile uint32_t PLLI2SCFGR;     
} RCC_TypeDef;



 

typedef struct
{
  volatile uint32_t TR;       
  volatile uint32_t DR;       
  volatile uint32_t CR;       
  volatile uint32_t ISR;      
  volatile uint32_t PRER;     
  volatile uint32_t WUTR;     
  volatile uint32_t CALIBR;   
  volatile uint32_t ALRMAR;   
  volatile uint32_t ALRMBR;   
  volatile uint32_t WPR;      
  volatile uint32_t SSR;      
  volatile uint32_t SHIFTR;   
  volatile uint32_t TSTR;     
  volatile uint32_t TSDR;     
  volatile uint32_t TSSSR;    
  volatile uint32_t CALR;     
  volatile uint32_t TAFCR;    
  volatile uint32_t ALRMASSR; 
  volatile uint32_t ALRMBSSR; 
  uint32_t RESERVED7;     
  volatile uint32_t BKP0R;    
  volatile uint32_t BKP1R;    
  volatile uint32_t BKP2R;    
  volatile uint32_t BKP3R;    
  volatile uint32_t BKP4R;    
  volatile uint32_t BKP5R;    
  volatile uint32_t BKP6R;    
  volatile uint32_t BKP7R;    
  volatile uint32_t BKP8R;    
  volatile uint32_t BKP9R;    
  volatile uint32_t BKP10R;   
  volatile uint32_t BKP11R;   
  volatile uint32_t BKP12R;   
  volatile uint32_t BKP13R;   
  volatile uint32_t BKP14R;   
  volatile uint32_t BKP15R;   
  volatile uint32_t BKP16R;   
  volatile uint32_t BKP17R;   
  volatile uint32_t BKP18R;   
  volatile uint32_t BKP19R;   
} RTC_TypeDef;



 

typedef struct
{
  volatile uint32_t POWER;           
  volatile uint32_t CLKCR;           
  volatile uint32_t ARG;             
  volatile uint32_t CMD;             
  volatile const uint32_t  RESPCMD;         
  volatile const uint32_t  RESP1;           
  volatile const uint32_t  RESP2;           
  volatile const uint32_t  RESP3;           
  volatile const uint32_t  RESP4;           
  volatile uint32_t DTIMER;          
  volatile uint32_t DLEN;            
  volatile uint32_t DCTRL;           
  volatile const uint32_t  DCOUNT;          
  volatile const uint32_t  STA;             
  volatile uint32_t ICR;             
  volatile uint32_t MASK;            
  uint32_t      RESERVED0[2];    
  volatile const uint32_t  FIFOCNT;         
  uint32_t      RESERVED1[13];   
  volatile uint32_t FIFO;            
} SDIO_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;         
  uint16_t      RESERVED0;   
  volatile uint16_t CR2;         
  uint16_t      RESERVED1;   
  volatile uint16_t SR;          
  uint16_t      RESERVED2;   
  volatile uint16_t DR;          
  uint16_t      RESERVED3;   
  volatile uint16_t CRCPR;       
  uint16_t      RESERVED4;   
  volatile uint16_t RXCRCR;      
  uint16_t      RESERVED5;   
  volatile uint16_t TXCRCR;      
  uint16_t      RESERVED6;   
  volatile uint16_t I2SCFGR;     
  uint16_t      RESERVED7;   
  volatile uint16_t I2SPR;       
  uint16_t      RESERVED8;   
} SPI_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;          
  uint16_t      RESERVED0;    
  volatile uint16_t CR2;          
  uint16_t      RESERVED1;    
  volatile uint16_t SMCR;         
  uint16_t      RESERVED2;    
  volatile uint16_t DIER;         
  uint16_t      RESERVED3;    
  volatile uint16_t SR;           
  uint16_t      RESERVED4;    
  volatile uint16_t EGR;          
  uint16_t      RESERVED5;    
  volatile uint16_t CCMR1;        
  uint16_t      RESERVED6;    
  volatile uint16_t CCMR2;        
  uint16_t      RESERVED7;    
  volatile uint16_t CCER;         
  uint16_t      RESERVED8;    
  volatile uint32_t CNT;          
  volatile uint16_t PSC;          
  uint16_t      RESERVED9;    
  volatile uint32_t ARR;          
  volatile uint16_t RCR;          
  uint16_t      RESERVED10;   
  volatile uint32_t CCR1;         
  volatile uint32_t CCR2;         
  volatile uint32_t CCR3;         
  volatile uint32_t CCR4;         
  volatile uint16_t BDTR;         
  uint16_t      RESERVED11;   
  volatile uint16_t DCR;          
  uint16_t      RESERVED12;   
  volatile uint16_t DMAR;         
  uint16_t      RESERVED13;   
  volatile uint16_t OR;           
  uint16_t      RESERVED14;   
} TIM_TypeDef;



 
 
typedef struct
{
  volatile uint16_t SR;          
  uint16_t      RESERVED0;   
  volatile uint16_t DR;          
  uint16_t      RESERVED1;   
  volatile uint16_t BRR;         
  uint16_t      RESERVED2;   
  volatile uint16_t CR1;         
  uint16_t      RESERVED3;   
  volatile uint16_t CR2;         
  uint16_t      RESERVED4;   
  volatile uint16_t CR3;         
  uint16_t      RESERVED5;   
  volatile uint16_t GTPR;        
  uint16_t      RESERVED6;   
} USART_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;    
  volatile uint32_t CFR;   
  volatile uint32_t SR;    
} WWDG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;      
  volatile uint32_t SR;      
  volatile uint32_t DR;      
  volatile uint32_t DOUT;    
  volatile uint32_t DMACR;   
  volatile uint32_t IMSCR;   
  volatile uint32_t RISR;    
  volatile uint32_t MISR;    
  volatile uint32_t K0LR;    
  volatile uint32_t K0RR;    
  volatile uint32_t K1LR;    
  volatile uint32_t K1RR;    
  volatile uint32_t K2LR;    
  volatile uint32_t K2RR;    
  volatile uint32_t K3LR;    
  volatile uint32_t K3RR;    
  volatile uint32_t IV0LR;   
  volatile uint32_t IV0RR;   
  volatile uint32_t IV1LR;   
  volatile uint32_t IV1RR;   
} CRYP_TypeDef;



 
  
typedef struct 
{
  volatile uint32_t CR;         
  volatile uint32_t DIN;        
  volatile uint32_t STR;        
  volatile uint32_t HR[5];      
  volatile uint32_t IMR;        
  volatile uint32_t SR;         
  uint32_t  RESERVED[52];   
  volatile uint32_t CSR[51];      
} HASH_TypeDef;



 
  
typedef struct 
{
  volatile uint32_t CR;   
  volatile uint32_t SR;   
  volatile uint32_t DR;   
} RNG_TypeDef;



 
  


 
#line 1015 ".\\src\\stm32f4xx.h"







 




 





 
#line 1061 ".\\src\\stm32f4xx.h"

 
#line 1078 ".\\src\\stm32f4xx.h"

 
#line 1115 ".\\src\\stm32f4xx.h"

 





 






 




 
  


   
#line 1222 ".\\src\\stm32f4xx.h"



 



 
  
  

 
    
 
 
 

 
 
 
 
 
 
#line 1251 ".\\src\\stm32f4xx.h"

 
#line 1277 ".\\src\\stm32f4xx.h"
  
 
#line 1303 ".\\src\\stm32f4xx.h"

 
#line 1341 ".\\src\\stm32f4xx.h"

 
#line 1383 ".\\src\\stm32f4xx.h"

 


 


 


 


 


 


 
#line 1432 ".\\src\\stm32f4xx.h"

 
#line 1470 ".\\src\\stm32f4xx.h"

 
#line 1508 ".\\src\\stm32f4xx.h"

 
#line 1537 ".\\src\\stm32f4xx.h"

 


 


 


 


 



 
#line 1573 ".\\src\\stm32f4xx.h"

 
#line 1595 ".\\src\\stm32f4xx.h"

 



 
 
 
 
 
 
 
#line 1616 ".\\src\\stm32f4xx.h"

 
#line 1627 ".\\src\\stm32f4xx.h"

 
#line 1645 ".\\src\\stm32f4xx.h"











 





 





 
#line 1683 ".\\src\\stm32f4xx.h"

 












 
#line 1704 ".\\src\\stm32f4xx.h"

 
 






 




 





 





 






 




 





 





 






   




 





 





 





 




 





 





 





 




 





 





 
 


 
#line 1844 ".\\src\\stm32f4xx.h"

 
#line 1861 ".\\src\\stm32f4xx.h"

 
#line 1878 ".\\src\\stm32f4xx.h"

 
#line 1895 ".\\src\\stm32f4xx.h"

 
#line 1929 ".\\src\\stm32f4xx.h"

 
#line 1963 ".\\src\\stm32f4xx.h"

 
#line 1997 ".\\src\\stm32f4xx.h"

 
#line 2031 ".\\src\\stm32f4xx.h"

 
#line 2065 ".\\src\\stm32f4xx.h"

 
#line 2099 ".\\src\\stm32f4xx.h"

 
#line 2133 ".\\src\\stm32f4xx.h"

 
#line 2167 ".\\src\\stm32f4xx.h"

 
#line 2201 ".\\src\\stm32f4xx.h"

 
#line 2235 ".\\src\\stm32f4xx.h"

 
#line 2269 ".\\src\\stm32f4xx.h"

 
#line 2303 ".\\src\\stm32f4xx.h"

 
#line 2337 ".\\src\\stm32f4xx.h"

 
#line 2371 ".\\src\\stm32f4xx.h"

 
#line 2405 ".\\src\\stm32f4xx.h"

 
#line 2439 ".\\src\\stm32f4xx.h"

 
#line 2473 ".\\src\\stm32f4xx.h"

 
#line 2507 ".\\src\\stm32f4xx.h"

 
#line 2541 ".\\src\\stm32f4xx.h"

 
#line 2575 ".\\src\\stm32f4xx.h"

 
#line 2609 ".\\src\\stm32f4xx.h"

 
#line 2643 ".\\src\\stm32f4xx.h"

 
#line 2677 ".\\src\\stm32f4xx.h"

 
#line 2711 ".\\src\\stm32f4xx.h"

 
#line 2745 ".\\src\\stm32f4xx.h"

 
#line 2779 ".\\src\\stm32f4xx.h"

 
#line 2813 ".\\src\\stm32f4xx.h"

 
#line 2847 ".\\src\\stm32f4xx.h"

 
 
 
 
 
 



 



 


 
 
 
 
 
 


#line 2884 ".\\src\\stm32f4xx.h"

#line 2893 ".\\src\\stm32f4xx.h"
 





 


 


 


 



 
 
 
 
 
 









































 



 


 


 


 


 


 


 



 



 



 


 


 



 
 
 
 
 

 
 
 
 
 
 
#line 3029 ".\\src\\stm32f4xx.h"

 




 






 






 






 






 
 
 
 
 
  
#line 3104 ".\\src\\stm32f4xx.h"

 
#line 3123 ".\\src\\stm32f4xx.h"

  
#line 3134 ".\\src\\stm32f4xx.h"

  
#line 3156 ".\\src\\stm32f4xx.h"

  
#line 3178 ".\\src\\stm32f4xx.h"

  
#line 3200 ".\\src\\stm32f4xx.h"

  
#line 3222 ".\\src\\stm32f4xx.h"

 
 
 
 
 
 
#line 3249 ".\\src\\stm32f4xx.h"

 
#line 3271 ".\\src\\stm32f4xx.h"

 
#line 3293 ".\\src\\stm32f4xx.h"

 
#line 3315 ".\\src\\stm32f4xx.h"

 
#line 3337 ".\\src\\stm32f4xx.h"

 
#line 3359 ".\\src\\stm32f4xx.h"

 
 
 
 
 
 
#line 3375 ".\\src\\stm32f4xx.h"

#line 3383 ".\\src\\stm32f4xx.h"

 
#line 3392 ".\\src\\stm32f4xx.h"

 
#line 3406 ".\\src\\stm32f4xx.h"

 
#line 3436 ".\\src\\stm32f4xx.h"

 
 
 
 
 
 











#line 3464 ".\\src\\stm32f4xx.h"

 











#line 3487 ".\\src\\stm32f4xx.h"

 











#line 3510 ".\\src\\stm32f4xx.h"

 











#line 3533 ".\\src\\stm32f4xx.h"

 








































 








































 








































 








































 


































 


































 


































 


































 



























 



























 



























 
#line 3930 ".\\src\\stm32f4xx.h"

 
#line 3939 ".\\src\\stm32f4xx.h"

 
#line 3948 ".\\src\\stm32f4xx.h"

 
#line 3959 ".\\src\\stm32f4xx.h"

#line 3969 ".\\src\\stm32f4xx.h"

#line 3979 ".\\src\\stm32f4xx.h"

#line 3989 ".\\src\\stm32f4xx.h"

 
#line 4000 ".\\src\\stm32f4xx.h"

#line 4010 ".\\src\\stm32f4xx.h"

#line 4020 ".\\src\\stm32f4xx.h"

#line 4030 ".\\src\\stm32f4xx.h"

 
#line 4041 ".\\src\\stm32f4xx.h"

#line 4051 ".\\src\\stm32f4xx.h"

#line 4061 ".\\src\\stm32f4xx.h"

#line 4071 ".\\src\\stm32f4xx.h"

 
#line 4082 ".\\src\\stm32f4xx.h"

#line 4092 ".\\src\\stm32f4xx.h"

#line 4102 ".\\src\\stm32f4xx.h"

#line 4112 ".\\src\\stm32f4xx.h"

 
#line 4123 ".\\src\\stm32f4xx.h"

#line 4133 ".\\src\\stm32f4xx.h"

#line 4143 ".\\src\\stm32f4xx.h"

#line 4153 ".\\src\\stm32f4xx.h"

 
#line 4164 ".\\src\\stm32f4xx.h"

#line 4174 ".\\src\\stm32f4xx.h"

#line 4184 ".\\src\\stm32f4xx.h"

#line 4194 ".\\src\\stm32f4xx.h"

 
#line 4205 ".\\src\\stm32f4xx.h"

#line 4215 ".\\src\\stm32f4xx.h"

#line 4225 ".\\src\\stm32f4xx.h"

#line 4235 ".\\src\\stm32f4xx.h"

 


 


 
 
 
 
 
 
































































 
#line 4329 ".\\src\\stm32f4xx.h"

 
































































 
































































 
#line 4477 ".\\src\\stm32f4xx.h"
 
#line 4494 ".\\src\\stm32f4xx.h"

 
#line 4512 ".\\src\\stm32f4xx.h"
 
#line 4529 ".\\src\\stm32f4xx.h"

 
#line 4563 ".\\src\\stm32f4xx.h"

 
 
 
 
 
 
#line 4584 ".\\src\\stm32f4xx.h"

 
#line 4593 ".\\src\\stm32f4xx.h"

 



 





 
 
 
 
 
 
#line 4624 ".\\src\\stm32f4xx.h"

 
#line 4633 ".\\src\\stm32f4xx.h"







 



#line 4654 ".\\src\\stm32f4xx.h"



 



 


 
#line 4679 ".\\src\\stm32f4xx.h"

 
#line 4689 ".\\src\\stm32f4xx.h"

 




 


 
 
 
 
 
 


 





 


 



 
 
 
 
 
 












 
#line 4746 ".\\src\\stm32f4xx.h"




 


 
#line 4761 ".\\src\\stm32f4xx.h"
 


 
 
 
 
 
 



#line 4779 ".\\src\\stm32f4xx.h"

#line 4789 ".\\src\\stm32f4xx.h"

#line 4798 ".\\src\\stm32f4xx.h"

 
#line 4807 ".\\src\\stm32f4xx.h"

#line 4818 ".\\src\\stm32f4xx.h"















 
 








 








 






#line 4868 ".\\src\\stm32f4xx.h"

 











 











 
#line 4900 ".\\src\\stm32f4xx.h"

 




















 
#line 4943 ".\\src\\stm32f4xx.h"

 
#line 4959 ".\\src\\stm32f4xx.h"

 






 


 
#line 4994 ".\\src\\stm32f4xx.h"

 
#line 5007 ".\\src\\stm32f4xx.h"
 


 
#line 5031 ".\\src\\stm32f4xx.h"

 






 


 
#line 5066 ".\\src\\stm32f4xx.h"

 
#line 5081 ".\\src\\stm32f4xx.h"

 
#line 5105 ".\\src\\stm32f4xx.h"

 






 


 
#line 5140 ".\\src\\stm32f4xx.h"

 
#line 5155 ".\\src\\stm32f4xx.h"

 











 
#line 5179 ".\\src\\stm32f4xx.h"

 





 



 
 
 
 
 
 



 






 
 
 
 
 
 
#line 5239 ".\\src\\stm32f4xx.h"

 
#line 5269 ".\\src\\stm32f4xx.h"

 
#line 5297 ".\\src\\stm32f4xx.h"

 
#line 5314 ".\\src\\stm32f4xx.h"

 



 


 



 
#line 5367 ".\\src\\stm32f4xx.h"

 
#line 5409 ".\\src\\stm32f4xx.h"

 


 


 



 
#line 5448 ".\\src\\stm32f4xx.h"

 
#line 5468 ".\\src\\stm32f4xx.h"

 


 
#line 5486 ".\\src\\stm32f4xx.h"

 
#line 5506 ".\\src\\stm32f4xx.h"

 
#line 5514 ".\\src\\stm32f4xx.h"

 
#line 5522 ".\\src\\stm32f4xx.h"

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 
 
 
 
 
 




 












 


 






#line 5623 ".\\src\\stm32f4xx.h"

 


 


 


 


 


 


 


 


 
















 


 
#line 5693 ".\\src\\stm32f4xx.h"

 
#line 5708 ".\\src\\stm32f4xx.h"

 
#line 5734 ".\\src\\stm32f4xx.h"

 


 


 
 
 
 
 
 









#line 5766 ".\\src\\stm32f4xx.h"

 
#line 5774 ".\\src\\stm32f4xx.h"

 
#line 5784 ".\\src\\stm32f4xx.h"

 


 


 


 


 





















 




 
 
 
 
 
   




 

 


 






  
#line 5856 ".\\src\\stm32f4xx.h"


  
#line 5868 ".\\src\\stm32f4xx.h"


  
#line 5880 ".\\src\\stm32f4xx.h"


  
#line 5892 ".\\src\\stm32f4xx.h"

 






  
#line 5910 ".\\src\\stm32f4xx.h"


  
#line 5922 ".\\src\\stm32f4xx.h"


  
#line 5934 ".\\src\\stm32f4xx.h"


  
#line 5946 ".\\src\\stm32f4xx.h"

 




           


  
#line 5965 ".\\src\\stm32f4xx.h"


  
#line 5977 ".\\src\\stm32f4xx.h"


  
#line 5989 ".\\src\\stm32f4xx.h"


  
#line 6001 ".\\src\\stm32f4xx.h"

 






  
#line 6018 ".\\src\\stm32f4xx.h"


  
#line 6029 ".\\src\\stm32f4xx.h"


  
#line 6040 ".\\src\\stm32f4xx.h"


  
#line 6051 ".\\src\\stm32f4xx.h"

   



 
 
 
 
 
 
















 









#line 6096 ".\\src\\stm32f4xx.h"

 

























 
#line 6139 ".\\src\\stm32f4xx.h"

 
#line 6153 ".\\src\\stm32f4xx.h"

 
#line 6163 ".\\src\\stm32f4xx.h"

 




























 





















 




























 





















 
#line 6282 ".\\src\\stm32f4xx.h"

 


 


 


 


 


 


 


 


 
#line 6317 ".\\src\\stm32f4xx.h"





#line 6328 ".\\src\\stm32f4xx.h"

 
#line 6336 ".\\src\\stm32f4xx.h"

#line 6343 ".\\src\\stm32f4xx.h"

 


 
#line 6354 ".\\src\\stm32f4xx.h"


 
 
 
 
 
 
#line 6372 ".\\src\\stm32f4xx.h"

 


 



 
#line 6396 ".\\src\\stm32f4xx.h"

 
#line 6405 ".\\src\\stm32f4xx.h"







 
#line 6425 ".\\src\\stm32f4xx.h"

 
#line 6436 ".\\src\\stm32f4xx.h"



 
 
 
 
 
 
#line 6453 ".\\src\\stm32f4xx.h"



 
#line 6465 ".\\src\\stm32f4xx.h"







 



 
 
 
 
 
 



 









 
#line 6513 ".\\src\\stm32f4xx.h"
 


 






 
 
 
 
 
 
#line 6557 ".\\src\\stm32f4xx.h"

 
#line 6573 ".\\src\\stm32f4xx.h"

 


 


 
#line 6591 ".\\src\\stm32f4xx.h"
  
 


 
#line 6607 ".\\src\\stm32f4xx.h"

 



  


 








 

  
#line 6634 ".\\src\\stm32f4xx.h"

 






 



 


 


 
#line 6663 ".\\src\\stm32f4xx.h"

 


 
#line 6678 ".\\src\\stm32f4xx.h"

 


 
#line 6693 ".\\src\\stm32f4xx.h"

 


 
 
 

 
#line 6708 ".\\src\\stm32f4xx.h"

 




 




 




 




 


 


 


 


 


 


 
 
 

 
#line 6761 ".\\src\\stm32f4xx.h"

#line 6768 ".\\src\\stm32f4xx.h"

 


 


 



 


 



 


 


 


 



 
 
 

 
#line 6843 ".\\src\\stm32f4xx.h"

 


 


 


 


 




   
#line 6894 ".\\src\\stm32f4xx.h"

 
#line 6920 ".\\src\\stm32f4xx.h"

 
#line 6937 ".\\src\\stm32f4xx.h"

 





 


 


 


 




 

 

  

#line 1 ".\\inc\\stm32f4xx_conf.h"



















  

 




 




 
 
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_adc.h"




















 

 







 
#line 1 ".\\src\\stm32f4xx.h"




































  



 



 
    
#line 6995 ".\\src\\stm32f4xx.h"



 

  

 

 
#line 33 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_adc.h"



 



  

 



  
typedef struct
{
  uint32_t ADC_Resolution;                
                                    
  FunctionalState ADC_ScanConvMode;       


  
  FunctionalState ADC_ContinuousConvMode; 

 
  uint32_t ADC_ExternalTrigConvEdge;      


 
  uint32_t ADC_ExternalTrigConv;          


 
  uint32_t ADC_DataAlign;                 

 
  uint8_t  ADC_NbrOfConversion;           


 
}ADC_InitTypeDef;
  


  
typedef struct 
{
  uint32_t ADC_Mode;                      

                                               
  uint32_t ADC_Prescaler;                 

 
  uint32_t ADC_DMAAccessMode;             


 
  uint32_t ADC_TwoSamplingDelay;          

 
  
}ADC_CommonInitTypeDef;


 



  






  
#line 135 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_adc.h"


  




  
#line 151 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_adc.h"


  




  
#line 167 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_adc.h"
                                     


  




  
#line 208 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_adc.h"
                                     


  




  
#line 225 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_adc.h"
                                      


  




  
#line 242 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_adc.h"


  




  
#line 282 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_adc.h"


  




  






  




  
#line 321 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_adc.h"





#line 345 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_adc.h"


  




  
#line 369 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_adc.h"


  




  
#line 385 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_adc.h"
                                            


  




  
#line 426 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_adc.h"


  




  
#line 442 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_adc.h"


  




  
#line 464 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_adc.h"


  




  
#line 478 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_adc.h"


  




  
#line 492 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_adc.h"
  
#line 500 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_adc.h"


  




  



  




  



  




  



  




  



  




  



  




  



  




  



  




  

 
   

   
void ADC_DeInit(void);

 
void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct);
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct);
void ADC_CommonInit(ADC_CommonInitTypeDef* ADC_CommonInitStruct);
void ADC_CommonStructInit(ADC_CommonInitTypeDef* ADC_CommonInitStruct);
void ADC_Cmd(ADC_TypeDef* ADCx, FunctionalState NewState);

 
void ADC_AnalogWatchdogCmd(ADC_TypeDef* ADCx, uint32_t ADC_AnalogWatchdog);
void ADC_AnalogWatchdogThresholdsConfig(ADC_TypeDef* ADCx, uint16_t HighThreshold,uint16_t LowThreshold);
void ADC_AnalogWatchdogSingleChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel);

 
void ADC_TempSensorVrefintCmd(FunctionalState NewState);
void ADC_VBATCmd(FunctionalState NewState);

 
void ADC_RegularChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void ADC_SoftwareStartConv(ADC_TypeDef* ADCx);
FlagStatus ADC_GetSoftwareStartConvStatus(ADC_TypeDef* ADCx);
void ADC_EOCOnEachRegularChannelCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_ContinuousModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_DiscModeChannelCountConfig(ADC_TypeDef* ADCx, uint8_t Number);
void ADC_DiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
uint16_t ADC_GetConversionValue(ADC_TypeDef* ADCx);
uint32_t ADC_GetMultiModeConversionValue(void);

 
void ADC_DMACmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_DMARequestAfterLastTransferCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_MultiModeDMARequestAfterLastTransferCmd(FunctionalState NewState);

 
void ADC_InjectedChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void ADC_InjectedSequencerLengthConfig(ADC_TypeDef* ADCx, uint8_t Length);
void ADC_SetInjectedOffset(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel, uint16_t Offset);
void ADC_ExternalTrigInjectedConvConfig(ADC_TypeDef* ADCx, uint32_t ADC_ExternalTrigInjecConv);
void ADC_ExternalTrigInjectedConvEdgeConfig(ADC_TypeDef* ADCx, uint32_t ADC_ExternalTrigInjecConvEdge);
void ADC_SoftwareStartInjectedConv(ADC_TypeDef* ADCx);
FlagStatus ADC_GetSoftwareStartInjectedConvCmdStatus(ADC_TypeDef* ADCx);
void ADC_AutoInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_InjectedDiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
uint16_t ADC_GetInjectedConversionValue(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel);

 
void ADC_ITConfig(ADC_TypeDef* ADCx, uint16_t ADC_IT, FunctionalState NewState);
FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCx, uint8_t ADC_FLAG);
void ADC_ClearFlag(ADC_TypeDef* ADCx, uint8_t ADC_FLAG);
ITStatus ADC_GetITStatus(ADC_TypeDef* ADCx, uint16_t ADC_IT);
void ADC_ClearITPendingBit(ADC_TypeDef* ADCx, uint16_t ADC_IT);









  



  

 
#line 35 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_can.h"




















 

 







 
#line 33 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_can.h"



 



 

 






 
typedef struct
{
  uint16_t CAN_Prescaler;   
 
  
  uint8_t CAN_Mode;         
 

  uint8_t CAN_SJW;          


 

  uint8_t CAN_BS1;          

 

  uint8_t CAN_BS2;          
 
  
  FunctionalState CAN_TTCM; 
 
  
  FunctionalState CAN_ABOM;  
 

  FunctionalState CAN_AWUM;  
 

  FunctionalState CAN_NART;  
 

  FunctionalState CAN_RFLM;  
 

  FunctionalState CAN_TXFP;  
 
} CAN_InitTypeDef;



 
typedef struct
{
  uint16_t CAN_FilterIdHigh;         

 

  uint16_t CAN_FilterIdLow;          

 

  uint16_t CAN_FilterMaskIdHigh;     


 

  uint16_t CAN_FilterMaskIdLow;      


 

  uint16_t CAN_FilterFIFOAssignment; 
 
  
  uint8_t CAN_FilterNumber;           

  uint8_t CAN_FilterMode;            
 

  uint8_t CAN_FilterScale;           
 

  FunctionalState CAN_FilterActivation; 
 
} CAN_FilterInitTypeDef;



 
typedef struct
{
  uint32_t StdId;  
 

  uint32_t ExtId;  
 

  uint8_t IDE;     

 

  uint8_t RTR;     

 

  uint8_t DLC;     

 

  uint8_t Data[8]; 
 
} CanTxMsg;



 
typedef struct
{
  uint32_t StdId;  
 

  uint32_t ExtId;  
 

  uint8_t IDE;     

 

  uint8_t RTR;     

 

  uint8_t DLC;     
 

  uint8_t Data[8]; 
 

  uint8_t FMI;     

 
} CanRxMsg;

 



 



 





 




 



 












 


 


   










 
  



   





 



 









 



 
#line 283 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_can.h"




 



 
#line 300 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_can.h"




 



 



 



 



 



 







 



 







 



 





 




 



 



 



 






 



 





 




 



 




 




 



 





 	






 



 






 



 



 	




 



 



 




 




                                                          
#line 475 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_can.h"


 



 

 

 

 




 
#line 499 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_can.h"

 



 

 





#line 520 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_can.h"








 

  


  


 
#line 543 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_can.h"

 



 






 





#line 568 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_can.h"

#line 575 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_can.h"


 



 

 
   

  
void CAN_DeInit(CAN_TypeDef* CANx);

  
uint8_t CAN_Init(CAN_TypeDef* CANx, CAN_InitTypeDef* CAN_InitStruct);
void CAN_FilterInit(CAN_FilterInitTypeDef* CAN_FilterInitStruct);
void CAN_StructInit(CAN_InitTypeDef* CAN_InitStruct);
void CAN_SlaveStartBank(uint8_t CAN_BankNumber); 
void CAN_DBGFreeze(CAN_TypeDef* CANx, FunctionalState NewState);
void CAN_TTComModeCmd(CAN_TypeDef* CANx, FunctionalState NewState);

 
uint8_t CAN_Transmit(CAN_TypeDef* CANx, CanTxMsg* TxMessage);
uint8_t CAN_TransmitStatus(CAN_TypeDef* CANx, uint8_t TransmitMailbox);
void CAN_CancelTransmit(CAN_TypeDef* CANx, uint8_t Mailbox);

 
void CAN_Receive(CAN_TypeDef* CANx, uint8_t FIFONumber, CanRxMsg* RxMessage);
void CAN_FIFORelease(CAN_TypeDef* CANx, uint8_t FIFONumber);
uint8_t CAN_MessagePending(CAN_TypeDef* CANx, uint8_t FIFONumber);

 
uint8_t CAN_OperatingModeRequest(CAN_TypeDef* CANx, uint8_t CAN_OperatingMode);
uint8_t CAN_Sleep(CAN_TypeDef* CANx);
uint8_t CAN_WakeUp(CAN_TypeDef* CANx);

 
uint8_t CAN_GetLastErrorCode(CAN_TypeDef* CANx);
uint8_t CAN_GetReceiveErrorCounter(CAN_TypeDef* CANx);
uint8_t CAN_GetLSBTransmitErrorCounter(CAN_TypeDef* CANx);

 
void CAN_ITConfig(CAN_TypeDef* CANx, uint32_t CAN_IT, FunctionalState NewState);
FlagStatus CAN_GetFlagStatus(CAN_TypeDef* CANx, uint32_t CAN_FLAG);
void CAN_ClearFlag(CAN_TypeDef* CANx, uint32_t CAN_FLAG);
ITStatus CAN_GetITStatus(CAN_TypeDef* CANx, uint32_t CAN_IT);
void CAN_ClearITPendingBit(CAN_TypeDef* CANx, uint32_t CAN_IT);









 



 

 
#line 36 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_crc.h"




















 

 







 
#line 33 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_crc.h"



 



 

 
 



 



 

 
   

void CRC_ResetDR(void);
uint32_t CRC_CalcCRC(uint32_t Data);
uint32_t CRC_CalcBlockCRC(uint32_t pBuffer[], uint32_t BufferLength);
uint32_t CRC_GetCRC(void);
void CRC_SetIDRegister(uint8_t IDValue);
uint8_t CRC_GetIDRegister(void);









 



 

 
#line 37 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_cryp.h"




















 

 







 
#line 33 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_cryp.h"



 



  

 



  
typedef struct
{
  uint16_t CRYP_AlgoDir;   
 
  uint16_t CRYP_AlgoMode;  

 
  uint16_t CRYP_DataType;  
  
  uint16_t CRYP_KeySize;   

 
}CRYP_InitTypeDef;



  
typedef struct
{
  uint32_t CRYP_Key0Left;   
  uint32_t CRYP_Key0Right;  
  uint32_t CRYP_Key1Left;   
  uint32_t CRYP_Key1Right;  
  uint32_t CRYP_Key2Left;   
  uint32_t CRYP_Key2Right;  
  uint32_t CRYP_Key3Left;   
  uint32_t CRYP_Key3Right;  
}CRYP_KeyInitTypeDef;


  
typedef struct
{
  uint32_t CRYP_IV0Left;   
  uint32_t CRYP_IV0Right;  
  uint32_t CRYP_IV1Left;   
  uint32_t CRYP_IV1Right;  
}CRYP_IVInitTypeDef;



  
typedef struct
{
   
  uint32_t CR_bits9to2;
   
  uint32_t CRYP_IV0LR;
  uint32_t CRYP_IV0RR;
  uint32_t CRYP_IV1LR;
  uint32_t CRYP_IV1RR;
   
  uint32_t CRYP_K0LR;
  uint32_t CRYP_K0RR;
  uint32_t CRYP_K1LR;
  uint32_t CRYP_K1RR;
  uint32_t CRYP_K2LR;
  uint32_t CRYP_K2RR;
  uint32_t CRYP_K3LR;
  uint32_t CRYP_K3RR;
}CRYP_Context;


 



 



 







  
 


 

 



 



 





#line 154 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_cryp.h"


  
 


 
#line 169 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_cryp.h"


 
                                     


 
#line 182 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_cryp.h"


 



 
#line 201 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_cryp.h"

#line 209 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_cryp.h"


 



 







 



 





 



 





  



  

 
 

 
void CRYP_DeInit(void);

 
void CRYP_Init(CRYP_InitTypeDef* CRYP_InitStruct);
void CRYP_StructInit(CRYP_InitTypeDef* CRYP_InitStruct);
void CRYP_KeyInit(CRYP_KeyInitTypeDef* CRYP_KeyInitStruct);
void CRYP_KeyStructInit(CRYP_KeyInitTypeDef* CRYP_KeyInitStruct);
void CRYP_IVInit(CRYP_IVInitTypeDef* CRYP_IVInitStruct);
void CRYP_IVStructInit(CRYP_IVInitTypeDef* CRYP_IVInitStruct);
void CRYP_Cmd(FunctionalState NewState);

 
void CRYP_DataIn(uint32_t Data);
uint32_t CRYP_DataOut(void);
void CRYP_FIFOFlush(void);

 
ErrorStatus CRYP_SaveContext(CRYP_Context* CRYP_ContextSave,
                             CRYP_KeyInitTypeDef* CRYP_KeyInitStruct);
void CRYP_RestoreContext(CRYP_Context* CRYP_ContextRestore);

 
void CRYP_DMACmd(uint8_t CRYP_DMAReq, FunctionalState NewState);

 
void CRYP_ITConfig(uint8_t CRYP_IT, FunctionalState NewState);
ITStatus CRYP_GetITStatus(uint8_t CRYP_IT);
FlagStatus CRYP_GetFlagStatus(uint8_t CRYP_FLAG);

 
ErrorStatus CRYP_AES_ECB(uint8_t Mode,
                         uint8_t *Key, uint16_t Keysize,
                         uint8_t *Input, uint32_t Ilength,
                         uint8_t *Output);

ErrorStatus CRYP_AES_CBC(uint8_t Mode,
                         uint8_t InitVectors[16],
                         uint8_t *Key, uint16_t Keysize,
                         uint8_t *Input, uint32_t Ilength,
                         uint8_t *Output);

ErrorStatus CRYP_AES_CTR(uint8_t Mode,
                         uint8_t InitVectors[16],
                         uint8_t *Key, uint16_t Keysize,
                         uint8_t *Input, uint32_t Ilength,
                         uint8_t *Output);

 
ErrorStatus CRYP_TDES_ECB(uint8_t Mode,
                           uint8_t Key[24], 
                           uint8_t *Input, uint32_t Ilength,
                           uint8_t *Output);

ErrorStatus CRYP_TDES_CBC(uint8_t Mode,
                          uint8_t Key[24],
                          uint8_t InitVectors[8],
                          uint8_t *Input, uint32_t Ilength,
                          uint8_t *Output);

 
ErrorStatus CRYP_DES_ECB(uint8_t Mode,
                         uint8_t Key[8],
                         uint8_t *Input, uint32_t Ilength,
                         uint8_t *Output);

ErrorStatus CRYP_DES_CBC(uint8_t Mode,
                         uint8_t Key[8],
                         uint8_t InitVectors[8],
                         uint8_t *Input,uint32_t Ilength,
                         uint8_t *Output);









 



  

 
#line 38 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dac.h"




















 

 







 
#line 33 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dac.h"



 



 

 



 

typedef struct
{
  uint32_t DAC_Trigger;                      
 

  uint32_t DAC_WaveGeneration;               

 

  uint32_t DAC_LFSRUnmask_TriangleAmplitude; 

 

  uint32_t DAC_OutputBuffer;                 
 
}DAC_InitTypeDef;

 



 



 

#line 83 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dac.h"




#line 96 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dac.h"



 



 

#line 111 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dac.h"


 



 

#line 143 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dac.h"

#line 168 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dac.h"


 



 







 



 







 



 

#line 206 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dac.h"


 



 







 



 




 
  


    





  



  
  





 



 

 
   

   
void DAC_DeInit(void);

 
void DAC_Init(uint32_t DAC_Channel, DAC_InitTypeDef* DAC_InitStruct);
void DAC_StructInit(DAC_InitTypeDef* DAC_InitStruct);
void DAC_Cmd(uint32_t DAC_Channel, FunctionalState NewState);
void DAC_SoftwareTriggerCmd(uint32_t DAC_Channel, FunctionalState NewState);
void DAC_DualSoftwareTriggerCmd(FunctionalState NewState);
void DAC_WaveGenerationCmd(uint32_t DAC_Channel, uint32_t DAC_Wave, FunctionalState NewState);
void DAC_SetChannel1Data(uint32_t DAC_Align, uint16_t Data);
void DAC_SetChannel2Data(uint32_t DAC_Align, uint16_t Data);
void DAC_SetDualChannelData(uint32_t DAC_Align, uint16_t Data2, uint16_t Data1);
uint16_t DAC_GetDataOutputValue(uint32_t DAC_Channel);

 
void DAC_DMACmd(uint32_t DAC_Channel, FunctionalState NewState);

 
void DAC_ITConfig(uint32_t DAC_Channel, uint32_t DAC_IT, FunctionalState NewState);
FlagStatus DAC_GetFlagStatus(uint32_t DAC_Channel, uint32_t DAC_FLAG);
void DAC_ClearFlag(uint32_t DAC_Channel, uint32_t DAC_FLAG);
ITStatus DAC_GetITStatus(uint32_t DAC_Channel, uint32_t DAC_IT);
void DAC_ClearITPendingBit(uint32_t DAC_Channel, uint32_t DAC_IT);









 



 

 
#line 39 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dbgmcu.h"



















 

 







 
#line 32 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dbgmcu.h"



 



  

 
 



  





#line 70 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dbgmcu.h"

#line 77 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dbgmcu.h"


  

 
  
uint32_t DBGMCU_GetREVID(void);
uint32_t DBGMCU_GetDEVID(void);
void DBGMCU_Config(uint32_t DBGMCU_Periph, FunctionalState NewState);
void DBGMCU_APB1PeriphConfig(uint32_t DBGMCU_Periph, FunctionalState NewState);
void DBGMCU_APB2PeriphConfig(uint32_t DBGMCU_Periph, FunctionalState NewState);









  



  

 
#line 40 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dcmi.h"



















 

 







 
#line 32 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dcmi.h"



 



  

 


  
typedef struct
{
  uint16_t DCMI_CaptureMode;      
 

  uint16_t DCMI_SynchroMode;      
 

  uint16_t DCMI_PCKPolarity;      
 

  uint16_t DCMI_VSPolarity;       
 

  uint16_t DCMI_HSPolarity;       
 

  uint16_t DCMI_CaptureRate;      
 

  uint16_t DCMI_ExtendedDataMode; 
 
} DCMI_InitTypeDef;



  
typedef struct
{
  uint16_t DCMI_VerticalStartLine;      
 

  uint16_t DCMI_HorizontalOffsetCount;  
 

  uint16_t DCMI_VerticalLineCount;      
 

  uint16_t DCMI_CaptureCount;           

 
} DCMI_CROPInitTypeDef;



  
typedef struct
{
  uint8_t DCMI_FrameStartCode;  
  uint8_t DCMI_LineStartCode;   
  uint8_t DCMI_LineEndCode;     
  uint8_t DCMI_FrameEndCode;    
} DCMI_CodesInitTypeDef;

 



 



  
#line 114 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dcmi.h"


  




  
#line 128 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dcmi.h"


  




  






  




  






  




  






  




  
#line 178 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dcmi.h"


  




  
#line 194 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dcmi.h"


  




  
#line 213 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dcmi.h"


  




  


  





  







  
#line 256 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dcmi.h"
                                



  



  

 
  

  
void DCMI_DeInit(void);

 
void DCMI_Init(DCMI_InitTypeDef* DCMI_InitStruct);
void DCMI_StructInit(DCMI_InitTypeDef* DCMI_InitStruct);
void DCMI_CROPConfig(DCMI_CROPInitTypeDef* DCMI_CROPInitStruct);
void DCMI_CROPCmd(FunctionalState NewState);
void DCMI_SetEmbeddedSynchroCodes(DCMI_CodesInitTypeDef* DCMI_CodesInitStruct);
void DCMI_JPEGCmd(FunctionalState NewState);

 
void DCMI_Cmd(FunctionalState NewState);
void DCMI_CaptureCmd(FunctionalState NewState);
uint32_t DCMI_ReadData(void);

 
void DCMI_ITConfig(uint16_t DCMI_IT, FunctionalState NewState);
FlagStatus DCMI_GetFlagStatus(uint16_t DCMI_FLAG);
void DCMI_ClearFlag(uint16_t DCMI_FLAG);
ITStatus DCMI_GetITStatus(uint16_t DCMI_IT);
void DCMI_ClearITPendingBit(uint16_t DCMI_IT);









  



  

 
#line 41 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dma.h"




















  

 







 
#line 33 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dma.h"



 



 

 



 

typedef struct
{
  uint32_t DMA_Channel;            
 
 
  uint32_t DMA_PeripheralBaseAddr;  

  uint32_t DMA_Memory0BaseAddr;    

 

  uint32_t DMA_DIR;                

 

  uint32_t DMA_BufferSize;         

 

  uint32_t DMA_PeripheralInc;      
 

  uint32_t DMA_MemoryInc;          
 

  uint32_t DMA_PeripheralDataSize; 
 

  uint32_t DMA_MemoryDataSize;     
 

  uint32_t DMA_Mode;               


 

  uint32_t DMA_Priority;           
 

  uint32_t DMA_FIFOMode;          


 

  uint32_t DMA_FIFOThreshold;      
 

  uint32_t DMA_MemoryBurst;        


 

  uint32_t DMA_PeripheralBurst;    


   
}DMA_InitTypeDef;

 



 

#line 128 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dma.h"






  
#line 143 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dma.h"

#line 152 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dma.h"


  




  









  




  



  




  







  




  







  




  









  




  









  




  







  




  











  




  







  




  











  




  











  




  











  




 
#line 340 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dma.h"

#line 347 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dma.h"


  



 
#line 394 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dma.h"




#line 418 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dma.h"


  




  









  




  
#line 481 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dma.h"





#line 506 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_dma.h"


  




  







  




  







  




  






  



  

 
  

  
void DMA_DeInit(DMA_Stream_TypeDef* DMAy_Streamx);

 
void DMA_Init(DMA_Stream_TypeDef* DMAy_Streamx, DMA_InitTypeDef* DMA_InitStruct);
void DMA_StructInit(DMA_InitTypeDef* DMA_InitStruct);
void DMA_Cmd(DMA_Stream_TypeDef* DMAy_Streamx, FunctionalState NewState);

 
void DMA_PeriphIncOffsetSizeConfig(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_Pincos);
void DMA_FlowControllerConfig(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_FlowCtrl);

 
void DMA_SetCurrDataCounter(DMA_Stream_TypeDef* DMAy_Streamx, uint16_t Counter);
uint16_t DMA_GetCurrDataCounter(DMA_Stream_TypeDef* DMAy_Streamx);

 
void DMA_DoubleBufferModeConfig(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t Memory1BaseAddr,
                                uint32_t DMA_CurrentMemory);
void DMA_DoubleBufferModeCmd(DMA_Stream_TypeDef* DMAy_Streamx, FunctionalState NewState);
void DMA_MemoryTargetConfig(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t MemoryBaseAddr,
                            uint32_t DMA_MemoryTarget);
uint32_t DMA_GetCurrentMemoryTarget(DMA_Stream_TypeDef* DMAy_Streamx);

 
FunctionalState DMA_GetCmdStatus(DMA_Stream_TypeDef* DMAy_Streamx);
uint32_t DMA_GetFIFOStatus(DMA_Stream_TypeDef* DMAy_Streamx);
FlagStatus DMA_GetFlagStatus(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_FLAG);
void DMA_ClearFlag(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_FLAG);
void DMA_ITConfig(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_IT, FunctionalState NewState);
ITStatus DMA_GetITStatus(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_IT);
void DMA_ClearITPendingBit(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_IT);









 



 


 
#line 42 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_exti.h"




















 

 







 
#line 33 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_exti.h"



 



 

 



 

typedef enum
{
  EXTI_Mode_Interrupt = 0x00,
  EXTI_Mode_Event = 0x04
}EXTIMode_TypeDef;





 

typedef enum
{
  EXTI_Trigger_Rising = 0x08,
  EXTI_Trigger_Falling = 0x0C,  
  EXTI_Trigger_Rising_Falling = 0x10
}EXTITrigger_TypeDef;






 

typedef struct
{
  uint32_t EXTI_Line;               
 
   
  EXTIMode_TypeDef EXTI_Mode;       
 

  EXTITrigger_TypeDef EXTI_Trigger; 
 

  FunctionalState EXTI_LineCmd;     
  
}EXTI_InitTypeDef;

 



 



 

#line 122 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_exti.h"
                                          


#line 137 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_exti.h"
                    


 



 

 
 

 
void EXTI_DeInit(void);

 
void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_StructInit(EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_GenerateSWInterrupt(uint32_t EXTI_Line);

 
FlagStatus EXTI_GetFlagStatus(uint32_t EXTI_Line);
void EXTI_ClearFlag(uint32_t EXTI_Line);
ITStatus EXTI_GetITStatus(uint32_t EXTI_Line);
void EXTI_ClearITPendingBit(uint32_t EXTI_Line);









 



 

 
#line 43 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_flash.h"




















 

 







 
#line 33 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_flash.h"



 



  

 


  
typedef enum
{ 
  FLASH_BUSY = 1,
  FLASH_ERROR_PGS,
  FLASH_ERROR_PGP,
  FLASH_ERROR_PGA,
  FLASH_ERROR_WRP,
  FLASH_ERROR_PROGRAM,
  FLASH_ERROR_OPERATION,
  FLASH_COMPLETE
}FLASH_Status;

 



   



  
#line 75 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_flash.h"

#line 84 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_flash.h"


  



  











  



  
#line 127 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_flash.h"


  



  
#line 147 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_flash.h"




 



 


  
 





  



  





  



  





  




  





 
  


   
#line 207 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_flash.h"


 



  





  



  
#line 236 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_flash.h"


 



 







  



  







  



  



  






  

 
  
 
 
void FLASH_SetLatency(uint32_t FLASH_Latency);
void FLASH_PrefetchBufferCmd(FunctionalState NewState);
void FLASH_InstructionCacheCmd(FunctionalState NewState);
void FLASH_DataCacheCmd(FunctionalState NewState);
void FLASH_InstructionCacheReset(void);
void FLASH_DataCacheReset(void);

    
void FLASH_Unlock(void);
void FLASH_Lock(void);
FLASH_Status FLASH_EraseSector(uint32_t FLASH_Sector, uint8_t VoltageRange);
FLASH_Status FLASH_EraseAllSectors(uint8_t VoltageRange);
FLASH_Status FLASH_ProgramDoubleWord(uint32_t Address, uint64_t Data);
FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);
FLASH_Status FLASH_ProgramByte(uint32_t Address, uint8_t Data);

  
void FLASH_OB_Unlock(void);
void FLASH_OB_Lock(void);
void FLASH_OB_WRPConfig(uint32_t OB_WRP, FunctionalState NewState);
void FLASH_OB_RDPConfig(uint8_t OB_RDP);
void FLASH_OB_UserConfig(uint8_t OB_IWDG, uint8_t OB_STOP, uint8_t OB_STDBY);
void FLASH_OB_BORConfig(uint8_t OB_BOR);
FLASH_Status FLASH_OB_Launch(void);
uint8_t FLASH_OB_GetUser(void);
uint16_t FLASH_OB_GetWRP(void);
FlagStatus FLASH_OB_GetRDP(void);
uint8_t FLASH_OB_GetBOR(void);

 
void FLASH_ITConfig(uint32_t FLASH_IT, FunctionalState NewState);
FlagStatus FLASH_GetFlagStatus(uint32_t FLASH_FLAG);
void FLASH_ClearFlag(uint32_t FLASH_FLAG);
FLASH_Status FLASH_GetStatus(void);
FLASH_Status FLASH_WaitForLastOperation(void);









  



  

 
#line 44 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_fsmc.h"




















 

 







 
#line 33 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_fsmc.h"



 



 

 



 
typedef struct
{
  uint32_t FSMC_AddressSetupTime;       


 

  uint32_t FSMC_AddressHoldTime;        


 

  uint32_t FSMC_DataSetupTime;          


 

  uint32_t FSMC_BusTurnAroundDuration;  


 

  uint32_t FSMC_CLKDivision;            

 

  uint32_t FSMC_DataLatency;            





 

  uint32_t FSMC_AccessMode;             
 
}FSMC_NORSRAMTimingInitTypeDef;



 
typedef struct
{
  uint32_t FSMC_Bank;                
 

  uint32_t FSMC_DataAddressMux;      

 

  uint32_t FSMC_MemoryType;          

 

  uint32_t FSMC_MemoryDataWidth;     
 

  uint32_t FSMC_BurstAccessMode;     

 

  uint32_t FSMC_AsynchronousWait;     

                                           

  uint32_t FSMC_WaitSignalPolarity;  

 

  uint32_t FSMC_WrapMode;            

 

  uint32_t FSMC_WaitSignalActive;    


 

  uint32_t FSMC_WriteOperation;      
 

  uint32_t FSMC_WaitSignal;          

 

  uint32_t FSMC_ExtendedMode;        
 

  uint32_t FSMC_WriteBurst;          
  

  FSMC_NORSRAMTimingInitTypeDef* FSMC_ReadWriteTimingStruct;    

  FSMC_NORSRAMTimingInitTypeDef* FSMC_WriteTimingStruct;            
}FSMC_NORSRAMInitTypeDef;



 
typedef struct
{
  uint32_t FSMC_SetupTime;      



 

  uint32_t FSMC_WaitSetupTime;  



 

  uint32_t FSMC_HoldSetupTime;  




 

  uint32_t FSMC_HiZSetupTime;   



 
}FSMC_NAND_PCCARDTimingInitTypeDef;



 
typedef struct
{
  uint32_t FSMC_Bank;              
 

  uint32_t FSMC_Waitfeature;      
 

  uint32_t FSMC_MemoryDataWidth;  
 

  uint32_t FSMC_ECC;              
 

  uint32_t FSMC_ECCPageSize;      
 

  uint32_t FSMC_TCLRSetupTime;    

 

  uint32_t FSMC_TARSetupTime;     

  

  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_CommonSpaceTimingStruct;     

  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_AttributeSpaceTimingStruct;  
}FSMC_NANDInitTypeDef;



 

typedef struct
{
  uint32_t FSMC_Waitfeature;    
 

  uint32_t FSMC_TCLRSetupTime;  

 

  uint32_t FSMC_TARSetupTime;   

  

  
  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_CommonSpaceTimingStruct;  

  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_AttributeSpaceTimingStruct;    
  
  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_IOSpaceTimingStruct;    
}FSMC_PCCARDInitTypeDef;

 



 



 






 



   




 



     



 



















 



 







 



 

#line 308 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_fsmc.h"


 



 







 



 







 
    


 






 



 






 



 






 



 






 



 






 



 






 



 







 



 







 



 



 



 



 



 



 



 



 



 



 



 



 



 
#line 485 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_fsmc.h"


 



 
  


 



 






 




 






 



 
#line 535 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_fsmc.h"


 



 



 



 



 



 



 



 



 



 



 



 



 



 
#line 597 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_fsmc.h"


 



 
#line 612 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_fsmc.h"




 



 



 

 
  

 
void FSMC_NORSRAMDeInit(uint32_t FSMC_Bank);
void FSMC_NORSRAMInit(FSMC_NORSRAMInitTypeDef* FSMC_NORSRAMInitStruct);
void FSMC_NORSRAMStructInit(FSMC_NORSRAMInitTypeDef* FSMC_NORSRAMInitStruct);
void FSMC_NORSRAMCmd(uint32_t FSMC_Bank, FunctionalState NewState);

 
void FSMC_NANDDeInit(uint32_t FSMC_Bank);
void FSMC_NANDInit(FSMC_NANDInitTypeDef* FSMC_NANDInitStruct);
void FSMC_NANDStructInit(FSMC_NANDInitTypeDef* FSMC_NANDInitStruct);
void FSMC_NANDCmd(uint32_t FSMC_Bank, FunctionalState NewState);
void FSMC_NANDECCCmd(uint32_t FSMC_Bank, FunctionalState NewState);
uint32_t FSMC_GetECC(uint32_t FSMC_Bank);

 
void FSMC_PCCARDDeInit(void);
void FSMC_PCCARDInit(FSMC_PCCARDInitTypeDef* FSMC_PCCARDInitStruct);
void FSMC_PCCARDStructInit(FSMC_PCCARDInitTypeDef* FSMC_PCCARDInitStruct);
void FSMC_PCCARDCmd(FunctionalState NewState);

 
void FSMC_ITConfig(uint32_t FSMC_Bank, uint32_t FSMC_IT, FunctionalState NewState);
FlagStatus FSMC_GetFlagStatus(uint32_t FSMC_Bank, uint32_t FSMC_FLAG);
void FSMC_ClearFlag(uint32_t FSMC_Bank, uint32_t FSMC_FLAG);
ITStatus FSMC_GetITStatus(uint32_t FSMC_Bank, uint32_t FSMC_IT);
void FSMC_ClearITPendingBit(uint32_t FSMC_Bank, uint32_t FSMC_IT);








 



  

 
#line 45 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_hash.h"




















 

 







 
#line 33 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_hash.h"



 



  

 



  
typedef struct
{
  uint32_t HASH_AlgoSelection; 
 
  uint32_t HASH_AlgoMode;      
 
  uint32_t HASH_DataType;      

 
  uint32_t HASH_HMACKeyType;   
 
}HASH_InitTypeDef;



  
typedef struct
{
  uint32_t Data[5];      
 
} HASH_MsgDigest; 



  
typedef struct
{
  uint32_t HASH_IMR; 
  uint32_t HASH_STR;      
  uint32_t HASH_CR;     
  uint32_t HASH_CSR[51];       
}HASH_Context;

 



  



  







 



  







 



   











 



  







 



   




 



   





				   


 



   

















  



  

 
  
  
 
void HASH_DeInit(void);

 
void HASH_Init(HASH_InitTypeDef* HASH_InitStruct);
void HASH_StructInit(HASH_InitTypeDef* HASH_InitStruct);
void HASH_Reset(void);

 
void HASH_DataIn(uint32_t Data);
uint8_t HASH_GetInFIFOWordsNbr(void);
void HASH_SetLastWordValidBitsNbr(uint16_t ValidNumber);
void HASH_StartDigest(void);
void HASH_GetDigest(HASH_MsgDigest* HASH_MessageDigest);

 
void HASH_SaveContext(HASH_Context* HASH_ContextSave);
void HASH_RestoreContext(HASH_Context* HASH_ContextRestore);

 
void HASH_DMACmd(FunctionalState NewState);

 
void HASH_ITConfig(uint8_t HASH_IT, FunctionalState NewState);
FlagStatus HASH_GetFlagStatus(uint16_t HASH_FLAG);
void HASH_ClearFlag(uint16_t HASH_FLAG);
ITStatus HASH_GetITStatus(uint8_t HASH_IT);
void HASH_ClearITPendingBit(uint8_t HASH_IT);

 
ErrorStatus HASH_SHA1(uint8_t *Input, uint32_t Ilen, uint8_t Output[20]);
ErrorStatus HMAC_SHA1(uint8_t *Key, uint32_t Keylen,
                      uint8_t *Input, uint32_t Ilen,
                      uint8_t Output[20]);

 
ErrorStatus HASH_MD5(uint8_t *Input, uint32_t Ilen, uint8_t Output[16]);
ErrorStatus HMAC_MD5(uint8_t *Key, uint32_t Keylen,
                     uint8_t *Input, uint32_t Ilen,
                     uint8_t Output[16]);









  



  

 
#line 46 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_gpio.h"




















 

 







 
#line 33 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_gpio.h"



 



  

 

#line 53 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_gpio.h"
                                                                


    
typedef enum
{ 
  GPIO_Mode_IN   = 0x00,  
  GPIO_Mode_OUT  = 0x01,  
  GPIO_Mode_AF   = 0x02,  
  GPIO_Mode_AN   = 0x03   
}GPIOMode_TypeDef;





   
typedef enum
{ 
  GPIO_OType_PP = 0x00,
  GPIO_OType_OD = 0x01
}GPIOOType_TypeDef;





   
typedef enum
{ 
  GPIO_Speed_2MHz   = 0x00,  
  GPIO_Speed_25MHz  = 0x01,  
  GPIO_Speed_50MHz  = 0x02,  
  GPIO_Speed_100MHz = 0x03   
}GPIOSpeed_TypeDef;





  
typedef enum
{ 
  GPIO_PuPd_NOPULL = 0x00,
  GPIO_PuPd_UP     = 0x01,
  GPIO_PuPd_DOWN   = 0x02
}GPIOPuPd_TypeDef;





  
typedef enum
{ 
  Bit_RESET = 0,
  Bit_SET
}BitAction;





  
typedef struct
{
  uint32_t GPIO_Pin;              
 

  GPIOMode_TypeDef GPIO_Mode;     
 

  GPIOSpeed_TypeDef GPIO_Speed;   
 

  GPIOOType_TypeDef GPIO_OType;   
 

  GPIOPuPd_TypeDef GPIO_PuPd;     
 
}GPIO_InitTypeDef;

 



  



  
#line 161 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_gpio.h"

#line 179 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_gpio.h"


  




  
#line 203 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_gpio.h"

#line 220 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_gpio.h"


  



  


  








  





  






  







  






  





  




  







  






  








  





  




  






  




  


#line 345 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_gpio.h"


  



 
    








 



 

 
  

 
void GPIO_DeInit(GPIO_TypeDef* GPIOx);

 
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_PinLockConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

 
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx);
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal);
void GPIO_ToggleBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

 
void GPIO_PinAFConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinSource, uint8_t GPIO_AF);









  



  

 
#line 47 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_i2c.h"




















  

 







 
#line 33 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_i2c.h"



 



 

 



 

typedef struct
{
  uint32_t I2C_ClockSpeed;          
 

  uint16_t I2C_Mode;                
 

  uint16_t I2C_DutyCycle;           
 

  uint16_t I2C_OwnAddress1;         
 

  uint16_t I2C_Ack;                 
 

  uint16_t I2C_AcknowledgedAddress; 
 
}I2C_InitTypeDef;

 




 






 

#line 89 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_i2c.h"


 



 







  



 







 



 







 



 







  



 

#line 163 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_i2c.h"


 



 







  



 







 



 







  



 







  



 

#line 233 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_i2c.h"



#line 243 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_i2c.h"


 



 



 

#line 262 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_i2c.h"



 

#line 281 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_i2c.h"



#line 295 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_i2c.h"


 



 





 








 
 

























 

 


 





























 

  
 


 
 

 







 

























 

    
 



 



 



























 

  
 

 


 
 


 






 

#line 501 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_i2c.h"


 



 




 



 




 



 

 
  

 
void I2C_DeInit(I2C_TypeDef* I2Cx);

 
void I2C_Init(I2C_TypeDef* I2Cx, I2C_InitTypeDef* I2C_InitStruct);
void I2C_StructInit(I2C_InitTypeDef* I2C_InitStruct);
void I2C_Cmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GenerateSTART(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GenerateSTOP(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_Send7bitAddress(I2C_TypeDef* I2Cx, uint8_t Address, uint8_t I2C_Direction);
void I2C_AcknowledgeConfig(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_OwnAddress2Config(I2C_TypeDef* I2Cx, uint8_t Address);
void I2C_DualAddressCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GeneralCallCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_SoftwareResetCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_StretchClockCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_FastModeDutyCycleConfig(I2C_TypeDef* I2Cx, uint16_t I2C_DutyCycle);
void I2C_NACKPositionConfig(I2C_TypeDef* I2Cx, uint16_t I2C_NACKPosition);
void I2C_SMBusAlertConfig(I2C_TypeDef* I2Cx, uint16_t I2C_SMBusAlert);
void I2C_ARPCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);

  
void I2C_SendData(I2C_TypeDef* I2Cx, uint8_t Data);
uint8_t I2C_ReceiveData(I2C_TypeDef* I2Cx);

  
void I2C_TransmitPEC(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_PECPositionConfig(I2C_TypeDef* I2Cx, uint16_t I2C_PECPosition);
void I2C_CalculatePEC(I2C_TypeDef* I2Cx, FunctionalState NewState);
uint8_t I2C_GetPEC(I2C_TypeDef* I2Cx);

 
void I2C_DMACmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_DMALastTransferCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);

 
uint16_t I2C_ReadRegister(I2C_TypeDef* I2Cx, uint8_t I2C_Register);
void I2C_ITConfig(I2C_TypeDef* I2Cx, uint16_t I2C_IT, FunctionalState NewState);




















































































 





 
ErrorStatus I2C_CheckEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT);




 
uint32_t I2C_GetLastEvent(I2C_TypeDef* I2Cx);




 
FlagStatus I2C_GetFlagStatus(I2C_TypeDef* I2Cx, uint32_t I2C_FLAG);


void I2C_ClearFlag(I2C_TypeDef* I2Cx, uint32_t I2C_FLAG);
ITStatus I2C_GetITStatus(I2C_TypeDef* I2Cx, uint32_t I2C_IT);
void I2C_ClearITPendingBit(I2C_TypeDef* I2Cx, uint32_t I2C_IT);









  



  

 
#line 48 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_iwdg.h"




















 

 







 
#line 33 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_iwdg.h"



 



 

 
 



 
  


 






 



 
#line 77 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_iwdg.h"


 



 






 



 

 
 

 
void IWDG_WriteAccessCmd(uint16_t IWDG_WriteAccess);
void IWDG_SetPrescaler(uint8_t IWDG_Prescaler);
void IWDG_SetReload(uint16_t Reload);
void IWDG_ReloadCounter(void);

 
void IWDG_Enable(void);

 
FlagStatus IWDG_GetFlagStatus(uint16_t IWDG_FLAG);









 



 

 
#line 49 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_pwr.h"




















  

 







 
#line 33 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_pwr.h"



 



  

 
 



  



  

#line 61 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_pwr.h"







 

  


 







 



 







 




 


 



 









 



 








 



 

 
  

  
void PWR_DeInit(void);

  
void PWR_BackupAccessCmd(FunctionalState NewState);

  
void PWR_PVDLevelConfig(uint32_t PWR_PVDLevel);
void PWR_PVDCmd(FunctionalState NewState);

  
void PWR_WakeUpPinCmd(FunctionalState NewState);

  
void PWR_BackupRegulatorCmd(FunctionalState NewState);
void PWR_MainRegulatorModeConfig(uint32_t PWR_Regulator_Voltage);

  
void PWR_FlashPowerDownCmd(FunctionalState NewState);

  
void PWR_EnterSTOPMode(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry);
void PWR_EnterSTANDBYMode(void);

  
FlagStatus PWR_GetFlagStatus(uint32_t PWR_FLAG);
void PWR_ClearFlag(uint32_t PWR_FLAG);









 



 

 
#line 50 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rcc.h"



















 

 







 
#line 32 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rcc.h"



 



  

 
typedef struct
{
  uint32_t SYSCLK_Frequency;  
  uint32_t HCLK_Frequency;    
  uint32_t PCLK1_Frequency;   
  uint32_t PCLK2_Frequency;   
}RCC_ClocksTypeDef;

 



 
  


 







  
  


 
#line 79 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rcc.h"
 




  
  


 
#line 95 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rcc.h"


  
  


 
#line 116 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rcc.h"


  
  


 
#line 131 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rcc.h"


  
  


 
#line 151 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rcc.h"


  
  


 







  
  


 
#line 234 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rcc.h"


  
  


 






  
  


  
#line 278 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rcc.h"


  
  


   
#line 291 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rcc.h"


  
  


  




  
  


  
#line 331 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rcc.h"


  
  


  
#line 354 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rcc.h"


  
  


 
#line 372 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rcc.h"
                                   





  
  


 
#line 394 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rcc.h"
                                   





  
  


 
#line 426 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rcc.h"


  



  

 
  

 
void RCC_DeInit(void);

 
void RCC_HSEConfig(uint8_t RCC_HSE);
ErrorStatus RCC_WaitForHSEStartUp(void);
void RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue);
void RCC_HSICmd(FunctionalState NewState);
void RCC_LSEConfig(uint8_t RCC_LSE);
void RCC_LSICmd(FunctionalState NewState);

void RCC_PLLConfig(uint32_t RCC_PLLSource, uint32_t PLLM, uint32_t PLLN, uint32_t PLLP, uint32_t PLLQ);
void RCC_PLLCmd(FunctionalState NewState);
void RCC_PLLI2SConfig(uint32_t PLLI2SN, uint32_t PLLI2SR);
void RCC_PLLI2SCmd(FunctionalState NewState);

void RCC_ClockSecuritySystemCmd(FunctionalState NewState);
void RCC_MCO1Config(uint32_t RCC_MCO1Source, uint32_t RCC_MCO1Div);
void RCC_MCO2Config(uint32_t RCC_MCO2Source, uint32_t RCC_MCO2Div);

 
void RCC_SYSCLKConfig(uint32_t RCC_SYSCLKSource);
uint8_t RCC_GetSYSCLKSource(void);
void RCC_HCLKConfig(uint32_t RCC_SYSCLK);
void RCC_PCLK1Config(uint32_t RCC_HCLK);
void RCC_PCLK2Config(uint32_t RCC_HCLK);
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);

 
void RCC_RTCCLKConfig(uint32_t RCC_RTCCLKSource);
void RCC_RTCCLKCmd(FunctionalState NewState);
void RCC_BackupResetCmd(FunctionalState NewState);
void RCC_I2SCLKConfig(uint32_t RCC_I2SCLKSource); 

void RCC_AHB1PeriphClockCmd(uint32_t RCC_AHB1Periph, FunctionalState NewState);
void RCC_AHB2PeriphClockCmd(uint32_t RCC_AHB2Periph, FunctionalState NewState);
void RCC_AHB3PeriphClockCmd(uint32_t RCC_AHB3Periph, FunctionalState NewState);
void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);

void RCC_AHB1PeriphResetCmd(uint32_t RCC_AHB1Periph, FunctionalState NewState);
void RCC_AHB2PeriphResetCmd(uint32_t RCC_AHB2Periph, FunctionalState NewState);
void RCC_AHB3PeriphResetCmd(uint32_t RCC_AHB3Periph, FunctionalState NewState);
void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);

void RCC_AHB1PeriphClockLPModeCmd(uint32_t RCC_AHB1Periph, FunctionalState NewState);
void RCC_AHB2PeriphClockLPModeCmd(uint32_t RCC_AHB2Periph, FunctionalState NewState);
void RCC_AHB3PeriphClockLPModeCmd(uint32_t RCC_AHB3Periph, FunctionalState NewState);
void RCC_APB1PeriphClockLPModeCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void RCC_APB2PeriphClockLPModeCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);

 
void RCC_ITConfig(uint8_t RCC_IT, FunctionalState NewState);
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG);
void RCC_ClearFlag(void);
ITStatus RCC_GetITStatus(uint8_t RCC_IT);
void RCC_ClearITPendingBit(uint8_t RCC_IT);









  



  

 
#line 51 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rng.h"




















 

 







 
#line 33 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rng.h"



 



  

 
  



 
  


  











  



   







  



  

 
  

  
void RNG_DeInit(void);

 
void RNG_Cmd(FunctionalState NewState);

 
uint32_t RNG_GetRandomNumber(void);

 
void RNG_ITConfig(FunctionalState NewState);
FlagStatus RNG_GetFlagStatus(uint8_t RNG_FLAG);
void RNG_ClearFlag(uint8_t RNG_FLAG);
ITStatus RNG_GetITStatus(uint8_t RNG_IT);
void RNG_ClearITPendingBit(uint8_t RNG_IT);









  



  

 
#line 52 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rtc.h"




















 

 







 
#line 33 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rtc.h"



 



  

 



  
typedef struct
{
  uint32_t RTC_HourFormat;   
 
  
  uint32_t RTC_AsynchPrediv; 
 
  
  uint32_t RTC_SynchPrediv;  
 
}RTC_InitTypeDef;



 
typedef struct
{
  uint8_t RTC_Hours;    


 

  uint8_t RTC_Minutes;  
 
  
  uint8_t RTC_Seconds;  
 

  uint8_t RTC_H12;      
 
}RTC_TimeTypeDef; 



 
typedef struct
{
  uint8_t RTC_WeekDay; 
 
  
  uint8_t RTC_Month;   
 

  uint8_t RTC_Date;     
 
  
  uint8_t RTC_Year;     
 
}RTC_DateTypeDef;



 
typedef struct
{
  RTC_TimeTypeDef RTC_AlarmTime;      

  uint32_t RTC_AlarmMask;            
 

  uint32_t RTC_AlarmDateWeekDaySel;  
 
  
  uint8_t RTC_AlarmDateWeekDay;      



 
}RTC_AlarmTypeDef;

 



  




  






  



  

 


  




  




  



  







  



  






  



  




  



  

 
#line 205 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rtc.h"



  



  
  
#line 228 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rtc.h"


  




  
#line 244 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rtc.h"



  




  








  




  
#line 274 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rtc.h"



  



  







  

  

  
#line 343 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rtc.h"


  



  





  



  
#line 373 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rtc.h"


  



  






  



  




 







  



  






  




  








  

 

  






  



  
#line 453 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rtc.h"
                                          


  



  
#line 468 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rtc.h"



  



  




 



  











  



  
#line 509 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rtc.h"



  



  


#line 529 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rtc.h"


  



  
#line 560 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rtc.h"



 

  

  
#line 576 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rtc.h"







 



  





 



  






  



  






  



  







  



  






  



  




 



 

#line 693 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rtc.h"


  



  






  



  
#line 733 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rtc.h"


  



  
#line 746 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_rtc.h"









  



  





  



  

 
  

 
ErrorStatus RTC_DeInit(void);

 
ErrorStatus RTC_Init(RTC_InitTypeDef* RTC_InitStruct);
void RTC_StructInit(RTC_InitTypeDef* RTC_InitStruct);
void RTC_WriteProtectionCmd(FunctionalState NewState);
ErrorStatus RTC_EnterInitMode(void);
void RTC_ExitInitMode(void);
ErrorStatus RTC_WaitForSynchro(void);
ErrorStatus RTC_RefClockCmd(FunctionalState NewState);
void RTC_BypassShadowCmd(FunctionalState NewState);

 
ErrorStatus RTC_SetTime(uint32_t RTC_Format, RTC_TimeTypeDef* RTC_TimeStruct);
void RTC_TimeStructInit(RTC_TimeTypeDef* RTC_TimeStruct);
void RTC_GetTime(uint32_t RTC_Format, RTC_TimeTypeDef* RTC_TimeStruct);
uint32_t RTC_GetSubSecond(void);
ErrorStatus RTC_SetDate(uint32_t RTC_Format, RTC_DateTypeDef* RTC_DateStruct);
void RTC_DateStructInit(RTC_DateTypeDef* RTC_DateStruct);
void RTC_GetDate(uint32_t RTC_Format, RTC_DateTypeDef* RTC_DateStruct);

 
void RTC_SetAlarm(uint32_t RTC_Format, uint32_t RTC_Alarm, RTC_AlarmTypeDef* RTC_AlarmStruct);
void RTC_AlarmStructInit(RTC_AlarmTypeDef* RTC_AlarmStruct);
void RTC_GetAlarm(uint32_t RTC_Format, uint32_t RTC_Alarm, RTC_AlarmTypeDef* RTC_AlarmStruct);
ErrorStatus RTC_AlarmCmd(uint32_t RTC_Alarm, FunctionalState NewState);
void RTC_AlarmSubSecondConfig(uint32_t RTC_Alarm, uint32_t RTC_AlarmSubSecondValue, uint32_t RTC_AlarmSubSecondMask);
uint32_t RTC_GetAlarmSubSecond(uint32_t RTC_Alarm);

 
void RTC_WakeUpClockConfig(uint32_t RTC_WakeUpClock);
void RTC_SetWakeUpCounter(uint32_t RTC_WakeUpCounter);
uint32_t RTC_GetWakeUpCounter(void);
ErrorStatus RTC_WakeUpCmd(FunctionalState NewState);

 
void RTC_DayLightSavingConfig(uint32_t RTC_DayLightSaving, uint32_t RTC_StoreOperation);
uint32_t RTC_GetStoreOperation(void);

 
void RTC_OutputConfig(uint32_t RTC_Output, uint32_t RTC_OutputPolarity);

 
ErrorStatus RTC_CoarseCalibConfig(uint32_t RTC_CalibSign, uint32_t Value);
ErrorStatus RTC_CoarseCalibCmd(FunctionalState NewState);
void RTC_CalibOutputCmd(FunctionalState NewState);
void RTC_CalibOutputConfig(uint32_t RTC_CalibOutput);
ErrorStatus RTC_SmoothCalibConfig(uint32_t RTC_SmoothCalibPeriod, 
                                  uint32_t RTC_SmoothCalibPlusPulses,
                                  uint32_t RTC_SmouthCalibMinusPulsesValue);

 
void RTC_TimeStampCmd(uint32_t RTC_TimeStampEdge, FunctionalState NewState);
void RTC_GetTimeStamp(uint32_t RTC_Format, RTC_TimeTypeDef* RTC_StampTimeStruct,
                                      RTC_DateTypeDef* RTC_StampDateStruct);
uint32_t RTC_GetTimeStampSubSecond(void);

 
void RTC_TamperTriggerConfig(uint32_t RTC_Tamper, uint32_t RTC_TamperTrigger);
void RTC_TamperCmd(uint32_t RTC_Tamper, FunctionalState NewState);
void RTC_TamperFilterConfig(uint32_t RTC_TamperFilter);
void RTC_TamperSamplingFreqConfig(uint32_t RTC_TamperSamplingFreq);
void RTC_TamperPinsPrechargeDuration(uint32_t RTC_TamperPrechargeDuration);
void RTC_TimeStampOnTamperDetectionCmd(FunctionalState NewState);
void RTC_TamperPullUpCmd(FunctionalState NewState);

 
void RTC_WriteBackupRegister(uint32_t RTC_BKP_DR, uint32_t Data);
uint32_t RTC_ReadBackupRegister(uint32_t RTC_BKP_DR);


 
void RTC_TamperPinSelection(uint32_t RTC_TamperPin);
void RTC_TimeStampPinSelection(uint32_t RTC_TimeStampPin);
void RTC_OutputTypeConfig(uint32_t RTC_OutputType);

 
ErrorStatus RTC_SynchroShiftConfig(uint32_t RTC_ShiftAdd1S, uint32_t RTC_ShiftSubFS);

 
void RTC_ITConfig(uint32_t RTC_IT, FunctionalState NewState);
FlagStatus RTC_GetFlagStatus(uint32_t RTC_FLAG);
void RTC_ClearFlag(uint32_t RTC_FLAG);
ITStatus RTC_GetITStatus(uint32_t RTC_IT);
void RTC_ClearITPendingBit(uint32_t RTC_IT);









  



  

 
#line 53 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_sdio.h"




















 

 







 
#line 33 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_sdio.h"



 



 

 

typedef struct
{
  uint32_t SDIO_ClockEdge;            
 

  uint32_t SDIO_ClockBypass;          

 

  uint32_t SDIO_ClockPowerSave;       

 

  uint32_t SDIO_BusWide;              
 

  uint32_t SDIO_HardwareFlowControl;  
 

  uint8_t SDIO_ClockDiv;              
 
                                           
} SDIO_InitTypeDef;

typedef struct
{
  uint32_t SDIO_Argument;  


 

  uint32_t SDIO_CmdIndex;   

  uint32_t SDIO_Response;  
 

  uint32_t SDIO_Wait;      
 

  uint32_t SDIO_CPSM;      

 
} SDIO_CmdInitTypeDef;

typedef struct
{
  uint32_t SDIO_DataTimeOut;     

  uint32_t SDIO_DataLength;      
 
  uint32_t SDIO_DataBlockSize;  
 
 
  uint32_t SDIO_TransferDir;    

 
 
  uint32_t SDIO_TransferMode;   
 
 
  uint32_t SDIO_DPSM;           

 
} SDIO_DataInitTypeDef;


 



 



 







 



 







  



 







 



 









 



 







 



 






  




 

#line 219 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_sdio.h"


  



 




 



 

#line 242 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_sdio.h"


 



 








 



 






  



 

#line 280 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_sdio.h"


 



 




 



 

#line 327 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_sdio.h"


 



 







 



 







 



 






 



 

#line 418 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_sdio.h"



#line 445 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_sdio.h"





 



 







 



 

 
 
 
void SDIO_DeInit(void);

 
void SDIO_Init(SDIO_InitTypeDef* SDIO_InitStruct);
void SDIO_StructInit(SDIO_InitTypeDef* SDIO_InitStruct);
void SDIO_ClockCmd(FunctionalState NewState);
void SDIO_SetPowerState(uint32_t SDIO_PowerState);
uint32_t SDIO_GetPowerState(void);

 
void SDIO_SendCommand(SDIO_CmdInitTypeDef *SDIO_CmdInitStruct);
void SDIO_CmdStructInit(SDIO_CmdInitTypeDef* SDIO_CmdInitStruct);
uint8_t SDIO_GetCommandResponse(void);
uint32_t SDIO_GetResponse(uint32_t SDIO_RESP);

 
void SDIO_DataConfig(SDIO_DataInitTypeDef* SDIO_DataInitStruct);
void SDIO_DataStructInit(SDIO_DataInitTypeDef* SDIO_DataInitStruct);
uint32_t SDIO_GetDataCounter(void);
uint32_t SDIO_ReadData(void);
void SDIO_WriteData(uint32_t Data);
uint32_t SDIO_GetFIFOCount(void);

 
void SDIO_StartSDIOReadWait(FunctionalState NewState);
void SDIO_StopSDIOReadWait(FunctionalState NewState);
void SDIO_SetSDIOReadWaitMode(uint32_t SDIO_ReadWaitMode);
void SDIO_SetSDIOOperation(FunctionalState NewState);
void SDIO_SendSDIOSuspendCmd(FunctionalState NewState);

 
void SDIO_CommandCompletionCmd(FunctionalState NewState);
void SDIO_CEATAITCmd(FunctionalState NewState);
void SDIO_SendCEATACmd(FunctionalState NewState);

 
void SDIO_DMACmd(FunctionalState NewState);

 
void SDIO_ITConfig(uint32_t SDIO_IT, FunctionalState NewState);
FlagStatus SDIO_GetFlagStatus(uint32_t SDIO_FLAG);
void SDIO_ClearFlag(uint32_t SDIO_FLAG);
ITStatus SDIO_GetITStatus(uint32_t SDIO_IT);
void SDIO_ClearITPendingBit(uint32_t SDIO_IT);









 



 

 
#line 54 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_spi.h"




















  

 







 
#line 33 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_spi.h"



 



  

 



 

typedef struct
{
  uint16_t SPI_Direction;           
 

  uint16_t SPI_Mode;                
 

  uint16_t SPI_DataSize;            
 

  uint16_t SPI_CPOL;                
 

  uint16_t SPI_CPHA;                
 

  uint16_t SPI_NSS;                 

 
 
  uint16_t SPI_BaudRatePrescaler;   



 

  uint16_t SPI_FirstBit;            
 

  uint16_t SPI_CRCPolynomial;        
}SPI_InitTypeDef;



 

typedef struct
{

  uint16_t I2S_Mode;         
 

  uint16_t I2S_Standard;     
 

  uint16_t I2S_DataFormat;   
 

  uint16_t I2S_MCLKOutput;   
 

  uint32_t I2S_AudioFreq;    
 

  uint16_t I2S_CPOL;         
 
}I2S_InitTypeDef;

 



 

























 
  
#line 147 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_spi.h"


 



 







 



 







  



 







 



 







 



 







  



 

#line 231 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_spi.h"


  



 







 



 

#line 259 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_spi.h"


 
  



 

#line 278 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_spi.h"


 
  


 

#line 294 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_spi.h"


 



 







 



 

#line 324 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_spi.h"






 
            


 







 



 






 



 







 



 






 



 







 



 























 



 

#line 431 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_spi.h"

#line 438 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_spi.h"


 



 




 



 

#line 474 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_spi.h"


 
  


 

 
  

  
void SPI_I2S_DeInit(SPI_TypeDef* SPIx);

 
void SPI_Init(SPI_TypeDef* SPIx, SPI_InitTypeDef* SPI_InitStruct);
void I2S_Init(SPI_TypeDef* SPIx, I2S_InitTypeDef* I2S_InitStruct);
void SPI_StructInit(SPI_InitTypeDef* SPI_InitStruct);
void I2S_StructInit(I2S_InitTypeDef* I2S_InitStruct);
void SPI_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void I2S_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_DataSizeConfig(SPI_TypeDef* SPIx, uint16_t SPI_DataSize);
void SPI_BiDirectionalLineConfig(SPI_TypeDef* SPIx, uint16_t SPI_Direction);
void SPI_NSSInternalSoftwareConfig(SPI_TypeDef* SPIx, uint16_t SPI_NSSInternalSoft);
void SPI_SSOutputCmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_TIModeCmd(SPI_TypeDef* SPIx, FunctionalState NewState);

void I2S_FullDuplexConfig(SPI_TypeDef* I2Sxext, I2S_InitTypeDef* I2S_InitStruct);

  
void SPI_I2S_SendData(SPI_TypeDef* SPIx, uint16_t Data);
uint16_t SPI_I2S_ReceiveData(SPI_TypeDef* SPIx);

 
void SPI_CalculateCRC(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_TransmitCRC(SPI_TypeDef* SPIx);
uint16_t SPI_GetCRC(SPI_TypeDef* SPIx, uint8_t SPI_CRC);
uint16_t SPI_GetCRCPolynomial(SPI_TypeDef* SPIx);

 
void SPI_I2S_DMACmd(SPI_TypeDef* SPIx, uint16_t SPI_I2S_DMAReq, FunctionalState NewState);

 
void SPI_I2S_ITConfig(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT, FunctionalState NewState);
FlagStatus SPI_I2S_GetFlagStatus(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG);
void SPI_I2S_ClearFlag(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG);
ITStatus SPI_I2S_GetITStatus(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT);
void SPI_I2S_ClearITPendingBit(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT);









 



 

 
#line 55 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_syscfg.h"




















 

 







 
#line 33 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_syscfg.h"



 



  

 
 
  


  



  
#line 61 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_syscfg.h"
                                      
#line 71 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_syscfg.h"


  




  
#line 111 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_syscfg.h"


  




  




   






  




  







  



  

 
  
 
void SYSCFG_DeInit(void);
void SYSCFG_MemoryRemapConfig(uint8_t SYSCFG_MemoryRemap);
void SYSCFG_EXTILineConfig(uint8_t EXTI_PortSourceGPIOx, uint8_t EXTI_PinSourcex);
void SYSCFG_ETH_MediaInterfaceConfig(uint32_t SYSCFG_ETH_MediaInterface); 
void SYSCFG_CompensationCellCmd(FunctionalState NewState); 
FlagStatus SYSCFG_GetCompensationCellStatus(void);









  



  

 
#line 56 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"




















 

 







 
#line 33 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"



 



  

 




 

typedef struct
{
  uint16_t TIM_Prescaler;         
 

  uint16_t TIM_CounterMode;       
 

  uint32_t TIM_Period;            

  

  uint16_t TIM_ClockDivision;     
 

  uint8_t TIM_RepetitionCounter;  






 
} TIM_TimeBaseInitTypeDef; 



 

typedef struct
{
  uint16_t TIM_OCMode;        
 

  uint16_t TIM_OutputState;   
 

  uint16_t TIM_OutputNState;  

 

  uint32_t TIM_Pulse;         
 

  uint16_t TIM_OCPolarity;    
 

  uint16_t TIM_OCNPolarity;   

 

  uint16_t TIM_OCIdleState;   

 

  uint16_t TIM_OCNIdleState;  

 
} TIM_OCInitTypeDef;



 

typedef struct
{

  uint16_t TIM_Channel;      
 

  uint16_t TIM_ICPolarity;   
 

  uint16_t TIM_ICSelection;  
 

  uint16_t TIM_ICPrescaler;  
 

  uint16_t TIM_ICFilter;     
 
} TIM_ICInitTypeDef;




 

typedef struct
{

  uint16_t TIM_OSSRState;        
 

  uint16_t TIM_OSSIState;        
 

  uint16_t TIM_LOCKLevel;        
  

  uint16_t TIM_DeadTime;         

 

  uint16_t TIM_Break;            
 

  uint16_t TIM_BreakPolarity;    
 

  uint16_t TIM_AutomaticOutput;  
 
} TIM_BDTRInitTypeDef;

 



 

#line 183 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"
                                          
#line 196 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"
                                     
 
#line 206 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"
 
#line 213 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"
 


 
#line 225 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"
                                






 

#line 254 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"


 



 







  



 





                                 




                                 







  



 

#line 303 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"


 



 

#line 321 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"


  



 







 



 
  






 



 







  



 







  



 







  



 







  



 







  



 







  



 







  



 

#line 445 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"


  



 







 



 







  



 







  



 







  



 

#line 507 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"


  



 

#line 523 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"


  



 

#line 539 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"


  



 

#line 556 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"

#line 565 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"


  



 

#line 613 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"


  



 

#line 657 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"


  



 

#line 673 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"



  



 

#line 690 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"


  



 

#line 718 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"


  



 







  



  






 



 







  



 







  



 

#line 779 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"


  




 

#line 797 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"
  


  



 

#line 812 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"


  



 







  



 





                                     


  



 







  



 

#line 873 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"


  



 

#line 889 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"


  



 







  


 














#line 931 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"



  


 

#line 963 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"



  



 




  



 




  



 

#line 1008 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_tim.h"


 



 

 
  

 
void TIM_DeInit(TIM_TypeDef* TIMx);
void TIM_TimeBaseInit(TIM_TypeDef* TIMx, TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
void TIM_PrescalerConfig(TIM_TypeDef* TIMx, uint16_t Prescaler, uint16_t TIM_PSCReloadMode);
void TIM_CounterModeConfig(TIM_TypeDef* TIMx, uint16_t TIM_CounterMode);
void TIM_SetCounter(TIM_TypeDef* TIMx, uint32_t Counter);
void TIM_SetAutoreload(TIM_TypeDef* TIMx, uint32_t Autoreload);
uint32_t TIM_GetCounter(TIM_TypeDef* TIMx);
uint16_t TIM_GetPrescaler(TIM_TypeDef* TIMx);
void TIM_UpdateDisableConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_UpdateRequestConfig(TIM_TypeDef* TIMx, uint16_t TIM_UpdateSource);
void TIM_ARRPreloadConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectOnePulseMode(TIM_TypeDef* TIMx, uint16_t TIM_OPMode);
void TIM_SetClockDivision(TIM_TypeDef* TIMx, uint16_t TIM_CKD);
void TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState);

 
void TIM_OC1Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC2Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC3Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC4Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OCStructInit(TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_SelectOCxM(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_OCMode);
void TIM_SetCompare1(TIM_TypeDef* TIMx, uint32_t Compare1);
void TIM_SetCompare2(TIM_TypeDef* TIMx, uint32_t Compare2);
void TIM_SetCompare3(TIM_TypeDef* TIMx, uint32_t Compare3);
void TIM_SetCompare4(TIM_TypeDef* TIMx, uint32_t Compare4);
void TIM_ForcedOC1Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC2Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC3Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC4Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_OC1PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC2PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC3PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC4PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC1FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC2FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC3FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC4FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_ClearOC1Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC2Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC3Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC4Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_OC1PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC1NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC2PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC2NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC3PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC3NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC4PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_CCxCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCx);
void TIM_CCxNCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCxN);

 
void TIM_ICInit(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_ICStructInit(TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_PWMIConfig(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
uint32_t TIM_GetCapture1(TIM_TypeDef* TIMx);
uint32_t TIM_GetCapture2(TIM_TypeDef* TIMx);
uint32_t TIM_GetCapture3(TIM_TypeDef* TIMx);
uint32_t TIM_GetCapture4(TIM_TypeDef* TIMx);
void TIM_SetIC1Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC2Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC3Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC4Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);

 
void TIM_BDTRConfig(TIM_TypeDef* TIMx, TIM_BDTRInitTypeDef *TIM_BDTRInitStruct);
void TIM_BDTRStructInit(TIM_BDTRInitTypeDef* TIM_BDTRInitStruct);
void TIM_CtrlPWMOutputs(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectCOM(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_CCPreloadControl(TIM_TypeDef* TIMx, FunctionalState NewState);

 
void TIM_ITConfig(TIM_TypeDef* TIMx, uint16_t TIM_IT, FunctionalState NewState);
void TIM_GenerateEvent(TIM_TypeDef* TIMx, uint16_t TIM_EventSource);
FlagStatus TIM_GetFlagStatus(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
void TIM_ClearFlag(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
ITStatus TIM_GetITStatus(TIM_TypeDef* TIMx, uint16_t TIM_IT);
void TIM_ClearITPendingBit(TIM_TypeDef* TIMx, uint16_t TIM_IT);
void TIM_DMAConfig(TIM_TypeDef* TIMx, uint16_t TIM_DMABase, uint16_t TIM_DMABurstLength);
void TIM_DMACmd(TIM_TypeDef* TIMx, uint16_t TIM_DMASource, FunctionalState NewState);
void TIM_SelectCCDMA(TIM_TypeDef* TIMx, FunctionalState NewState);

 
void TIM_InternalClockConfig(TIM_TypeDef* TIMx);
void TIM_ITRxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
void TIM_TIxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_TIxExternalCLKSource,
                                uint16_t TIM_ICPolarity, uint16_t ICFilter);
void TIM_ETRClockMode1Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                             uint16_t ExtTRGFilter);
void TIM_ETRClockMode2Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, 
                             uint16_t TIM_ExtTRGPolarity, uint16_t ExtTRGFilter);

 
void TIM_SelectInputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
void TIM_SelectOutputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_TRGOSource);
void TIM_SelectSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_SlaveMode);
void TIM_SelectMasterSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_MasterSlaveMode);
void TIM_ETRConfig(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                   uint16_t ExtTRGFilter);

    
void TIM_EncoderInterfaceConfig(TIM_TypeDef* TIMx, uint16_t TIM_EncoderMode,
                                uint16_t TIM_IC1Polarity, uint16_t TIM_IC2Polarity);
void TIM_SelectHallSensor(TIM_TypeDef* TIMx, FunctionalState NewState);

 
void TIM_RemapConfig(TIM_TypeDef* TIMx, uint16_t TIM_Remap);









  



 

 
#line 57 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_usart.h"




















  

 







 
#line 33 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_usart.h"



 



  

  



  
  
typedef struct
{
  uint32_t USART_BaudRate;            



 

  uint16_t USART_WordLength;          
 

  uint16_t USART_StopBits;            
 

  uint16_t USART_Parity;              




 
 
  uint16_t USART_Mode;                
 

  uint16_t USART_HardwareFlowControl; 

 
} USART_InitTypeDef;



  
  
typedef struct
{

  uint16_t USART_Clock;   
 

  uint16_t USART_CPOL;    
 

  uint16_t USART_CPHA;    
 

  uint16_t USART_LastBit; 

 
} USART_ClockInitTypeDef;

 



  
  
#line 110 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_usart.h"








  
  


                                    




  



  
  
#line 141 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_usart.h"


  



  
  
#line 155 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_usart.h"


  



  
  





  



  
#line 182 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_usart.h"


  



  






  



 
  






  



 







 



 







  



 
  
#line 249 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_usart.h"



 



 

#line 270 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_usart.h"


 



 







  



 







 



 
  







 



 







  



 

#line 342 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_usart.h"
                              








  



  

 
   

  
void USART_DeInit(USART_TypeDef* USARTx);

 
void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct);
void USART_StructInit(USART_InitTypeDef* USART_InitStruct);
void USART_ClockInit(USART_TypeDef* USARTx, USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_ClockStructInit(USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SetPrescaler(USART_TypeDef* USARTx, uint8_t USART_Prescaler);
void USART_OverSampling8Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_OneBitMethodCmd(USART_TypeDef* USARTx, FunctionalState NewState);

  
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
uint16_t USART_ReceiveData(USART_TypeDef* USARTx);

 
void USART_SetAddress(USART_TypeDef* USARTx, uint8_t USART_Address);
void USART_WakeUpConfig(USART_TypeDef* USARTx, uint16_t USART_WakeUp);
void USART_ReceiverWakeUpCmd(USART_TypeDef* USARTx, FunctionalState NewState);

 
void USART_LINBreakDetectLengthConfig(USART_TypeDef* USARTx, uint16_t USART_LINBreakDetectLength);
void USART_LINCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SendBreak(USART_TypeDef* USARTx);

 
void USART_HalfDuplexCmd(USART_TypeDef* USARTx, FunctionalState NewState);

 
void USART_SmartCardCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SmartCardNACKCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SetGuardTime(USART_TypeDef* USARTx, uint8_t USART_GuardTime);

 
void USART_IrDAConfig(USART_TypeDef* USARTx, uint16_t USART_IrDAMode);
void USART_IrDACmd(USART_TypeDef* USARTx, FunctionalState NewState);

 
void USART_DMACmd(USART_TypeDef* USARTx, uint16_t USART_DMAReq, FunctionalState NewState);

 
void USART_ITConfig(USART_TypeDef* USARTx, uint16_t USART_IT, FunctionalState NewState);
FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG);
void USART_ClearFlag(USART_TypeDef* USARTx, uint16_t USART_FLAG);
ITStatus USART_GetITStatus(USART_TypeDef* USARTx, uint16_t USART_IT);
void USART_ClearITPendingBit(USART_TypeDef* USARTx, uint16_t USART_IT);









  



  

 
#line 58 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_wwdg.h"




















 

 







 
#line 33 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_wwdg.h"



 



  

 
 



  
  


 
  
#line 63 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\stm32f4xx_wwdg.h"



  



  

 
 
  
   
void WWDG_DeInit(void);

 
void WWDG_SetPrescaler(uint32_t WWDG_Prescaler);
void WWDG_SetWindowValue(uint8_t WindowValue);
void WWDG_EnableIT(void);
void WWDG_SetCounter(uint8_t Counter);

 
void WWDG_Enable(uint8_t Counter);

 
FlagStatus WWDG_GetFlagStatus(void);
void WWDG_ClearFlag(void);









  



  

 
#line 59 ".\\inc\\stm32f4xx_conf.h"
#line 1 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\misc.h"




















 
	

   



 







 
#line 38 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\misc.h"



 



 

 



 

typedef struct
{
  uint8_t NVIC_IRQChannel;                    


 

  uint8_t NVIC_IRQChannelPreemptionPriority;  


 

  uint8_t NVIC_IRQChannelSubPriority;         


 

  FunctionalState NVIC_IRQChannelCmd;         

    
} NVIC_InitTypeDef;
 
 



 



 







 



 

#line 103 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\misc.h"


 



 

#line 121 ".\\Libraries\\STM32F4xx_StdPeriph_Driver\\misc.h"















 



 







 



 

 
 

void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup);
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset);
void NVIC_SystemLPConfig(uint8_t LowPowerMode, FunctionalState NewState);
void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource);









 



 

 
#line 60 ".\\inc\\stm32f4xx_conf.h"

 
 



 
   



 
 

 
#line 91 ".\\inc\\stm32f4xx_conf.h"



 
#line 6966 ".\\src\\stm32f4xx.h"




 

















 









 

  

 

 
#line 28 ".\\inc\\usb_conf.h"




 
  



  



  






 





 



 



 



 


  

























 
 
 
#line 119 ".\\inc\\usb_conf.h"

 





 
 


 



















#line 163 ".\\inc\\usb_conf.h"

 

     
#line 185 ".\\inc\\usb_conf.h"

 
#line 196 ".\\inc\\usb_conf.h"



  




  


  




  


  



  


  



  


  







  



  
 

#line 28 "Libraries\\STM32_USB_OTG_Driver\\usb_core.h"
#line 1 "Libraries\\STM32_USB_OTG_Driver\\usb_regs.h"



















 

 



 
#line 28 "Libraries\\STM32_USB_OTG_Driver\\usb_regs.h"




 
  



  




  




#line 59 "Libraries\\STM32_USB_OTG_Driver\\usb_regs.h"









  



  



 
typedef struct _USB_OTG_GREGS  
{
  volatile uint32_t GOTGCTL;       
  volatile uint32_t GOTGINT;       
  volatile uint32_t GAHBCFG;       
  volatile uint32_t GUSBCFG;       
  volatile uint32_t GRSTCTL;       
  volatile uint32_t GINTSTS;       
  volatile uint32_t GINTMSK;       
  volatile uint32_t GRXSTSR;       
  volatile uint32_t GRXSTSP;       
  volatile uint32_t GRXFSIZ;       
  volatile uint32_t DIEPTXF0_HNPTXFSIZ;    
  volatile uint32_t HNPTXSTS;      
  volatile uint32_t GI2CCTL;       
  uint32_t Reserved34;   
  volatile uint32_t GCCFG;         
  volatile uint32_t CID;           
  uint32_t  Reserved40[48];    
  volatile uint32_t HPTXFSIZ;  
  volatile uint32_t DIEPTXF[15]; 
}
USB_OTG_GREGS;


 




 
typedef struct _USB_OTG_DREGS 
{
  volatile uint32_t DCFG;          
  volatile uint32_t DCTL;          
  volatile uint32_t DSTS;          
  uint32_t Reserved0C;            
  volatile uint32_t DIEPMSK;    
  volatile uint32_t DOEPMSK;   
  volatile uint32_t DAINT;      
  volatile uint32_t DAINTMSK;  
  uint32_t  Reserved20;           
  uint32_t Reserved9;        
  volatile uint32_t DVBUSDIS;     
  volatile uint32_t DVBUSPULSE;   
  volatile uint32_t DTHRCTL;      
  volatile uint32_t DIEPEMPMSK;  
  volatile uint32_t DEACHINT;     
  volatile uint32_t DEACHMSK;       
  uint32_t Reserved40;       
  volatile uint32_t DINEP1MSK;   
  uint32_t  Reserved44[15];       
  volatile uint32_t DOUTEP1MSK;     
}
USB_OTG_DREGS;


 




 
typedef struct _USB_OTG_INEPREGS
{
  volatile uint32_t DIEPCTL;  
  uint32_t Reserved04;              
  volatile uint32_t DIEPINT;  
  uint32_t Reserved0C;              
  volatile uint32_t DIEPTSIZ;  
  volatile uint32_t DIEPDMA;  
  volatile uint32_t DTXFSTS; 
  uint32_t Reserved18;              
}
USB_OTG_INEPREGS;


 




 
typedef struct _USB_OTG_OUTEPREGS
{
  volatile uint32_t DOEPCTL;        
  volatile uint32_t DOUTEPFRM;    
  volatile uint32_t DOEPINT;               
  uint32_t Reserved0C;                     
  volatile uint32_t DOEPTSIZ;  
  volatile uint32_t DOEPDMA;               
  uint32_t Reserved18[2];                  
}
USB_OTG_OUTEPREGS;


 




 
typedef struct _USB_OTG_HREGS
{
  volatile uint32_t HCFG;              
  volatile uint32_t HFIR;       
  volatile uint32_t HFNUM;          
  uint32_t Reserved40C;                    
  volatile uint32_t HPTXSTS;    
  volatile uint32_t HAINT;    
  volatile uint32_t HAINTMSK;    
}
USB_OTG_HREGS;


 




 
typedef struct _USB_OTG_HC_REGS
{
  volatile uint32_t HCCHAR;
  volatile uint32_t HCSPLT;
  volatile uint32_t HCINT;
  volatile uint32_t HCGINTMSK;
  volatile uint32_t HCTSIZ;
  volatile uint32_t HCDMA;
  uint32_t Reserved[2];
}
USB_OTG_HC_REGS;


 




 
typedef struct USB_OTG_core_regs 
{
  USB_OTG_GREGS         *GREGS;
  USB_OTG_DREGS         *DREGS;
  USB_OTG_HREGS         *HREGS;
  USB_OTG_INEPREGS      *INEP_REGS[15];
  USB_OTG_OUTEPREGS     *OUTEP_REGS[15];
  USB_OTG_HC_REGS       *HC_REGS[15];
  volatile uint32_t         *HPRT0;
  volatile uint32_t         *DFIFO[15];
  volatile uint32_t         *PCGCCTL;
}
USB_OTG_CORE_REGS , *PUSB_OTG_CORE_REGS;
typedef union _USB_OTG_OTGCTL_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t sesreqscs :
    1;
uint32_t sesreq :
    1;
uint32_t Reserved2_7 :
    6;
uint32_t hstnegscs :
    1;
uint32_t hnpreq :
    1;
uint32_t hstsethnpen :
    1;
uint32_t devhnpen :
    1;
uint32_t Reserved12_15 :
    4;
uint32_t conidsts :
    1;
uint32_t Reserved17 :
    1;
uint32_t asesvld :
    1;
uint32_t bsesvld :
    1;
uint32_t currmod :
    1;
uint32_t Reserved21_31 :
    11;
  }
  b;
} USB_OTG_OTGCTL_TypeDef ;
typedef union _USB_OTG_GOTGINT_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t Reserved0_1 :
    2;
uint32_t sesenddet :
    1;
uint32_t Reserved3_7 :
    5;
uint32_t sesreqsucstschng :
    1;
uint32_t hstnegsucstschng :
    1;
uint32_t reserver10_16 :
    7;
uint32_t hstnegdet :
    1;
uint32_t adevtoutchng :
    1;
uint32_t debdone :
    1;
uint32_t Reserved31_20 :
    12;
  }
  b;
} USB_OTG_GOTGINT_TypeDef ;
typedef union _USB_OTG_GAHBCFG_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t glblintrmsk :
    1;
uint32_t hburstlen :
    4;
uint32_t dmaenable :
    1;
uint32_t Reserved :
    1;
uint32_t nptxfemplvl_txfemplvl :
    1;
uint32_t ptxfemplvl :
    1;
uint32_t Reserved9_31 :
    23;
  }
  b;
} USB_OTG_GAHBCFG_TypeDef ;
typedef union _USB_OTG_GUSBCFG_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t toutcal :
    3;
uint32_t phyif :
    1;
uint32_t ulpi_utmi_sel :
    1;
uint32_t fsintf :
    1;
uint32_t physel :
    1;
uint32_t ddrsel :
    1;
uint32_t srpcap :
    1;
uint32_t hnpcap :
    1;
uint32_t usbtrdtim :
    4;
uint32_t nptxfrwnden :
    1;
uint32_t phylpwrclksel :
    1;
uint32_t otgutmifssel :
    1;
uint32_t ulpi_fsls :
    1;
uint32_t ulpi_auto_res :
    1;
uint32_t ulpi_clk_sus_m :
    1;
uint32_t ulpi_ext_vbus_drv :
    1;
uint32_t ulpi_int_vbus_indicator :
    1;
uint32_t term_sel_dl_pulse :
    1;
uint32_t Reserved :
    6;
uint32_t force_host :
    1;
uint32_t force_dev :
    1;
uint32_t corrupt_tx :
    1;
  }
  b;
} USB_OTG_GUSBCFG_TypeDef ;
typedef union _USB_OTG_GRSTCTL_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t csftrst :
    1;
uint32_t hsftrst :
    1;
uint32_t hstfrm :
    1;
uint32_t intknqflsh :
    1;
uint32_t rxfflsh :
    1;
uint32_t txfflsh :
    1;
uint32_t txfnum :
    5;
uint32_t Reserved11_29 :
    19;
uint32_t dmareq :
    1;
uint32_t ahbidle :
    1;
  }
  b;
} USB_OTG_GRSTCTL_TypeDef ;
typedef union _USB_OTG_GINTMSK_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t Reserved0 :
    1;
uint32_t modemismatch :
    1;
uint32_t otgintr :
    1;
uint32_t sofintr :
    1;
uint32_t rxstsqlvl :
    1;
uint32_t nptxfempty :
    1;
uint32_t ginnakeff :
    1;
uint32_t goutnakeff :
    1;
uint32_t Reserved8 :
    1;
uint32_t i2cintr :
    1;
uint32_t erlysuspend :
    1;
uint32_t usbsuspend :
    1;
uint32_t usbreset :
    1;
uint32_t enumdone :
    1;
uint32_t isooutdrop :
    1;
uint32_t eopframe :
    1;
uint32_t Reserved16 :
    1;
uint32_t epmismatch :
    1;
uint32_t inepintr :
    1;
uint32_t outepintr :
    1;
uint32_t incomplisoin :
    1;
uint32_t incomplisoout :
    1;
uint32_t Reserved22_23 :
    2;
uint32_t portintr :
    1;
uint32_t hcintr :
    1;
uint32_t ptxfempty :
    1;
uint32_t Reserved27 :
    1;
uint32_t conidstschng :
    1;
uint32_t disconnect :
    1;
uint32_t sessreqintr :
    1;
uint32_t wkupintr :
    1;
  }
  b;
} USB_OTG_GINTMSK_TypeDef ;
typedef union _USB_OTG_GINTSTS_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t curmode :
    1;
uint32_t modemismatch :
    1;
uint32_t otgintr :
    1;
uint32_t sofintr :
    1;
uint32_t rxstsqlvl :
    1;
uint32_t nptxfempty :
    1;
uint32_t ginnakeff :
    1;
uint32_t goutnakeff :
    1;
uint32_t Reserved8 :
    1;
uint32_t i2cintr :
    1;
uint32_t erlysuspend :
    1;
uint32_t usbsuspend :
    1;
uint32_t usbreset :
    1;
uint32_t enumdone :
    1;
uint32_t isooutdrop :
    1;
uint32_t eopframe :
    1;
uint32_t intimerrx :
    1;
uint32_t epmismatch :
    1;
uint32_t inepint:
    1;
uint32_t outepintr :
    1;
uint32_t incomplisoin :
    1;
uint32_t incomplisoout :
    1;
uint32_t Reserved22_23 :
    2;
uint32_t portintr :
    1;
uint32_t hcintr :
    1;
uint32_t ptxfempty :
    1;
uint32_t Reserved27 :
    1;
uint32_t conidstschng :
    1;
uint32_t disconnect :
    1;
uint32_t sessreqintr :
    1;
uint32_t wkupintr :
    1;
  }
  b;
} USB_OTG_GINTSTS_TypeDef ;
typedef union _USB_OTG_DRXSTS_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t epnum :
    4;
uint32_t bcnt :
    11;
uint32_t dpid :
    2;
uint32_t pktsts :
    4;
uint32_t fn :
    4;
uint32_t Reserved :
    7;
  }
  b;
} USB_OTG_DRXSTS_TypeDef ;
typedef union _USB_OTG_GRXSTS_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t chnum :
    4;
uint32_t bcnt :
    11;
uint32_t dpid :
    2;
uint32_t pktsts :
    4;
uint32_t Reserved :
    11;
  }
  b;
} USB_OTG_GRXFSTS_TypeDef ;
typedef union _USB_OTG_FSIZ_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t startaddr :
    16;
uint32_t depth :
    16;
  }
  b;
} USB_OTG_FSIZ_TypeDef ;
typedef union _USB_OTG_HNPTXSTS_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t nptxfspcavail :
    16;
uint32_t nptxqspcavail :
    8;
uint32_t nptxqtop_terminate :
    1;
uint32_t nptxqtop_timer :
    2;
uint32_t nptxqtop :
    2;
uint32_t chnum :
    2;    
uint32_t Reserved :
    1;
  }
  b;
} USB_OTG_HNPTXSTS_TypeDef ;
typedef union _USB_OTG_DTXFSTSn_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t txfspcavail :
    16;
uint32_t Reserved :
    16;
  }
  b;
} USB_OTG_DTXFSTSn_TypeDef ;
typedef union _USB_OTG_GI2CCTL_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t rwdata :
    8;
uint32_t regaddr :
    8;
uint32_t addr :
    7;
uint32_t i2cen :
    1;
uint32_t ack :
    1;
uint32_t i2csuspctl :
    1;
uint32_t i2cdevaddr :
    2;
uint32_t dat_se0:
    1;
uint32_t Reserved :
    1;
uint32_t rw :
    1;
uint32_t bsydne :
    1;
  }
  b;
} USB_OTG_GI2CCTL_TypeDef ;
typedef union _USB_OTG_GCCFG_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t Reserved_in :
    16;
uint32_t pwdn :
    1;
uint32_t i2cifen :
    1;
uint32_t vbussensingA :
    1;
uint32_t vbussensingB :
    1;
uint32_t sofouten :
    1;
uint32_t disablevbussensing :
    1;
uint32_t Reserved_out :
    10;
  }
  b;
} USB_OTG_GCCFG_TypeDef ;

typedef union _USB_OTG_DCFG_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t devspd :
    2;
uint32_t nzstsouthshk :
    1;
uint32_t Reserved3 :
    1;
uint32_t devaddr :
    7;
uint32_t perfrint :
    2;
uint32_t Reserved13_17 :
    5;
uint32_t epmscnt :
    4;
  }
  b;
} USB_OTG_DCFG_TypeDef ;
typedef union _USB_OTG_DCTL_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t rmtwkupsig :
    1;
uint32_t sftdiscon :
    1;
uint32_t gnpinnaksts :
    1;
uint32_t goutnaksts :
    1;
uint32_t tstctl :
    3;
uint32_t sgnpinnak :
    1;
uint32_t cgnpinnak :
    1;
uint32_t sgoutnak :
    1;
uint32_t cgoutnak :
    1;
uint32_t Reserved :
    21;
  }
  b;
} USB_OTG_DCTL_TypeDef ;
typedef union _USB_OTG_DSTS_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t suspsts :
    1;
uint32_t enumspd :
    2;
uint32_t errticerr :
    1;
uint32_t Reserved4_7:
    4;
uint32_t soffn :
    14;
uint32_t Reserved22_31 :
    10;
  }
  b;
} USB_OTG_DSTS_TypeDef ;
typedef union _USB_OTG_DIEPINTn_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t xfercompl :
    1;
uint32_t epdisabled :
    1;
uint32_t ahberr :
    1;
uint32_t timeout :
    1;
uint32_t intktxfemp :
    1;
uint32_t intknepmis :
    1;
uint32_t inepnakeff :
    1;
uint32_t emptyintr :
    1;
uint32_t txfifoundrn :
    1;
uint32_t Reserved08_31 :
    23;
  }
  b;
} USB_OTG_DIEPINTn_TypeDef ;
typedef union _USB_OTG_DIEPINTn_TypeDef   USB_OTG_DIEPMSK_TypeDef ;
typedef union _USB_OTG_DOEPINTn_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t xfercompl :
    1;
uint32_t epdisabled :
    1;
uint32_t ahberr :
    1;
uint32_t setup :
    1;
uint32_t Reserved04_31 :
    28;
  }
  b;
} USB_OTG_DOEPINTn_TypeDef ;
typedef union _USB_OTG_DOEPINTn_TypeDef   USB_OTG_DOEPMSK_TypeDef ;

typedef union _USB_OTG_DAINT_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t in :
    16;
uint32_t out :
    16;
  }
  ep;
} USB_OTG_DAINT_TypeDef ;

typedef union _USB_OTG_DTHRCTL_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t non_iso_thr_en :
    1;
uint32_t iso_thr_en :
    1;
uint32_t tx_thr_len :
    9;
uint32_t Reserved11_15 :
    5;
uint32_t rx_thr_en :
    1;
uint32_t rx_thr_len :
    9;
uint32_t Reserved26_31 :
    6;
  }
  b;
} USB_OTG_DTHRCTL_TypeDef ;
typedef union _USB_OTG_DEPCTL_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t mps :
    11;
uint32_t reserved :
    4;
uint32_t usbactep :
    1;
uint32_t dpid :
    1;
uint32_t naksts :
    1;
uint32_t eptype :
    2;
uint32_t snp :
    1;
uint32_t stall :
    1;
uint32_t txfnum :
    4;
uint32_t cnak :
    1;
uint32_t snak :
    1;
uint32_t setd0pid :
    1;
uint32_t setd1pid :
    1;
uint32_t epdis :
    1;
uint32_t epena :
    1;
  }
  b;
} USB_OTG_DEPCTL_TypeDef ;
typedef union _USB_OTG_DEPXFRSIZ_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t xfersize :
    19;
uint32_t pktcnt :
    10;
uint32_t mc :
    2;
uint32_t Reserved :
    1;
  }
  b;
} USB_OTG_DEPXFRSIZ_TypeDef ;
typedef union _USB_OTG_DEP0XFRSIZ_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t xfersize :
    7;
uint32_t Reserved7_18 :
    12;
uint32_t pktcnt :
    2;
uint32_t Reserved20_28 :
    9;
uint32_t supcnt :
    2;
    uint32_t Reserved31;
  }
  b;
} USB_OTG_DEP0XFRSIZ_TypeDef ;
typedef union _USB_OTG_HCFG_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t fslspclksel :
    2;
uint32_t fslssupp :
    1;
  }
  b;
} USB_OTG_HCFG_TypeDef ;
typedef union _USB_OTG_HFRMINTRVL_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t frint :
    16;
uint32_t Reserved :
    16;
  }
  b;
} USB_OTG_HFRMINTRVL_TypeDef ;

typedef union _USB_OTG_HFNUM_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t frnum :
    16;
uint32_t frrem :
    16;
  }
  b;
} USB_OTG_HFNUM_TypeDef ;
typedef union _USB_OTG_HPTXSTS_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t ptxfspcavail :
    16;
uint32_t ptxqspcavail :
    8;
uint32_t ptxqtop_terminate :
    1;
uint32_t ptxqtop_timer :
    2;
uint32_t ptxqtop :
    2;
uint32_t chnum :
    2;
uint32_t ptxqtop_odd :
    1;
  }
  b;
} USB_OTG_HPTXSTS_TypeDef ;
typedef union _USB_OTG_HPRT0_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t prtconnsts :
    1;
uint32_t prtconndet :
    1;
uint32_t prtena :
    1;
uint32_t prtenchng :
    1;
uint32_t prtovrcurract :
    1;
uint32_t prtovrcurrchng :
    1;
uint32_t prtres :
    1;
uint32_t prtsusp :
    1;
uint32_t prtrst :
    1;
uint32_t Reserved9 :
    1;
uint32_t prtlnsts :
    2;
uint32_t prtpwr :
    1;
uint32_t prttstctl :
    4;
uint32_t prtspd :
    2;
uint32_t Reserved19_31 :
    13;
  }
  b;
} USB_OTG_HPRT0_TypeDef ;
typedef union _USB_OTG_HAINT_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t chint :
    16;
uint32_t Reserved :
    16;
  }
  b;
} USB_OTG_HAINT_TypeDef ;
typedef union _USB_OTG_HAINTMSK_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t chint :
    16;
uint32_t Reserved :
    16;
  }
  b;
} USB_OTG_HAINTMSK_TypeDef ;
typedef union _USB_OTG_HCCHAR_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t mps :
    11;
uint32_t epnum :
    4;
uint32_t epdir :
    1;
uint32_t Reserved :
    1;
uint32_t lspddev :
    1;
uint32_t eptype :
    2;
uint32_t multicnt :
    2;
uint32_t devaddr :
    7;
uint32_t oddfrm :
    1;
uint32_t chdis :
    1;
uint32_t chen :
    1;
  }
  b;
} USB_OTG_HCCHAR_TypeDef ;
typedef union _USB_OTG_HCSPLT_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t prtaddr :
    7;
uint32_t hubaddr :
    7;
uint32_t xactpos :
    2;
uint32_t compsplt :
    1;
uint32_t Reserved :
    14;
uint32_t spltena :
    1;
  }
  b;
} USB_OTG_HCSPLT_TypeDef ;
typedef union _USB_OTG_HCINTn_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t xfercompl :
    1;
uint32_t chhltd :
    1;
uint32_t ahberr :
    1;
uint32_t stall :
    1;
uint32_t nak :
    1;
uint32_t ack :
    1;
uint32_t nyet :
    1;
uint32_t xacterr :
    1;
uint32_t bblerr :
    1;
uint32_t frmovrun :
    1;
uint32_t datatglerr :
    1;
uint32_t Reserved :
    21;
  }
  b;
} USB_OTG_HCINTn_TypeDef ;
typedef union _USB_OTG_HCTSIZn_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t xfersize :
    19;
uint32_t pktcnt :
    10;
uint32_t pid :
    2;
uint32_t dopng :
    1;
  }
  b;
} USB_OTG_HCTSIZn_TypeDef ;
typedef union _USB_OTG_HCGINTMSK_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t xfercompl :
    1;
uint32_t chhltd :
    1;
uint32_t ahberr :
    1;
uint32_t stall :
    1;
uint32_t nak :
    1;
uint32_t ack :
    1;
uint32_t nyet :
    1;
uint32_t xacterr :
    1;
uint32_t bblerr :
    1;
uint32_t frmovrun :
    1;
uint32_t datatglerr :
    1;
uint32_t Reserved :
    21;
  }
  b;
} USB_OTG_HCGINTMSK_TypeDef ;
typedef union _USB_OTG_PCGCCTL_TypeDef 
{
  uint32_t d32;
  struct
  {
uint32_t stoppclk :
    1;
uint32_t gatehclk :
    1;
uint32_t Reserved :
    30;
  }
  b;
} USB_OTG_PCGCCTL_TypeDef ;



  




  


  



  


  



  


  







  



  
 

#line 29 "Libraries\\STM32_USB_OTG_Driver\\usb_core.h"
#line 1 "Libraries\\STM32_USB_OTG_Driver\\usb_defines.h"



















 

 



 
#line 28 "Libraries\\STM32_USB_OTG_Driver\\usb_defines.h"



 
  



  




  


  




 














 




 
#line 85 "Libraries\\STM32_USB_OTG_Driver\\usb_defines.h"


 




 
#line 108 "Libraries\\STM32_USB_OTG_Driver\\usb_defines.h"


 




 
































 




 






















 




  

typedef enum
{
  USB_OTG_HS_CORE_ID = 0,
  USB_OTG_FS_CORE_ID = 1
}USB_OTG_CORE_ID_TypeDef;


  




  


  



  


  



  


  




 







 
enum USB_OTG_SPEED {
  USB_SPEED_UNKNOWN = 0,
  USB_SPEED_LOW,
  USB_SPEED_FULL,
  USB_SPEED_HIGH
};






  



  
 

#line 30 "Libraries\\STM32_USB_OTG_Driver\\usb_core.h"




 
  



  




  

#line 53 "Libraries\\STM32_USB_OTG_Driver\\usb_core.h"





 






  




  


typedef enum {
  USB_OTG_OK = 0,
  USB_OTG_FAIL
}USB_OTG_STS;

typedef enum {
  HC_IDLE = 0,
  HC_XFRC,
  HC_HALTED,
  HC_NAK,
  HC_NYET,
  HC_STALL,
  HC_XACTERR,  
  HC_BBLERR,   
  HC_DATATGLERR,  
}HC_STATUS;

typedef enum {
  URB_IDLE = 0,
  URB_DONE,
  URB_NOTREADY,
  URB_ERROR,
  URB_STALL
}URB_STATE;

typedef enum {
  CTRL_START = 0,
  CTRL_XFRC,
  CTRL_HALTED,
  CTRL_NAK,
  CTRL_STALL,
  CTRL_XACTERR,  
  CTRL_BBLERR,   
  CTRL_DATATGLERR,  
  CTRL_FAIL
}CTRL_STATUS;


typedef struct USB_OTG_hc
{
  uint8_t       dev_addr ;
  uint8_t       ep_num;
  uint8_t       ep_is_in;
  uint8_t       speed;
  uint8_t       do_ping;  
  uint8_t       ep_type;
  uint16_t      max_packet;
  uint8_t       data_pid;
  uint8_t       *xfer_buff;
  uint32_t      xfer_len;
  uint32_t      xfer_count;  
  uint8_t       toggle_in;
  uint8_t       toggle_out;
  uint32_t       dma_addr;  
}
USB_OTG_HC , *PUSB_OTG_HC;

typedef struct USB_OTG_ep
{
  uint8_t        num;
  uint8_t        is_in;
  uint8_t        is_stall;  
  uint8_t        type;
  uint8_t        data_pid_start;
  uint8_t        even_odd_frame;
  uint16_t       tx_fifo_num;
  uint32_t       maxpacket;
   
  uint8_t        *xfer_buff;
  uint32_t       dma_addr;  
  uint32_t       xfer_len;
  uint32_t       xfer_count;
     
  uint32_t       rem_data_len;
  uint32_t       total_data_len;
  uint32_t       ctl_data_len;  

}

USB_OTG_EP , *PUSB_OTG_EP;



typedef struct USB_OTG_core_cfg
{
  uint8_t       host_channels;
  uint8_t       dev_endpoints;
  uint8_t       speed;
  uint8_t       dma_enable;
  uint16_t      mps;
  uint16_t      TotalFifoSize;
  uint8_t       phy_itface;
  uint8_t       Sof_output;
  uint8_t       low_power;
  uint8_t       coreID;
 
}
USB_OTG_CORE_CFGS, *PUSB_OTG_CORE_CFGS;



typedef  struct  usb_setup_req {
    
    uint8_t   bmRequest;                      
    uint8_t   bRequest;                           
    uint16_t  wValue;                             
    uint16_t  wIndex;                             
    uint16_t  wLength;                            
} USB_SETUP_REQ;

typedef struct _Device_TypeDef
{
  uint8_t  *(*GetDeviceDescriptor)( uint8_t speed , uint16_t *length);  
  uint8_t  *(*GetLangIDStrDescriptor)( uint8_t speed , uint16_t *length); 
  uint8_t  *(*GetManufacturerStrDescriptor)( uint8_t speed , uint16_t *length);  
  uint8_t  *(*GetProductStrDescriptor)( uint8_t speed , uint16_t *length);  
  uint8_t  *(*GetSerialStrDescriptor)( uint8_t speed , uint16_t *length);  
  uint8_t  *(*GetConfigurationStrDescriptor)( uint8_t speed , uint16_t *length);  
  uint8_t  *(*GetInterfaceStrDescriptor)( uint8_t speed , uint16_t *length);   
} USBD_DEVICE, *pUSBD_DEVICE;

typedef struct USB_OTG_hPort
{
  void (*Disconnect) (void *phost);
  void (*Connect) (void *phost); 
  uint8_t ConnStatus;
  uint8_t DisconnStatus;
  uint8_t ConnHandled;
  uint8_t DisconnHandled;
} USB_OTG_hPort_TypeDef;

typedef struct _Device_cb
{
  uint8_t  (*Init)         (void *pdev , uint8_t cfgidx);
  uint8_t  (*DeInit)       (void *pdev , uint8_t cfgidx);
  
  uint8_t  (*Setup)        (void *pdev , USB_SETUP_REQ  *req);  
  uint8_t  (*EP0_TxSent)   (void *pdev );    
  uint8_t  (*EP0_RxReady)  (void *pdev );  
   
  uint8_t  (*DataIn)       (void *pdev , uint8_t epnum);   
  uint8_t  (*DataOut)      (void *pdev , uint8_t epnum); 
  uint8_t  (*SOF)          (void *pdev); 
  uint8_t  (*IsoINIncomplete)  (void *pdev); 
  uint8_t  (*IsoOUTIncomplete)  (void *pdev);   

  uint8_t  *(*GetConfigDescriptor)( uint8_t speed , uint16_t *length); 







  
} USBD_Class_cb_TypeDef;



typedef struct _USBD_USR_PROP
{
  void (*Init)(void);   
  void (*DeviceReset)(uint8_t speed); 
  void (*DeviceConfigured)(void);
  void (*DeviceSuspended)(void);
  void (*DeviceResumed)(void);  
  
  void (*DeviceConnected)(void);  
  void (*DeviceDisconnected)(void);    
  
}
USBD_Usr_cb_TypeDef;

typedef struct _DCD
{
  uint8_t        device_config;
  uint8_t        device_state;
  uint8_t        device_status;
  uint8_t        device_address;
  uint32_t       DevRemoteWakeup;
  USB_OTG_EP     in_ep   [15];
  USB_OTG_EP     out_ep  [15];
  uint8_t        setup_packet [8*3];
  USBD_Class_cb_TypeDef         *class_cb;
  USBD_Usr_cb_TypeDef           *usr_cb;
  USBD_DEVICE                   *usr_device;  
  uint8_t        *pConfig_descriptor;
 }
DCD_DEV , *DCD_PDEV;


typedef struct _HCD
{
  uint8_t                  Rx_Buffer [0xFF];  
  volatile uint32_t            ConnSts;
  volatile uint32_t            ErrCnt[15];
  volatile uint32_t            XferCnt[15];
  volatile HC_STATUS           HC_Status[15];  
  volatile URB_STATE           URB_State[15];
  USB_OTG_HC               hc [15];
  uint16_t                 channel [15];
  USB_OTG_hPort_TypeDef    *port_cb;  
}
HCD_DEV , *USB_OTG_USBH_PDEV;


typedef struct _OTG
{
  uint8_t    OTG_State;
  uint8_t    OTG_PrevState;  
  uint8_t    OTG_Mode;    
}
OTG_DEV , *USB_OTG_USBO_PDEV;

typedef struct USB_OTG_handle
{
  USB_OTG_CORE_CFGS    cfg;
  USB_OTG_CORE_REGS    regs;




  HCD_DEV     host;




}
USB_OTG_CORE_HANDLE , *PUSB_OTG_CORE_HANDLE;



  




  



  



  


  



  


USB_OTG_STS  USB_OTG_CoreInit        (USB_OTG_CORE_HANDLE *pdev);
USB_OTG_STS  USB_OTG_SelectCore      (USB_OTG_CORE_HANDLE *pdev, 
                                      USB_OTG_CORE_ID_TypeDef coreID);
USB_OTG_STS  USB_OTG_EnableGlobalInt (USB_OTG_CORE_HANDLE *pdev);
USB_OTG_STS  USB_OTG_DisableGlobalInt(USB_OTG_CORE_HANDLE *pdev);
void*           USB_OTG_ReadPacket   (USB_OTG_CORE_HANDLE *pdev ,
    uint8_t *dest,
    uint16_t len);
USB_OTG_STS  USB_OTG_WritePacket     (USB_OTG_CORE_HANDLE *pdev ,
    uint8_t *src,
    uint8_t ch_ep_num,
    uint16_t len);
USB_OTG_STS  USB_OTG_FlushTxFifo     (USB_OTG_CORE_HANDLE *pdev , uint32_t num);
USB_OTG_STS  USB_OTG_FlushRxFifo     (USB_OTG_CORE_HANDLE *pdev);

uint32_t     USB_OTG_ReadCoreItr     (USB_OTG_CORE_HANDLE *pdev);
uint32_t     USB_OTG_ReadOtgItr      (USB_OTG_CORE_HANDLE *pdev);
uint8_t      USB_OTG_IsHostMode      (USB_OTG_CORE_HANDLE *pdev);
uint8_t      USB_OTG_IsDeviceMode    (USB_OTG_CORE_HANDLE *pdev);
uint32_t     USB_OTG_GetMode         (USB_OTG_CORE_HANDLE *pdev);
USB_OTG_STS  USB_OTG_PhyInit         (USB_OTG_CORE_HANDLE *pdev);
USB_OTG_STS  USB_OTG_SetCurrentMode  (USB_OTG_CORE_HANDLE *pdev,
    uint8_t mode);

 

USB_OTG_STS  USB_OTG_CoreInitHost    (USB_OTG_CORE_HANDLE *pdev);
USB_OTG_STS  USB_OTG_EnableHostInt   (USB_OTG_CORE_HANDLE *pdev);
USB_OTG_STS  USB_OTG_HC_Init         (USB_OTG_CORE_HANDLE *pdev, uint8_t hc_num);
USB_OTG_STS  USB_OTG_HC_Halt         (USB_OTG_CORE_HANDLE *pdev, uint8_t hc_num);
USB_OTG_STS  USB_OTG_HC_StartXfer    (USB_OTG_CORE_HANDLE *pdev, uint8_t hc_num);
USB_OTG_STS  USB_OTG_HC_DoPing       (USB_OTG_CORE_HANDLE *pdev , uint8_t hc_num);
uint32_t     USB_OTG_ReadHostAllChannels_intr    (USB_OTG_CORE_HANDLE *pdev);
uint32_t     USB_OTG_ResetPort       (USB_OTG_CORE_HANDLE *pdev);
uint32_t     USB_OTG_ReadHPRT0       (USB_OTG_CORE_HANDLE *pdev);
void         USB_OTG_DriveVbus       (USB_OTG_CORE_HANDLE *pdev, uint8_t state);
void         USB_OTG_InitFSLSPClkSel (USB_OTG_CORE_HANDLE *pdev ,uint8_t freq);
uint8_t      USB_OTG_IsEvenFrame     (USB_OTG_CORE_HANDLE *pdev) ;
void         USB_OTG_StopHost        (USB_OTG_CORE_HANDLE *pdev);

 
#line 393 "Libraries\\STM32_USB_OTG_Driver\\usb_core.h"


  






  



  
 

#line 24 "Libraries\\STM32_USB_OTG_Driver\\usb_hcd.c"
#line 1 "Libraries\\STM32_USB_OTG_Driver\\usb_hcd.h"



















 

 



 
#line 28 "Libraries\\STM32_USB_OTG_Driver\\usb_hcd.h"
#line 29 "Libraries\\STM32_USB_OTG_Driver\\usb_hcd.h"




 
  



  




  


  




  


  




  


  



  


  



  
uint32_t  HCD_Init                 (USB_OTG_CORE_HANDLE *pdev ,
                                    USB_OTG_CORE_ID_TypeDef coreID);
uint32_t  HCD_HC_Init              (USB_OTG_CORE_HANDLE *pdev , 
                                    uint8_t hc_num); 
uint32_t  HCD_SubmitRequest        (USB_OTG_CORE_HANDLE *pdev , 
                                    uint8_t hc_num) ;
uint32_t  HCD_GetCurrentSpeed      (USB_OTG_CORE_HANDLE *pdev);
uint32_t  HCD_ResetPort            (USB_OTG_CORE_HANDLE *pdev);
uint32_t  HCD_IsDeviceConnected    (USB_OTG_CORE_HANDLE *pdev);
uint32_t  HCD_GetCurrentFrame      (USB_OTG_CORE_HANDLE *pdev) ;
URB_STATE HCD_GetURB_State         (USB_OTG_CORE_HANDLE *pdev,  uint8_t ch_num); 
uint32_t  HCD_GetXferCnt           (USB_OTG_CORE_HANDLE *pdev,  uint8_t ch_num); 
HC_STATUS HCD_GetHCState           (USB_OTG_CORE_HANDLE *pdev,  uint8_t ch_num) ;


  






  



  
 

#line 25 "Libraries\\STM32_USB_OTG_Driver\\usb_hcd.c"
#line 26 "Libraries\\STM32_USB_OTG_Driver\\usb_hcd.c"
#line 1 "Libraries\\STM32_USB_OTG_Driver\\usb_bsp.h"



















 

 



 
#line 28 "Libraries\\STM32_USB_OTG_Driver\\usb_bsp.h"
#line 1 ".\\Utilities\\STM32F4-Discovery\\stm32f4_discovery.h"




















  
  
 






                                              
 
#line 33 ".\\Utilities\\STM32F4-Discovery\\stm32f4_discovery.h"
   


 
  


 
      


  



 
typedef enum 
{
  LED4 = 0,
  LED3 = 1,
  LED5 = 2,
  LED6 = 3
} Led_TypeDef;

typedef enum 
{  
  BUTTON_USER = 0,
} Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;     


  



  



 





  



  



  





  
  


   




 
#line 114 ".\\Utilities\\STM32F4-Discovery\\stm32f4_discovery.h"


  
  


   


  




 
void STM_EVAL_LEDInit(Led_TypeDef Led);
void STM_EVAL_LEDOn(Led_TypeDef Led);
void STM_EVAL_LEDOff(Led_TypeDef Led);
void STM_EVAL_LEDToggle(Led_TypeDef Led);
void STM_EVAL_PBInit(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode);
uint32_t STM_EVAL_PBGetState(Button_TypeDef Button);


 
  







  



  



 

 

 
#line 29 "Libraries\\STM32_USB_OTG_Driver\\usb_bsp.h"



 
  



  




  


  




  


  




  


  



  


  



  
void BSP_Init(void);

void USB_OTG_BSP_Init (USB_OTG_CORE_HANDLE *pdev);
void USB_OTG_BSP_uDelay (const uint32_t usec);
void USB_OTG_BSP_mDelay (const uint32_t msec);
void USB_OTG_BSP_EnableInterrupt (USB_OTG_CORE_HANDLE *pdev);

void USB_OTG_BSP_ConfigVBUS(USB_OTG_CORE_HANDLE *pdev);
void USB_OTG_BSP_DriveVBUS(USB_OTG_CORE_HANDLE *pdev,uint8_t state);



  





  



  
 

#line 27 "Libraries\\STM32_USB_OTG_Driver\\usb_hcd.c"




 
  



 




  


  
 



  


  





  


  




  


  




  


  




  







 
uint32_t HCD_Init(USB_OTG_CORE_HANDLE *pdev , 
                  USB_OTG_CORE_ID_TypeDef coreID)
{
  uint8_t i = 0;
  pdev->host.ConnSts = 0;
  
  for (i= 0; i< 15; i++)
  {
  pdev->host.ErrCnt[i]  = 0;
  pdev->host.XferCnt[i]   = 0;
  pdev->host.HC_Status[i]   = HC_IDLE;
  }
  pdev->host.hc[0].max_packet  = 8; 

  USB_OTG_SelectCore(pdev, coreID);

  USB_OTG_DisableGlobalInt(pdev);
  USB_OTG_CoreInit(pdev);

   
  USB_OTG_SetCurrentMode(pdev , 1);
  USB_OTG_CoreInitHost(pdev);
  USB_OTG_EnableGlobalInt(pdev);

   
  return 0;
}







 

uint32_t HCD_GetCurrentSpeed (USB_OTG_CORE_HANDLE *pdev)
{    
    USB_OTG_HPRT0_TypeDef  HPRT0;
    HPRT0.d32 = (*(volatile uint32_t *)pdev->regs . HPRT0);
    
    return HPRT0.b.prtspd;
}






 
uint32_t HCD_ResetPort(USB_OTG_CORE_HANDLE *pdev)
{
  




 
  
  USB_OTG_ResetPort(pdev); 
  return 0;
}







 
uint32_t HCD_IsDeviceConnected(USB_OTG_CORE_HANDLE *pdev)
{
  return (pdev->host.ConnSts);
}







 
uint32_t HCD_GetCurrentFrame (USB_OTG_CORE_HANDLE *pdev) 
{
 return ((*(volatile uint32_t *)&pdev->regs . HREGS->HFNUM) & 0xFFFF) ;
}







 
URB_STATE HCD_GetURB_State (USB_OTG_CORE_HANDLE *pdev , uint8_t ch_num) 
{
  return pdev->host.URB_State[ch_num] ;
}







 
uint32_t HCD_GetXferCnt (USB_OTG_CORE_HANDLE *pdev, uint8_t ch_num) 
{
  return pdev->host.XferCnt[ch_num] ;
}









 
HC_STATUS HCD_GetHCState (USB_OTG_CORE_HANDLE *pdev ,  uint8_t ch_num) 
{
  return pdev->host.HC_Status[ch_num] ;
}







 
uint32_t HCD_HC_Init (USB_OTG_CORE_HANDLE *pdev , uint8_t hc_num) 
{
  return USB_OTG_HC_Init(pdev, hc_num);  
}







 
uint32_t HCD_SubmitRequest (USB_OTG_CORE_HANDLE *pdev , uint8_t hc_num) 
{
  
  pdev->host.URB_State[hc_num] =   URB_IDLE;  
  pdev->host.hc[hc_num].xfer_count = 0 ;
  return USB_OTG_HC_StartXfer(pdev, hc_num);
}




  



  



 

 
