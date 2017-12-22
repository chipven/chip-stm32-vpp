#ifndef STM32F103C8T6_H
#define STM32F103C8T6_H

#define __I volatile const /*!< Defines 'read only' permissions */
#define __O volatile       /*!< Defines 'write only' permissions */
#define __IO volatile      /*!< Defines 'read / write' permissions */
#define __IM \
    volatile const    /*! Defines 'read only' structure member permissions */
#define __OM volatile /*! Defines 'write only' structure member permissions */
#define __IOM \
    volatile /*! Defines 'read / write' structure member permissions */

typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;
typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;


/**
  \brief  Union type to access the Application Program Status Register (APSR).
 */
typedef union {
    struct {
        u32 _reserved0 : 27; /*!< bit:  0..26  Reserved */
        u32 Q : 1;           /*!< bit:     27  Saturation condition flag */
        u32 V : 1; /*!< bit:     28  Overflow condition code flag */
        u32 C : 1; /*!< bit:     29  Carry condition code flag */
        u32 Z : 1; /*!< bit:     30  Zero condition code flag */
        u32 N : 1; /*!< bit:     31  Negative condition code flag */
    } b;                /*!< Structure used for bit  access */
    u32 w;         /*!< Type      used for word access */
} APSR_Type;

/**
  \brief  Union type to access the Interrupt Program Status Register (IPSR).
 */
typedef union {
    struct {
        u32 ISR : 9;         /*!< bit:  0.. 8  Exception number */
        u32 _reserved0 : 23; /*!< bit:  9..31  Reserved */
    } b;                          /*!< Structure used for bit  access */
    u32 w;                   /*!< Type      used for word access */
} IPSR_Type;

/**
  \brief  Union type to access the Special-Purpose Program Status Registers
  (xPSR).
 */
typedef union {
    struct {
        u32 ISR : 9;         /*!< bit:  0.. 8  Exception number */
        u32 _reserved0 : 15; /*!< bit:  9..23  Reserved */
        u32 T : 1;           /*!< bit:     24  Thumb bit        (read 0) */
        u32 IT : 2;          /*!< bit: 25..26  saved IT state   (read 0) */
        u32 Q : 1;           /*!< bit:     27  Saturation condition flag */
        u32 V : 1; /*!< bit:     28  Overflow condition code flag */
        u32 C : 1; /*!< bit:     29  Carry condition code flag */
        u32 Z : 1; /*!< bit:     30  Zero condition code flag */
        u32 N : 1; /*!< bit:     31  Negative condition code flag */
    } b;                /*!< Structure used for bit  access */
    u32 w;         /*!< Type      used for word access */
} xPSR_Type;

/**
  \brief  Union type to access the Control Registers (CONTROL).
 */
typedef union {
    struct {
        u32
            nPRIV : 1; /*!< bit:      0  Execution privilege in Thread mode */
        u32 SPSEL : 1;       /*!< bit:      1  Stack to be used */
        u32 _reserved1 : 30; /*!< bit:  2..31  Reserved */
    } b;                          /*!< Structure used for bit  access */
    u32 w;                   /*!< Type      used for word access */
} CONTROL_Type;

typedef struct {
    __IOM u32
        ISER[8U]; /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
    u32 RESERVED0[24U];
    __IOM u32
        ICER[8U]; /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
    u32 RSERVED1[24U];
    __IOM u32
        ISPR[8U]; /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
    u32 RESERVED2[24U];
    __IOM u32
        ICPR[8U]; /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
    u32 RESERVED3[24U];
    __IOM u32
        IABR[8U]; /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
    u32 RESERVED4[56U];
    __IOM u8
        IP[240U]; /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit
                     wide) */
    u32 RESERVED5[644U];
    __OM u32
        STIR; /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
} NVIC_Type;

/**
  \brief  Structure type to access the System Control Block (SCB).
 */
typedef struct {
    __IM u32 CPUID; /*!< Offset: 0x000 (R/ )  CPUID Base Register */
    __IOM u32
        ICSR; /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
    __IOM u32
        VTOR; /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
    __IOM u32 AIRCR; /*!< Offset: 0x00C (R/W)  Application Interrupt and
                             Reset Control Register */
    __IOM u32 SCR;   /*!< Offset: 0x010 (R/W)  System Control Register */
    __IOM u32
        CCR; /*!< Offset: 0x014 (R/W)  Configuration Control Register */
    __IOM u8 SHP[12U]; /*!< Offset: 0x018 (R/W)  System Handlers Priority
                               Registers (4-7, 8-11, 12-15) */
    __IOM u32 SHCSR;   /*!< Offset: 0x024 (R/W)  System Handler Control and
                               State Register */
    __IOM u32
        CFSR; /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
    __IOM u32 HFSR; /*!< Offset: 0x02C (R/W)  HardFault Status Register */
    __IOM u32
        DFSR; /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
    __IOM u32
        MMFAR; /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
    __IOM u32 BFAR; /*!< Offset: 0x038 (R/W)  BusFault Address Register */
    __IOM u32
        AFSR; /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
    __IM u32
        PFR[2U];       /*!< Offset: 0x040 (R/ )  Processor Feature Register */
    __IM u32 DFR; /*!< Offset: 0x048 (R/ )  Debug Feature Register */
    __IM u32 ADR; /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
    __IM u32
        MMFR[4U]; /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
    __IM u32 ISAR
        [5U]; /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
    u32 RESERVED0[5U];
    __IOM u32
        CPACR; /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
} SCB_Type;

/**
  \brief  Structure type to access the System Timer (SysTick).
 */
typedef struct {
    __IOM u32
        CTRL; /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
    __IOM u32
        LOAD; /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register */
    __IOM u32
        VAL; /*!< Offset: 0x008 (R/W)  SysTick Current Value Register */
    __IM u32
        CALIB; /*!< Offset: 0x00C (R/ )  SysTick Calibration Register */
} SysTick_Type;

/**
  \brief  Structure type to access the Instrumentation Trace Macrocell Register
  (ITM).
 */
typedef struct {
    __OM union {
        __OM unsigned char u8;   /*!< Offset: 0x000 ( /W)  ITM Stimulus Port 8-bit */
        __OM unsigned short u16; /*!< Offset: 0x000 ( /W)  ITM Stimulus Port 16-bit */
        __OM unsigned int u32; /*!< Offset: 0x000 ( /W)  ITM Stimulus Port 32-bit */
    } PORT[32U]; /*!< Offset: 0x000 ( /W)  ITM Stimulus Port Registers */
    u32 RESERVED0[864U];
    __IOM u32 TER; /*!< Offset: 0xE00 (R/W)  ITM Trace Enable Register */
    u32 RESERVED1[15U];
    __IOM u32
        TPR; /*!< Offset: 0xE40 (R/W)  ITM Trace Privilege Register */
    u32 RESERVED2[15U];
    __IOM u32 TCR; /*!< Offset: 0xE80 (R/W)  ITM Trace Control Register */
    u32 RESERVED3[29U];
    __OM u32
        IWR; /*!< Offset: 0xEF8 ( /W)  ITM Integration Write Register */
    __IM u32
        IRR; /*!< Offset: 0xEFC (R/ )  ITM Integration Read Register */
    __IOM u32
        IMCR; /*!< Offset: 0xF00 (R/W)  ITM Integration Mode Control Register */
    u32 RESERVED4[43U];
    __OM u32 LAR; /*!< Offset: 0xFB0 ( /W)  ITM Lock Access Register */
    __IM u32 LSR; /*!< Offset: 0xFB4 (R/ )  ITM Lock Status Register */
    u32 RESERVED5[6U];
    __IM u32 PID4; /*!< Offset: 0xFD0 (R/ )  ITM Peripheral Identification
                           Register #4 */
    __IM u32 PID5; /*!< Offset: 0xFD4 (R/ )  ITM Peripheral Identification
                           Register #5 */
    __IM u32 PID6; /*!< Offset: 0xFD8 (R/ )  ITM Peripheral Identification
                           Register #6 */
    __IM u32 PID7; /*!< Offset: 0xFDC (R/ )  ITM Peripheral Identification
                           Register #7 */
    __IM u32 PID0; /*!< Offset: 0xFE0 (R/ )  ITM Peripheral Identification
                           Register #0 */
    __IM u32 PID1; /*!< Offset: 0xFE4 (R/ )  ITM Peripheral Identification
                           Register #1 */
    __IM u32 PID2; /*!< Offset: 0xFE8 (R/ )  ITM Peripheral Identification
                           Register #2 */
    __IM u32 PID3; /*!< Offset: 0xFEC (R/ )  ITM Peripheral Identification
                           Register #3 */
    __IM u32 CID0; /*!< Offset: 0xFF0 (R/ )  ITM Component  Identification
                           Register #0 */
    __IM u32 CID1; /*!< Offset: 0xFF4 (R/ )  ITM Component  Identification
                           Register #1 */
    __IM u32 CID2; /*!< Offset: 0xFF8 (R/ )  ITM Component  Identification
                           Register #2 */
    __IM u32 CID3; /*!< Offset: 0xFFC (R/ )  ITM Component  Identification
                           Register #3 */
} ITM_Type;

/**
  \brief  Structure type to access the Data Watchpoint and Trace Register (DWT).
 */
typedef struct {
    __IOM u32 CTRL;   /*!< Offset: 0x000 (R/W)  Control Register */
    __IOM u32 CYCCNT; /*!< Offset: 0x004 (R/W)  Cycle Count Register */
    __IOM u32 CPICNT; /*!< Offset: 0x008 (R/W)  CPI Count Register */
    __IOM u32
        EXCCNT; /*!< Offset: 0x00C (R/W)  Exception Overhead Count Register */
    __IOM u32 SLEEPCNT; /*!< Offset: 0x010 (R/W)  Sleep Count Register */
    __IOM u32 LSUCNT;   /*!< Offset: 0x014 (R/W)  LSU Count Register */
    __IOM u32
        FOLDCNT; /*!< Offset: 0x018 (R/W)  Folded-instruction Count Register */
    __IM u32
        PCSR; /*!< Offset: 0x01C (R/ )  Program Counter Sample Register */
    __IOM u32 COMP0;     /*!< Offset: 0x020 (R/W)  Comparator Register 0 */
    __IOM u32 MASK0;     /*!< Offset: 0x024 (R/W)  Mask Register 0 */
    __IOM u32 FUNCTION0; /*!< Offset: 0x028 (R/W)  Function Register 0 */
    u32 RESERVED0[1U];
    __IOM u32 COMP1;     /*!< Offset: 0x030 (R/W)  Comparator Register 1 */
    __IOM u32 MASK1;     /*!< Offset: 0x034 (R/W)  Mask Register 1 */
    __IOM u32 FUNCTION1; /*!< Offset: 0x038 (R/W)  Function Register 1 */
    u32 RESERVED1[1U];
    __IOM u32 COMP2;     /*!< Offset: 0x040 (R/W)  Comparator Register 2 */
    __IOM u32 MASK2;     /*!< Offset: 0x044 (R/W)  Mask Register 2 */
    __IOM u32 FUNCTION2; /*!< Offset: 0x048 (R/W)  Function Register 2 */
    u32 RESERVED2[1U];
    __IOM u32 COMP3;     /*!< Offset: 0x050 (R/W)  Comparator Register 3 */
    __IOM u32 MASK3;     /*!< Offset: 0x054 (R/W)  Mask Register 3 */
    __IOM u32 FUNCTION3; /*!< Offset: 0x058 (R/W)  Function Register 3 */
} DWT_Type;

/**
  \brief  Structure type to access the Trace Port Interface Register (TPI).
 */
typedef struct {
    __IOM u32
        SSPSR; /*!< Offset: 0x000 (R/ )  Supported Parallel Port Size Register
                */
    __IOM u32
        CSPSR; /*!< Offset: 0x004 (R/W)  Current Parallel Port Size Register */
    u32 RESERVED0[2U];
    __IOM u32
        ACPR; /*!< Offset: 0x010 (R/W)  Asynchronous Clock Prescaler Register */
    u32 RESERVED1[55U];
    __IOM u32
        SPPR; /*!< Offset: 0x0F0 (R/W)  Selected Pin Protocol Register */
    u32 RESERVED2[131U];
    __IM u32
        FFSR; /*!< Offset: 0x300 (R/ )  Formatter and Flush Status Register */
    __IOM u32
        FFCR; /*!< Offset: 0x304 (R/W)  Formatter and Flush Control Register */
    __IM u32 FSCR; /*!< Offset: 0x308 (R/ )  Formatter Synchronization
                           Counter Register */
    u32 RESERVED3[759U];
    __IM u32 TRIGGER;   /*!< Offset: 0xEE8 (R/ )  TRIGGER */
    __IM u32 FIFO0;     /*!< Offset: 0xEEC (R/ )  Integration ETM Data */
    __IM u32 ITATBCTR2; /*!< Offset: 0xEF0 (R/ )  ITATBCTR2 */
    u32 RESERVED4[1U];
    __IM u32 ITATBCTR0; /*!< Offset: 0xEF8 (R/ )  ITATBCTR0 */
    __IM u32 FIFO1;     /*!< Offset: 0xEFC (R/ )  Integration ITM Data */
    __IOM u32 ITCTRL; /*!< Offset: 0xF00 (R/W)  Integration Mode Control */
    u32 RESERVED5[39U];
    __IOM u32 CLAIMSET; /*!< Offset: 0xFA0 (R/W)  Claim tag set */
    __IOM u32 CLAIMCLR; /*!< Offset: 0xFA4 (R/W)  Claim tag clear */
    u32 RESERVED7[8U];
    __IM u32 DEVID;   /*!< Offset: 0xFC8 (R/ )  TPIU_DEVID */
    __IM u32 DEVTYPE; /*!< Offset: 0xFCC (R/ )  TPIU_DEVTYPE */
} TPI_Type;

/**
  \brief  Structure type to access the Memory Protection Unit (MPU).
 */
typedef struct {
    __IM u32 TYPE;  /*!< Offset: 0x000 (R/ )  MPU Type Register */
    __IOM u32 CTRL; /*!< Offset: 0x004 (R/W)  MPU Control Register */
    __IOM u32 RNR;  /*!< Offset: 0x008 (R/W)  MPU Region RNRber Register */
    __IOM u32
        RBAR; /*!< Offset: 0x00C (R/W)  MPU Region Base Address Register */
    __IOM u32
        RASR; /*!< Offset: 0x010 (R/W)  MPU Region Attribute and Size Register
               */
    __IOM u32 RBAR_A1; /*!< Offset: 0x014 (R/W)  MPU Alias 1 Region Base
                               Address Register */
    __IOM u32
        RASR_A1; /*!< Offset: 0x018 (R/W)  MPU Alias 1 Region Attribute and Size
         Register */
    __IOM u32 RBAR_A2; /*!< Offset: 0x01C (R/W)  MPU Alias 2 Region Base
                               Address Register */
    __IOM u32
        RASR_A2; /*!< Offset: 0x020 (R/W)  MPU Alias 2 Region Attribute and Size
         Register */
    __IOM u32 RBAR_A3; /*!< Offset: 0x024 (R/W)  MPU Alias 3 Region Base
                               Address Register */
    __IOM u32
        RASR_A3; /*!< Offset: 0x028 (R/W)  MPU Alias 3 Region Attribute and Size
                    Register */
} MPU_Type;

/**
  \brief  Structure type to access the Core Debug Register (CoreDebug).
 */
typedef struct {
    __IOM u32 DHCSR; /*!< Offset: 0x000 (R/W)  Debug Halting Control and
                             Status Register */
    __OM u32
        DCRSR; /*!< Offset: 0x004 ( /W)  Debug Core Register Selector Register
                */
    __IOM u32
        DCRDR; /*!< Offset: 0x008 (R/W)  Debug Core Register Data Register */
    __IOM u32 DEMCR; /*!< Offset: 0x00C (R/W)  Debug Exception and Monitor
                             Control Register */
} CoreDebug_Type;

/* Memory mapping of Cortex-M3 Hardware */
#define SCS_BASE (0xE000E000UL)       /*!< System Control Space Base Address */
#define ITM_BASE (0xE0000000UL)       /*!< ITM Base Address */
#define DWT_BASE (0xE0001000UL)       /*!< DWT Base Address */
#define TPI_BASE (0xE0040000UL)       /*!< TPI Base Address */
#define CoreDebug_BASE (0xE000EDF0UL) /*!< Core Debug Base Address */
#define SysTick_BASE (SCS_BASE + 0x0010UL) /*!< SysTick Base Address */
#define NVIC_BASE (SCS_BASE + 0x0100UL)    /*!< NVIC Base Address */
#define SCB_BASE                                                               \
    (SCS_BASE + 0x0D00UL) /*!< System Control Block Base Address \ \ \ \ \ \ \ \
                           * \ \ \ \ \ \ \                                     \
                           * \ \ \ \ \ \                                       \
                           * \ \ \ \ \ \                                       \
                           * \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \               \
                           */

#define SCnSCB \
    ((SCnSCB_Type *)SCS_BASE)      /*!< System control Register not in SCB */
#define SCB ((SCB_Type *)SCB_BASE) /*!< SCB configuration struct */
#define SysTick \
    ((SysTick_Type *)SysTick_BASE)    /*!< SysTick configuration struct */
#define NVIC ((NVIC_Type *)NVIC_BASE) /*!< NVIC configuration struct */
#define ITM ((ITM_Type *)ITM_BASE)    /*!< ITM configuration struct */
#define DWT ((DWT_Type *)DWT_BASE)    /*!< DWT configuration struct */
#define TPI ((TPI_Type *)TPI_BASE)    /*!< TPI configuration struct */
#define CoreDebug \
    ((CoreDebug_Type *)CoreDebug_BASE) /*!< Core Debug configuration struct */

/**
 * @brief STM32F10x Interrupt Number Definition, according to the selected
 * device
 *        in @ref Library_configuration_section
 */

/*!< Interrupt Number Definition */
typedef enum {
    /******  Cortex-M3 Processor Exceptions Numbers
     ***************************************************/
    NonMaskableInt_IRQn = -14,   /*!< 2 Non Maskable Interrupt */
    HardFault_IRQn = -13,        /*!< 3 Cortex-M3 Hard Fault Interrupt */
    MemoryManagement_IRQn = -12, /*!< 4 Cortex-M3 Memory Management Interrupt */
    BusFault_IRQn = -11,         /*!< 5 Cortex-M3 Bus Fault Interrupt */
    UsageFault_IRQn = -10,       /*!< 6 Cortex-M3 Usage Fault Interrupt */
    SVCall_IRQn = -5,       /*!< 11 Cortex-M3 SV Call Interrupt            */
    DebugMonitor_IRQn = -4, /*!< 12 Cortex-M3 Debug Monitor Interrupt */
    PendSV_IRQn = -2,       /*!< 14 Cortex-M3 Pend SV Interrupt            */
    SysTick_IRQn = -1,      /*!< 15 Cortex-M3 System Tick Interrupt */

    /******  STM32 specific Interrupt Numbers
     *********************************************************/
    WWDG_IRQn = 0, /*!< Window WatchDog Interrupt                            */
    PVD_IRQn = 1,  /*!< PVD through EXTI Line detection Interrupt            */
    TAMPER_IRQn = 2, /*!< Tamper Interrupt */
    RTC_IRQn = 3,   /*!< RTC global Interrupt                                 */
    FLASH_IRQn = 4, /*!< FLASH global Interrupt                               */
    RCC_IRQn = 5,   /*!< RCC global Interrupt                                 */
    EXTI0_IRQn = 6, /*!< EXTI Line0 Interrupt                                 */
    EXTI1_IRQn = 7, /*!< EXTI Line1 Interrupt                                 */
    EXTI2_IRQn = 8, /*!< EXTI Line2 Interrupt                                 */
    EXTI3_IRQn = 9, /*!< EXTI Line3 Interrupt                                 */
    EXTI4_IRQn = 10,         /*!< EXTI Line4 Interrupt         */
    DMA1_Channel1_IRQn = 11, /*!< DMA1 Channel 1 global Interrupt */
    DMA1_Channel2_IRQn = 12, /*!< DMA1 Channel 2 global Interrupt */
    DMA1_Channel3_IRQn = 13, /*!< DMA1 Channel 3 global Interrupt */
    DMA1_Channel4_IRQn = 14, /*!< DMA1 Channel 4 global Interrupt */
    DMA1_Channel5_IRQn = 15, /*!< DMA1 Channel 5 global Interrupt */
    DMA1_Channel6_IRQn = 16, /*!< DMA1 Channel 6 global Interrupt */
    DMA1_Channel7_IRQn = 17, /*!< DMA1 Channel 7 global Interrupt */
    ADC1_2_IRQn = 18,        /*!< ADC1 and ADC2 global Interrupt        */
    USB_HP_CAN1_TX_IRQn =
        19, /*!< USB Device High Priority or CAN1 TX Interrupts       */
    USB_LP_CAN1_RX0_IRQn =
        20, /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
    CAN1_RX1_IRQn = 21,     /*!< CAN1 RX1 Interrupt */
    CAN1_SCE_IRQn = 22,     /*!< CAN1 SCE Interrupt */
    EXTI9_5_IRQn = 23,      /*!< External Line[9:5] Interrupts */
    TIM1_BRK_IRQn = 24,     /*!< TIM1 Break Interrupt */
    TIM1_UP_IRQn = 25,      /*!< TIM1 Update Interrupt */
    TIM1_TRG_COM_IRQn = 26, /*!< TIM1 Trigger and Commutation Interrupt */
    TIM1_CC_IRQn = 27,      /*!< TIM1 Capture Compare Interrupt */
    TIM2_IRQn = 28, /*!< TIM2 global Interrupt                                */
    TIM3_IRQn = 29, /*!< TIM3 global Interrupt                                */
    TIM4_IRQn = 30, /*!< TIM4 global Interrupt                                */
    I2C1_EV_IRQn = 31, /*!< I2C1 Event Interrupt */
    I2C1_ER_IRQn = 32, /*!< I2C1 Error Interrupt */
    I2C2_EV_IRQn = 33, /*!< I2C2 Event Interrupt */
    I2C2_ER_IRQn = 34, /*!< I2C2 Error Interrupt */
    SPI1_IRQn = 35, /*!< SPI1 global Interrupt                                */
    SPI2_IRQn = 36, /*!< SPI2 global Interrupt                                */
    USART1_IRQn = 37,    /*!< USART1 global Interrupt    */
    USART2_IRQn = 38,    /*!< USART2 global Interrupt    */
    USART3_IRQn = 39,    /*!< USART3 global Interrupt    */
    EXTI15_10_IRQn = 40, /*!< External Line[15:10] Interrupts */
    RTC_Alarm_IRQn = 41, /*!< RTC Alarm through EXTI Line Interrupt */
    USBWakeUp_IRQn =
        42, /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
} IRQn_Type;

/**
 * @brief Analog to Digital Converter
 */

typedef struct {
    __IO u32 SR;
    __IO u32 CR1;
    __IO u32 CR2;
    __IO u32 SMPR1;
    __IO u32 SMPR2;
    __IO u32 JOFR1;
    __IO u32 JOFR2;
    __IO u32 JOFR3;
    __IO u32 JOFR4;
    __IO u32 HTR;
    __IO u32 LTR;
    __IO u32 SQR1;
    __IO u32 SQR2;
    __IO u32 SQR3;
    __IO u32 JSQR;
    __IO u32 JDR1;
    __IO u32 JDR2;
    __IO u32 JDR3;
    __IO u32 JDR4;
    __IO u32 DR;
} ADC_TypeDef;

typedef struct {
    __IO u32 SR;  /*!< ADC status register,    used for ADC multimode (bits
                          common to several ADC instances). Address offset: ADC1
                          base address         */
    __IO u32 CR1; /*!< ADC control register 1, used for ADC multimode (bits
                          common to several ADC instances). Address offset: ADC1
                          base address + 0x04  */
    __IO u32 CR2; /*!< ADC control register 2, used for ADC multimode (bits
                          common to several ADC instances). Address offset: ADC1
                          base address + 0x08  */
    u32 RESERVED[16];
    __IO u32 DR; /*!< ADC data register,      used for ADC multimode (bits
                         common to several ADC instances). Address offset: ADC1
                         base address + 0x4C  */
} ADC_Common_TypeDef;

/**
 * @brief Backup Registers
 */

typedef struct {
    u32 RESERVED0;
    __IO u32 DR1;
    __IO u32 DR2;
    __IO u32 DR3;
    __IO u32 DR4;
    __IO u32 DR5;
    __IO u32 DR6;
    __IO u32 DR7;
    __IO u32 DR8;
    __IO u32 DR9;
    __IO u32 DR10;
    __IO u32 RTCCR;
    __IO u32 CR;
    __IO u32 CSR;
} BKP_TypeDef;

/**
 * @brief Controller Area Network TxMailBox
 */

typedef struct {
    __IO u32 TIR;
    __IO u32 TDTR;
    __IO u32 TDLR;
    __IO u32 TDHR;
} CAN_TxMailBox_TypeDef;

/**
 * @brief Controller Area Network FIFOMailBox
 */

typedef struct {
    __IO u32 RIR;
    __IO u32 RDTR;
    __IO u32 RDLR;
    __IO u32 RDHR;
} CAN_FIFOMailBox_TypeDef;

/**
 * @brief Controller Area Network FilterRegister
 */

typedef struct {
    __IO u32 FR1;
    __IO u32 FR2;
} CAN_FilterRegister_TypeDef;

/**
 * @brief Controller Area Network
 */

typedef struct {
    __IO u32 MCR;
    __IO u32 MSR;
    __IO u32 TSR;
    __IO u32 RF0R;
    __IO u32 RF1R;
    __IO u32 IER;
    __IO u32 ESR;
    __IO u32 BTR;
    u32 RESERVED0[88];
    CAN_TxMailBox_TypeDef sTxMailBox[3];
    CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
    u32 RESERVED1[12];
    __IO u32 FMR;
    __IO u32 FM1R;
    u32 RESERVED2;
    __IO u32 FS1R;
    u32 RESERVED3;
    __IO u32 FFA1R;
    u32 RESERVED4;
    __IO u32 FA1R;
    u32 RESERVED5[8];
    CAN_FilterRegister_TypeDef sFilterRegister[14];
} CAN_TypeDef;

/**
 * @brief CRC calculation unit
 */

typedef struct {
    __IO u32 DR;  /*!< CRC Data register,                           Address
                          offset: 0x00 */
    __IO u8 IDR;  /*!< CRC Independent data register,               Address
                          offset: 0x04 */
    u8 RESERVED0; /*!< Reserved,                                    Address
                          offset: 0x05 */
    u16 RESERVED1; /*!< Reserved, Address offset: 0x06 */
    __IO u32 CR; /*!< CRC Control register,                        Address
                         offset: 0x08 */
} CRC_TypeDef;

/**
 * @brief Debug MCU
 */

typedef struct {
    __IO u32 IDCODE;
    __IO u32 CR;
} DBGMCU_TypeDef;

/**
 * @brief DMA Controller
 */

typedef struct {
    __IO u32 CCR;
    __IO u32 CNDTR;
    __IO u32 CPAR;
    __IO u32 CMAR;
} DMA_Channel_TypeDef;

typedef struct {
    __IO u32 ISR;
    __IO u32 IFCR;
} DMA_TypeDef;

/**
 * @brief External Interrupt/Event Controller
 */

typedef struct {
    __IO u32 IMR;
    __IO u32 EMR;
    __IO u32 RTSR;
    __IO u32 FTSR;
    __IO u32 SWIER;
    __IO u32 PR;
} EXTI_TypeDef;

/**
 * @brief FLASH Registers
 */

typedef struct {
    __IO u32 ACR;
    __IO u32 KEYR;
    __IO u32 OPTKEYR;
    __IO u32 SR;
    __IO u32 CR;
    __IO u32 AR;
    __IO u32 RESERVED;
    __IO u32 OBR;
    __IO u32 WRPR;
} FLASH_TypeDef;

/**
 * @brief Option Bytes Registers
 */

typedef struct {
    __IO u16 RDP;
    __IO u16 USER;
    __IO u16 Data0;
    __IO u16 Data1;
    __IO u16 WRP0;
    __IO u16 WRP1;
    __IO u16 WRP2;
    __IO u16 WRP3;
} OB_TypeDef;

/**
 * @brief General Purpose I/O
 */

typedef struct {
    __IO u32 CRL;
    __IO u32 CRH;
    __IO u32 IDR;
    __IO u32 ODR;
    __IO u32 BSRR;
    __IO u32 BRR;
    __IO u32 LCKR;
} GPIO_TypeDef;

/**
 * @brief Alternate Function I/O
 */

typedef struct {
    __IO u32 EVCR;
    __IO u32 MAPR;
    __IO u32 EXTICR[4];
    u32 RESERVED0;
    __IO u32 MAPR2;
} AFIO_TypeDef;
/**
 * @brief Inter Integrated Circuit Interface
 */

typedef struct {
    __IO u32 CR1;
    __IO u32 CR2;
    __IO u32 OAR1;
    __IO u32 OAR2;
    __IO u32 DR;
    __IO u32 SR1;
    __IO u32 SR2;
    __IO u32 CCR;
    __IO u32 TRISE;
} I2C_TypeDef;

/**
 * @brief Independent WATCHDOG
 */

typedef struct {
    __IO u32 KR;  /*!< Key register,                                Address
                          offset: 0x00 */
    __IO u32 PR;  /*!< Prescaler register,                          Address
                          offset: 0x04 */
    __IO u32 RLR; /*!< Reload register,                             Address
                          offset: 0x08 */
    __IO u32 SR;  /*!< Status register,                             Address
                          offset: 0x0C */
} IWDG_TypeDef;

/**
 * @brief Power Control
 */

typedef struct {
    __IO u32 CR;
    __IO u32 CSR;
} PWR_TypeDef;

/**
 * @brief Reset and Clock Control
 */

typedef struct {
    __IO u32 CR;
    __IO u32 CFGR;
    __IO u32 CIR;
    __IO u32 APB2RSTR;
    __IO u32 APB1RSTR;
    __IO u32 AHBENR;
    __IO u32 APB2ENR;
    __IO u32 APB1ENR;
    __IO u32 BDCR;
    __IO u32 CSR;

} RCC_TypeDef;

/**
 * @brief Real-Time Clock
 */

typedef struct {
    __IO u32 CRH;
    __IO u32 CRL;
    __IO u32 PRLH;
    __IO u32 PRLL;
    __IO u32 DIVH;
    __IO u32 DIVL;
    __IO u32 CNTH;
    __IO u32 CNTL;
    __IO u32 ALRH;
    __IO u32 ALRL;
} RTC_TypeDef;

/**
 * @brief SD host Interface
 */

typedef struct {
    __IO u32 POWER;
    __IO u32 CLKCR;
    __IO u32 ARG;
    __IO u32 CMD;
    __I u32 RESPCMD;
    __I u32 RESP1;
    __I u32 RESP2;
    __I u32 RESP3;
    __I u32 RESP4;
    __IO u32 DTIMER;
    __IO u32 DLEN;
    __IO u32 DCTRL;
    __I u32 DCOUNT;
    __I u32 STA;
    __IO u32 ICR;
    __IO u32 MASK;
    u32 RESERVED0[2];
    __I u32 FIFOCNT;
    u32 RESERVED1[13];
    __IO u32 FIFO;
} SDIO_TypeDef;

/**
 * @brief Serial Peripheral Interface
 */

typedef struct {
    __IO u32 CR1;
    __IO u32 CR2;
    __IO u32 SR;
    __IO u32 DR;
    __IO u32 CRCPR;
    __IO u32 RXCRCR;
    __IO u32 TXCRCR;
    __IO u32 I2SCFGR;
} SPI_TypeDef;

/**
 * @brief TIM Timers
 */
typedef struct {
    __IO u32 CR1; /*!< TIM control register 1,                      Address
                          offset: 0x00 */
    __IO u32 CR2; /*!< TIM control register 2,                      Address
                          offset: 0x04 */
    __IO u32 SMCR; /*!< TIM slave Mode Control register, Address offset:
                           0x08 */
    __IO u32 DIER; /*!< TIM DMA/interrupt enable register, Address offset:
                  0x0C */
    __IO u32 SR;  /*!< TIM status register,                         Address
                          offset: 0x10 */
    __IO u32 EGR; /*!< TIM event generation register,               Address
                          offset: 0x14 */
    __IO u32
        CCMR1; /*!< TIM  capture/compare mode register 1,        Address offset:
                  0x18 */
    __IO u32
        CCMR2; /*!< TIM  capture/compare mode register 2,        Address offset:
                  0x1C */
    __IO u32
        CCER; /*!< TIM capture/compare enable register,         Address offset:
        0x20 */
    __IO u32 CNT; /*!< TIM counter register,                        Address
                          offset: 0x24 */
    __IO u32 PSC; /*!< TIM prescaler register,                      Address
                          offset: 0x28 */
    __IO u32 ARR; /*!< TIM auto-reload register,                    Address
                          offset: 0x2C */
    __IO u32 RCR; /*!< TIM  repetition counter register,            Address
                          offset: 0x30 */
    __IO u32 CCR1; /*!< TIM capture/compare register 1, Address offset:
                           0x34 */
    __IO u32 CCR2; /*!< TIM capture/compare register 2, Address offset:
                           0x38 */
    __IO u32 CCR3; /*!< TIM capture/compare register 3, Address offset:
                           0x3C */
    __IO u32 CCR4; /*!< TIM capture/compare register 4, Address offset:
                           0x40 */
    __IO u32 BDTR; /*!< TIM break and dead-time register, Address offset:
                  0x44 */
    __IO u32 DCR; /*!< TIM DMA control register,                    Address
                          offset: 0x48 */
    __IO u32
        DMAR; /*!< TIM DMA address for full transfer register,  Address offset:
         0x4C */
    __IO u32 OR; /*!< TIM option register,                         Address
                         offset: 0x50 */
} TIM_TypeDef;

/**
 * @brief Universal Synchronous Asynchronous Receiver Transmitter
 */

typedef struct {
    __IO u32 SR;   /*!< USART Status register,                   Address
                           offset: 0x00 */
    __IO u32 DR;   /*!< USART Data register,                     Address
                           offset: 0x04 */
    __IO u32 BRR;  /*!< USART Baud rate register,                Address
                           offset: 0x08 */
    __IO u32 CR1;  /*!< USART Control register 1,                Address
                           offset: 0x0C */
    __IO u32 CR2;  /*!< USART Control register 2,                Address
                           offset: 0x10 */
    __IO u32 CR3;  /*!< USART Control register 3,                Address
                           offset: 0x14 */
    __IO u32 GTPR; /*!< USART Guard time and prescaler register, Address
                           offset: 0x18 */
} USART_TypeDef;

/**
 * @brief Universal Serial Bus Full Speed Device
 */

typedef struct {
    __IO u16 EP0R; /*!< USB Endpoint 0 register,                   Address
                  offset: 0x00 */
    __IO u16 RESERVED0; /*!< Reserved */
    __IO u16 EP1R; /*!< USB Endpoint 1 register,                   Address
                  offset: 0x04 */
    __IO u16 RESERVED1; /*!< Reserved */
    __IO u16 EP2R; /*!< USB Endpoint 2 register,                   Address
                  offset: 0x08 */
    __IO u16 RESERVED2; /*!< Reserved */
    __IO u16 EP3R; /*!< USB Endpoint 3 register,                   Address
                  offset: 0x0C */
    __IO u16 RESERVED3; /*!< Reserved */
    __IO u16 EP4R; /*!< USB Endpoint 4 register,                   Address
                  offset: 0x10 */
    __IO u16 RESERVED4; /*!< Reserved */
    __IO u16 EP5R; /*!< USB Endpoint 5 register,                   Address
                  offset: 0x14 */
    __IO u16 RESERVED5; /*!< Reserved */
    __IO u16 EP6R; /*!< USB Endpoint 6 register,                   Address
                  offset: 0x18 */
    __IO u16 RESERVED6; /*!< Reserved */
    __IO u16 EP7R; /*!< USB Endpoint 7 register,                   Address
                  offset: 0x1C */
    __IO u16 RESERVED7[17]; /*!< Reserved */
    __IO u16 CNTR; /*!< Control register,                          Address
                  offset: 0x40 */
    __IO u16 RESERVED8; /*!< Reserved */
    __IO u16 ISTR; /*!< Interrupt status register,                 Address
                  offset: 0x44 */
    __IO u16 RESERVED9; /*!< Reserved */
    __IO u16 FNR; /*!< Frame number register,                     Address
                offset: 0x48 */
    __IO u16 RESERVEDA; /*!< Reserved */
    __IO u16 DADDR; /*!< Device address register,                   Address
                    offset: 0x4C */
    __IO u16 RESERVEDB; /*!< Reserved */
    __IO u16 BTABLE; /*!< Buffer Table address register,    Address offset:
                0x50 */
    __IO u16 RESERVEDC; /*!< Reserved */
} USB_TypeDef;

/**
 * @brief Window WATCHDOG
 */

typedef struct {
    __IO u32 CR;  /*!< WWDG Control register,       Address offset: 0x00 */
    __IO u32 CFR; /*!< WWDG Configuration register, Address offset: 0x04 */
    __IO u32 SR;  /*!< WWDG Status register,        Address offset: 0x08 */
} WWDG_TypeDef;

#define FLASH_BASE 0x08000000U /*!< FLASH base address in the alias region */
#define FLASH_BANK1_END 0x0801FFFFU /*!< FLASH END address of bank1 */
#define SRAM_BASE 0x20000000U /*!< SRAM base address in the alias region */
#define PERIPH_BASE \
    0x40000000U /*!< Peripheral base address in the alias region */

#define SRAM_BB_BASE \
    0x22000000U /*!< SRAM base address in the bit-band region */
#define PERIPH_BB_BASE \
    0x42000000U /*!< Peripheral base address in the bit-band region */

/*!< Peripheral memory map */
#define APB1PERIPH_BASE PERIPH_BASE
#define APB2PERIPH_BASE (PERIPH_BASE + 0x00010000U)
#define AHBPERIPH_BASE (PERIPH_BASE + 0x00020000U)

#define TIM2_BASE (APB1PERIPH_BASE + 0x00000000U)
#define TIM3_BASE (APB1PERIPH_BASE + 0x00000400U)
#define TIM4_BASE (APB1PERIPH_BASE + 0x00000800U)
#define RTC_BASE (APB1PERIPH_BASE + 0x00002800U)
#define WWDG_BASE (APB1PERIPH_BASE + 0x00002C00U)
#define IWDG_BASE (APB1PERIPH_BASE + 0x00003000U)
#define SPI2_BASE (APB1PERIPH_BASE + 0x00003800U)
#define USART2_BASE (APB1PERIPH_BASE + 0x00004400U)
#define USART3_BASE (APB1PERIPH_BASE + 0x00004800U)
#define I2C1_BASE (APB1PERIPH_BASE + 0x00005400U)
#define I2C2_BASE (APB1PERIPH_BASE + 0x5800)
#define CAN1_BASE (APB1PERIPH_BASE + 0x00006400U)
#define BKP_BASE (APB1PERIPH_BASE + 0x00006C00U)
#define PWR_BASE (APB1PERIPH_BASE + 0x00007000U)
#define AFIO_BASE (APB2PERIPH_BASE + 0x00000000U)
#define EXTI_BASE (APB2PERIPH_BASE + 0x00000400U)
#define GPIOA_BASE (APB2PERIPH_BASE + 0x00000800U)
#define GPIOB_BASE (APB2PERIPH_BASE + 0x00000C00U)
#define GPIOC_BASE (APB2PERIPH_BASE + 0x00001000U)
#define GPIOD_BASE (APB2PERIPH_BASE + 0x00001400U)
#define GPIOE_BASE (APB2PERIPH_BASE + 0x00001800U)
#define ADC1_BASE (APB2PERIPH_BASE + 0x00002400U)
#define ADC2_BASE (APB2PERIPH_BASE + 0x00002800U)
#define TIM1_BASE (APB2PERIPH_BASE + 0x00002C00U)
#define SPI1_BASE (APB2PERIPH_BASE + 0x00003000U)
#define USART1_BASE (APB2PERIPH_BASE + 0x00003800U)

#define SDIO_BASE (PERIPH_BASE + 0x00018000U)

#define DMA1_BASE (AHBPERIPH_BASE + 0x00000000U)
#define DMA1_Channel1_BASE (AHBPERIPH_BASE + 0x00000008U)
#define DMA1_Channel2_BASE (AHBPERIPH_BASE + 0x0000001CU)
#define DMA1_Channel3_BASE (AHBPERIPH_BASE + 0x00000030U)
#define DMA1_Channel4_BASE (AHBPERIPH_BASE + 0x00000044U)
#define DMA1_Channel5_BASE (AHBPERIPH_BASE + 0x00000058U)
#define DMA1_Channel6_BASE (AHBPERIPH_BASE + 0x0000006CU)
#define DMA1_Channel7_BASE (AHBPERIPH_BASE + 0x00000080U)
#define RCC_BASE (AHBPERIPH_BASE + 0x00001000U)
#define CRC_BASE (AHBPERIPH_BASE + 0x00003000U)

#define FLASH_R_BASE \
    (AHBPERIPH_BASE + 0x00002000U) /*!< Flash registers base address */
#define FLASHSIZE_BASE 0x1FFFF7E0U /*!< FLASH Size register base address */
#define UID_BASE 0x1FFFF7E8U /*!< Unique device ID register base address */
#define OB_BASE 0x1FFFF800U  /*!< Flash Option Bytes base address */

#define DBGMCU_BASE 0xE0042000U /*!< Debug MCU registers base address */

/* USB device FS */
#define USB_BASE       \
    (APB1PERIPH_BASE + \
     0x00005C00U) /*!< USB_IP Peripheral Registers base address */
#define USB_PMAADDR                                               \
    (APB1PERIPH_BASE +                                            \
     0x00006000U) /*!< USB_IP Packet Memory Area base address \ \ \
                   * \ \ \ \ \ \ \ \ \ \                      \ \ \
                   */

#define WWDG ((WWDG_TypeDef *)WWDG_BASE)
#define IWDG ((IWDG_TypeDef *)IWDG_BASE)
#define SPI2 ((SPI_TypeDef *)SPI2_BASE)
#define USART2 ((USART_TypeDef *)USART2_BASE)
#define USART3 ((USART_TypeDef *)USART3_BASE)
#define I2C1 ((I2C_TypeDef *)I2C1_BASE)
#define I2C2 ((I2C_TypeDef *)I2C2_BASE)
#define USB ((USB_TypeDef *)USB_BASE)
#define CAN1 ((CAN_TypeDef *)CAN1_BASE)
#define BKP ((BKP_TypeDef *)BKP_BASE)
#define PWR ((PWR_TypeDef *)PWR_BASE)
#define AFIO ((AFIO_TypeDef *)AFIO_BASE)
#define EXTI ((EXTI_TypeDef *)EXTI_BASE)
#define GPIOA ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD ((GPIO_TypeDef *)GPIOD_BASE)
#define GPIOE ((GPIO_TypeDef *)GPIOE_BASE)
#define ADC1 ((ADC_TypeDef *)ADC1_BASE)
#define ADC2 ((ADC_TypeDef *)ADC2_BASE)
#define ADC12_COMMON ((ADC_Common_TypeDef *)ADC1_BASE)
#define TIM1 ((TIM_TypeDef *)TIM1_BASE)
#define TIM2 ((TIM_TypeDef *)TIM2_BASE)
#define SPI1 ((SPI_TypeDef *)SPI1_BASE)
#define USART1 ((USART_TypeDef *)USART1_BASE)
#define SDIO ((SDIO_TypeDef *)SDIO_BASE)
#define DMA1 ((DMA_TypeDef *)DMA1_BASE)
#define DMA1_Channel1 ((DMA_Channel_TypeDef *)DMA1_Channel1_BASE)
#define DMA1_Channel2 ((DMA_Channel_TypeDef *)DMA1_Channel2_BASE)
#define DMA1_Channel3 ((DMA_Channel_TypeDef *)DMA1_Channel3_BASE)
#define DMA1_Channel4 ((DMA_Channel_TypeDef *)DMA1_Channel4_BASE)
#define DMA1_Channel5 ((DMA_Channel_TypeDef *)DMA1_Channel5_BASE)
#define DMA1_Channel6 ((DMA_Channel_TypeDef *)DMA1_Channel6_BASE)
#define DMA1_Channel7 ((DMA_Channel_TypeDef *)DMA1_Channel7_BASE)
#define RCC ((RCC_TypeDef *)RCC_BASE)
#define CRC ((CRC_TypeDef *)CRC_BASE)
#define FLASH ((FLASH_TypeDef *)FLASH_R_BASE)
#define OB ((OB_TypeDef *)OB_BASE)
#define DBGMCU ((DBGMCU_TypeDef *)DBGMCU_BASE)

#endif /* STM32F103C8T6_H */
