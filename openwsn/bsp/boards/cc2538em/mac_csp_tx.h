#ifndef mac_csp_tx_h
#define mac_csp_tx_h


#define MAX_DELTA 10
typedef struct {
	uint8_t macTxCsmaBackoffDelay;
	uint8_t maxBe;
	uint8_t minBe;
	uint8_t macTxBe ;
	uint8_t nb;
	uint32_t countBusy;
	uint32_t countok;
	uint32_t counterr;
	uint32_t capturedTime0;
    uint32_t capturedTime1;
    uint32_t rfftxendok;
    uint32_t rfftxstop2;
    uint32_t rfftxwait;
    uint32_t rfftxbusy;
	uint32_t delta[MAX_DELTA];
    uint32_t deltaMax;
	uint8_t deltapos;
} radio_csma_vars_t;


/* bit value used to form values of macRxActive */
#define MAC_RX_ACTIVE_PHYSICAL_BV       0x80

#define MAC_RX_ACTIVE_NO_ACTIVITY       0x00  /* zero reserved for boolean use, e.g. !macRxActive */
#define MAC_RX_ACTIVE_STARTED           (0x01 | MAC_RX_ACTIVE_PHYSICAL_BV)
#define MAC_RX_ACTIVE_DONE              0x02


#define MAC_RX_IS_PHYSICALLY_ACTIVE()   ((macRxActive & MAC_RX_ACTIVE_PHYSICAL_BV) || macRxOutgoingAckFlag)

/* ------------------------------------------------------------------------------------------------
 *                                        CC253x rev numbers
 * ------------------------------------------------------------------------------------------------
 */
#define REV_A          0x00    /* workaround turned off */
#define REV_B          0x11    /* PG1.1 */
#define CHIPID         0x40

#ifndef BV
#define BV(n)      (1 << (n))
#endif

#ifndef BF
#define BF(x,b,s)  (((x) & (b)) >> (s))
#endif

typedef unsigned char halIntState_t;

#define T2M_SEL(x)      (x)

/* ------------------------------------------------------------------------------------------------
 *                                   CSP Defines / Macros
 * ------------------------------------------------------------------------------------------------
 */
/* immediate strobe commands */
#define ISSTART     0xE1
#define ISSTOP      0xE2
#define ISCLEAR     0xFF


#define HAL_ENTER_CRITICAL_SECTION(x)   st( x = EA;  HAL_DISABLE_INTERRUPTS(); )
#define HAL_EXIT_CRITICAL_SECTION(x)    st( EA = x; )

/* Hal Critical statement definition */
#define HAL_CRITICAL_STATEMENT(x)       st( halIntState_t s; HAL_ENTER_CRITICAL_SECTION(s); x; HAL_EXIT_CRITICAL_SECTION(s); )

#define HAL_CPU_CLOCK_MHZ     32UL

// The following are defines for the SYS_CTRL register offsets.

/* CCACTRL0 */
#define CCA_THR                       0xFCUL   /* -4-76=-80dBm when CC2538 operated alone or with CC2591 in LGM */
/* CC2538-CC2591 has 6db gain whereas CC2538-CC2592 has 3db. Choosing the lower value for threshold */
#define CCA_THR_HGM                   0x03UL   /* 3-76=-73dBm when CC2538 operated with CC2592 in HGM */
#define CCA_THR_MINUS_20              0x38UL


/* T2IRQF */
#define TIMER2_OVF_COMPARE2F  BV(5)
#define TIMER2_OVF_COMPARE1F  BV(4)
#define TIMER2_OVF_PERF       BV(3)
#define TIMER2_COMPARE2F      BV(2)
#define TIMER2_COMPARE1F      BV(1)
#define TIMER2_PERF           BV(0)

/* RFIRQM1 */
#define IM_TXACKDONE    BV(0)
#define IM_TXDONE       BV(1)
#define IM_CSP_MANINT   BV(3)
#define IM_CSP_STOP     BV(4)
#define IM_CSP_WAIT     BV(5)

/* RFIRQF1 */
#define IRQ_TXACKDONE   BV(0)
#define IRQ_TXDONE      BV(1)
#define IRQ_CSP_MANINT  BV(3)
#define IRQ_CSP_STOP    BV(4)
#define IRQ_CSP_WAIT    BV(5)

/* ------------------------------------------------------------------------------------------------
 *                                       MAC Timer Macros
 * ------------------------------------------------------------------------------------------------
 */

#define T2M_OVF_BITS    (BV(6) | BV(5) | BV(4))
#define T2M_BITS        (BV(2) | BV(1) | BV(0))

#define T2M_OVFSEL(x)   ((x) << 4)
#define T2M_SEL(x)      (x)

#define T2M_T2OVF       T2M_OVFSEL(0UL)
#define T2M_T2OVF_CAP   T2M_OVFSEL(1UL)
#define T2M_T2OVF_PER   T2M_OVFSEL(2UL)
#define T2M_T2OVF_CMP1  T2M_OVFSEL(3UL)
#define T2M_T2OVF_CMP2  T2M_OVFSEL(4UL)

#define T2M_T2TIM       T2M_SEL(0UL)
#define T2M_T2_CAP      T2M_SEL(1UL)
#define T2M_T2_PER      T2M_SEL(2UL)
#define T2M_T2_CMP1     T2M_SEL(3UL)
#define T2M_T2_CMP2     T2M_SEL(4UL)

#define MAC_MCU_T2_ACCESS_OVF_COUNT_VALUE()   st( T2MSEL = T2M_T2OVF; )
#define MAC_MCU_T2_ACCESS_OVF_CAPTURE_VALUE() st( T2MSEL = T2M_T2OVF_CAP; )
#define MAC_MCU_T2_ACCESS_OVF_PERIOD_VALUE()  st( T2MSEL = T2M_T2OVF_PER; )
#define MAC_MCU_T2_ACCESS_OVF_CMP1_VALUE()    st( T2MSEL = T2M_T2OVF_CMP1; )
#define MAC_MCU_T2_ACCESS_OVF_CMP2_VALUE()    st( T2MSEL = T2M_T2OVF_CMP2; )

#define MAC_MCU_T2_ACCESS_COUNT_VALUE()       st( T2MSEL = T2M_T2TIM; )
#define MAC_MCU_T2_ACCESS_CAPTURE_VALUE()     st( T2MSEL = T2M_T2_CAP; )
#define MAC_MCU_T2_ACCESS_PERIOD_VALUE()      st( T2MSEL = T2M_T2_PER; )
#define MAC_MCU_T2_ACCESS_CMP1_VALUE()        st( T2MSEL = T2M_T2_CMP1; )
#define MAC_MCU_T2_ACCESS_CMP2_VALUE()        st( T2MSEL = T2M_T2_CMP2; )

#define MAC_MCU_CONFIG_CSP_EVENT1()           st( T2CSPCFG = 1UL; )

//#define HAL_CLOCK_STABLE()    st( while (CLKCONSTA != (CLKCONCMD_32MHZ | OSC_32KHZ)); )

#define MAC_RADIO_TIMER_SLEEP()                       st( T2CTRL &= ~TIMER2_RUN; while(T2CTRL & TIMER2_STATE); )
#define MAC_RADIO_TIMER_WAKE_UP()                     st( HAL_CLOCK_STABLE(); \
                                                          T2CTRL |= (TIMER2_RUN | TIMER2_SYNC); \
                                                          while(!(T2CTRL & TIMER2_STATE)); )
/* ----- values specific to 2450 MHz PHY ----- */
/* The number of symbols forming a basic CSMA-CA time period */
#define MAC_A_UNIT_BACKOFF_PERIOD       20

/* minimum receiver sensitivity in dBm (see 6.5.3.3) */
#define MAC_SPEC_MIN_RECEIVER_SENSITIVITY   -85

/* Length of preamble field in symbols */
#define MAC_SPEC_PREAMBLE_FIELD_LENGTH      8

/* Length of SFD field in symbols */
#define MAC_SPEC_SFD_FIELD_LENGTH           2

/* Microseconds in one symbol */
#define MAC_SPEC_USECS_PER_SYMBOL           16

/* Microseconds in one backoff period */
#define MAC_SPEC_USECS_PER_BACKOFF          (MAC_SPEC_USECS_PER_SYMBOL * MAC_A_UNIT_BACKOFF_PERIOD)

/* octets (or bytes) per symbol */
#define MAC_SPEC_OCTETS_PER_SYMBOL          2


#define MAC_RADIO_TIMER_TICKS_PER_USEC()              HAL_CPU_CLOCK_MHZ /* never fractional */
#define MAC_RADIO_TIMER_TICKS_PER_BACKOFF()           (HAL_CPU_CLOCK_MHZ * MAC_SPEC_USECS_PER_BACKOFF)
#define MAC_RADIO_TIMER_TICKS_PER_SYMBOL()            (HAL_CPU_CLOCK_MHZ * MAC_SPEC_USECS_PER_SYMBOL)

/* SFR registers */
#define T2CSPCFG                     HWREG(RFCORE_SFR_MTCSPCFG)
#define T2CTRL                       HWREG(RFCORE_SFR_MTCTRL)
#define T2IRQM                       HWREG(RFCORE_SFR_MTIRQM)
#define T2IRQF                       HWREG(RFCORE_SFR_MTIRQF)
#define T2MSEL                       HWREG(RFCORE_SFR_MTMSEL)
#define T2M0                         HWREG(RFCORE_SFR_MTM0)
#define T2M1                         HWREG(RFCORE_SFR_MTM1)
#define T2MOVF2                      HWREG(RFCORE_SFR_MTMOVF2)
#define T2MOVF1                      HWREG(RFCORE_SFR_MTMOVF1)
#define T2MOVF0                      HWREG(RFCORE_SFR_MTMOVF0)
#define RFD                          HWREG(RFCORE_SFR_RFDATA)
#define RFERRF                       HWREG(RFCORE_SFR_RFERRF)
#define RFIRQF1                      HWREG(RFCORE_SFR_RFIRQF1)
#define RFIRQF0                      HWREG(RFCORE_SFR_RFIRQF0)
#define RFST                         HWREG(RFCORE_SFR_RFST)

/* T2CTRL */
#define LATCH_MODE            BV(3)
#define TIMER2_STATE          BV(2)
#define TIMER2_SYNC           BV(1)
#define TIMER2_RUN            BV(0)

/* FRMCTRL0 */
#define FRMCTRL0_RESET_VALUE          0x40UL
#define ENERGY_SCAN                   BV(4)
#define AUTOACK                       BV(5)
#define RX_MODE(x)                    ((x) << 2)
#define RX_MODE_INFINITE_RECEPTION    RX_MODE(2UL)
#define RX_MODE_NORMAL_OPERATION      RX_MODE(0UL)

/* RFIRQF0 */
#define IRQ_SFD         BV(1)
#define IRQ_FIFOP       BV(2)

/* immediate strobe commands */
#define ISTXCAL       0xECUL
#define ISRXON        0xE3UL
#define ISTXON        0xE9UL
#define ISTXONCCA     0xEAUL
#define ISRFOFF       0xEFUL
#define ISFLUSHRX     0xEDUL
#define ISFLUSHTX     0xEEUL
#define ISACK         0xE6UL
#define ISACKPEND     0xE7UL
#define ISNACK        0xE8UL

#define MAC_RADIO_RX_ON                             HWREG(RFCORE_SFR_RFST) = ISRXON;
#define MAC_RADIO_RXTX_OFF                          HWREG(RFCORE_SFR_RFST) = ISRFOFF;

/* Proprietary PIB Get-only Attributes */
#define MAC_RANDOM_SEED                   0xEF  /* An array of MAC_RANDOM_SEED_LEN bytes of random bits */
/* The length of the random seed is set for maximum requirement which is
 * 32 for ZigBeeIP
 */
#define MAC_RANDOM_SEED_LEN         32

/* strobe processor command instructions */
#define SSTOP       (0xD2)    /* stop program execution                                      */
#define SNOP        (0xD0)    /* no operation                                                */
#define STXCAL      (0xDC)    /* enable and calibrate frequency synthesizer for TX           */
#define SRXON       (0xD3)    /* turn on receiver                                            */
#define STXON       (0xD9)    /* transmit after calibration                                  */
#define STXONCCA    (0xDA)    /* transmit after calibration if CCA indicates clear channel   */
#define SRFOFF      (0xDF)    /* turn off RX/TX                                              */
#define SFLUSHRX    (0xDD)    /* flush receive FIFO                                          */
#define SFLUSHTX    (0xDE)    /* flush transmit FIFO                                         */
#define SACK        (0xD6)    /* send ACK frame                                              */
#define SACKPEND    (0xD7)    /* send ACK frame with pending bit set                         */


#define C_NEGATE(c)   ((c) | 0x08)

/* conditions for use with instructions SKIP and RPT */
#define C_CCA_IS_VALID        0x00
#define C_SFD_IS_ACTIVE       0x01
#define C_CPU_CTRL_IS_ON      0x02
#define C_END_INSTR_MEM       0x03
#define C_CSPX_IS_ZERO        0x04
#define C_CSPY_IS_ZERO        0x05
#define C_CSPZ_IS_ZERO        0x06
#define C_RSSI_IS_VALID       0x07

/* negated conditions for use with instructions SKIP and RPT */
#define C_NEGATE(c)   ((c) | 0x08)
#define C_CCA_IS_INVALID      C_NEGATE(C_CCA_IS_VALID)
#define C_SFD_IS_INACTIVE     C_NEGATE(C_SFD_IS_ACTIVE)
#define C_CPU_CTRL_IS_OFF     C_NEGATE(C_CPU_CTRL_IS_ON)
#define C_NOT_END_INSTR_MEM   C_NEGATE(C_END_INSTR_MEM)
#define C_CSPX_IS_NON_ZERO    C_NEGATE(C_CSPX_IS_ZERO)
#define C_CSPY_IS_NON_ZERO    C_NEGATE(C_CSPY_IS_ZERO)
#define C_CSPZ_IS_NON_ZERO    C_NEGATE(C_CSPZ_IS_ZERO)
#define C_RSSI_IS_INVALID     C_NEGATE(C_RSSI_IS_VALID)

/* strobe processor instructions */
#define SKIP(s,c)   (0x00 | (((s) & 0x07) << 4) | ((c) & 0x0F))   /* skip 's' instructions if 'c' is true  */
#define WHILE(c)    SKIP(0,c)              /* pend while 'c' is true (derived instruction)        */
#define WAITW(w)    (0x80 | ((w) & 0x1F))  /* wait for 'w' number of MAC timer overflows          */
#define WEVENT1     (0xB8)                 /* wait for MAC timer compare                          */
#define WAITX       (0xBC)                 /* wait for CSPX number of MAC timer overflows         */
#define LABEL       (0xBB)                 /* set next instruction as start of loop               */
#define RPT(c)      (0xA0 | ((c) & 0x0F))  /* if condition is true jump to last label             */
#define INT         (0xBA)                 /* assert IRQ_CSP_INT interrupt                        */
#define INCY        (0xC1)                 /* increment CSPY                                      */
#define INCMAXY(m)  (0xC8 | ((m) & 0x07))  /* increment CSPY but not above maximum value of 'm'   */
#define DECX        (0xC3)                 /* decrement CSPX                                      */
#define DECY        (0xC4)                 /* decrement CSPY                                      */
#define DECZ        (0xC5)                 /* decrement CSPZ                                      */
#define RANDXY      (0xBD)                 /* load the lower CSPY bits of CSPX with random value  */

/* strobe processor command instructions */
#define SSTOP       (0xD2)    /* stop program execution                                      */
#define SNOP        (0xD0)    /* no operation                                                */
#define STXCAL      (0xDC)    /* enable and calibrate frequency synthesizer for TX           */
#define SRXON       (0xD3)    /* turn on receiver                                            */
#define STXON       (0xD9)    /* transmit after calibration                                  */
#define STXONCCA    (0xDA)    /* transmit after calibration if CCA indicates clear channel   */
#define SRFOFF      (0xDF)    /* turn off RX/TX                                              */
#define SFLUSHRX    (0xDD)    /* flush receive FIFO                                          */
#define SFLUSHTX    (0xDE)    /* flush transmit FIFO                                         */
#define SACK        (0xD6)    /* send ACK frame                                              */
#define SACKPEND    (0xD7)    /* send ACK frame with pending bit set                         */

/* bit of proprietary FCS format that indicates if the CRC is OK */
#define PROPRIETARY_FCS_CRC_OK_BIT  0x80

#define PROPRIETARY_FCS_RSSI(p)                 ((int8)((p)[0]))
#define PROPRIETARY_FCS_CRC_OK(p)               ((p)[1] & PROPRIETARY_FCS_CRC_OK_BIT)
#define PROPRIETARY_FCS_CORRELATION_VALUE(p)    ((p)[1] & ~PROPRIETARY_FCS_CRC_OK_BIT)
#define MAC_RADIO_RECEIVER_SENSITIVITY_DBM      -97 /* dBm */
#define MAC_RADIO_RECEIVER_SATURATION_DBM       10  /* dBm */
/* Maximum value for energy detect */
#define MAC_SPEC_ED_MAX                 0xFF

/* Threshold above receiver sensitivity for minimum energy detect in dBm (see 6.9.7) */
#define MAC_SPEC_ED_MIN_DBM_ABOVE_RECEIVER_SENSITIVITY    10
#define ED_RF_POWER_MIN_DBM   (MAC_RADIO_RECEIVER_SENSITIVITY_DBM + MAC_SPEC_ED_MIN_DBM_ABOVE_RECEIVER_SENSITIVITY)
#define ED_RF_POWER_MAX_DBM   MAC_RADIO_RECEIVER_SATURATION_DBM


/* CSPZ return values from CSP program */
#define CSPZ_CODE_TX_DONE           0
#define CSPZ_CODE_CHANNEL_BUSY      1
#define CSPZ_CODE_TX_ACK_TIME_OUT   2

/* ADCCON1 */
//*****************************************************************************
//
// The following are defines for the SOC_ADC register offsets.
//
//*****************************************************************************
#define SOC_ADC_ADCCON1         0x400D7000  // This register controls the ADC.
#define SOC_ADC_ADCCON2         0x400D7004  // This register controls the ADC.
#define SOC_ADC_ADCCON3         0x400D7008  // This register controls the ADC.
#define SOC_ADC_ADCL            0x400D700C  // This register contains the
                                            // least-significant part of ADC
                                            // conversion result.
#define SOC_ADC_ADCH            0x400D7010  // This register contains the
                                            // most-significant part of ADC
                                            // conversion result.
#define SOC_ADC_RNDL            0x400D7014  // This registers contains
                                            // random-number-generator data;
                                            // low byte.
#define SOC_ADC_RNDH            0x400D7018  // This register contains
                                            // random-number-generator data;
                                            // high byte.
#define SOC_ADC_CMPCTL          0x400D7024  // Analog comparator control and
                                            // status register.


/* SFR registers */
#define T2CSPCFG                     HWREG(RFCORE_SFR_MTCSPCFG)
#define T2CTRL                       HWREG(RFCORE_SFR_MTCTRL)
#define T2IRQM                       HWREG(RFCORE_SFR_MTIRQM)
#define T2IRQF                       HWREG(RFCORE_SFR_MTIRQF)
#define T2MSEL                       HWREG(RFCORE_SFR_MTMSEL)
#define T2M0                         HWREG(RFCORE_SFR_MTM0)
#define T2M1                         HWREG(RFCORE_SFR_MTM1)
#define T2MOVF2                      HWREG(RFCORE_SFR_MTMOVF2)
#define T2MOVF1                      HWREG(RFCORE_SFR_MTMOVF1)
#define T2MOVF0                      HWREG(RFCORE_SFR_MTMOVF0)
#define RFD                          HWREG(RFCORE_SFR_RFDATA)
#define RFERRF                       HWREG(RFCORE_SFR_RFERRF)
#define RFIRQF1                      HWREG(RFCORE_SFR_RFIRQF1)
#define RFIRQF0                      HWREG(RFCORE_SFR_RFIRQF0)
#define RFST                         HWREG(RFCORE_SFR_RFST)


/* SMWDT registers */
#define WDCTL                        HWREG(SMWDTHROSC_WDCTL)
#define ST0                          HWREG(SMWDTHROSC_ST0)
#define ST1                          HWREG(SMWDTHROSC_ST1)
#define ST2                          HWREG(SMWDTHROSC_ST2)
#define ST3                          HWREG(SMWDTHROSC_ST3)
#define STLOAD                       HWREG(SMWDTHROSC_STLOAD)
#define STCC                         HWREG(SMWDTHROSC_STCC)
#define STCS                         HWREG(SMWDTHROSC_STCS)
#define STV0                         HWREG(SMWDTHROSC_STCV0)
#define STV1                         HWREG(SMWDTHROSC_STCV1)
#define STV2                         HWREG(SMWDTHROSC_STCV2)
#define STV3                         HWREG(SMWDTHROSC_STCV3)
#define HSOSCCAL                     HWREG(SMWDTHROSC_HSOSCCAL)
#define RCOSCCAL                     HWREG(SMWDTHROSC_RCOSCCAL)


/* XREG registers */
/* Address information for RX Control */
#define FRMFILT0                     HWREG(RFCORE_XREG_FRMFILT0)
#define FRMFILT1                     HWREG(RFCORE_XREG_FRMFILT1)
#define SRCMATCH                     HWREG(RFCORE_XREG_SRCMATCH)
#define SRCSHORTEN0                  HWREG(RFCORE_XREG_SRCSHORTEN0)
#define SRCSHORTEN1                  HWREG(RFCORE_XREG_SRCSHORTEN1)
#define SRCSHORTEN2                  HWREG(RFCORE_XREG_SRCSHORTEN2)
#define SRCEXTEN0                    HWREG(RFCORE_XREG_SRCEXTEN0)
#define SRCEXTEN1                    HWREG(RFCORE_XREG_SRCEXTEN1)
#define SRCEXTEN2                    HWREG(RFCORE_XREG_SRCEXTEN2)

/* Radio Control */
#define FRMCTRL0                     HWREG(RFCORE_XREG_FRMCTRL0)
#define FRMCTRL1                     HWREG(RFCORE_XREG_FRMCTRL1)
#define RXENABLE                     HWREG(RFCORE_XREG_RXENABLE)
#define RXMASKSET                    HWREG(RFCORE_XREG_RXMASKSET)
#define RXMASKCLR                    HWREG(RFCORE_XREG_RXMASKCLR)
#define FREQTUNE                     HWREG(RFCORE_XREG_FREQTUNE)
#define FREQCTRL                     HWREG(RFCORE_XREG_FREQCTRL)
#define TXPOWER                      HWREG(RFCORE_XREG_TXPOWER)
#define TXCTRL                       HWREG(RFCORE_XREG_TXCTRL)
#define FSMSTAT0                     HWREG(RFCORE_XREG_FSMSTAT0)
#define FSMSTAT1                     HWREG(RFCORE_XREG_FSMSTAT1)
#define FIFOPCTRL                    HWREG(RFCORE_XREG_FIFOPCTRL)
#define FSMCTRL                      HWREG(RFCORE_XREG_FSMCTRL)
#define CCACTRL0                     HWREG(RFCORE_XREG_CCACTRL0)
#define CCACTRL1                     HWREG(RFCORE_XREG_CCACTRL1)
#define RSSI                         HWREG(RFCORE_XREG_RSSI)
#define RSSISTAT                     HWREG(RFCORE_XREG_RSSISTAT)
#define RXFIRST                      HWREG(RFCORE_XREG_RXFIRST)
#define RXFIFOCNT                    HWREG(RFCORE_XREG_RXFIFOCNT)
#define TXFIFOCNT                    HWREG(RFCORE_XREG_TXFIFOCNT)
#define RXFIRST_PTR                  HWREG(RFCORE_XREG_RXFIRST_PTR)
#define RXLAST_PTR                   HWREG(RFCORE_XREG_RXLAST_PTR)
#define RXP1_PTR                     HWREG(RFCORE_XREG_RXP1_PTR)
#define RXP2_PTR                     HWREG(RFCORE_XREG_RXP2_PTR)
#define TXFIRST_PTR                  HWREG(RFCORE_XREG_TXFIRST_PTR)
#define TXLAST_PTR                   HWREG(RFCORE_XREG_TXLAST_PTR)

/* Interrupt Controller Registers */
#define RFIRQM0                      HWREG(RFCORE_XREG_RFIRQM0)
#define RFIRQM1                      HWREG(RFCORE_XREG_RFIRQM1)
#define RFERRM                       HWREG(RFCORE_XREG_RFERRM)
#define D18_SPARE_OPAMPMC            HWREG(RFCORE_XREG_D18_SPARE_OPAMPMC)

/* Random Number Generator */
#define RFRND                        HWREG(RFCORE_XREG_RFRND)

/* Analog and Digital Radio Test And Tuning */
#define MDMCTRL0                     HWREG(RFCORE_XREG_MDMCTRL0)
#define MDMCTRL1                     HWREG(RFCORE_XREG_MDMCTRL1)
#define FREQEST                      HWREG(RFCORE_XREG_FREQEST)
#define RXCTRL                       HWREG(RFCORE_XREG_RXCTRL)
#define FSCTRL                       HWREG(RFCORE_XREG_FSCTRL)
#define FSCAL0                       HWREG(RFCORE_XREG_FSCAL0)
#define FSCAL1                       HWREG(RFCORE_XREG_FSCAL1)
#define FSCAL2                       HWREG(RFCORE_XREG_FSCAL2)
#define FSCAL3                       HWREG(RFCORE_XREG_FSCAL3)
#define AGCCTRL0                     HWREG(RFCORE_XREG_AGCCTRL0)
#define AGCCTRL1                     HWREG(RFCORE_XREG_AGCCTRL1)
#define AGCCTRL2                     HWREG(RFCORE_XREG_AGCCTRL2)
#define AGCCTRL3                     HWREG(RFCORE_XREG_AGCCTRL3)
#define ADCTEST0                     HWREG(RFCORE_XREG_ADCTEST0)
#define ADCTEST1                     HWREG(RFCORE_XREG_ADCTEST1)
#define ADCTEST2                     HWREG(RFCORE_XREG_ADCTEST2)
#define MDMTEST0                     HWREG(RFCORE_XREG_MDMTEST0)
#define MDMTEST1                     HWREG(RFCORE_XREG_MDMTEST1)
#define DACTEST0                     HWREG(RFCORE_XREG_DACTEST0)
#define DACTEST1                     HWREG(RFCORE_XREG_DACTEST1)
#define DACTEST2                     HWREG(RFCORE_XREG_DACTEST2)
#define ATEST                        HWREG(RFCORE_XREG_ATEST)
#define PTEST0                       HWREG(RFCORE_XREG_PTEST0)
#define PTEST1                       HWREG(RFCORE_XREG_PTEST1)

/*CSP */
#define CSPPROG0                     HWREG(RFCORE_XREG_CSPPROG0)
#define CSPPROG1                     HWREG(RFCORE_XREG_CSPPROG1)
#define CSPPROG2                     HWREG(RFCORE_XREG_CSPPROG2)
#define CSPPROG3                     HWREG(RFCORE_XREG_CSPPROG3)
#define CSPPROG4                     HWREG(RFCORE_XREG_CSPPROG4)
#define CSPPROG5                     HWREG(RFCORE_XREG_CSPPROG5)
#define CSPPROG6                     HWREG(RFCORE_XREG_CSPPROG6)
#define CSPPROG7                     HWREG(RFCORE_XREG_CSPPROG7)
#define CSPPROG8                     HWREG(RFCORE_XREG_CSPPROG8)
#define CSPPROG9                     HWREG(RFCORE_XREG_CSPPROG9)
#define CSPPROG10                    HWREG(RFCORE_XREG_CSPPROG10)
#define CSPPROG11                    HWREG(RFCORE_XREG_CSPPROG11)
#define CSPPROG12                    HWREG(RFCORE_XREG_CSPPROG12)
#define CSPPROG13                    HWREG(RFCORE_XREG_CSPPROG13)
#define CSPPROG14                    HWREG(RFCORE_XREG_CSPPROG14)
#define CSPPROG15                    HWREG(RFCORE_XREG_CSPPROG15)
#define CSPPROG16                    HWREG(RFCORE_XREG_CSPPROG16)
#define CSPPROG17                    HWREG(RFCORE_XREG_CSPPROG17)
#define CSPPROG18                    HWREG(RFCORE_XREG_CSPPROG18)
#define CSPPROG19                    HWREG(RFCORE_XREG_CSPPROG19)
#define CSPPROG20                    HWREG(RFCORE_XREG_CSPPROG20)
#define CSPPROG21                    HWREG(RFCORE_XREG_CSPPROG21)
#define CSPPROG22                    HWREG(RFCORE_XREG_CSPPROG22)
#define CSPPROG23                    HWREG(RFCORE_XREG_CSPPROG23)
#define CSPCTRL                      HWREG(RFCORE_XREG_CSPCTRL)
#define CSPSTAT                      HWREG(RFCORE_XREG_CSPSTAT)
#define CSPX                         HWREG(RFCORE_XREG_CSPX)
#define CSPY                         HWREG(RFCORE_XREG_CSPY)
#define CSPZ                         HWREG(RFCORE_XREG_CSPZ)
#define CSPT                         HWREG(RFCORE_XREG_CSPT)

#define RFC_DUTY_CYCLE               HWREG(RFCORE_XREG_RFC_DUTY_CYCLE)
#define RFC_OBS_CTRL0                HWREG(RFCORE_XREG_RFC_OBS_CTRL0)
#define RFC_OBS_CTRL1                HWREG(RFCORE_XREG_RFC_OBS_CTRL1)
#define RFC_OBS_CTRL2                HWREG(RFCORE_XREG_RFC_OBS_CTRL2)

#define ACOMPGAINI                   HWREG(RFCORE_XREG_ACOMPGAINI)
#define ACOMPGAINQ                   HWREG(RFCORE_XREG_ACOMPGAINQ)
#define ACOMPDCI                     HWREG(RFCORE_XREG_ACOMPDCI)
#define ACOMPDCQ                     HWREG(RFCORE_XREG_ACOMPDCQ)
#define ACOMPQS                      HWREG(RFCORE_XREG_ACOMPQS)
#define ACOMPCFG                     HWREG(RFCORE_XREG_ACOMPCFG)
#define ACOMPCALIL                   HWREG(RFCORE_XREG_ACOMPCALIL)
#define ACOMPCALIH                   HWREG(RFCORE_XREG_ACOMPCALIH)
#define ACOMPCALQL                   HWREG(RFCORE_XREG_ACOMPCALQL)
#define ACOMPCALQH                   HWREG(RFCORE_XREG_ACOMPCALQH)
#define TXFILTCFG                    HWREG(RFCORE_XREG_TXFILTCFG)
#define TXMIXCFG                     HWREG(RFCORE_XREG_TXMIXCFG)
#define TXMIXSTAT                    HWREG(RFCORE_XREG_TXMIXSTAT)


/* FFSM registers */
/* Source Address Matching Control */
#define SRCRESMASK0                  HWREG(RFCORE_FFSM_SRCRESMASK0)
#define SRCRESMASK1                  HWREG(RFCORE_FFSM_SRCRESMASK1)
#define SRCRESMASK2                  HWREG(RFCORE_FFSM_SRCRESMASK2)
#define SRCRESINDEX                  HWREG(RFCORE_FFSM_SRCRESINDEX)
#define SRCEXTPENDEN0                HWREG(RFCORE_FFSM_SRCEXTPENDEN0)
#define SRCEXTPENDEN1                HWREG(RFCORE_FFSM_SRCEXTPENDEN1)
#define SRCEXTPENDEN2                HWREG(RFCORE_FFSM_SRCEXTPENDEN2)
#define SRCSHORTPENDEN0              HWREG(RFCORE_FFSM_SRCSHORTPENDEN0)
#define SRCSHORTPENDEN1              HWREG(RFCORE_FFSM_SRCSHORTPENDEN1)
#define SRCSHORTPENDEN2              HWREG(RFCORE_FFSM_SRCSHORTPENDEN2)

/* Local Address Information */
#define EXT_ADDR0                    HWREG(RFCORE_FFSM_EXT_ADDR0)
#define EXT_ADDR1                    HWREG(RFCORE_FFSM_EXT_ADDR1)
#define EXT_ADDR2                    HWREG(RFCORE_FFSM_EXT_ADDR2)
#define EXT_ADDR3                    HWREG(RFCORE_FFSM_EXT_ADDR3)
#define EXT_ADDR4                    HWREG(RFCORE_FFSM_EXT_ADDR4)
#define EXT_ADDR5                    HWREG(RFCORE_FFSM_EXT_ADDR5)
#define EXT_ADDR6                    HWREG(RFCORE_FFSM_EXT_ADDR6)
#define EXT_ADDR7                    HWREG(RFCORE_FFSM_EXT_ADDR7)
#define PAN_ID0                      HWREG(RFCORE_FFSM_PAN_ID0)
#define PAN_ID1                      HWREG(RFCORE_FFSM_PAN_ID1)
#define SHORT_ADDR0                  HWREG(RFCORE_FFSM_SHORT_ADDR0)
#define SHORT_ADDR1                  HWREG(RFCORE_FFSM_SHORT_ADDR1)


/* CC Test registers*/
#define OBSSEL0                      HWREG(CCTEST_OBSSEL0)
#define OBSSEL1                      HWREG(CCTEST_OBSSEL1)
#define OBSSEL2                      HWREG(CCTEST_OBSSEL2)
#define OBSSEL3                      HWREG(CCTEST_OBSSEL3)
#define OBSSEL4                      HWREG(CCTEST_OBSSEL4)
#define OBSSEL5                      HWREG(CCTEST_OBSSEL5)
#define OBSSEL6                      HWREG(CCTEST_OBSSEL6)
#define OBSSEL7                      HWREG(CCTEST_OBSSEL7)
#define RCTRL1                        BV(3)
#define RCTRL0                        BV(2)
#define RCTRL_BITS                    (RCTRL1 | RCTRL0)
#define RCTRL_CLOCK_LFSR              RCTRL0

#define ADCCON1                       HWREG(SOC_ADC_ADCCON1)
#define RNDH                          HWREG(SOC_ADC_RNDH)



/* macTxFrame() parameter values for txType */
#define MAC_TX_TYPE_SLOTTED_CSMA            0x00
#define MAC_TX_TYPE_UNSLOTTED_CSMA          0x01
#define MAC_TX_TYPE_SLOTTED                 0x02
#define MAC_TX_TYPE_GREEN_POWER             0x03

#define maxCsmaBackoffs 3


//Macros do Auto Ack...usado pelo protocolo AMAC
#define EXT_ADDR_INDEX_SIZE                  2
#define MAC_SRCMATCH_ENABLE_BITMAP_LEN       3
#define MAC_SRCMATCH_SHORT_ENTRY_SIZE        4
#define MAC_SRCMATCH_SHORT_MAX_NUM_ENTRIES   24
#define MAC_SRCMATCH_EXT_MAX_NUM_ENTRIES     12
#define Z_EXTADDR_LEN                         8
#define MAC_SRCMATCH_EXT_ENTRY_SIZE          Z_EXTADDR_LEN


#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

#define MAC_RADIO_GET_SRC_SHORTPENDEN(p)              macMemReadRam( &SRCSHORTPENDEN0, (p), 3 )
#define MAC_RADIO_GET_SRC_EXTENPEND(p)                macMemReadRam( &SRCEXTPENDEN0, (p), 3 )
#define MAC_RADIO_GET_SRC_SHORTEN(p)                  macMemReadRam( &SRCSHORTEN0, (p), 3 )
#define MAC_RADIO_GET_SRC_EXTEN(p)                    macMemReadRam( &SRCEXTEN0, (p), 3 )


#define MAC_RADIO_SRC_MATCH_GET_SHORTADDR_EN()        macSrcMatchGetShortAddrEnableBit()
#define MAC_RADIO_SRC_MATCH_GET_EXTADDR_EN()          macSrcMatchGetExtAddrEnableBit()
#define MAC_RADIO_SRC_MATCH_GET_SHORTADDR_PENDEN()    macSrcMatchGetShortAddrPendEnBit()
#define MAC_RADIO_SRC_MATCH_GET_EXTADDR_PENDEN()      macSrcMatchGetExtAddrPendEnBit()

#define MAC_RADIO_SRC_MATCH_SET_SHORTEN(x)            osal_buffer_uint24( (uint8_t*)&SRCSHORTEN0, (x) )
#define MAC_RADIO_SRC_MATCH_SET_EXTEN(x)              osal_buffer_uint24( (uint8_t*)&SRCEXTEN0, (x) )

#define MAC_RADIO_SRC_MATCH_SET_SHORTPENDEN(p)        macMemWriteRam( &SRCSHORTPENDEN0, (p), 3 )
#define MAC_RADIO_SRC_MATCH_SET_EXTPENDEN(p)          macMemWriteRam( &SRCEXTPENDEN0, (p), 3 )

#define MAC_RADIO_SRC_MATCH_TABLE_WRITE(offset, p, len)    macMemWriteRam( (macRam_t *)(SRC_ADDR_TABLE + (offset*4)), (p), (len) )
#define MAC_RADIO_SRC_MATCH_TABLE_READ(offset, p, len)     macMemReadRam( (macRam_t *)(SRC_ADDR_TABLE + (offset*4)), (p), (len))

/* takes a byte out of a uint32 : var - uint32,  ByteNum - byte to take out (0 - 3) */
#define BREAK_UINT32( var, ByteNum )                 (uint8_t)((uint32_t)(((var) >>((ByteNum) * 8)) & 0x00FF))

#define BUILD_UINT32(Byte0, Byte1, Byte2, Byte3) \
          ((uint32_t)((uint32_t)((Byte0) & 0x00FF) \
          + ((uint32_t)((Byte1) & 0x00FF) << 8) \
          + ((uint32_t)((Byte2) & 0x00FF) << 16) \
          + ((uint32_t)((Byte3) & 0x00FF) << 24)))

#define MAC_INVALID_PARAMETER       0xE8  /* The API function parameter is out of range */
#define MAC_DUPLICATED_ENTRY        0x1E  /* For internal use only - A duplicated entry is added to the source matching table */
#define MAC_NO_RESOURCES            0x1A  /* The operation could not be completed because no
                                             memory resources were available */

#define MAC_SRCMATCH_INVALID_INDEX  0xFF



/* Extended address length */
#define SADDR_EXT_LEN   8

/* Address modes */
#define SADDR_MODE_NONE       0       /* Address not present */
#define SADDR_MODE_SHORT      2       /* Short address */
#define SADDR_MODE_EXT        3       /* Extended address */

#define SRC_ADDR_TABLE                      0x40088400

/* HWREG switched to C99 format uint32_t. Hence macRam_t
 * uses c99 format to remove compiler warning
 */
typedef volatile uint32_t macRam_t;

/* Extended address */
typedef uint8_t sAddrExt_t[SADDR_EXT_LEN];

/* Combined short/extended device address */
typedef struct
{
  union
  {
    uint16_t     shortAddr;    /* Short address */
    sAddrExt_t  extAddr;      /* Extended address */
  } addr;
  uint8_t         addrMode;     /* Address mode */
} sAddr_t;

//-------------------------------------
// Prototypes
void macCspTxGoCsma(void);
void txCsmaPrep(void);
void radio_csma_init(void);
void macCspTxStopIsr(void);
void macTxChannelBusyCallback(void);
void radiotimer_isr_private(void);
void csmavarsinit(void);
uint8_t updatecsmanb(void);
void macCspTxStopIsr(void);
void macCspTxIntIsr(uint32_t *ptimestamp,uint16_t *ptimestamp2);
void macTxChannelBusyCallback(void);
uint32_t macMcuOverflowCapture(void);
uint16_t macMcuTimerCapture(void);
void macBackoffTimerCompareIsr(void);
uint8_t macRadioRandomByte(void);
void MAC_SrcMatchEnable (void);
uint8_t MAC_SrcMatchAddEntry ( uint16_t srcaddr,uint16_t dstaddr, uint16_t panID );
uint8_t macRadioComputeLQI(int8_t rssiDbm, uint8_t corr);
uint8_t radioComputeED(int8_t rssiDbm);
#endif
