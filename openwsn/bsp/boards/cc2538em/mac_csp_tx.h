#ifndef mac_csp_tx_h
#define mac_csp_tx_h


typedef struct {
	uint8_t maxCsmaBackoffs;
	uint8_t macTxCsmaBackoffDelay;
	uint8_t maxBe;
	uint8_t macTxBe ;
	uint8_t nb;
} radio_csma_vars_t;

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

#define HAL_CPU_CLOCK_MHZ     32UL       //TODO!!! ESTA CERTO ISSO



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

/* RFIRQF1 */
#define IRQ_TXACKDONE   BV(0)
#define IRQ_TXDONE      BV(1)
#define IRQ_CSP_MANINT  BV(3)
#define IRQ_CSP_STOP    BV(4)

#define T2M_T2TIM       T2M_SEL(0UL)
#define T2M_T2_CAP      T2M_SEL(1UL)
#define T2M_T2_PER      T2M_SEL(2UL)
#define T2M_T2_CMP1     T2M_SEL(3UL)
#define T2M_T2_CMP2     T2M_SEL(4UL)


/* ----- values specific to 2450 MHz PHY ----- */

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



// Prototypes
void macCspTxGoCsma(void);
void txCsmaPrep(void);
void radio_csma_init(void);
void macCspTxStopIsr(void);
void macTxChannelBusyCallback(void);

#endif
