/**
\brief CC2538-specific definition of the "radio" bsp module.
  aqui somente vou tirar algo do codigo TIMAC
  existe duas funcoes...RF04 e RF05...o codigo abaixo é baseado na RF05

\author Renato Fernandes <renfernand@gmail.com>, feb 2015.
*/

#include "board.h"
#include "radio.h"
#include "mac_csp_tx.h"
#include "leds.h"
#include "stdio.h"
#include "string.h"
#include "radiotimer.h"
#include "debugpins.h"
#include "hw_rfcore_sfr.h"
#include "interrupt.h"
#include "sys_ctrl.h"
#include "hw_ints.h"
#include "hw_types.h"
#include "sys_ctrl.h"
#include "hw_rfcore_xreg.h"
#include "hw_rfcore_sfr.h"
#include "cc2538rf.h"
#include "hw_ana_regs.h"

#if ( ENABLE_CSMA_CA == 1)
//=========================== defines =========================================

//=========================== variables =======================================
extern radio_vars_t radio_vars;
radio_csma_vars_t radio_csma_vars;

uint8_t    macTxActive;
uint8_t    macTxType;

uint8_t    macTxGpInterframeDelay;

uint32_t   macChipVersion = 0;



/* Function pointer for the 16 byte random seed callback */
typedef void (*macRNGFcn_t )(uint8_t * seed);

/* Function pointer for the random seed callback */
//static macRNGFcn_t pRandomSeedCB = NULL;
static uint8_t randomSeed[MAC_RANDOM_SEED_LEN];

//=========================== prototypes =======================================
//void macTxFrame(void);


/* TODO!!!!! AQUI TEM DE SER NAO VOLATIL ?????
 * Store the 16 byte random seed in NV
 */
//void SSP_StoreRandomSeedNV( uint8 *pSeed ){
//}


void radio_csma_init(void) {

	uint8_t i,j;
	uint16_t rndSeed;
	uint8_t rndByte = 0;

	  //macTxType = MAC_TX_TYPE_UNSLOTTED_CSMA;

	  macChipVersion = CHIPID >> 16;
	  radio_csma_vars.maxCsmaBackoffs = 3;
	  radio_csma_vars.maxBe = 3;
	  radio_csma_vars.macTxCsmaBackoffDelay = 3;

#if 0
	  // JA ESTA SENDO HABILITADO O timer (DE FORMA DIFERENTE) NO RADIOTIMER.C
	  /*-------------------------------------------------------------------------------
	   *  Initialize MAC timer.
	   */

	  /* set timer rollover */
	  //HAL_ENTER_CRITICAL_SECTION(s);
	  //MAC_MCU_T2_ACCESS_PERIOD_VALUE();
	  HWREG(RFCORE_SFR_MTMSEL) = T2M_T2_PER;

	  HWREG(RFCORE_SFR_MTM0) = MAC_RADIO_TIMER_TICKS_PER_BACKOFF() & 0xFFUL;
	  HWREG(RFCORE_SFR_MTM1) = MAC_RADIO_TIMER_TICKS_PER_BACKOFF() >> 8UL;
	  //HAL_EXIT_CRITICAL_SECTION(s);

	  /* start timer */
	  /* MAC_RADIO_TIMER_WAKE_UP();
                           st( HAL_CLOCK_STABLE(); \
                                                          T2CTRL |= (TIMER2_RUN | TIMER2_SYNC); \
                                                          while(!(T2CTRL & TIMER2_STATE)); )  */
	  while ( !((HWREG(SYS_CTRL_CLOCK_STA)) & (SYS_CTRL_CLOCK_STA_XOSC_STB))) ;
	  HWREG(RFCORE_SFR_MTCTRL) |= (TIMER2_RUN | TIMER2_SYNC);
	  while(!(HWREG(RFCORE_SFR_MTCTRL) & TIMER2_STATE));

	  /* Enable latch mode and T2 SYNC start. OSAL timer is based on MAC timer.
	   * The SYNC start msut be on when POWER_SAVING is on for this design to work.
	   */
	  HWREG(RFCORE_SFR_MTCTRL) |= (LATCH_MODE | TIMER2_SYNC);

	  /* enable timer interrupts */
	  IntEnable(INT_MACTIMR);
#endif

	 /*----------------------------------------------------------------------------------------------
	  *  Initialize random seed value.
	  */

	  /*
	   *  Set radio for infinite reception.  Once radio reaches this state,
	   *  it will stay in receive mode regardless RF activity.
	   */
	  HWREG(RFCORE_XREG_FRMCTRL0) = FRMCTRL0_RESET_VALUE | RX_MODE_INFINITE_RECEPTION;

	  /* turn on the receiver */
	  //macRxOn();
	  MAC_RADIO_RX_ON;

	  /*
	   *  Wait for radio to reach infinite reception state by checking RSSI valid flag.
	   *  Once it does, the least significant bit of ADTSTH should be pretty random.
	   */
	  while (!(HWREG(RFCORE_XREG_RSSISTAT) & 0x01));

	  /* put 16 random bits into the seed value */

	  /* Read MAC_RANDOM_SEED_LEN*8 random bits and store them in flash for
	   * future use in random key generation for CBKE key establishment
	   */
	  {
		rndSeed = 0;

		for(i=0; i<16; i++)
		{
		  /* use most random bit of analog to digital receive conversion to populate the random seed */
		  rndSeed = (rndSeed << 1) | (HWREG(RFCORE_XREG_RFRND) & 0x01);
		}

		/*
		 *  The seed value must not be zero or 0x0380 (0x8003 in the polynomial).  If it is, the psuedo
		 *  random sequence won’t be random.  There is an extremely small chance this seed could randomly
		 *  be zero or 0x0380.  The following check makes sure this does not happen.
		 */
		if (rndSeed == 0x0000 || rndSeed == 0x0380)
		{
		  rndSeed = 0xBABE; /* completely arbitrary "random" value */
		}

		/*
		 *  Two writes to RNDL will set the random seed.  A write to RNDL copies current contents
		 *  of RNDL to RNDH before writing new the value to RNDL.
		 */
		HWREG(SOC_ADC_RNDL) = rndSeed & 0xFF;
		HWREG(SOC_ADC_RNDL) = rndSeed >> 8;
	  }

		for(i = 0; i < MAC_RANDOM_SEED_LEN; i++)
		{
		  for(j = 0; j < 8; j++)
		  {
			/* use most random bit of analog to digital receive conversion to
			   populate the random seed */
			rndByte = (rndByte << 1) | (HWREG(RFCORE_XREG_RFRND) & 0x01);
		  }
		  randomSeed[i] = rndByte;

		}

		//TODO!!!! AQUI ELE SALVA EM AREA NV
		//pRandomSeedCB( randomSeed );

	  /* turn off the receiver */
	  //macRxOff();
	  MAC_RADIO_RXTX_OFF;

	  /* take receiver out of infinite reception mode; set back to normal operation */
	  HWREG(RFCORE_XREG_FRMCTRL0) = FRMCTRL0_RESET_VALUE | RX_MODE_NORMAL_OPERATION;

	  /* Turn on autoack ??????*/
	  //MAC_RADIO_TURN_ON_AUTO_ACK();
	  HWREG(RFCORE_XREG_FRMCTRL0) |= AUTOACK;

	  /* Initialize SRCEXTPENDEN and SRCSHORTPENDEN to zeros */
/*
	  //MAC_RADIO_SRC_MATCH_INIT_EXTPENDEN();
	  HWREG(RFCORE_FFSM_SRCEXTPENDEN0) = 0;
	  HWREG(RFCORE_FFSM_SRCEXTPENDEN1) = 0;
	  HWREG(RFCORE_FFSM_SRCEXTPENDEN2) = 0;

	  //MAC_RADIO_SRC_MATCH_INIT_SHORTPENDEN();
	  HWREG(RFCORE_FFSM_SRCSHORTPENDEN0) = 0;
	  HWREG(RFCORE_FFSM_SRCSHORTPENDEN1) = 0;
	  HWREG(RFCORE_FFSM_SRCSHORTPENDEN2) = 0;
*/

}

#if 0
void macTxFrame(void){

	  /*-------------------------------------------------------------------------------
	   *  Prepare for transmit. unslotted csma_ca
	   */

		txCsmaPrep();

	  /*-------------------------------------------------------------------------------
	   *  Load transmit FIFO unless this is a retransmit.  No need to write
	   *  the FIFO again in that case.
	   *  NESTE CASO CONSIDERO QUE JA FOI CARREGADO O FRAME NA FIFO!!!!!!!!!!!!!
	   */

	  /*-------------------------------------------------------------------------------
	   *  If not receiving, start the transmit.  If receive is active
	   *  queue up the transmit.
	   *
	   *  Critical sections around the state change prevents any sort of race condition
	   *  with  macTxStartQueuedFrame().  This guarantees function txGo() will only be
	   *  called once.
	   *  AQUI NAO SEI SE SERIA O CASO DE TER UMA RECEPCAO PENDENTE...VOU SOMENTE ENVIAR HABILITAR O ENVIO!!!!
	   *

	  {
		halIntState_t  s;

		HAL_ENTER_CRITICAL_SECTION(s);
		if (!macRxActive && !macRxOutgoingAckFlag)
		{
		  macTxActive = MAC_TX_ACTIVE_GO;
		  HAL_EXIT_CRITICAL_SECTION(s);
		  txCsmaGo();
		}
		else
		{
		  macTxActive = MAC_TX_ACTIVE_QUEUED;
		  HAL_EXIT_CRITICAL_SECTION(s);
		}
	  }*/

	  macCspTxGoCsma();
}
#endif

/*=================================================================================================
 * @fn          cspPrepForTxProgram
 *
 * @brief       Prepare and initialize for transmit CSP program.
 *              Call *before* loading the CSP program!
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
static void cspPrepForTxProgram(void)
{
  //MAC_ASSERT(!(RFIRQM1 & IM_CSP_STOP)); /* already an active CSP program */

  /* set CSP EVENT1 to T2 CMP1 */
  //MAC_MCU_CONFIG_CSP_EVENT1();
  HWREG(RFCORE_SFR_MTCSPCFG) = 1UL;

  /* set up parameters for CSP transmit program */
  HWREG(RFCORE_XREG_CSPZ) = CSPZ_CODE_CHANNEL_BUSY;

  /* clear the currently loaded CSP, this generates a stop interrupt which must be cleared */
  //CSP_STOP_AND_CLEAR_PROGRAM();
  HWREG(RFCORE_SFR_RFST) = ISSTOP;
  HWREG(RFCORE_SFR_RFST) = ISCLEAR;

  //MAC_MCU_CSP_STOP_CLEAR_INTERRUPT();
  //MAC_MCU_WRITE_RFIRQF1(~IRQ_CSP_STOP);
  //HAL_CRITICAL_STATEMENT(
  IntPendClear(INT_RFCORERTX);
  RFIRQF1 = ~IRQ_CSP_STOP;

  //MAC_MCU_CSP_INT_CLEAR_INTERRUPT();
  //MAC_MCU_WRITE_RFIRQF1(~IRQ_CSP_MANINT);
  //HAL_CRITICAL_STATEMENT(
  IntPendClear(INT_RFCORERTX);
  RFIRQF1 =~IRQ_CSP_MANINT;
}


/**************************************************************************************************
 * @fn          macCspTxPrepCsmaUnslotted
 *
 * @brief       Prepare CSP for "Unslotted CSMA" transmit.  Load CSP program and set CSP parameters.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macCspTxPrepCsmaUnslotted(void)
{
  cspPrepForTxProgram();

  /*----------------------------------------------------------------------
   *  Load CSP program :  Unslotted CSMA transmit
   */

  /*
   *  Wait for X number of backoffs, then wait for intra-backoff count
   *  to reach value set for WEVENT1.
   */
  HWREG(RFCORE_SFR_RFST) = (uint32_t) WAITX;
  HWREG(RFCORE_SFR_RFST) = (uint32_t) WEVENT1;

  /* wait until RSSI is valid */
  HWREG(RFCORE_SFR_RFST) = (uint32_t) WHILE(C_RSSI_IS_INVALID);

  /* Note that the CCA signal is updated four clock cycles (system clock)
   * after the RSSI_VALID signal has been set.
   */
  HWREG(RFCORE_SFR_RFST) = SNOP;
  HWREG(RFCORE_SFR_RFST) = SNOP;
  HWREG(RFCORE_SFR_RFST) = SNOP;
  HWREG(RFCORE_SFR_RFST) = SNOP;

  /* sample CCA, if it fails exit from here, CSPZ indicates result */
  HWREG(RFCORE_SFR_RFST) = (uint32_t) SKIP(1, C_CCA_IS_VALID);
  HWREG(RFCORE_SFR_RFST) = (uint32_t) SSTOP;

  /* CSMA has passed so transmit (actual frame starts one backoff from when strobe is sent) */
  HWREG(RFCORE_SFR_RFST) = (uint32_t) STXON;

  /*
   *  Wait for the start of frame delimiter of the transmitted frame.  If SFD happens to
   *  already be active when STXON is strobed, it gets forced low.  How long this takes
   *  though, is not certain.  For bulletproof operation, the first step is to wait
   *  until SFD is inactive (which should be very fast if even necessary), and then wait
   *  for it to go active.
   */
  HWREG(RFCORE_SFR_RFST) = (uint32_t) WHILE(C_SFD_IS_ACTIVE);
  HWREG(RFCORE_SFR_RFST) = (uint32_t) WHILE(C_SFD_IS_INACTIVE);

  /*
   *  Record the timestamp.  The INT instruction causes an interrupt to fire.
   *  The ISR for this interrupt records the timestamp (which was just captured
   *  when SFD went high).
   */
  HWREG(RFCORE_SFR_RFST) = (uint32_t) INT;

  /*
   *  Wait for SFD to go inactive which is the end of transmit.  Decrement CSPZ to indicate
   *  the transmit was successful.
   */
  HWREG(RFCORE_SFR_RFST) = (uint32_t) WHILE(C_SFD_IS_ACTIVE);
  HWREG(RFCORE_SFR_RFST) = (uint32_t) DECZ;

  /*
   * CC2530 requires SSTOP to generate CSP_STOP interrupt.
   */
  HWREG(RFCORE_SFR_RFST) = (uint32_t) SSTOP;
}

/**************************************************************************************************
 * @fn          macRadioRandomByte
 *
 * @brief       Return a random byte derived from previously set random seed.
 * @brief       Returns a random byte using a special hardware feature that generates new
 *              random values based on the truly random seed set earlier.
 *
 * @param       none
 *
 * @return      a random byte
 **************************************************************************************************
 */

uint8_t macRadioRandomByte(void)
{
  /* clock the random generator to get a new random value */
	HWREG(SOC_ADC_ADCCON1) = (HWREG(SOC_ADC_ADCCON1) & ~RCTRL_BITS) | RCTRL_CLOCK_LFSR;

  /* return new randomized value from hardware */
  return(HWREG(SOC_ADC_RNDH));
}


/*=================================================================================================
 * @fn          txCsmaPrep
 *
 * @brief       Prepare/initialize for a CSMA transmit.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
void txCsmaPrep(void)
{
#if 0
  macTxCsmaBackoffDelay = macRadioRandomByte() & ((1 << macTxBe) - 1);

  macCspTxPrepCsmaUnslotted();
#else

  radio_csma_vars.nb = 0;
  radio_csma_vars.macTxBe = 3;
  radio_csma_vars.macTxCsmaBackoffDelay = macRadioRandomByte() & ((1 << radio_csma_vars.macTxBe) - 1);

  //MAC_ASSERT(!(RFIRQM1 & IM_CSP_STOP)); /* already an active CSP program */

  /* set CSP EVENT1 to T2 CMP1 -   MAC Timer event configuration  */
  //MAC_MCU_CONFIG_CSP_EVENT1();
  HWREG(RFCORE_SFR_MTCSPCFG) = 1UL;

  /* set up parameters for CSP transmit program */
  HWREG(RFCORE_XREG_CSPZ) = CSPZ_CODE_CHANNEL_BUSY;

  /* clear the currently loaded CSP, this generates a stop interrupt which must be cleared */
  //CSP_STOP_AND_CLEAR_PROGRAM();
  HWREG(RFCORE_SFR_RFST) = ISSTOP;
  HWREG(RFCORE_SFR_RFST) = ISCLEAR;

  //MAC_MCU_CSP_STOP_CLEAR_INTERRUPT();
  //MAC_MCU_WRITE_RFIRQF1(~IRQ_CSP_STOP);
  //HAL_CRITICAL_STATEMENT(
  IntPendClear(INT_RFCORERTX);
  HWREG(RFCORE_SFR_RFIRQF1) = ~IRQ_CSP_STOP;

  //MAC_MCU_CSP_INT_CLEAR_INTERRUPT();
  //MAC_MCU_WRITE_RFIRQF1(~IRQ_CSP_MANINT);
  //HAL_CRITICAL_STATEMENT(
  IntPendClear(INT_RFCORERTX);
  HWREG(RFCORE_SFR_RFIRQF1) =~IRQ_CSP_MANINT;

  /*----------------------------------------------------------------------
   *  Load CSP program :  Unslotted CSMA transmit
   */
  /*
   *  Wait for X number of backoffs, then wait for intra-backoff count
   *  to reach value set for WEVENT1.
   */
  HWREG(RFCORE_SFR_RFST) = (uint32_t) WAITX;
  HWREG(RFCORE_SFR_RFST) = (uint32_t) WEVENT1;

  /* wait until RSSI is valid */
  HWREG(RFCORE_SFR_RFST) = (uint32_t) WHILE(C_RSSI_IS_INVALID);

  /* Note that the CCA signal is updated four clock cycles (system clock)
   * after the RSSI_VALID signal has been set.
   */
  HWREG(RFCORE_SFR_RFST) = SNOP;
  HWREG(RFCORE_SFR_RFST) = SNOP;
  HWREG(RFCORE_SFR_RFST) = SNOP;
  HWREG(RFCORE_SFR_RFST) = SNOP;

  /* sample CCA, if it fails exit from here, CSPZ indicates result */
  HWREG(RFCORE_SFR_RFST) = (uint32_t) SKIP(1, C_CCA_IS_VALID);
  HWREG(RFCORE_SFR_RFST) = (uint32_t) SSTOP;

  /* CSMA has passed so transmit (actual frame starts one backoff from when strobe is sent) */
  HWREG(RFCORE_SFR_RFST) = (uint32_t) STXON;

  /*
   *  Wait for the start of frame delimiter of the transmitted frame.  If SFD happens to
   *  already be active when STXON is strobed, it gets forced low.  How long this takes
   *  though, is not certain.  For bulletproof operation, the first step is to wait
   *  until SFD is inactive (which should be very fast if even necessary), and then wait
   *  for it to go active.
   */
  HWREG(RFCORE_SFR_RFST) = (uint32_t) WHILE(C_SFD_IS_ACTIVE);
  HWREG(RFCORE_SFR_RFST) = (uint32_t) WHILE(C_SFD_IS_INACTIVE);

  /*
   *  Record the timestamp.  The INT instruction causes an interrupt to fire.
   *  The ISR for this interrupt records the timestamp (which was just captured
   *  when SFD went high).
   */
  HWREG(RFCORE_SFR_RFST) = (uint32_t) INT;

  /*
   *  Wait for SFD to go inactive which is the end of transmit.  Decrement CSPZ to indicate
   *  the transmit was successful.
   */
  HWREG(RFCORE_SFR_RFST) = (uint32_t) WHILE(C_SFD_IS_ACTIVE);
  HWREG(RFCORE_SFR_RFST) = (uint32_t) DECZ;

  /*
   * CC2530 requires SSTOP to generate CSP_STOP interrupt.
   */
  HWREG(RFCORE_SFR_RFST) = (uint32_t) SSTOP;

#endif

}

/*=================================================================================================
 * @fn          cspWeventSetTriggerNow
 *
 * @brief       sets the WEVENT1 trigger point at the current timer count
 *
 * @param       none
 *
 * @return      symbols
 *=================================================================================================
 */

static void cspWeventSetTriggerNow(void)
{
  uint8_t  temp0, temp1;

  /* Clear the compare interrupt flag for debugging purpose. */
  //CSP_WEVENT_CLEAR_TRIGGER();
  HWREG(RFCORE_SFR_MTIRQF) = ~TIMER2_COMPARE1F;

  /* copy current timer count to compare */
  DISABLE_INTERRUPTS();

  //MAC_MCU_T2_ACCESS_COUNT_VALUE();
  T2MSEL = T2M_T2TIM;

  temp0 = T2M0;
  temp1 = T2M1;

  /* MAC timer bug on the cc2530 PG1 made it impossible to use
   * compare = 0 for both the timer and the overflow counter.
   */
  if ((macChipVersion <= REV_B) && (temp0 == 0) && (temp1 == 0))
  {
    temp0++;
  }

  T2MSEL = T2M_T2_CMP1;
  T2M0 = temp0;
  T2M1 = temp1;

  ENABLE_INTERRUPTS();
}

/**************************************************************************************************
 * @fn          macCspTxGoCsma
 *
 * @brief       Run previously loaded CSP program for CSMA transmit.  Handles either
 *              slotted or unslotted CSMA transmits.  When CSP program has finished,
 *              an interrupt occurs and macCspTxStopIsr() is called.  This ISR will in
 *              turn call macTxDoneCallback().
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macCspTxGoCsma(void)
{
  /*
   *  Set CSPX with the countdown time of the CSMA delay.
   */
   HWREG(RFCORE_XREG_CSPX) = (uint32_t ) radio_csma_vars.macTxCsmaBackoffDelay;

  /*
   *  Set WEVENT to trigger at the current value of the timer.  This allows
   *  unslotted CSMA to transmit just a little bit sooner.
   */
   cspWeventSetTriggerNow();

  /*
   *  Enable interrupt that fires when CSP program stops.
   *  Also enable interrupt that fires when INT instruction
   *  is executed.
   */
  //MAC_MCU_CSP_STOP_ENABLE_INTERRUPT();
   HWREG(RFCORE_XREG_RFIRQM1) |= IM_CSP_STOP;

  //MAC_MCU_CSP_INT_ENABLE_INTERRUPT();
   HWREG(RFCORE_XREG_RFIRQM1) |= IM_CSP_MANINT;

  /*
   *  Turn on the receiver if it is not already on.  Receiver must be 'on' for at
   *  least one backoff before performing clear channel assessment (CCA).
   */
  //macRxOn();
   radio_rxNow();

  /* start the CSP program */
  HWREG(RFCORE_SFR_RFST) = ISSTART;
}


/**************************************************************************************************
 * @fn          macTxChannelBusyCallback
 *
 * @brief       This callback is executed if a CSMA transmit was attempted but the channel
 *              was busy.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
#if 0
void macTxChannelBusyCallback(void)
{
  //MAC_ASSERT((macTxType == MAC_TX_TYPE_SLOTTED_CSMA) || (macTxType == MAC_TX_TYPE_UNSLOTTED_CSMA));

  /* turn off receiver if allowed */
  //macTxActive = MAC_TX_ACTIVE_CHANNEL_BUSY;
  //macRxOffRequest();
  radio_off();

  /*  clear channel assement failed, follow through with CSMA algorithm */
  nb++;
  if (nb > maxCsmaBackoffs)
  {
      // TODO!!!! AQUI DEVERIA SINALIZAR QUE HOUVE ERRO!!!!
      radio_vars.state = RADIOSTATE_TXRX_DONE;
      if (radio_vars.endFrame_cb!=NULL) {
         // call the callback
         radio_vars.endFrame_cb(capturedTime);
         // kick the OS
         return;
      } else {
         while(1);
      }
  }
  else
  {
	  radio_csma_vars.macTxBe = MIN(radio_csma_vars.macTxBe+1, radio_csma_vars.maxBe);
    radio_txNow();
  }
}
#endif


#endif
