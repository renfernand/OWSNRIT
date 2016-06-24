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


//=========================== defines =========================================

uint32_t   macChipVersion = 0;
uint8_t  errorcount;
uint8_t  backofftimercount=0;
extern uint8_t macRxOnFlag;
extern uint8_t macRxEnableFlags;

#if  (DEBUG_CSMA == 1)
radio_csma_vars_t radio_csma_vars;
#endif

extern uint8_t macRxOutgoingAckFlag;
extern radio_vars_t radio_vars;

uint8_t    macTxActive;
uint8_t    macTxType;

uint8_t    macTxGpInterframeDelay;

/* Function pointer for the 16 byte random seed callback */
typedef void (*macRNGFcn_t )(uint8_t * seed);

/* Function pointer for the random seed callback */
//static macRNGFcn_t pRandomSeedCB = NULL;
uint8_t randomSeed[MAC_RANDOM_SEED_LEN];

//=========================== prototypes =======================================


/**************************************************************************************************
 * @fn          macBackoffTimerCompareIsr
 *
 * @brief       Interrupt service routine that fires when the backoff count is equal
 *              to the trigger count.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macBackoffTimerCompareIsr(void)
{

  //macBackoffTimerTriggerCallback(); nao existe esta funcao...ou esta na lib
}
/**************************************************************************************************
 * @fn          macMcuTimerCapture
 *
 * @brief       Returns the last timer capture.  This capture should have occurred at the
 *              receive time of the last frame (the last time SFD transitioned to active).
 *
 * @param       none
 *
 * @return      last capture of hardware timer (full 16-bit value)
 **************************************************************************************************
 */
uint16_t macMcuTimerCapture(void)
{
  uint16_t         timerCapture;

  //HAL_ENTER_CRITICAL_SECTION(s);
  //MAC_MCU_T2_ACCESS_CAPTURE_VALUE();
  T2MSEL = T2M_T2_CAP;
  timerCapture = T2M1 << 8;
  timerCapture |= T2M0;
  //HAL_EXIT_CRITICAL_SECTION(s);

  return (timerCapture);
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
#if ( ENABLE_CSMA_CA == 1)

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
#if (CSMA_802154e_ALGORITM == 0)
void txCsmaPrep(void)
{
  //-----------------------------
  //macCspTxPrepCsmaUnslotted();

  //MACTIMER macTxCsmaBackoffDelay = macRadioRandomByte() & ((1 << macTxBe) - 1);
  radio_csma_vars.macTxCsmaBackoffDelay = macRadioRandomByte() & ((1 << radio_csma_vars.macTxBe) - 1);

  // cspPrepForTxProgram();
  /* set CSP EVENT1 to T2 CMP1 - Setou o bit do MAC Timer habilitando para gerar evento1 CSP Compare1  */
   //MACTIMER - MAC_MCU_CONFIG_CSP_EVENT1();
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

  /*----------------------------------------------------------------------
   *  Load CSP program :  Unslotted CSMA transmit
   *  Wait for X number of backoffs, then wait for intra-backoff count
   *  to reach value set for WEVENT1.
   */
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
  HWREG(RFCORE_SFR_RFST) = (uint32_t) WAITX;
  //HWREG(RFCORE_SFR_RFST) = (uint32_t) WEVENT1;
  //HWREG(RFCORE_SFR_RFST) = (uint32_t) SSTOP;

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
#else

void txCsmaPrep(void)
{
   radio_csma_vars.macTxCsmaBackoffDelay = macRadioRandomByte() & ((1 << radio_csma_vars.macTxBe) - 1);

    /* set CSP EVENT1 to T2 CMP1 - Setou o bit do MAC Timer habilitando para gerar evento1 CSP Compare1  */
   HWREG(RFCORE_SFR_MTCSPCFG) = 1UL;

   /* set up parameters for CSP transmit program */
   HWREG(RFCORE_XREG_CSPZ) = CSPZ_CODE_CHANNEL_BUSY;

   /* clear the currently loaded CSP, this generates a stop interrupt which must be cleared */
   HWREG(RFCORE_SFR_RFST) = ISSTOP;
   HWREG(RFCORE_SFR_RFST) = ISCLEAR;

   IntPendClear(INT_RFCORERTX);
   RFIRQF1 = ~IRQ_CSP_STOP;

   IntPendClear(INT_RFCORERTX);
   RFIRQF1 =~IRQ_CSP_MANINT;

  /*----------------------------------------------------------------------
   *  Load CSP program :  Unslotted CSMA transmit
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
#if 1
  HWREG(RFCORE_SFR_RFST) = (uint32_t) SKIP(1, C_CCA_IS_VALID);
  HWREG(RFCORE_SFR_RFST) = (uint32_t) SSTOP;
#endif

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
 // HWREG(RFCORE_SFR_RFST) = (uint32_t) INT;

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

#endif
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
  //DISABLE_INTERRUPTS();

  //MAC_MCU_T2_ACCESS_COUNT_VALUE();
  HWREG(RFCORE_SFR_MTMSEL) = T2M_T2TIM;

  temp0 = HWREG(RFCORE_SFR_MTM0);
  temp1 = HWREG(RFCORE_SFR_MTM1);

  /* MAC timer bug on the cc2530 PG1 made it impossible to use
   * compare = 0 for both the timer and the overflow counter.
   */
  if ((macChipVersion <= REV_B) && (temp0 == 0) && (temp1 == 0))
  {
    temp0++;
  }

  HWREG(RFCORE_SFR_MTMSEL) = T2M_T2_CMP1;  //seleciona MT_cmp1 (timer compare 1)
  HWREG(RFCORE_SFR_MTM0) = temp0;
  HWREG(RFCORE_SFR_MTM1) = temp1;

  //ENABLE_INTERRUPTS();

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
   HWREG(RFCORE_XREG_CSPT) = (uint32_t ) 0xFF;  //nao decrementa...
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
   HWREG(RFCORE_XREG_RFIRQM1)   |= IM_CSP_STOP;
   HWREG(RFCORE_XREG_RFIRQM1)   |= IM_CSP_MANINT;
#if (CSMA_802154e_ALGORITM == 0)
   HWREG(RFCORE_XREG_RFIRQM1)   |= IM_CSP_WAIT;
#endif
  /*
   *  Turn on the receiver if it is not already on.  Receiver must be 'on' for at
   *  least one backoff before performing clear channel assessment (CCA).
   */
  //macRxOn();
  DISABLE_INTERRUPTS();
  if (!macRxOnFlag)
  {
    macRxOnFlag = 1;
    HWREG(RFCORE_SFR_RFST) = ISRXON;  //MAC_RADIO_RX_ON();
    //MAC_DEBUG_TURN_ON_RX_LED();
  }
  ENABLE_INTERRUPTS();


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

void csmavarsinit(void){
	//macTxType = MAC_TX_TYPE_UNSLOTTED_CSMA;
	macChipVersion = CHIPID >> 16;

	radio_csma_vars.maxBe = 5;
	radio_csma_vars.minBe = 3;
	radio_csma_vars.macTxCsmaBackoffDelay = 3;

}

#endif
