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
#include "hw_rfcore_ffsm.h"
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
uint8_t macSrcMatchCheckSrcAddr ( sAddr_t *addr, uint16_t panID  );
uint8_t macSrcMatchFindEmptyEntry( uint8_t macSrcMatchAddrMode );
void macMemWriteRam(uint32_t * pRam, uint8_t * pData, uint8_t len);
void macSrcMatchSetPendEnBit( uint8_t index, uint8_t macSrcMatchAddrMode );
void macSrcMatchSetEnableBit(uint8_t index,bool option, uint8_t macSrcMatchAddrMode);
uint32_t macSrcMatchGetShortAddrPendEnBit( void );
uint32_t macSrcMatchGetExtAddrPendEnBit( void );
uint32_t macSrcMatchGetShortAddrEnableBit( void );
uint32_t macSrcMatchGetExtAddrEnableBit( void );
bool macSrcMatchCheckEnableBit( uint8_t index, uint32_t enable);

uint8_t *osal_buffer_uint24( uint8_t *buf, uint32_t val );
uint32_t osal_build_uint32( uint8_t *swapped, uint8_t len );
uint8_t osal_memcmp( const void *src1, const void *src2, unsigned int len );
void macMemReadRam(macRam_t * pRam, uint8_t * pData, uint8_t len);
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


void rxackPrep(void)
{
   /* clear the currently loaded CSP, this generates a stop interrupt which must be cleared */
   HWREG(RFCORE_SFR_RFST) = ISSTOP;
   HWREG(RFCORE_SFR_RFST) = ISCLEAR;

   IntPendClear(INT_RFCORERTX);
   RFIRQF1 = ~IRQ_CSP_STOP;

   IntPendClear(INT_RFCORERTX);
   RFIRQF1 =~IRQ_CSP_MANINT;

  HWREG(RFCORE_SFR_RFST) = SFLUSHRX;
  HWREG(RFCORE_SFR_RFST) = SACK;

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


#if 1 //(TESTE_AUTOACK == 1)
/*********************************************************************
 * @fn          MAC_SrcMatchEnable
 *
 * @brief      Enabled AUTOPEND and source address matching.
 *             This function shall be not be called from
 *             ISR. It is not thread safe.
 *
 * @param     none
 *
 * @return     none
 */
void MAC_SrcMatchEnable (void)
{
  /* Turn on Frame Filter (TIMAC enables frame filter by default), TBD */
	//HWREG(RFCORE_XREG_FRMFILT0) |= (RFCORE_XREG_FRMFILT0_FRAME_FILTER_EN | RFCORE_XREG_FRMFILT0_PAN_COORDINATOR | RFCORE_XREG_FRMFILT0_MAX_FRAME_VERSION_S);
	HWREG(RFCORE_XREG_FRMFILT0) = (RFCORE_XREG_FRMFILT0_FRAME_FILTER_EN_M | RFCORE_XREG_FRMFILT0_MAX_FRAME_VERSION_M);

	HWREG(RFCORE_XREG_FRMFILT1) = RFCORE_XREG_FRMFILT1_ACCEPT_FT_1_DATA_M | RFCORE_XREG_FRMFILT1_ACCEPT_FT_3_MAC_CMD_M;
  /* Turn on Auto ACK (TIMAC turn on Auto ACK by default), TBD */
  //HWREG(RFCORE_XREG_FRMCTRL0) |= RFCORE_XREG_FRMCTRL0_AUTOACK;

  /* Turn on Autopend: set SRCMATCH.AUTOPEND and SRCMATCH.SRC_MATCH_EN */
	HWREG(RFCORE_XREG_SRCMATCH) |= RFCORE_XREG_SRCMATCH_SRC_MATCH_EN_M;

  /* Set SRCMATCH.AUTOPEND */
	//HWREG(RFCORE_XREG_SRCMATCH) |= RFCORE_XREG_SRCMATCH_AUTOPEND;

  /* AUTOPEND function requires that the received frame is a DATA REQUEST MAC command frame
    //MAC_RADIO_TURN_ON_AUTOPEND_DATAREQ_ONLY(); */
	//	HWREG(RFCORE_XREG_SRCMATCH) |= RFCORE_XREG_SRCMATCH_PEND_DATAREQ_ONLY;
}

void MAC_AckEnable (void)
{
  /* Turn on Frame Filter (TIMAC enables frame filter by default), TBD */
	//HWREG(RFCORE_XREG_FRMFILT0) |= (RFCORE_XREG_FRMFILT0_FRAME_FILTER_EN | RFCORE_XREG_FRMFILT0_PAN_COORDINATOR | RFCORE_XREG_FRMFILT0_MAX_FRAME_VERSION_S);
	HWREG(RFCORE_XREG_FRMFILT0) = (RFCORE_XREG_FRMFILT0_FRAME_FILTER_EN_M | RFCORE_XREG_FRMFILT0_MAX_FRAME_VERSION_M);

	HWREG(RFCORE_XREG_FRMFILT1) = RFCORE_XREG_FRMFILT1_ACCEPT_FT_1_DATA_M | RFCORE_XREG_FRMFILT1_ACCEPT_FT_3_MAC_CMD_M;
  /* Turn on Auto ACK (TIMAC turn on Auto ACK by default), TBD */
  //HWREG(RFCORE_XREG_FRMCTRL0) |= RFCORE_XREG_FRMCTRL0_AUTOACK;

  /* Turn on Autopend: set SRCMATCH.AUTOPEND and SRCMATCH.SRC_MATCH_EN */
	HWREG(RFCORE_XREG_SRCMATCH) |= RFCORE_XREG_SRCMATCH_SRC_MATCH_EN_M;

  /* Set SRCMATCH.AUTOPEND */
	//HWREG(RFCORE_XREG_SRCMATCH) |= RFCORE_XREG_SRCMATCH_AUTOPEND;

  /* AUTOPEND function requires that the received frame is a DATA REQUEST MAC command frame
    //MAC_RADIO_TURN_ON_AUTOPEND_DATAREQ_ONLY(); */
	//	HWREG(RFCORE_XREG_SRCMATCH) |= RFCORE_XREG_SRCMATCH_PEND_DATAREQ_ONLY;
}

/*********************************************************************
 * @fn          MAC_SrcMatchAddEntry
 *
 * @brief       Add a short or extended address to source address table. This
 *              function shall be not be called from ISR. It is not thread safe.
 *
 * @param       srcaddr - is the source of the packet to be filter...not my address..
 *
 * @param       dstaddr - is the destination in the packet...in general is my address or broadcast
 *
 * @param       panID - the device PAN ID. It is only used when the addr is
 *                      using short address

 * @return      MAC_SUCCESS or MAC_NO_RESOURCES (source address table full)
 *              or MAC_DUPLICATED_ENTRY (the entry added is duplicated),
 *              or MAC_INVALID_PARAMETER if the input parameters are invalid.
 */

uint8_t MAC_SrcMatchAddEntry ( uint16_t srcaddr,uint16_t dstaddr, uint16_t panID )
{
	uint8_t index;
	uint8_t entry[MAC_SRCMATCH_SHORT_ENTRY_SIZE];
//	uint8_t buf[MAC_SRCMATCH_ENABLE_BITMAP_LEN];
	uint32_t *pucAux;
//	uint32_t enable;

	//PAN destination
	HWREG(RFCORE_FFSM_PAN_ID0) = LO_UINT16( panID );
	HWREG(RFCORE_FFSM_PAN_ID1) = HI_UINT16( panID );

	//destination Address
	HWREG(RFCORE_FFSM_SHORT_ADDR1) = LO_UINT16( dstaddr );
	HWREG(RFCORE_FFSM_SHORT_ADDR0) = HI_UINT16( dstaddr );

	//Habilita filtro do source address (Posicao zero)

	//#define MAC_RADIO_SRC_MATCH_TABLE_WRITE(offset, p, len)    macMemWriteRam( (macRam_t *)(SRC_ADDR_TABLE + (offset*4)), (p), (len) )
	index = 0;
	 /* Little Endian for the radio RAM */
	entry[0] = LO_UINT16( panID );
	entry[1] = HI_UINT16( panID );
	entry[2] = LO_UINT16( srcaddr );
	entry[3] = HI_UINT16( srcaddr );

	pucAux = (uint32_t *)(SRC_ADDR_TABLE + (index*4));
	macMemWriteRam (pucAux, entry, MAC_SRCMATCH_SHORT_ENTRY_SIZE);

	HWREG(RFCORE_XREG_SRCSHORTEN0) = 0x01;

  /* Set the Autopend enable bits */
  //macSrcMatchSetPendEnBit( index, addr->addrMode );
 /*
	index = 0;
	//enable = macSrcMatchGetShortAddrPendEnBit();
	MAC_RADIO_GET_SRC_SHORTPENDEN( buf );
	enable=osal_build_uint32( buf, MAC_SRCMATCH_ENABLE_BITMAP_LEN );

	enable |= ( (uint32_t)0x01 << index );
	osal_buffer_uint24( buf, enable );
	MAC_RADIO_SRC_MATCH_SET_SHORTPENDEN( buf );
*/
	//HWREG(RFCORE_FFSM_SRCSHORTPENDEN0) = 0x01;

  /* Set the Src Match enable bits */
  //macSrcMatchSetEnableBit( index, TRUE, addr->addrMode);
	//enable = macSrcMatchGetShortAddrEnableBit();
/*
	MAC_RADIO_GET_SRC_SHORTEN( buf );
	enable = osal_build_uint32( buf, MAC_SRCMATCH_ENABLE_BITMAP_LEN );
	enable |= ( (uint32_t )0x01 << index );
	osal_buffer_uint24( buf, enable );
	MAC_RADIO_SRC_MATCH_SET_SHORTEN( buf );
*/


  return 0;
}

/*********************************************************************
 * @fn          macSrcMatchSetPendEnBit
 *
 * @brief       Set the enable bit in the source address table
 *
 * @param       index - index of the entry in the source address table
 * @param       macSrcMatchAddrMode - Address Mode for the entry. Valid values
 *              are SADDR_MODE_SHORT or SADDR_MODE_EXT
 * @return      none
 */
void macSrcMatchSetPendEnBit( uint8_t index, uint8_t macSrcMatchAddrMode )
{
  uint32_t enable;
  uint8_t buf[MAC_SRCMATCH_ENABLE_BITMAP_LEN];

  if( macSrcMatchAddrMode == SADDR_MODE_SHORT )
  {
    enable = macSrcMatchGetShortAddrPendEnBit();
    enable |= ( (uint32_t)0x01 << index );
    osal_buffer_uint24( buf, enable );
   // MAC_RADIO_SRC_MATCH_SET_SHORTPENDEN( buf );
  }
  else
  {
    enable = macSrcMatchGetExtAddrPendEnBit();
    enable |= ( (uint32_t)0x01 << ( index * EXT_ADDR_INDEX_SIZE ) );
    enable |= ( (uint32_t)0x01 << ( ( index * EXT_ADDR_INDEX_SIZE ) + 1 ) );
    osal_buffer_uint24( buf, enable );
    //MAC_RADIO_SRC_MATCH_SET_EXTPENDEN( buf );
  }
}


/*********************************************************************
 * @fn          macSrcMatchSetEnableBit
 *
 * @brief       Set or clear the enable bit in the SRCMATCH EN register
 *
 * @param       index - index of the entry in the source address table
 * @param       option - true (set the enable bit), or false (clear the
 *              enable bit)
 * @param       macSrcMatchAddrMode - Address Mode for the entry. Valid values
 *              are SADDR_MODE_SHORT or SADDR_MODE_EXT
 *
 * @return      none
 */
#if 0
void macSrcMatchSetEnableBit(uint8_t index,bool option, uint8_t macSrcMatchAddrMode)
{
  uint32_t enable;
  uint8_t buf[MAC_SRCMATCH_ENABLE_BITMAP_LEN];

  if( option == TRUE )
  {
    if( macSrcMatchAddrMode == SADDR_MODE_SHORT )
    {
      enable = macSrcMatchGetShortAddrEnableBit();
      enable |= ( (uint32_t )0x01 << index );
      osal_buffer_uint24( buf, enable );
      MAC_RADIO_SRC_MATCH_SET_SHORTEN( buf );
    }
    else
    {
      enable = MAC_RADIO_SRC_MATCH_GET_EXTADDR_EN();
      enable |= ( (uint32_t)0x01 << ( index *  EXT_ADDR_INDEX_SIZE) );
      osal_buffer_uint24( buf, enable );
      MAC_RADIO_SRC_MATCH_SET_EXTEN( buf );
    }
  }
  else
  {
    if( macSrcMatchAddrMode == SADDR_MODE_SHORT )
    {
      enable = MAC_RADIO_SRC_MATCH_GET_SHORTADDR_EN();
      enable &= ~( (uint32_t)0x01 << index );
      osal_buffer_uint24( buf, enable );
      MAC_RADIO_SRC_MATCH_SET_SHORTEN( buf );
    }
    else
    {
      enable = MAC_RADIO_SRC_MATCH_GET_EXTADDR_EN();
      enable &= ~( (uint32_t)0x01 << ( index * EXT_ADDR_INDEX_SIZE ) );
      osal_buffer_uint24( buf, enable );
      MAC_RADIO_SRC_MATCH_SET_EXTEN( buf );
    }
  }
}
#endif

/*********************************************************************
 * @fn          macSrcMatchGetShortAddrPendEnBit
 *
 * @brief       Return the SRCMATCH ShortAddr Pend enable bitmap
 *
 * @param       none
 *
 * @return      uint24 - 24 bits bitmap
 */
uint32_t macSrcMatchGetShortAddrPendEnBit( void )
{
  uint8_t buf[MAC_SRCMATCH_ENABLE_BITMAP_LEN];

  MAC_RADIO_GET_SRC_SHORTPENDEN( buf );

  return osal_build_uint32( buf, MAC_SRCMATCH_ENABLE_BITMAP_LEN );
}


/*********************************************************************
 * @fn          macSrcMatchGetExtAddrPendEnBit
 *
 * @brief       Return the SRCMATCH Extended Address Pend enable bitmap
 *
 * @param       none
 *
 * @return      uint24 - 24 bits bitmap
 */
uint32_t macSrcMatchGetExtAddrPendEnBit( void )
{
  uint8_t buf[MAC_SRCMATCH_ENABLE_BITMAP_LEN];

  MAC_RADIO_GET_SRC_EXTENPEND( buf );

  return osal_build_uint32( buf, MAC_SRCMATCH_ENABLE_BITMAP_LEN );
}

/*********************************************************************
 * @fn         macSrcMatchCheckSrcAddr
 *
 * @brief      Check if a short or extended address is in the source address table.
 *             This function shall not be called from ISR. It is not thread safe.
 *
 * @param      addr - a pointer to sAddr_t which contains addrMode
 *                    and a union of a short 16-bit MAC address or an extended
 *                    64-bit MAC address to be checked in the source address table.
 * @param      panID - the device PAN ID. It is only used when the addr is
 *                     using short address

 * @return     uint8 - index of the entry in the table. Return
 *                     MAC_SRCMATCH_INVALID_INDEX (0xFF) if address not found.
 */
uint8_t macSrcMatchCheckSrcAddr ( sAddr_t *addr, uint16_t panID  )
{
  uint8_t index;
  uint8_t *pAddr;
  uint8_t entrySize;
  uint8_t indexUsed;
  uint8_t indexSize;
  uint8_t entry[MAC_SRCMATCH_SHORT_ENTRY_SIZE];
  uint8_t ramEntry[MAC_SRCMATCH_EXT_ENTRY_SIZE];
  uint32_t enable;

  /*
   Currently, shadow memory is not supported to optimize SPI traffic.
  */
  if( addr->addrMode ==  SADDR_MODE_SHORT )
  {
    entry[0] = LO_UINT16( panID );  /* Little Endian for the radio RAM */
    entry[1] = HI_UINT16( panID );
    entry[2] = LO_UINT16( addr->addr.shortAddr );
    entry[3] = HI_UINT16( addr->addr.shortAddr );
    pAddr = entry;
    entrySize = MAC_SRCMATCH_SHORT_ENTRY_SIZE;
    indexSize = 1;
    enable = MAC_RADIO_SRC_MATCH_GET_SHORTADDR_EN();
  }
  else
  {
    pAddr = addr->addr.extAddr;
    entrySize = MAC_SRCMATCH_EXT_ENTRY_SIZE;
    indexSize = 2;
    enable = MAC_RADIO_SRC_MATCH_GET_EXTADDR_EN();
  }

  for( index = 0; index < MAC_SRCMATCH_SHORT_MAX_NUM_ENTRIES; index += indexSize )
  {
    /* Check if the entry is enabled */
    if( macSrcMatchCheckEnableBit( index, enable ) == FALSE )
    {
      continue;
    }

    indexUsed = index / indexSize;

    /* Compare the short address or extended address */
    MAC_RADIO_SRC_MATCH_TABLE_READ( ( indexUsed * entrySize ), ramEntry, entrySize );

    if( osal_memcmp( pAddr, ramEntry, entrySize ) == TRUE )
    {
      /* Match found */
      return indexUsed;
    }
  }

  return MAC_SRCMATCH_INVALID_INDEX;
}
/*********************************************************************
 * @fn          macSrcMatchFindEmptyEntry
 *
 * @brief       return index of the first empty entry found
 *
 * @param       macSrcMatchAddrMode - Address Mode for the entry. Valid values
 *              are SADDR_MODE_SHORT or SADDR_MODE_EXT
 *
 * @return      uint8 - return index of the first empty entry found
 */

uint8_t macSrcMatchFindEmptyEntry( uint8_t macSrcMatchAddrMode )
{
  uint8_t  index;
  uint32_t shortAddrEnable = MAC_RADIO_SRC_MATCH_GET_SHORTADDR_EN();
  uint32_t extAddrEnable = MAC_RADIO_SRC_MATCH_GET_EXTADDR_EN();
  uint32_t enable = shortAddrEnable | extAddrEnable;

  if( macSrcMatchAddrMode == SADDR_MODE_SHORT )
   {
     for( index = 0; index < MAC_SRCMATCH_SHORT_MAX_NUM_ENTRIES; index ++ )
     {
       /* Both 2n bit of extAddrEnable and corresponding bit of shortAddrEnable must be clear
        * in order to assume that the entry location for a short address is not used.
        */
       if( (extAddrEnable & ((uint32_t)0x01 << ((index/2)*2))) == 0 &&
           (shortAddrEnable & ((uint32_t)0x01 << index)) == 0 )
       {
         return index;
       }
     }
   }
   else
   {
     for( index = 0; index < MAC_SRCMATCH_EXT_MAX_NUM_ENTRIES; index++ )
     {
       /* Both 2n bit of extAddrEnable and
        * 2n bit and 2n+1 bit of shortAddrEnable must be clear in order
        * to assume that the entry location for an extended address
        * is not used.
        */
       if( (enable & ((uint32_t)0x03 << (index*2))) == 0 )
       {
         return index;
       }
     }
   }
  return index;
}

/*********************************************************************
 * @fn      osal_buffer_uint24
 *
 * @brief
 *
 *   Buffer an uint24 value - LSB first. Note that type uint24 is
 *   typedef to uint32 in comdef.h
 *
 * @param   buf - buffer
 * @param   val - uint24 value
 *
 * @return  pointer to end of destination buffer
 */
uint8_t * osal_buffer_uint24( uint8_t *buf, uint32_t val )
{
  *buf++ = BREAK_UINT32( val, 0 );
  *buf++ = BREAK_UINT32( val, 1 );
  *buf++ = BREAK_UINT32( val, 2 );

  return buf;
}

/*********************************************************************
 * @fn      osal_build_uint32
 *
 * @brief
 *
 *   Build a uint32 out of sequential bytes.
 *
 * @param   swapped - sequential bytes
 * @param   len - number of bytes in the uint8 array
 *
 * @return  uint32
 */
uint32_t osal_build_uint32( uint8_t *swapped, uint8_t len )
{
  if ( len == 2 )
    return ( BUILD_UINT32( swapped[0], swapped[1], 0L, 0L ) );
  else if ( len == 3 )
    return ( BUILD_UINT32( swapped[0], swapped[1], swapped[2], 0L ) );
  else if ( len == 4 )
    return ( BUILD_UINT32( swapped[0], swapped[1], swapped[2], swapped[3] ) );
  else
    return ( (uint32_t)swapped[0] );
}

/*********************************************************************
 * @fn      osal_memcmp
 *
 * @brief
 *
 *   Generic memory compare.
 *
 * @param   src1 - source 1 addrexx
 * @param   src2 - source 2 address
 * @param   len - number of bytes to compare
 *
 * @return  TRUE - same, FALSE - different
 */
uint8_t osal_memcmp( const void *src1, const void *src2, unsigned int len )
{
  const uint8_t *pSrc1;
  const uint8_t *pSrc2;

  pSrc1 = src1;
  pSrc2 = src2;

  while ( len-- )
  {
    if( *pSrc1++ != *pSrc2++ )
      return FALSE;
  }
  return TRUE;
}


/*********************************************************************
 * @fn          macSrcMatchGetShortAddrEnableBit
 *
 * @brief       Return the SRCMATCH ShortAddr enable bitmap
 *
 * @param       none
 *
 * @return      uint24 - 24 bits bitmap
 */
uint32_t macSrcMatchGetShortAddrEnableBit( void )
{
  uint8_t buf[MAC_SRCMATCH_ENABLE_BITMAP_LEN];

  MAC_RADIO_GET_SRC_SHORTEN( buf );

  return osal_build_uint32( buf, MAC_SRCMATCH_ENABLE_BITMAP_LEN );
}

/*********************************************************************
 * @fn          macSrcMatchGetExtAddrEnBit
 *
 * @brief       Return the SRCMATCH ExtAddr enable bitmap
 *
 * @param       none
 *
 * @return      uint24 - 24 bits bitmap
 */
uint32_t macSrcMatchGetExtAddrEnableBit( void )
{
  uint8_t buf[MAC_SRCMATCH_ENABLE_BITMAP_LEN];

  MAC_RADIO_GET_SRC_EXTEN( buf );

  return osal_build_uint32( buf, MAC_SRCMATCH_ENABLE_BITMAP_LEN );
}

/*********************************************************************
 * @fn          macSrcMatchCheckEnableBit
 *
 * @brief       Check the enable bit in the source address table
 *
 * @param       index - index of the entry in the source address table
 * @param       enable - enable register should be read before passing
 *              it here
 *
 * @return      TRUE or FALSE
 */
bool macSrcMatchCheckEnableBit( uint8_t index, uint32_t enable)
{
  if( enable & ((uint32_t)0x01 << index ))
  {
    return TRUE;
  }

  return FALSE;
}


/**************************************************************************************************
 * @fn          macMemWriteRam
 *
 * @brief       Write multiple bytes to RAM.
 *
 * @param       pRam  - pointer to RAM to be written to
 * @param       pData - pointer to data to write
 * @param       len   - number of bytes to write
 *
 * @return      none
 **************************************************************************************************
 */
void macMemWriteRam(uint32_t * pRam, uint8_t * pData, uint8_t len)
{
  while (len)
  {
    len--;
    *pRam = (unsigned long)(*pData);
    pRam++;
    pData++;
  }
}

#endif //#if AUTO_ACK


/**************************************************************************************************
 * @fn          macMemReadRam
 *
 * @brief       Read multiple bytes from RAM.
 *
 * @param       pRam  - pointer to RAM to be read from
 * @param       pData - pointer to location to store read data
 * @param       len   - number of bytes to read
 *
 * @return      none
 **************************************************************************************************
 */
void macMemReadRam(macRam_t * pRam, uint8_t * pData, uint8_t len)
{
  while (len)
  {
    len--;
    *pData = (unsigned char)(*pRam & 0xFF);
    pRam++;
    pData++;
  }
}

/**************************************************************************************************
 * @fn          macRadioComputeLQI
 *
 * @brief       Compute link quality indication.
 *
 * @param       rssi - raw RSSI value from radio hardware
 *              corr - correlation value from radio hardware
 *
 * @return      link quality indicator value
 **************************************************************************************************
 */
uint8_t macRadioComputeLQI(int8_t rssiDbm, uint8_t corr)
{
	  uint8_t ed;
	  int8_t min, max;
	  (void) corr; /* suppress compiler warning of unused parameter */

  /*
   *  Note : Currently the LQI value is simply the energy detect measurement.
   *         A more accurate value could be derived by using the correlation
   *         value along with the RSSI value.
   */
	  /*
	   *  Keep RF power between minimum and maximum values.
	   *  This min/max range is derived from datasheet and specification.
	   */

	  min = ED_RF_POWER_MIN_DBM;
	  max = ED_RF_POWER_MAX_DBM;

	  if (rssiDbm < min)
	  {
		rssiDbm = min;
	  }
	  else if (rssiDbm > max)
	  {
		rssiDbm = max;
	  }

	  /*
	   *  Create energy detect measurement by normalizing and scaling RF power level.
	   *
	   *  Note : The division operation below is designed for maximum accuracy and
	   *         best granularity.  This is done by grouping the math operations to
	   *         compute the entire numerator before doing any division.
	   */
	  ed = (MAC_SPEC_ED_MAX * (rssiDbm - ED_RF_POWER_MIN_DBM)) / (ED_RF_POWER_MAX_DBM - ED_RF_POWER_MIN_DBM);

	  return(ed);
}

#endif
