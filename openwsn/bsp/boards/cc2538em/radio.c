/**
\brief CC2538-specific definition of the "radio" bsp module.

\author Xavier Vilajosana <xvilajosana@eecs.berkeley.edu>, Sept 2013.
*/

#include "board.h"
#include "radio.h"
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
#include "IEEE802154E.h"
#if ( ENABLE_CSMA_CA == 1)
#include "hw_sys_ctrl.h"
#include "mac_csp_tx.h"
#include "aes.h"
#endif
//=========================== defines =========================================
/* Bit Masks for the last byte in the RX FIFO */
#define CRC_BIT_MASK 0x80
#define LQI_BIT_MASK 0x7F
/* RSSI Offset */
#define RSSI_OFFSET 73
#define CHECKSUM_LEN 2

//=========================== variables =======================================
radio_vars_t radio_vars;
uint8_t macRxOnFlag;
uint8_t macRxEnableFlags;
uint8_t macRxOutgoingAckFlag;
uint8_t dbgTimeMesureCount=0;



#if (ENABLE_CSMA_CA == 1)
uint8_t randomSeed[MAC_RANDOM_SEED_LEN];
extern radio_csma_vars_t radio_csma_vars;

#endif
//=========================== prototypes ======================================

port_INLINE void     enable_radio_interrupts(void);
port_INLINE void     disable_radio_interrupts(void);

port_INLINE void     radio_on(void);
port_INLINE void     radio_off(void);

port_INLINE void     radio_error_isr(void);
port_INLINE void     radio_isr_internal(void);
void radio_cleartimerovf(void);
//=========================== public ==========================================

//===== admin

#if (ENABLE_CSMA_CA == 0)
void radio_init() {
   
   // clear variables
   memset(&radio_vars,0,sizeof(radio_vars_t));
   
   // change state
   radio_vars.state          = RADIOSTATE_STOPPED;
   //flush fifos
   CC2538_RF_CSP_ISFLUSHRX();
   CC2538_RF_CSP_ISFLUSHTX();
   
   radio_off();
   
   //disable radio interrupts
   disable_radio_interrupts();
   
   /*
   This CORR_THR value should be changed to 0x14 before attempting RX. Testing has shown that
   too many false frames are received if the reset value is used. Make it more likely to detect
   sync by removing the requirement that both symbols in the SFD must have a correlation value
   above the correlation threshold, and make sync word detection less likely by raising the
   correlation threshold.
   */
   HWREG(RFCORE_XREG_MDMCTRL1)    = 0x14;
   /* tuning adjustments for optimal radio performance; details available in datasheet */
   
   HWREG(RFCORE_XREG_RXCTRL)      = 0x3F;
   /* Adjust current in synthesizer; details available in datasheet. */
   HWREG(RFCORE_XREG_FSCTRL)      = 0x55;
   
     /* Makes sync word detection less likely by requiring two zero symbols before the sync word.
      * details available in datasheet.
      */
   HWREG(RFCORE_XREG_MDMCTRL0)    = 0x85;
   
   /* Adjust current in VCO; details available in datasheet. */
   HWREG(RFCORE_XREG_FSCAL1)      = 0x01;
   /* Adjust target value for AGC control loop; details available in datasheet. */
   HWREG(RFCORE_XREG_AGCCTRL1)    = 0x15;
   
   /* Tune ADC performance, details available in datasheet. */
   HWREG(RFCORE_XREG_ADCTEST0)    = 0x10;
   HWREG(RFCORE_XREG_ADCTEST1)    = 0x0E;
   HWREG(RFCORE_XREG_ADCTEST2)    = 0x03;
   
   //update CCA register to -81db as indicated by manual.. won't be used..
   HWREG(RFCORE_XREG_CCACTRL0)    = 0xF8;
   /*
    * Changes from default values
    * See User Guide, section "Register Settings Update"
    */
   HWREG(RFCORE_XREG_TXFILTCFG)   = 0x09;    /** TX anti-aliasing filter bandwidth */
   HWREG(RFCORE_XREG_AGCCTRL1)    = 0x15;     /** AGC target value */
   HWREG(ANA_REGS_O_IVCTRL)       = 0x0B;        /** Bias currents */
   
   /* disable the CSPT register compare function */
   HWREG(RFCORE_XREG_CSPT)        = 0xFFUL;
   /*
    * Defaults:
    * Auto CRC; Append RSSI, CRC-OK and Corr. Val.; CRC calculation;
    * RX and TX modes with FIFOs
    */
   HWREG(RFCORE_XREG_FRMCTRL0)    = RFCORE_XREG_FRMCTRL0_AUTOCRC;
   
   //poipoi disable frame filtering by now.. sniffer mode.
   HWREG(RFCORE_XREG_FRMFILT0)   &= ~RFCORE_XREG_FRMFILT0_FRAME_FILTER_EN;
   
   /* Disable source address matching and autopend */
   HWREG(RFCORE_XREG_SRCMATCH)    = 0;
   
   /* MAX FIFOP threshold */
   HWREG(RFCORE_XREG_FIFOPCTRL)   = CC2538_RF_MAX_PACKET_LEN;
   
   HWREG(RFCORE_XREG_TXPOWER)     = CC2538_RF_TX_POWER;
   HWREG(RFCORE_XREG_FREQCTRL)    = CC2538_RF_CHANNEL_MIN;
   
   /* Enable RF interrupts  see page 751  */
   // enable_radio_interrupts();
   
   //register interrupt
   IntRegister(INT_RFCORERTX, radio_isr_internal);
   IntRegister(INT_RFCOREERR, radio_error_isr);
   
   IntPrioritySet(INT_RFCORERTX, HAL_INT_PRIOR_MAC);
   IntPrioritySet(INT_RFCOREERR, HAL_INT_PRIOR_MAC);
   
   IntEnable(INT_RFCORERTX);
   
   /* Enable all RF Error interrupts */
   HWREG(RFCORE_XREG_RFERRM)      = RFCORE_XREG_RFERRM_RFERRM_M; //all errors
   IntEnable(INT_RFCOREERR);
   //radio_on();
   
   // change state
   radio_vars.state               = RADIOSTATE_RFOFF;
}

#else

void radio_init(void) {
	uint8_t i,j;
	uint16_t rndSeed;
	uint8_t rndByte = 0;

   // clear variables
   memset(&radio_vars,0,sizeof(radio_vars_t));

   radio_csma_vars.countBusy = 0;
   radio_csma_vars.counterr = 0;
   radio_csma_vars.countok = 0;

   radio_csma_vars.rfftxendok = 0;
   radio_csma_vars.rfftxbusy  = 0;
   radio_csma_vars.rfftxwait  = 0;
   radio_csma_vars.rfftxstop2 = 0;
   radio_csma_vars.nb = 0;
   radio_csma_vars.countBusy =0 ;
   radio_csma_vars.counterr=0;

   // change state
   radio_vars.state = RADIOSTATE_STOPPED;

   //flush fifos
   CC2538_RF_CSP_ISFLUSHRX();
   CC2538_RF_CSP_ISFLUSHTX();

   radio_off();

   //disable radio interrupts
   disable_radio_interrupts();

   /*
   This CORR_THR value should be changed to 0x14 before attempting RX. Testing has shown that
   too many false frames are received if the reset value is used. Make it more likely to detect
   sync by removing the requirement that both symbols in the SFD must have a correlation value
   above the correlation threshold, and make sync word detection less likely by raising the
   correlation threshold.
   */
   HWREG(RFCORE_XREG_MDMCTRL1)    = 0x14; 
   /* tuning adjustments for optimal radio performance; details available in datasheet */

   HWREG(RFCORE_XREG_RXCTRL)      = 0x3F;  
  /* Adjust current in synthesizer; details available in datasheet. */
   HWREG(RFCORE_XREG_FSCTRL)      = 0x55; 

//TODO!!! VERIFICAR SE PRECISA DISSO
//#if !(defined HAL_PA_LNA || defined HAL_PA_LNA_CC2590 ||  defined HAL_PA_LNA_CC2592)
  /* Raises the CCA threshold from about -108 dBm to about -80 dBm input level.
   */
 //  HWREG(RFCORE_XREG_CCACTRL0) = CCA_THR;  //OK
//#endif

  /* Makes sync word detection less likely by requiring two zero symbols before the sync word.
   * details available in datasheet.
   */
   HWREG(RFCORE_XREG_MDMCTRL0)    = 0x85;   //C
   //update CCA register to -81db as indicated by manual.. won't be used..
   //HWREG(RFCORE_XREG_CCACTRL0)    = 0xF8;

   /* Adjust current in VCO; details available in datasheet. */
   HWREG(RFCORE_XREG_FSCAL1)      = 0x01;  

   /* Adjust target value for AGC control loop; details available in datasheet. */
   HWREG(RFCORE_XREG_AGCCTRL1)    = 0x15;   

  /* Disable source address matching an autopend for now */
   HWREG(RFCORE_XREG_SRCMATCH) = 0;

   /* Tune ADC performance, details available in datasheet. */
   HWREG(RFCORE_XREG_ADCTEST0)    = 0x10;   
   HWREG(RFCORE_XREG_ADCTEST1)    = 0x0E;  
   HWREG(RFCORE_XREG_ADCTEST2)    = 0x03;  

   /*
	* Changes from default values
	* See User Guide, section "Register Settings Update"
	*/
  /* Sets TX anti-aliasing filter to appropriate bandwidth.
   * Reduces spurious emissions close to signal.
   */
   HWREG(RFCORE_XREG_TXFILTCFG)   = 0x09; 

   /*Controls bias currents */
   HWREG(ANA_REGS_O_IVCTRL)       = 0x0B;  

   /* disable the CSPT register compare function */
   HWREG(RFCORE_XREG_CSPT)        = 0xFFUL;

   /*
	* Defaults:
	* Auto CRC; Append RSSI, CRC-OK and Corr. Val.; CRC calculation;
	* RX and TX modes with FIFOs 
	*/
   //poipoi disable frame filtering by now.. sniffer mode.
   HWREG(RFCORE_XREG_FRMFILT0)   &= ~RFCORE_XREG_FRMFILT0_FRAME_FILTER_EN;

   /* MAX FIFOP threshold */
   HWREG(RFCORE_XREG_FIFOPCTRL)   = CC2538_RF_MAX_PACKET_LEN;
   HWREG(RFCORE_XREG_TXPOWER)     = CC2538_RF_TX_POWER;
   HWREG(RFCORE_XREG_FREQCTRL)    = CC2538_RF_CHANNEL_MIN;

   // END ????????????????

   //register interrupt
   IntRegister(INT_RFCORERTX, radio_isr_internal);    // OK
   IntRegister(INT_RFCOREERR, radio_error_isr);       // OK

   IntPrioritySet(INT_RFCORERTX, HAL_INT_PRIOR_MAC);  // OK
   IntPrioritySet(INT_RFCOREERR, HAL_INT_PRIOR_MAC);  // OK

   IntEnable(INT_RFCORERTX);   //AQUI ELE SO INICIOU PONTUAL>>

   /* Enable all RF Error interrupts */
   HWREG(RFCORE_XREG_RFERRM)      = RFCORE_XREG_RFERRM_RFERRM_M; //all errors
   IntEnable(INT_RFCOREERR);     //AQUI ELE SO INICIOU PONTUAL>>
   //radio_on();

   // change state
   radio_vars.state               = RADIOSTATE_RFOFF;

    csmavarsinit();
	//errorcount =0;
	//macTxBe = radio_csma_vars.minBe;

	 /*----------------------------------------------------------------------------------------------
	  *  Initialize random seed value.
	  */

	  /*
	   *  Set radio for infinite reception.  Once radio reaches this state,Auto CRC;
	   *  it will stay in receive mode regardless RF activity.
	   */
	  HWREG(RFCORE_XREG_FRMCTRL0) = FRMCTRL0_RESET_VALUE | RX_MODE_INFINITE_RECEPTION | RFCORE_XREG_FRMCTRL0_AUTOCRC;

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
	  RFST = ISRFOFF;

	  /* take receiver out of infinite reception mode; set back to normal operation */
	  HWREG(RFCORE_XREG_FRMCTRL0) = FRMCTRL0_RESET_VALUE | RX_MODE_NORMAL_OPERATION;

	  /* Turn on autoack ??????*/
	  HWREG(RFCORE_XREG_FRMCTRL0) |= AUTOACK;    //MAC_RADIO_TURN_ON_AUTO_ACK();

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

#endif


void radio_setOverflowCb(radiotimer_compare_cbt cb) {
   radiotimer_setOverflowCb(cb);
}

void radio_setCompareCb(radiotimer_compare_cbt cb) {
   radiotimer_setCompareCb(cb);
}

void radio_setStartFrameCb(radiotimer_capture_cbt cb) {
   radio_vars.startFrame_cb  = cb;
}

void radio_setEndFrameCb(radiotimer_capture_cbt cb) {
   radio_vars.endFrame_cb    = cb;
}

//===== reset

void radio_reset() {
   /* Wait for ongoing TX to complete (e.g. this could be an outgoing ACK) */
   while(HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE);
   
   //flush fifos
   CC2538_RF_CSP_ISFLUSHRX();
   CC2538_RF_CSP_ISFLUSHTX();
   
   /* Don't turn off if we are off as this will trigger a Strobe Error */
   if(HWREG(RFCORE_XREG_RXENABLE) != 0) {
      CC2538_RF_CSP_ISRFOFF();
   }
   radio_init();
}

//===== timer

void radio_startTimer(PORT_TIMER_WIDTH period) {
   radiotimer_start(period);
}

PORT_TIMER_WIDTH radio_getTimerValue() {
   return radiotimer_getValue();
}

void radio_setTimerPeriod(PORT_TIMER_WIDTH period) {
   radiotimer_setPeriod(period);
}

PORT_TIMER_WIDTH radio_getTimerPeriod() {
   return radiotimer_getPeriod();
}

//===== RF admin

void radio_setFrequency(uint8_t frequency) {

   // change state
   radio_vars.state = RADIOSTATE_SETTING_FREQUENCY;

   radio_off();
   // configure the radio to the right frequecy
   if((frequency < CC2538_RF_CHANNEL_MIN) || (frequency > CC2538_RF_CHANNEL_MAX)) {
      while(1);
   }
   
   /* Changes to FREQCTRL take effect after the next recalibration */
   HWREG(RFCORE_XREG_FREQCTRL) = (CC2538_RF_CHANNEL_MIN
      + (frequency - CC2538_RF_CHANNEL_MIN) * CC2538_RF_CHANNEL_SPACING);
   
   //radio_on();
   
   // change state
   radio_vars.state = RADIOSTATE_FREQUENCY_SET;
}

void radio_rfOn() {
   //radio_on();
}

void radio_rfOff() {
   
   // change state
   radio_vars.state = RADIOSTATE_TURNING_OFF;
   radio_off();
   // wiggle debug pin
   debugpins_radio_clr();
   //leds_radio_off();
   //enable radio interrupts
   disable_radio_interrupts();
   
   macRxOnFlag = 0;
   // change state
   radio_vars.state = RADIOSTATE_RFOFF;
}

//===== TX

void radio_loadPacket(uint8_t* packet, uint8_t len) {
   uint8_t i=0;
   
   // change state
   radio_vars.state = RADIOSTATE_LOADING_PACKET;
   
   // load packet in TXFIFO
   /*
   When we transmit in very quick bursts, make sure previous transmission
   is not still in progress before re-writing to the TX FIFO
   */
   while(HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE);
   
   CC2538_RF_CSP_ISFLUSHTX();
   
   /* Send the phy length byte first */
    HWREG(RFCORE_SFR_RFDATA) = len; //crc len is included
   
   for(i = 0; i < len; i++) {
      HWREG(RFCORE_SFR_RFDATA) = packet[i];
   }
   
   // change state
   radio_vars.state = RADIOSTATE_PACKET_LOADED;
}

void radio_txEnable() {
   
   // change state
   radio_vars.state = RADIOSTATE_ENABLING_TX;
   
   // wiggle debug pin
   debugpins_radio_set();
   //leds_radio_on();
   
   //do nothing -- radio is activated by the strobe on rx or tx
   //radio_rfOn();
   
   // change state
   radio_vars.state = RADIOSTATE_TX_ENABLED;
}

#if (ENABLE_CSMA_CA == 1)
/*
 * quando usando CSMA_CA tenho que esperar a linha estar desocupada...
 * aqui ja considero que carreguei o frame e ja liguei o mecanismo do csma em load_packet
 */
void radio_txNow(uint8_t flagUseCsma) {
   PORT_TIMER_WIDTH count;

   // change state
   radio_vars.state = RADIOSTATE_TRANSMITTING;

   if (flagUseCsma == 0)
   {
	   //enable radio interrupts
	   enable_radio_interrupts();

	   //make sure we are not transmitting already
	   while(HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE);

	   // send packet by STON strobe see pag 669
	   CC2538_RF_CSP_ISTXON();
	   //wait 192uS
	   count=0;
	   while(!((HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE))){
	      count++; //debug
	   }
   }
   else
   {
	   // leds_debug_on();
	   radio_csma_vars.capturedTime0 = radiotimer_getCapturedTime();

	   radio_csma_vars.nb = 0;   //contador de retries
	   radio_csma_vars.macTxBe = 3;
	   //leds_debug_on();

	   // Prepare for transmit. unslotted csma_ca
	   txCsmaPrep();

	   // Run previously loaded CSP program for CSMA transmit.
	   macCspTxGoCsma();
   }

}
#else
void radio_txNow() {
   PORT_TIMER_WIDTH count;

   // change state
   radio_vars.state = RADIOSTATE_TRANSMITTING;

   //enable radio interrupts
   enable_radio_interrupts();

   //make sure we are not transmitting already
   while(HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE);

   // send packet by STON strobe see pag 669

   CC2538_RF_CSP_ISTXON();
   //wait 192uS
   count=0;
   while(!((HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE))){
      count++; //debug
   }
}

#endif
//===== RX

void radio_rxEnable() {
   
   // change state
   radio_vars.state = RADIOSTATE_ENABLING_RX;
   
   //enable radio interrupts
   
   // do nothing as we do not want to receive anything yet.
   // wiggle debug pin
   debugpins_radio_set();
   //leds_radio_on();
   
   // change state
   radio_vars.state = RADIOSTATE_LISTENING;
}

void radio_rxNow() {
   //empty buffer before receiving
   //CC2538_RF_CSP_ISFLUSHRX();
   
   //enable radio interrupts
   CC2538_RF_CSP_ISFLUSHRX();
   enable_radio_interrupts();
   
   CC2538_RF_CSP_ISRXON();
   // busy wait until radio really listening
   while(!((HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_RX_ACTIVE)));
}

void radio_getReceivedFrame(uint8_t* pBufRead,
                            uint8_t* pLenRead,
                            uint8_t  maxBufLen,
                             int8_t* pRssi,
                            uint8_t* pLqi,
                               bool* pCrc) {
   uint8_t crc_corr,i;
   
   uint8_t len=0;
   
   /* Check the length */
   len = HWREG(RFCORE_SFR_RFDATA); //first byte is len
   
   
   /* Check for validity */
   if(len > CC2538_RF_MAX_PACKET_LEN) {
      /* wrong len */
      CC2538_RF_CSP_ISFLUSHRX();
      return;
   }
   
   
   if(len <= CC2538_RF_MIN_PACKET_LEN) {
      //too short
      CC2538_RF_CSP_ISFLUSHRX();
      return;
   }
   
   //check if this fits to the buffer
   if(len > maxBufLen) {
      CC2538_RF_CSP_ISFLUSHRX();
      return;
   }
   
   // when reading the packet from the RX buffer, you get the following:
   // - *[1B]     length byte
   // -  [0-125B] packet (excluding CRC)
   // -  [1B]     RSSI
   // - *[2B]     CRC
   
   //skip first byte is len
   for(i = 0; i < len - 2; i++) {
      pBufRead[i] = HWREG(RFCORE_SFR_RFDATA);
   }
   
   *pRssi     = ((int8_t)(HWREG(RFCORE_SFR_RFDATA)) - RSSI_OFFSET);
   crc_corr   = HWREG(RFCORE_SFR_RFDATA);
   *pCrc      = crc_corr & CRC_BIT_MASK;
   *pLenRead  = len;
   
   //flush it
   CC2538_RF_CSP_ISFLUSHRX();
}

//=========================== private =========================================

port_INLINE  void enable_radio_interrupts(void){
#if (IEEE802154E_TSCH == 1)
   /* Enable RF interrupts 0, RXPKTDONE,SFD,FIFOP only -- see page 751  */
   HWREG(RFCORE_XREG_RFIRQM0) |= ((0x06|0x02|0x01) << RFCORE_XREG_RFIRQM0_RFIRQM_S) & RFCORE_XREG_RFIRQM0_RFIRQM_M;
#else
   /* Enable RF interrupts 0, RXPKTDONE,FIFOP only -- see page 751  */
   HWREG(RFCORE_XREG_RFIRQM0) |= ((0x06|0x02) << RFCORE_XREG_RFIRQM0_RFIRQM_S) & RFCORE_XREG_RFIRQM0_RFIRQM_M;
#endif
   /* Enable RF interrupts 1, TXDONE only */
   HWREG(RFCORE_XREG_RFIRQM1) |= ((0x02) << RFCORE_XREG_RFIRQM1_RFIRQM_S) & RFCORE_XREG_RFIRQM1_RFIRQM_M;
}

port_INLINE  void disable_radio_interrupts(void){
   /* Enable RF interrupts 0, RXPKTDONE,SFD,FIFOP only -- see page 751  */
   HWREG(RFCORE_XREG_RFIRQM0) = 0;
   /* Enable RF interrupts 1, TXDONE only */
   HWREG(RFCORE_XREG_RFIRQM1) = 0;
}


port_INLINE void radio_on(void){
   // CC2538_RF_CSP_ISFLUSHRX();
    CC2538_RF_CSP_ISRXON();
}

port_INLINE void radio_off(void){
   /* Wait for ongoing TX to complete (e.g. this could be an outgoing ACK) */
   while(HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE);
   //CC2538_RF_CSP_ISFLUSHRX();
   /* Don't turn off if we are off as this will trigger a Strobe Error */
   if(HWREG(RFCORE_XREG_RXENABLE) != 0) {
      CC2538_RF_CSP_ISRFOFF();
      //clear fifo isr flag
      HWREG(RFCORE_SFR_RFIRQF0) = ~(RFCORE_SFR_RFIRQF0_FIFOP|RFCORE_SFR_RFIRQF0_RXPKTDONE);
   }
}


void radio_flushfifos(void){
	//flush fifos
	CC2538_RF_CSP_ISFLUSHRX();
	CC2538_RF_CSP_ISFLUSHTX();
}

/*
 *
 void radio_reset() {

   while(HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE);

   //flush fifos
   CC2538_RF_CSP_ISFLUSHRX();
   CC2538_RF_CSP_ISFLUSHTX();

   if(HWREG(RFCORE_XREG_RXENABLE) != 0) {
      CC2538_RF_CSP_ISRFOFF();
   }
   radio_init();
}
 *
 */

//=========================== callbacks =======================================
#if (ENABLE_CSMA_CA == 1)

void calculadeltatime(void){


  radio_csma_vars.delta[radio_csma_vars.deltapos] = radio_csma_vars.capturedTime1 - radio_csma_vars.capturedTime0;

  if (radio_csma_vars.delta[radio_csma_vars.deltapos] > radio_csma_vars.deltaMax)
	  radio_csma_vars.deltaMax = radio_csma_vars.delta[radio_csma_vars.deltapos];

  if (radio_csma_vars.deltapos >= MAX_DELTA)
	  radio_csma_vars.deltapos = 0;
  radio_csma_vars.deltapos++;



}
/**************************************************************************************************
 * @fn          macCspTxStopIsr
 *
 * @brief       Interrupt service routine for handling STOP type interrupts from CSP.
 *              This interrupt occurs when the CSP program stops by 1) reaching the end of the
 *              program, 2) executing SSTOP within the program, 3) executing immediate
 *              instruction ISSTOP.
 *
 *              The value of CSPZ indicates if interrupt is being used for ACK timeout or
 *              is the end of a transmit.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macCspTxStopIsr(void)
{
  uint32_t capturedTime=0;
  macRxOnFlag =0;
  if (HWREG(RFCORE_XREG_CSPZ) == CSPZ_CODE_TX_DONE)
  {
#if (DEBUG_CSMA == 1)
	  // leds_debug_off();
	  radio_csma_vars.capturedTime1 = radiotimer_getCapturedTime();

      calculadeltatime();
#endif

	  // change state
      radio_vars.state = RADIOSTATE_TXRX_DONE;
	  radio_csma_vars.countok++;

	  //leds_debug_off();

      if (radio_vars.endFrame_cb!=NULL) {
         // call the callback
         radio_vars.endFrame_cb(capturedTime);
         // kick the OS
         return;
      } else {
         while(1);
      }
  }
  else if (HWREG(RFCORE_XREG_CSPZ) == CSPZ_CODE_CHANNEL_BUSY)
  {
	  radio_csma_vars.rfftxbusy++;

	  macCspTxBusyIsr(capturedTime);
	  //leds_sync_toggle();
  }
  else
  {
     radio_csma_vars.rfftxstop2++;
	//AQUI EH UMA CONDICAO INESPERADA...NAO SEI O QUE FAZER...
    //MAC_ASSERT((HWREG(RFCORE_XREG_CSPZ) == CSPZ_CODE_TX_ACK_TIME_OUT);
    //macTxAckNotReceivedCallback();

  }

}
#endif

//=========================== interrupt handlers ==============================

/**
\brief Stub function for the CC2538.

In MSP430 platforms the CPU status after servicing an interrupt can be managed
toggling some bits in a special register, e.g. CPUOFF, LPM1, etc, within the
interrupt context itself. By default, after servicing an interrupt the CPU will
be off so it makes sense to return a value and enable it if something has
happened that needs the scheduler to run (a packet has been received that needs
to be processed). Otherwise, the CPU is kept in sleep mode without even
reaching the main loop.

In the CC2538, however, the default behaviour is the contrary. After servicing
an interrupt the CPU will be on by default and it is the responsability of the
main thread to put it back to sleep (which is already done). This means that
the scheduler will always be kicked in after servicing an interrupt. This
behaviour can be changed by modifying the SLEEPEXIT field in the SYSCTRL
regiser (see page 131 of the CC2538 manual).
*/
kick_scheduler_t radio_isr() {
   return DO_NOT_KICK_SCHEDULER;
}

#if (ENABLE_CSMA_CA == 1)

void macCspTxBusyIsr(PORT_TIMER_WIDTH capturedTime){

	if (macRxOnFlag)
		{
			macRxOnFlag = 0;
			RFST = ISRFOFF;
			/* just in case a receive was about to start, flush the receive FIFO */
			//MAC_RADIO_FLUSH_RX_FIFO(); --> neste caso tenho que carregar o frame novamente na fifo (load_packet)

			/* clear any receive interrupt that happened to squeak through */
			//MAC_RADIO_CLEAR_RX_THRESHOLD_INTERRUPT_FLAG();
			IntPendClear(INT_RFCORERTX);
			RFIRQF0 = (IRQ_FIFOP ^ 0xFF);
		}

		radio_csma_vars.nb++;
		radio_csma_vars.countBusy++;
		//leds_sync_toggle();

		if (radio_csma_vars.nb > maxCsmaBackoffs)
		{
			 // TODO!!!! AQUI DEVERIA SINALIZAR QUE HOUVE ERRO!!!!
			 radio_vars.state = RADIOSTATE_TURNING_OFF;
			 radio_csma_vars.counterr++;
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
			txCsmaPrep();
			macCspTxGoCsma();
		}

}

void radio_isr_internal(void) {
   volatile PORT_TIMER_WIDTH capturedTime;
   uint8_t  irq_status0,irq_status1;
   volatile uint32_t capture1;
   volatile uint16_t capture2;
   uint8_t nb=0;
   
   // capture the time
   capturedTime = radiotimer_getCapturedTime();
   //capturedTime =  radiotimer_getfreerunning();
   // reading IRQ_STATUS
   irq_status0 = HWREG(RFCORE_SFR_RFIRQF0);
   irq_status1 = HWREG(RFCORE_SFR_RFIRQF1);
   
   IntPendClear(INT_RFCORERTX);
   
   //clear interrupt
   HWREG(RFCORE_SFR_RFIRQF0) = 0;
   HWREG(RFCORE_SFR_RFIRQF1) = 0;

   //STATUS1 Register
#if (ENABLE_CSMA_CA == 1)
 /*
   if ((irq_status1 & IRQ_CSP_MANINT) ==  IRQ_CSP_MANINT)   {
     * This interrupt happens when the CSP instruction INT is executed.  It occurs once the SFD signal
      *  goes high indicating that transmit has successfully started.
      *  The timer value has been captured at this point and timestamp can be stored.
      *  Important!  Because of how the CSP programs are written, CSP_INT interrupts should
      *  be processed before CSP_STOP interrupts.  This becomes an issue when there are
      *  long critical sections.

	 HWREG(RFCORE_SFR_RFIRQF1) = ~IRQ_CSP_MANINT;

     return;
   }
*/
   if ((irq_status1 & RFCORE_SFR_RFIRQF1_CSP_STOP) == RFCORE_SFR_RFIRQF1_CSP_STOP) {

	   HWREG(RFCORE_SFR_RFIRQF1) = ~IRQ_CSP_STOP;    /* clear flag */
	   macCspTxStopIsr();
       return;
   }
   else if ((irq_status1 & IRQ_CSP_WAIT) ==  IRQ_CSP_WAIT)   {
		HWREG(RFCORE_SFR_RFIRQF1) = ~IRQ_CSP_WAIT;    /* clear flag */
		radio_csma_vars.rfftxwait++;
		macCspTxBusyIsr(capturedTime);
		return;
   }
   else if (((irq_status1 & RFCORE_SFR_RFIRQF1_TXDONE) == RFCORE_SFR_RFIRQF1_TXDONE)) {
	   // end of frame event --either end of tx .
	  radio_vars.state = RADIOSTATE_TXRX_DONE;

	  if (radio_vars.endFrame_cb!=NULL) {
         // call the callback
		 radio_csma_vars.rfftxendok++;

         radio_vars.endFrame_cb(capturedTime);
         // kick the OS
         return;
      } else {
         while(1);
      }
   }
#else
   if (((irq_status1 & RFCORE_SFR_RFIRQF1_TXDONE) == RFCORE_SFR_RFIRQF1_TXDONE)) {
	   // end of frame event --either end of tx .
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
   
#endif


   //STATUS0 Register
   // start of frame event
   if ((irq_status0 & RFCORE_SFR_RFIRQF0_SFD) == RFCORE_SFR_RFIRQF0_SFD) {
#if (IEEE802154E_TSCH == 1)
	   // change state
      radio_vars.state = RADIOSTATE_RECEIVING;
      if (radio_vars.startFrame_cb!=NULL) {
         // call the callback
         radio_vars.startFrame_cb(capturedTime);
         // kick the OS
         return;
      } else {
         while(1);
      }
#endif
   }
   
   //or RXDONE is full -- we have a packet.
   if (((irq_status0 & RFCORE_SFR_RFIRQF0_RXPKTDONE) ==  RFCORE_SFR_RFIRQF0_RXPKTDONE)) {
      // change state
      radio_vars.state = RADIOSTATE_TXRX_DONE;

      if (radio_vars.endFrame_cb!=NULL) {
         // call the callback
         radio_vars.endFrame_cb(capturedTime);
         // kick the OS
         return;
      } else {
         leds_error_on();
    	 while(1);
      }
   }
   
   // or FIFOP is full -- we have a packet.
   if (((irq_status0 & RFCORE_SFR_RFIRQF0_FIFOP) ==  RFCORE_SFR_RFIRQF0_FIFOP)) {
      // change state
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
   


   return;
}

#else //ENABLE_CSMA_CA == 0

void radio_isr_internal(void) {
   volatile PORT_TIMER_WIDTH capturedTime;
   uint8_t  irq_status0,irq_status1;
   
   // capture the time
   capturedTime = radiotimer_getCapturedTime();
   
   // reading IRQ_STATUS
   irq_status0 = HWREG(RFCORE_SFR_RFIRQF0);
   irq_status1 = HWREG(RFCORE_SFR_RFIRQF1);
   
   IntPendClear(INT_RFCORERTX);
   
   //clear interrupt
   HWREG(RFCORE_SFR_RFIRQF0) = 0;
   HWREG(RFCORE_SFR_RFIRQF1) = 0;
   
   //STATUS0 Register
   // start of frame event
   if ((irq_status0 & RFCORE_SFR_RFIRQF0_SFD) == RFCORE_SFR_RFIRQF0_SFD) {
#if (IEEE802154E_TSCH == 1)
	   // change state
      radio_vars.state = RADIOSTATE_RECEIVING;
      if (radio_vars.startFrame_cb!=NULL) {
         // call the callback
         radio_vars.startFrame_cb(capturedTime);
         // kick the OS
         return;
      } else {
         while(1);
      }
#endif
   }
   
   //or RXDONE is full -- we have a packet.
   if (((irq_status0 & RFCORE_SFR_RFIRQF0_RXPKTDONE) ==  RFCORE_SFR_RFIRQF0_RXPKTDONE)) {
      // change state
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
   
   // or FIFOP is full -- we have a packet.
   if (((irq_status0 & RFCORE_SFR_RFIRQF0_FIFOP) ==  RFCORE_SFR_RFIRQF0_FIFOP)) {
      // change state
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
   
   //STATUS1 Register
   // end of frame event --either end of tx .
   if (((irq_status1 & RFCORE_SFR_RFIRQF1_TXDONE) == RFCORE_SFR_RFIRQF1_TXDONE)) {
      // change state
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

   return;
}
#endif

void radio_error_isr(void){
   uint8_t rferrm;
   
   rferrm = (uint8_t)HWREG(RFCORE_XREG_RFERRM);
   
   if ((HWREG(RFCORE_XREG_RFERRM) & (((0x02)<<RFCORE_XREG_RFERRM_RFERRM_S)&RFCORE_XREG_RFERRM_RFERRM_M)) & ((uint32_t)rferrm))
   {
      HWREG(RFCORE_XREG_RFERRM) = ~(((0x02)<<RFCORE_XREG_RFERRM_RFERRM_S)&RFCORE_XREG_RFERRM_RFERRM_M);
      //poipoi  -- todo handle error
   }
}
