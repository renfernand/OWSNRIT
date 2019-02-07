/*
 * Protocolo RIT da camada MAC baseado no mecanismo RIT assincrono
 * Proposto pela norma IEEE802.15.4e2012
 */

#include "opendefs.h"
#include "IEEE802154E.h"
#if (IEEE802154E_RIT == 1)
#include "IEEE802154RIT.h"
#include "radio.h"
#include "radiotimer.h"
#include "IEEE802154.h"
#include "openqueue.h"
#include "idmanager.h"
#include "openserial.h"
#include "schedule.h"
#include "packetfunctions.h"
#include "scheduler.h"
#include "leds.h"
#include "neighbors.h"
#include "debugpins.h"
#include "sixtop.h"
#include "adaptive_sync.h"
#include "processIE.h"
#include "openqueue.h"
#include "SysTick.h"
#include "mac_csp_tx.h"
#include "topology.h"
#include "openbridge.h"
#include "idmanager.h"
#if (WATCHDOG_CONF_ENABLE == 1)
#include "watchdog.h"
#endif
#include "stdlib.h"
#include "debug.h"

#define TESTE_TIMER 0
#define TEST_MEASURE_NEIGHBOR_RATE 0

#define TRATA_ACK 1

//=========================== variables =======================================
extern uint8_t Uart0ErrorOccur;
extern radio_csma_vars_t radio_csma_vars;
extern uint32_t ritTimingclock;
extern uint32_t systickcount;
extern uint32_t lastradiotimer_freerunnig;
extern uint8_t ucFlagForwarding;
extern scheduler_vars_t scheduler_vars;
extern scheduler_dbg_t  scheduler_dbg;
extern sRITqueue pvObjList[MAX_RIT_LIST_ELEM];
extern uint8_t frameDioPending;
extern uint8_t olaasn;
extern uint8_t livelistasn;
extern uint32_t txwaitola;
uint32_t radiotxola;

uint8_t nrErrorRetries;
uint8_t COAPSimCount;
uint8_t RxFlagImprimeRoute = 0;
uint8_t lastRITstatewastx;
uint8_t actualrxolaasn;

uint32_t tempo1;
uint32_t tempo2;
uint32_t livelistcount;
uint8_t livelistfirsttime;
uint8_t livelistPending;
uint8_t livelistretries;

uint8_t isrmultichannelhello;
uint32_t slottimeref;
uint8_t flagSerialTx;
uint8_t lastslotwastx;
uint8_t testrff_isforwarding;
uint8_t ucFlagTxReOpen;
uint8_t lastRITstatewastx;
uint32_t radiotxduration;
uint32_t radiorxduration;
uint32_t radiorxdutycycle1;
uint32_t radiorxdutycycle2;
uint32_t radiotxdutycycle1;
uint32_t radiotxdutycycle2;

open_addr_t        actualsrcaddr;         //usado quando recebe um frame...para identificar quem enviou...
open_addr_t        actualdstaddr;
ieee154e_vars_t    ieee154e_vars;
ieee154e_stats_t   ieee154e_stats;
open_addr_t        address_1;
uint8_t            flagreschedule=0;
sRITelement        element;


//uint8_t            coappending;
//extern uint8_t RITQueue_ElementPending;   //salvo o elemento que acabou de enviar uma msg - para retornar apos o envio (apos o ola)
bool                txpending=FALSE;
OpenQueueEntry_t  sPendingDataPacket;

uint8_t macRITstate;

//uint8_t u8NrMsgQueue;          // numero de posicoes utilizadas na queue de RIT Pending Msg to TX
//uint8_t numAvailableElements;  // numero de lugares ocupados no pool de mensagem
extern uint8_t maxElements;
//uint8_t macRITActualPos;           //tem a informacao da posicao do atual Tx Msg Pending (antes do ola)
//uint32_t numOfQueueInsertionErrors;

sRITqueue sRIT_vars;

RIT_stats_t ritstat;

uint8_t macRIT_Pending_TX_frameType;  //PROVISORIO..>TESTE ONLY
uint8_t macRIT_Pending_RX_frameType;  //PROVISORIO..>TESTE ONLY

volatile uint32_t actualtimer0;

extern OpenQueueEntry_t advRIT;

#define DEBUG_ACK  0
uint8_t MsgNeedAck;

//uint32_t moteOKcount=0;
//uint32_t moteRetrycount=0;
//uint32_t testTx=0;

//=========================== prototypes ======================================
void ieee154e_init(void);
// interrupts
void     isr_ieee154e_newSlot(void);
void     isr_ieee154e_timer(void);
void     isr_multichannelhello(void);
port_INLINE void getRitRequest(void);
void clearstatistics(void);

port_INLINE void activitytx_olaackprepare(PORT_RADIOTIMER_WIDTH capturedTime);
bool RITQueue_ExistFramePending(void);
port_INLINE void activitytx_preparedata(PORT_RADIOTIMER_WIDTH capturedTime);

owerror_t openqueue_freePacketRITBuffer(OpenQueueEntry_t* pkt);
void     activity_ti1ORri1(void);

// SYNCHRONIZING
void     activity_synchronize_startOfFrame(PORT_RADIOTIMER_WIDTH capturedTime);
void     activity_synchronize_endOfFrame(PORT_RADIOTIMER_WIDTH capturedTime);

// TX
port_INLINE void StartTxMultiChannelHello (uint8_t pending);
port_INLINE void RITactivity_tx00(uint8_t elepending, uint8_t newtxframe);
port_INLINE uint8_t RITactivity_tx01(sRITelement *pmsgout,uint8_t txpending,uint8_t newtxframe);
port_INLINE void RITactivity_tx02(void);
port_INLINE void RITactivity_tx03(void);
port_INLINE void RITactivity_tx40(open_addr_t* address);
port_INLINE uint8_t RITactivity_tx45(void);
port_INLINE void RITactivity_tx06(void);
port_INLINE void RITactivity_tx07(PORT_RADIOTIMER_WIDTH capturedTime);
port_INLINE void RITactivity_tx08(void);
port_INLINE void RITactivity_tx81(void);
port_INLINE void RITactivity_txe08(void);
port_INLINE void RITactivity_tx09(PORT_RADIOTIMER_WIDTH capturedTime);
port_INLINE void RITactivity_txe03(void);
port_INLINE void RITactivity_tx04(PORT_RADIOTIMER_WIDTH capturedTime);

// RX
port_INLINE void RITactivity_rx00(void);
port_INLINE void RITactivity_rxe01(void);
port_INLINE void activityrx_waitolaecho(void);
port_INLINE void RITactivity_rx01(void);
port_INLINE void RITactivity_rx02(PORT_RADIOTIMER_WIDTH capturedTime);
port_INLINE void RITactivity_rx03(void);
port_INLINE void RITactivity_rxe03(void);
port_INLINE void RITactivity_rx04(PORT_RADIOTIMER_WIDTH capturedTime);
port_INLINE uint8_t RITactivity_rx05 (uint8_t ackRequested,uint8_t frameType, open_addr_t *rxaddr_dst);
port_INLINE void RITactivity_rx06(PORT_RADIOTIMER_WIDTH capturedTime);
port_INLINE void RITactivity_rx07(void);
port_INLINE void RITactivity_rxe07(void);
port_INLINE void RITactivity_rx08(PORT_RADIOTIMER_WIDTH capturedTime);
void     activity_rxwindowend(void);
port_INLINE uint8_t activityrx_reopenrxwindow(open_addr_t addr);

uint32_t calcdeltatime(uint32_t timebase);
port_INLINE sRITelement activityrx_preparemultichannelhello(uint8_t hellospec, uint8_t *frame,uint8_t len);
port_INLINE uint8_t activityrx_prepareritdatareq(uint8_t hello, uint16_t *destaddr);
void activity_RITDoNothing(void);
uint8_t toogleTxRxSerial(void);

#if (RIT_RX_SNIFFER == 1)
port_INLINE void RITactivity_RxSniffer(void);
port_INLINE void RITactivity_rxsniffer_timeout(void);
#endif

owerror_t sixtop_send_internal(
   OpenQueueEntry_t* msg,
   uint8_t iePresent,
   uint8_t frameVersion);

// frame validity check
bool     isValidRxFrame(ieee802154_header_iht* ieee802514_header);
bool     isValidAck(ieee802154_header_iht*     ieee802514_header,
                    OpenQueueEntry_t*          packetSent);
// IEs Handling
bool     ieee154e_processIEs(OpenQueueEntry_t* pkt, uint16_t* lenIE);
void     ieee154e_processSlotframeLinkIE(OpenQueueEntry_t* pkt,uint8_t * ptr);
// ASN handling
void     incrementAsnOffset(void);
void     asnStoreFromAdv(uint8_t* asn);
void     joinPriorityStoreFromAdv(uint8_t jp);
// synchronization
void     synchronizePacket(PORT_RADIOTIMER_WIDTH timeReceived);
void     synchronizeAck(PORT_SIGNED_INT_WIDTH timeCorrection);
void     changeIsSync(bool newIsSync);
// notifying upper layer
void     notif_sendDone(OpenQueueEntry_t* packetSent, owerror_t error);
void     notif_receive(OpenQueueEntry_t* packetReceived);
// statistics
void     resetStats(void);
void     updateStats(PORT_SIGNED_INT_WIDTH timeCorrection);
// misc
uint8_t  calculateFrequency(uint8_t channelOffset);
void     changeState(ieee154e_state_t newstate);
void     endSlot(void);
bool     debugPrint_asn(void);
bool     debugPrint_isSync(void);

//=========================== admin ===========================================

/**
\brief This function initializes this module.

Call this function once before any other function in this module, possibly
during boot-up.
*/
uint8_t toogleTxRxSerial(void)
{
#if 1
	if (flagSerialTx > OPENBRIDGE_MAX_TX_TO_RX)
	{
		flagSerialTx = 0;
	}
	else
	{
		flagSerialTx++;
	}
#else
	flagSerialTx = 0;
#endif

	return flagSerialTx;
}


/*
 * Calcula o tempo decorrido entre o timebase e o tempo atual
 * ele retorna o delta de tempo.
 * Quando o tempo decorrido eh maior que o timebase entao ele retorna zero.
 */

uint32_t calcdeltatime(uint32_t timebase){
	uint32_t timeelapsedms;
	uint32_t deltams;

	timeelapsedms = getdeltaslotperiod_ms();
	if (timeelapsedms > ieee154e_vars.lastCapturedTime)
		deltams = (timeelapsedms - ieee154e_vars.lastCapturedTime)*RADIOTIMER_TICS_MS;
	else
		deltams = timebase;

	ieee154e_vars.lastCapturedTime = timeelapsedms;


	if (timebase > deltams){

		deltams = timebase - deltams;
		if (deltams < 1)
			deltams = 1;
	}
	else{
		deltams = 0;
	}

#if 0 // ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
{
	uint8_t   pos=0;

	rffbuf[pos++]= RFF_IEEE802_RX;
	rffbuf[pos++]= 0x46;
	pos = printvar((uint8_t *)&timeelapsedms,sizeof(uint32_t),rffbuf,pos);
	pos = printvar((uint8_t *)&deltams,sizeof(uint32_t),rffbuf,pos);

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
}
#endif
	return deltams;
}

//=========================== public ==========================================

void ieee154e_init(void) {

   // initialize variables
   memset(&ieee154e_vars,0,sizeof(ieee154e_vars_t));
   memset(&ieee154e_dbg,0,sizeof(ieee154e_dbg_t));

   changeIsSync(TRUE);
   ucFlagForwarding = FALSE;
   ucFlagTxReOpen = FALSE;

   resetStats();
   ieee154e_stats.numDeSync  = 0;
 
//   COAPSimCount=0;
   flagSerialTx = 0;
   flagreschedule = 0;
   //moteOKcount=0;
   //moteRetrycount=0;
   //testTx=0;
   isrmultichannelhello = 0;
   macRITstate = 0;
   //sRIT_vars.livelistasn=0;
   livelistasn = 0;
   livelistPending = 0;
   olaasn=0;

   // switch radio on
   radio_rfOn();

	//set the period constant
//	macAckWaitPeriod       = TICK_RIT_ACK_WAIT_PERIOD;
//	macRITRXforTxPeriod    = TICK_MAC_RIT_RX_TO_TX_PERIOD;
//	macRITDataWaitDuration = TICK_MAC_RIT_RX_WIND_PERIOD;
#if (ENABLE_CSMA_CA == 0)
	macRITPeriod           = TICK_MAC_RIT_PERIOD;
	macRITTxPeriod         = TICK_MAC_RIT_TX_PERIOD;

    macRITsleepPeriod = (uint16_t) ((uint16_t)macRITPeriod-(uint16_t)macRITDataWaitDuration);
#endif
	//initiate RIT Queue
	RITQueue_Init();
	clearstatistics();
	livelistfirsttime = 0;
	livelistcount=0;
	testrff_isforwarding = 0;
	lastRITstatewastx = 0;
	lastslotwastx=0;
	txpending =0;
	memset(&sPendingDataPacket,0,sizeof(OpenQueueEntry_t));

   // set callback functions for the radio
   radio_setOverflowCb(isr_ieee154e_newSlot);     //timer indicando o inicio do slot
   radio_setCompareCb(isr_ieee154e_timer);        //timer diversos dentro do slot
   radio_setStartFrameCb(ieee154e_startOfFrame);  //indica inicio do pacote
   radio_setEndFrameCb(ieee154e_endOfFrame);      //indica fim do um pacote

#if (ENABLE_CSMA_CA == 0)
   // have the radio start its timer
   radio_startTimer(macRITPeriod);
#else
   RITTimer_Init();

#if (RIT_RX_SNIFFER == 0)
  opentimers_start(RIT_MULTICHANNELHELLO_MS, TIMER_PERIODIC, TIME_MS, (opentimers_cbt) isr_multichannelhello);
#endif

#endif
#if (USE_GPTIMER == 1)
   gptimer_init();
#endif
}



/**
/brief Difference between some older ASN and the current ASN.

\param[in] someASN some ASN to compare to the current

\returns The ASN difference, or 0xffff if more than 65535 different
*/
PORT_RADIOTIMER_WIDTH ieee154e_asnDiff(asn_t* someASN) {
   PORT_RADIOTIMER_WIDTH diff;
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   if (ieee154e_vars.asn.byte4 != someASN->byte4) {
      ENABLE_INTERRUPTS();
      return (PORT_RADIOTIMER_WIDTH)0xFFFFFFFF;;
   }

   diff = 0;
   if (ieee154e_vars.asn.bytes2and3 == someASN->bytes2and3) {
      ENABLE_INTERRUPTS();
      return ieee154e_vars.asn.bytes0and1-someASN->bytes0and1;
   } else if (ieee154e_vars.asn.bytes2and3-someASN->bytes2and3==1) {
      diff  = ieee154e_vars.asn.bytes0and1;
      diff += 0xffff-someASN->bytes0and1;
      diff += 1;
   } else {
      diff = (PORT_RADIOTIMER_WIDTH)0xFFFFFFFF;;
   }
   ENABLE_INTERRUPTS();
   return diff;
}

//======= events

/**
\brief Indicates a new service Multichannel Hello to check the neighbourhood state.

This function executes in ISR mode.
*/
void isr_multichannelhello (void) {

   isrmultichannelhello = TRUE;


#if (SINK_SIMULA_COAP == 1)
  //simula coap
   COAPSimCount++;

#if 1 //((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_TX == 1))
  {
	uint8_t   pos=0;

	rffbuf[pos++]= 0x00;
	rffbuf[pos++]= 0x00;
	rffbuf[pos++]= COAPSimCount;

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
  }
#endif

  if (COAPSimCount > 5){
	  COAPSimCount = 0;
	  openbridge_simucoap();
  }
#endif

}
/**
\brief Indicates a new slot has just started.

This function executes in ISR mode, when the new slot timer fires.
*/
/*
 * Network_StartRITProcedure - send Ola and open window for data
 *  */

void isr_ieee154e_newSlot() {
 // uint8_t ret;

  #if (WATCHDOG_CONF_ENABLE == 1)
  WatchdogClear();
#endif
  ieee154e_dbg.num_newSlot++;
  nrErrorRetries = 0;

  //zero o status de error apos algum tempo pois sempre na inicializacao ele entra com erro...nao sei por que.
  if (ieee154e_dbg.num_newSlot > 5){
	  Uart0ErrorOccur=0;
  }

  clearritroute();

#if (RIT_RX_SNIFFER == 1)
  //somente escuta a linha e imprime na serial..
  openserial_stop();
  openserial_startOutput();

  RITactivity_RxSniffer();

  radio_setTimerPeriod(RX_RIT_PERIOD_MS);

#else
  activity_ti1ORri1();
#endif
}



//=========================== private =========================================
void slotvarsInitialization(void) {

   ieee154e_vars.dataToSend = NULL;
   ieee154e_vars.lastCapturedTime = 0;
   actualtimer0 = radio_getTimerValue();
   slottimeref = actualtimer0;
   ieee154e_vars.radioRxOnTics = 0;
   ieee154e_vars.radioRxOnInit = 0;
   ieee154e_vars.radioTxOnTics = 0;
   ieee154e_vars.radioTxOnInit = 0;

}

/*
 * Aqui indica o inicio do slot
 * No inicio eh verificado se tem algo para enviar pendente...entao ele tem preferencia sobre o RX...
 * Neste caso ele nao habilita o RIT....
 */
port_INLINE void activity_ti1ORri1() {
   open_addr_t neighbor;
   uint8_t elePending;
   bool newtxframe=FALSE;
   uint32_t macRITPeriod;
   uint32_t macRITTxPeriod;
   slotOffset_t targetSlotOffset;
   volatile uint8_t	random;
   volatile uint32_t	rxritperiod;

   slotvarsInitialization();

   ieee154e_dbg.num_txslot++;

   radiotimer_cancel();


    //verifico se existe mensagem pendente para ser enviada
    ieee154e_vars.dataToSend = NULL;
    targetSlotOffset = ieee154e_vars.slotOffset;

#if (TESTRIT_ONLY_OLA == 0)
		schedule_syncSlotOffset(targetSlotOffset);
	    ieee154e_vars.nextActiveSlotOffset = schedule_getNextActiveSlotOffset();

	    schedule_getNeighbor(&neighbor);

	//if (neighbor.type > 0) {
    	    //verifico se ele tem mensagem pendende
    	    ieee154e_vars.dataToSend  = openqueue_macGetDataPacket(&neighbor);
	//}
#endif


	 txpending = FALSE;
#if (RIT_RX_ONLY == 0)
   if ((txpending == TRUE) || (ieee154e_vars.dataToSend != NULL)) {   // I have a packet to send
		macRITstate=S_RIT_TX_state;
		RITactivity_tx00(txpending,newtxframe);

		radio_setTimerPeriod(TX_RIT_PERIOD_MS);
        lastRITstatewastx = TRUE;
  }
#if  (BROADCAST_MAX_RETRIES > 0)
   else if ((lastRITstatewastx == FALSE) && (livelistPending > 0)) {
		macRITstate = S_RIT_multichnhello_state;
		StartTxMultiChannelHello(livelistPending);

		//Quando Livelist e nao eh mais a primeira vez...entao aumento o tempo de espera...
		if (livelistretries > 0) {
			radio_setTimerPeriod(TX_RIT_PERIOD_MS*1.5);
		}
		else{
			radio_setTimerPeriod(TX_RIT_PERIOD_MS);
		}

        lastRITstatewastx = TRUE;
   }
#endif
  else if ((lastRITstatewastx == FALSE) && (isrmultichannelhello == TRUE))  {

		isrmultichannelhello = FALSE;
		macRITstate = S_RIT_multichnhello_state;
		livelistretries = 0;
		livelistPending = 0;

#if (LIVELIST_FIXED_PACKETS == 1)
		if (livelistcount < NR_LIVELIST_FRAMES) {
			//livelistcount++;
			StartTxMultiChannelHello(0);
		}
		else {
			endSlot();
		}
#else
		StartTxMultiChannelHello(0);
#endif
		radio_setTimerPeriod(TX_RIT_PERIOD_MS);

		lastRITstatewastx = TRUE;
  }
  else {
		macRITstate=S_RIT_RX_state;

		ieee154e_dbg.num_rxslot++;

	    if (ieee154e_vars.dataReceived != NULL) {
		   // free the (invalid) received data buffer so RAM memory can be recycled
		   openqueue_freePacketBuffer(ieee154e_vars.dataReceived);
		   ieee154e_vars.dataReceived = NULL;
	    }

	    /* TODO!!! Esta escolha da serial ainda esta trazendo problemas...No OpenWSN ele tem um slottime para serialRX e serialTX
	     * Aqui a melhor configuracao foi esta somente no slot do Ola (RX) e chavear entre abertura da porta para serial RX
	     *  e serial TX...Lembrando que o bug da serial eh que depois de um tempo a serial para de receber input do OpenVis.
	     *  Talvez tenha relacao com o log que eu faco...
	     */
		#if  SINK
		 //start inputting serial data
		  if (toogleTxRxSerial()){
			  openserial_startOutput();
			  RITactivity_rx00();
		  }
		  else {
			  openserial_startInput();
			  RITactivity_rx00();
		  }
		#else
	      openserial_stop();
		  openserial_startOutput();
		  RITactivity_rx00();
		#endif

		random = 1; //macRadioRandomByte() & ((1 << 4) - 1);
		rxritperiod = (RX_RIT_PERIOD_MS + random);
		radio_setTimerPeriod(rxritperiod);

		lastRITstatewastx = FALSE;

  }
#else
    openserial_stop();
    openserial_startOutput();
	RITactivity_rx00();

	radio_setTimerPeriod(RX_RIT_PERIOD_MS);
#endif

  ieee154e_vars.slotOffset = ieee154e_vars.nextActiveSlotOffset;
  targetSlotOffset = targetSlotOffset;
}

/*
 * Esta rotina é utilizada para reprogramar o timer e voltar a ouvir na janela do RX_RIT
 * Neste caso será atualizado a livelist com este ola recebido
 * e sera reprogramado a maquina de estado para esperar um novo frame ainda dentro da janela do ola.
 *
 */
port_INLINE uint8_t activityrx_reopenrxwindow(open_addr_t actualsrcaddr) {
   uint32_t deltams=0;
   uint32_t timeelapsedms=0;

#if (MULTI_CHANNEL_HELLO_ENABLE == 0)
   // notifica o recebimento na tabela de vizinhos
   macneighbors_updtlivelist(actualsrcaddr);
#endif

    //REPROGRAMAR A JANELA COM UM TEMPO MENOR...
   deltams = calcdeltatime(RX_RIT_TIMEOUT_MS);

    if (deltams > 0){

		incroute(0x47);

		//como desliguei o radio anteriormente devo reprograma-lo para RX
		ieee154e_vars.freq = macneighbors_getMyBestChan();

		radio_setFrequency(ieee154e_vars.freq);

		RECORD_RADIO_RX_ON

		radio_rxEnable();

		radio_rxNow();

		//Volto ao estado anterior para indicar que estou novamente esperando um frame TX
		changeState(S_RIT_RXDATA);

		radiotimer_schedule(deltams);

#if 0 // ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
{
	uint8_t   pos=0;

	rffbuf[pos++]= RFF_IEEE802_RX;
	rffbuf[pos++]= 0x47;
	pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);
	pos = printvar((uint8_t *)&deltams,sizeof(uint32_t),rffbuf,pos);

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
}
#endif

		return DISCARD_YES_ENDSLOT_NO;
    }
    else
    {
		incroute(0x48);
		return DISCARD_YES_ENDSLOT_YES;
    }


}



#if (RIT_RX_SNIFFER == 1)
port_INLINE void RITactivity_RxSniffer(void) {

	uint8_t macRIT_Pending_TX_frameType=0;
    uint8_t notifyerror=0;

	uint8_t isBroadcast=0;
	uint8_t ret;

	changeState(S_RIT_RXPROC);

	radiotimer_cancel();

	radio_rfOff();

	ieee154e_vars.freq = 20;
    radio_setFrequency(ieee154e_vars.freq);

	radio_rxEnable();
	radio_rxNow();

	radiotimer_schedule(RX_RIT_TIMEOUT_MS);

}


port_INLINE void RITactivity_rxsniffer_newdata(PORT_RADIOTIMER_WIDTH capturedTime) {
    ieee802154_header_iht ieee802514_header;
    uint16_t lenIE=0;
	open_addr_t rxaddr_nexthop;
	uint8_t   *pauxframe;
    uint8_t ret=FALSE;
    uint8_t nrDIOsent=0;
	uint32_t timeelapsedms;
	uint32_t deltams;

	radiotimer_cancel();

	radio_rfOff();


   // get a buffer to put the (received) data in
   ieee154e_vars.dataReceived = openqueue_getFreePacketBuffer(COMPONENT_IEEE802154E);

   if (ieee154e_vars.dataReceived==NULL) {
      // log the error
      openserial_printError(COMPONENT_IEEE802154E,ERR_NO_FREE_PACKET_BUFFER,
                            (errorparameter_t)0,
                            (errorparameter_t)0);
      // abort
      endSlot();

      //teste rff - clear msg pendente
      //openqueue_removeAllOwnedBy(COMPONENT_SIXTOP_TO_IEEE802154E);

      return;
   }

   // declare ownership over that packet
   ieee154e_vars.dataReceived->creator = COMPONENT_IEEE802154E;
   ieee154e_vars.dataReceived->owner   = COMPONENT_IEEE802154E;

   do { // this "loop" is only executed once

      // retrieve the received data frame from the radio's Rx buffer
      ieee154e_vars.dataReceived->payload = &(ieee154e_vars.dataReceived->packet[FIRST_FRAME_BYTE]);

      radio_getReceivedFrame(       ieee154e_vars.dataReceived->payload,
                                   &ieee154e_vars.dataReceived->length,
                             sizeof(ieee154e_vars.dataReceived->packet),
                                   &ieee154e_vars.dataReceived->l1_rssi,
                                   &ieee154e_vars.dataReceived->l1_lqi,
                                   &ieee154e_vars.dataReceived->l1_crc);

      // break if wrong length
      if (ieee154e_vars.dataReceived->length<LENGTH_CRC || ieee154e_vars.dataReceived->length>LENGTH_IEEE154_MAX ) {
         // jump to the error code below this do-while loop
        openserial_printError(COMPONENT_IEEE802154E,ERR_INVALIDPACKETFROMRADIO,
                            (errorparameter_t)2,
                            ieee154e_vars.dataReceived->length);
         break;
      }

      if (ieee154e_vars.dataReceived->length == 13){
    	  break;
      }

#if 0
      // toss CRC (2 last bytes)
      packetfunctions_tossFooter(   ieee154e_vars.dataReceived, LENGTH_CRC);

      // if CRC doesn't check, stop
      if (ieee154e_vars.dataReceived->l1_crc==FALSE) {
  	    incroute(0x55);
         break;
      }

      // parse the IEEE802.15.4 header (RX DATA)
      // AQUI TAMBEM QUE ELE CHECA A TOPOLOGIA...SE O VIZINHO EH VALIDO...
      // Aqui se for invalido pode ser problema da topologia...entao volto a abrir a janela de RIT...
      ieee802154_retrieveHeader(ieee154e_vars.dataReceived,&ieee802514_header);

      if (ieee802514_header.valid==FALSE) {
   	    incroute(0x56);
        break;
      }

      // store header details in packet buffer
      ieee154e_vars.dataReceived->l2_frameType      = ieee802514_header.frameType;
      ieee154e_vars.dataReceived->l2_dsn            = ieee802514_header.dsn;
      ieee154e_vars.dataReceived->l2_IEListPresent  = ieee802514_header.ieListPresent;
      memcpy(&(ieee154e_vars.dataReceived->l2_nextORpreviousHop),&(ieee802514_header.src),sizeof(open_addr_t));

      // toss the IEEE802.15.4 header
      packetfunctions_tossHeader(ieee154e_vars.dataReceived,ieee802514_header.headerLength);

      // toss the IEs including Synch
      packetfunctions_tossHeader(ieee154e_vars.dataReceived,lenIE);

      // if I just received an invalid frame, stop
      // só eh valido beacon e data packet - ack nao sera valido!!!!!!
      if (isValidRxFrame(&ieee802514_header)==FALSE) {
  	    incroute(0x53);
      	  // jump to the error code below this do-while loop
         break;
      }

       memcpy(&rxaddr_nexthop, &(ieee154e_vars.dataReceived->l2_nextORpreviousHop),sizeof(open_addr_t));
       memcpy(&actualdstaddr, &(ieee802514_header.dest),sizeof(open_addr_t));
       memcpy(&actualsrcaddr, &(ieee802514_header.src),sizeof(open_addr_t));
#endif

		#if 1 // ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
		{
			uint8_t   pos=0, i=0,len;

			pauxframe = (uint8_t *) &ieee154e_vars.dataReceived->packet[0];
			//rffbuf[pos++]= RFF_IEEE802_TX;
			//rffbuf[pos++]= 0x04;
			rffbuf[pos++]= ieee154e_vars.dataReceived->length;

			if (ieee154e_vars.dataReceived->length < 30)
				len = ieee154e_vars.dataReceived->length;
			else
				len = 30;

			for (i=1;i<len;i++){
				rffbuf[pos++]= pauxframe[i];
			}

			//pos = printaddress(actualsrcaddr,rffbuf,pos);
			//pos = printaddress(actualsrcaddr,rffbuf,pos);

			openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);

		}
		#endif

		//descarto o frame recebido pois nao preciso mais dele
		if (ieee154e_vars.dataReceived!=NULL) {
		   openqueue_freePacketBuffer(ieee154e_vars.dataReceived);
		   ieee154e_vars.dataReceived = NULL;
		}

		return;

   } while(0);


   // free the (invalid) received data so RAM memory can be recycled
   if (ieee154e_vars.dataReceived!=NULL) {
	   openqueue_freePacketBuffer(ieee154e_vars.dataReceived);
	   ieee154e_vars.dataReceived = NULL;
   }

   //Reopen RX Window
   deltams = calcdeltatime(RX_RIT_TIMEOUT_MS);

   if (deltams > 0) {
		changeState(S_RIT_RXPROC);

		radio_setFrequency(ieee154e_vars.freq);
		radio_rxEnable();
		radio_rxNow();

		radiotimer_schedule(deltams);
	}
	else
	  endSlot();
}

port_INLINE void RITactivity_rxsniffer_timeout(void) {

	uint8_t nrPending=0;
	uint8_t nrDIOsent=0;

    radiotimer_cancel();

    radio_rfOff();

    #if 0// ((ENABLE_DEBUG_RFF ==1)  && (DBG_RADIO_POWER_CONS == 1))
	{
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_TX;
		rffbuf[pos++]= 0xE3;
		rffbuf[pos++]= livelistretries;
		pos = printaddress(ieee154e_vars.targetaddr,&rffbuf[0],pos);
		rffbuf[pos++]= 0xEE;
		pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
	#endif

   endSlot();

}
#endif






/* Rx Rit procedure - Aqui ele vai enviar o ola e abre a janela do RIT
 *  Calcula Frequencia
 *  prepara o frame do RIT request
 *  carrega o frame
 *  envia ele
 *  Proximo evento eh esperar um tempo para ligar o Rx
 */

port_INLINE void RITactivity_rx00(void) {

	uint32_t random;
	uint32_t rxritperiod;
	uint16_t destaddr;

	RxFlagImprimeRoute = 0;
	//incremento o olaasn...
	olaasn++;
	ieee154e_vars.asn.bytes0and1 = (uint16_t ) olaasn;

    RECORD_RADIO_TX_ON

	changeState(S_RIT_TXOLAPREPARE);

	if (idmanager_getIsDAGroot())
		incroute(0xDA);
	else
		incroute(0x00);

	// o ola eh sempre enviado para o meu melhor canal
	ieee154e_vars.freq = macneighbors_getMyBestChan();

	// configure the radio for that frequency
	radio_setFrequency(ieee154e_vars.freq);

	destaddr = 0xFFFF; //Broadcast
	activityrx_prepareritdatareq(HELLOTYPE_1,&destaddr);

	radio_txEnable();

	RECORD_RADIO_TX_INIT
	RECORD_RADIO_RX_INIT

	//Aqui gera um valor randomico para nao colidir com outro nó comecando no mesmo tempo...variando de 1 a 7ms
    random = macRadioRandomByte() & ((1 << 3) - 1);
	rxritperiod = (RX_RIT_TXOLAPREPARE_MS);  // + random);
	radiotimer_schedule(rxritperiod);


}

port_INLINE void RITactivity_rx01(void) {

  changeState(S_RIT_TXOLA);
  incroute(0x01);

  radiotimer_cancel();

  radio_txNow(TX_USE_CSMA_CA);

  radiotimer_schedule(RX_RIT_TXOLAECHO_MS);

}


/*
 *  aqui esta no inicio do slot RX e nao recebeu OLA ECHO...
 *  Reprograma o timer para a janela do RIT e volta a enviar o pacote.
 *  Problema aqui eh o radio estar travado e ele vai terminar o slot quando o timer estourar a janela.
 */
port_INLINE void RITactivity_rxe01(void) {

	uint32_t deltams;
	uint32_t timeelapsedms;

	radiotimer_cancel();

	radio_rfOff();

	changeState(S_RIT_TXOLAERROR);

	incroute(0xE1);

    RECORD_RADIO_TX_OFF (ieee154e_vars.lastCapturedTime)

	deltams = calcdeltatime(RX_RIT_TIMEOUT_MS);

	if (deltams > 0) {

		#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
		{
			uint8_t   pos=0;

			rffbuf[pos++]= RFF_IEEE802_RX;
			rffbuf[pos++]= 0xE1;
			pos = printvar((uint8_t *)&timeelapsedms,sizeof(uint32_t),rffbuf,pos);
			pos = printvar((uint8_t *)&deltams,sizeof(uint32_t),rffbuf,pos);

			openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
		}
		#endif

		//TODO!!!! ISTO PARECE QUE NAO DEU CERTO...teria de testar com um atraso antes de programar para tx
#if 0
	    //Volto ao estado anterior para indicar que estou novamente esperando um frame TX
		changeState(S_RIT_TXOLA);

		//como desliguei o radio anteriormente devo reprograma-lo para TX
		radio_setFrequency(ieee154e_vars.freq);

		radio_txEnable();

	    radio_txNow(TX_USE_CSMA_CA);

		radiotimer_schedule(deltams);
#else
		endSlot();
#endif

	}
	else{
		endSlot();
	}

}


port_INLINE void RITactivity_rx02(PORT_RADIOTIMER_WIDTH capturedTime) {

	radiotimer_cancel();

	radio_rfOff();

	changeState(S_RIT_RXDATAPREPARE);

    incroute(0x02);

    RECORD_RADIO_TX_OFF (ieee154e_vars.lastCapturedTime)
	//salvo o tempo do TxOla
    radiotxola = capturedTime;

#if 0// ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
{
	uint8_t   pos=0;

	rffbuf[pos++]= RFF_IEEE802_RX;
	rffbuf[pos++]= 0x02;
	pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);

}
#endif

	// recepcao da mensagem é no meu melhor canal
	ieee154e_vars.freq = macneighbors_getMyBestChan();

    // configure the radio for that frequency
    radio_setFrequency(ieee154e_vars.freq);

    // enable the radio in Rx mode. The radio does not actively listen yet.
	radio_rxEnable();

#if 1
	changeState(S_RIT_RXDATA);

	radiotimer_cancel();

	RECORD_RADIO_RX_ON

	radio_rxNow();

	radiotimer_schedule(RX_RIT_TIMEOUT_MS);

#else
	radiotimer_schedule_us(RX_RIT_DELAY_TX2RX_US);
#endif
}



//Abre a janela para recepcao...
port_INLINE void RITactivity_rx03(void) {

	radiotimer_cancel();

    changeState(S_RIT_RXDATA);

    RECORD_RADIO_RX_ON

	incroute(0x03);

    radio_rxNow();

    radiotimer_schedule(RX_RIT_TIMEOUT_MS);

}


/* ACABOU O RIT AQUI E NAO RECEBI NENHUMA MENSAGEM (macRITstate == S_RIT_RX_state)
 * Quando RX significa que abri a janela do RIT e nao teve nenhuma mensagem - caso normal.
 */
port_INLINE void RITactivity_rxe03() {

    radiotimer_cancel();

    // turn off the radio
    radio_rfOff();

	RECORD_RADIO_RX_OFF (ieee154e_vars.lastCapturedTime)

    // change state
    changeState(S_RIT_RXDATATIMEOUT);

	incroute(0xE3);

 	//aqui eu nao recebi nenhuma msg na janela...
	if (macRITstate == S_RIT_TX_state) {
		if (ieee154e_vars.dataToSend != NULL) {
			notif_sendDone(ieee154e_vars.dataToSend,E_SUCCESS);
			ieee154e_vars.dataToSend = NULL;
		}
	}

	endSlot();
}


/*
 * Aqui o frame foi recebido com sucesso...
 * se somente processo um frame por vez...fecho o radio e processo a resposta.
 *
 */
port_INLINE void RITactivity_rx04(PORT_RADIOTIMER_WIDTH capturedTime) {
    ieee802154_header_iht ieee802514_header;
    uint16_t lenIE=0;
//    uint8_t isFrameForMe=0;
    uint8_t discardframe = FALSE;
	open_addr_t rxaddr_nexthop;
//	open_addr_t rxaddr_dst;
//	open_addr_t rxaddr_src;
//	uint8_t elementpos;
	uint8_t   *pauxframe;
	uint32_t lastCapturedTime=0;

	radiotimer_cancel();

	radio_rfOff();

    RECORD_RADIO_RX_OFF (lastCapturedTime)

	changeState(S_RIT_RXNEWDATA);

	incroute(0x04);

   // get a buffer to put the (received) data in
   ieee154e_vars.dataReceived = openqueue_getFreePacketBuffer(COMPONENT_IEEE802154E);

   if (ieee154e_vars.dataReceived==NULL) {
      // log the error
      openserial_printError(COMPONENT_IEEE802154E,ERR_NO_FREE_PACKET_BUFFER,
                            (errorparameter_t)0,
                            (errorparameter_t)0);
      // abort
      endSlot();

      //teste rff - clear msg pendente
      //openqueue_removeAllOwnedBy(COMPONENT_SIXTOP_TO_IEEE802154E);

      return;
   }

   // declare ownership over that packet
   ieee154e_vars.dataReceived->creator = COMPONENT_IEEE802154E;
   ieee154e_vars.dataReceived->owner   = COMPONENT_IEEE802154E;


   /*
   The do-while loop that follows is a little parsing trick.
   Because it contains a while(0) condition, it gets executed only once.
   The behavior is:
   - if a break occurs inside the do{} body, the error code below the loop
     gets executed. This indicates something is wrong with the packet being
     parsed.
   - if a return occurs inside the do{} body, the error code below the loop
     does not get executed. This indicates the received packet is correct.
   */
   do { // this "loop" is only executed once

      // retrieve the received data frame from the radio's Rx buffer
      ieee154e_vars.dataReceived->payload = &(ieee154e_vars.dataReceived->packet[FIRST_FRAME_BYTE]);

      radio_getReceivedFrame(       ieee154e_vars.dataReceived->payload,
                                   &ieee154e_vars.dataReceived->length,
                             sizeof(ieee154e_vars.dataReceived->packet),
                                   &ieee154e_vars.dataReceived->l1_rssi,
                                   &ieee154e_vars.dataReceived->l1_lqi,
                                   &ieee154e_vars.dataReceived->l1_crc);

      // break if wrong length
      if (ieee154e_vars.dataReceived->length<LENGTH_CRC || ieee154e_vars.dataReceived->length>LENGTH_IEEE154_MAX ) {
         // jump to the error code below this do-while loop
        openserial_printError(COMPONENT_IEEE802154E,ERR_INVALIDPACKETFROMRADIO,
                            (errorparameter_t)2,
                            ieee154e_vars.dataReceived->length);
         break;
      }

      // toss CRC (2 last bytes)
      packetfunctions_tossFooter(   ieee154e_vars.dataReceived, LENGTH_CRC);

      // if CRC doesn't check, stop
      if (ieee154e_vars.dataReceived->l1_crc==FALSE) {
  	    incroute(0x45);
         break;
      }

      // parse the IEEE802.15.4 header (RX DATA)
      // AQUI TAMBEM QUE ELE CHECA A TOPOLOGIA...SE O VIZINHO EH VALIDO...
      // Aqui se for invalido pode ser problema da topologia...entao volto a abrir a janela de RIT...
      ieee802154_retrieveHeader(ieee154e_vars.dataReceived,&ieee802514_header);

      if (ieee802514_header.valid==FALSE) {
   	    incroute(0x46);
        break;
      }

      // store header details in packet buffer
      ieee154e_vars.dataReceived->l2_frameType      = ieee802514_header.frameType;
      ieee154e_vars.dataReceived->l2_dsn            = ieee802514_header.dsn;
      ieee154e_vars.dataReceived->l2_IEListPresent  = ieee802514_header.ieListPresent;
      memcpy(&(ieee154e_vars.dataReceived->l2_nextORpreviousHop),&(ieee802514_header.src),sizeof(open_addr_t));

      // toss the IEEE802.15.4 header
      packetfunctions_tossHeader(ieee154e_vars.dataReceived,ieee802514_header.headerLength);

      // toss the IEs including Synch
      packetfunctions_tossHeader(ieee154e_vars.dataReceived,lenIE);

      // if I just received an invalid frame, stop
      // só eh valido beacon e data packet - ack nao sera valido!!!!!!
      if (isValidRxFrame(&ieee802514_header)==FALSE) {
  	    incroute(0x53);
      	  // jump to the error code below this do-while loop
         break;
      }

       memcpy(&rxaddr_nexthop, &(ieee154e_vars.dataReceived->l2_nextORpreviousHop),sizeof(open_addr_t));
       memcpy(&actualdstaddr, &(ieee802514_header.dest),sizeof(open_addr_t));
       memcpy(&actualsrcaddr, &(ieee802514_header.src),sizeof(open_addr_t));

       //------------------------
       // ------ TREAT MESSAGE

#if 0// ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
{
	uint8_t   pos=0;

	pauxframe = (uint8_t *) &ieee154e_vars.dataReceived->packet[0];
	rffbuf[pos++]= RFF_IEEE802_RX;
	rffbuf[pos++]= 0x04;
	//rffbuf[pos++]= ieee154e_vars.dataReceived->l1_rssi;
	rffbuf[pos++]= ieee154e_vars.dataReceived->packet[1];
	rffbuf[pos++]= ieee154e_vars.dataReceived->packet[2];
	rffbuf[pos++]= ieee154e_vars.dataReceived->packet[10];
	pos = printaddress(actualsrcaddr,&rffbuf[0],pos);
	pos = printaddress(actualdstaddr,&rffbuf[0],pos);
	rffbuf[pos++]= 0xEE;
	pos = printvar((uint8_t *)&capturedTime,sizeof(uint32_t),rffbuf,pos);

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);

}
#endif

	   if (ieee802514_header.frameType != IEEE154_TYPE_OLA) {
		   discardframe = RITactivity_rx05(ieee802514_header.ackRequested,ieee802514_header.frameType, &actualdstaddr);
	   }
	   else {// é um CMD...pode ser live list

		   if ((ieee154e_vars.dataReceived->packet[10] == CMDLIVELIST) &&
	  			  (ieee154e_vars.dataReceived->packet[3] == olaasn)  && (idmanager_isMyAddress(&actualdstaddr))) {

                macRIT_Pending_RX_frameType = IANA_ICMPv6_RA_PREFIX_INFORMATION; //only  for debug....
				ritstat.rxola.countdatatxok++;
			#if (ENABLE_LIVELIST_ACK == 1)
				//descobre se o tx programou esperar um ack na livelist...
				if (ieee154e_vars.dataReceived->packet[11] == 0) {
					incroute(0x43);
					discardframe = DISCARD_YES_ENDSLOT_YES;
				}
				else {
					incroute(0x44);
					macRITstate = S_RIT_RX_livelist;
					changeState(S_RIT_LLTXACKOFFSET);
					radiotimer_schedule(TX_RIT_DELAYRXTX_MS);
					discardframe = DISCARD_NO_ENDSLOT_NO;
				}
			#else
				incroute(0x43);
				discardframe = DISCARD_YES_ENDSLOT_YES;
			#endif
		   }
		   else { //é um ola...volta a abrir a janela esperando pelo dado...
			   incroute(0x46);
			   discardframe = activityrx_reopenrxwindow(actualsrcaddr);
		   }
	   }

	   if (discardframe > 0) {
			incroute(0x49);

		   // clean up dataReceived
		   if ((discardframe == DISCARD_YES_ENDSLOT_YES) || (discardframe == DISCARD_YES_ENDSLOT_NO)) {
			   if (ieee154e_vars.dataReceived!=NULL) {
				  openqueue_freePacketBuffer(ieee154e_vars.dataReceived);
				  ieee154e_vars.dataReceived = NULL;
				}
		   }

		   if ((discardframe == DISCARD_YES_ENDSLOT_YES) || (discardframe == DISCARD_NO_ENDSLOT_YES)) {
		      endSlot();
	       }
	  }

	  // everything went well, return here not to execute the error code below
	  return;

   } while(0);


   // free the (invalid) received data so RAM memory can be recycled
   if (ieee154e_vars.dataReceived!=NULL) {
	   openqueue_freePacketBuffer(ieee154e_vars.dataReceived);
	   ieee154e_vars.dataReceived = NULL;
   }

   //Aqui ocorreu erro no frame recebido e volta a escutar a linha para RX.
   //Porem ele pode ficar travado aqui por algum motivo...entao ele somente vai fazer isso por tres tentativas
   nrErrorRetries++;
   if (nrErrorRetries < 5) {
	   incroute(0x49);
	   activityrx_reopenrxwindow(actualsrcaddr);
   }
   else {
	   endSlot();
   }

}


/* RITactivity_rx05 - trata msg recebida diferente de ola
 * return - 0 - nao descarta o frame e nao termina o slot
 *          1 - termina o slot e nao descarta o frame (pois ele ja foi descartado)
 *          2 - termina o slot e descarta o frame
 */

port_INLINE uint8_t RITactivity_rx05 (uint8_t ackRequested,uint8_t frameType, open_addr_t *rxaddr_dst) {
   uint8_t DiscardAndEndSlot = 0;
   uint32_t capturedTime=0;

    incroute(0x05);

	 //checa se frame é para o mote (mesmo endereco destino ou eh um frame broadcast)
	if (frameType == IEEE154_TYPE_UNDEFINED) {
		incroute(0x53);
		DiscardAndEndSlot = activityrx_reopenrxwindow(actualsrcaddr);
	}
	else if (packetfunctions_isBroadcastMulticast(rxaddr_dst)){

		incroute(0x50);

		//AQUI É UM DIO...ENTAO SOMENTO NOTIFICO AS CAMADAS SUPERIORES
		ritstat.rxdio++;

		// indicate reception to upper layer (no ACK asked)
		if (ieee154e_vars.dataReceived!=NULL) {
		   notif_receive(ieee154e_vars.dataReceived);
		   ieee154e_vars.dataReceived = NULL;

		   DiscardAndEndSlot = DISCARD_NO_ENDSLOT_YES;
		}
		else{
		  ritstat.rxola.countdatatxerr++;
		  DiscardAndEndSlot = DISCARD_YES_ENDSLOT_YES;
		}

	}
	else if  (idmanager_isMyAddress(rxaddr_dst)) { //FRAME IS DAO OU COAP

		// check if ack requested
		if (ackRequested==1) {
			incroute(0x51);

    		RITactivity_rx06(capturedTime);
			DiscardAndEndSlot = DISCARD_NO_ENDSLOT_NO;
		}
		else{
			//Aqui ele eh para mim mas nao precisa de ACK...nao sei que frame eh??????
			incroute(0x52);
			DiscardAndEndSlot = DISCARD_YES_ENDSLOT_YES;
		}
	}
	else { //aqui recebeu um frame que nao era ola e nao era para este mote....reprogramo a janela
		//Este caso nao eh para acontecer normalmente
		incroute(0x53);
		DiscardAndEndSlot = activityrx_reopenrxwindow(actualsrcaddr);
	}

	return DiscardAndEndSlot;
}


port_INLINE void RITactivity_rx06(PORT_RADIOTIMER_WIDTH capturedTime) {
   header_IE_ht header_desc;

   radiotimer_cancel();

   // change state
   changeState(S_RIT_TXACKPREPARE);

   incroute(0x06);

   // get a buffer to put the ack to send in
   ieee154e_vars.ackToSend = openqueue_getFreePacketBuffer(COMPONENT_IEEE802154E);

   ieee154e_vars.lastCapturedTime = capturedTime;

   if (ieee154e_vars.ackToSend==NULL) {
      // log the error
      openserial_printError(COMPONENT_IEEE802154E,ERR_NO_FREE_PACKET_BUFFER,
                            (errorparameter_t)0,
                            (errorparameter_t)0);
      // indicate we received a packet anyway (we don't want to loose any)
      notif_receive(ieee154e_vars.dataReceived);

      // free local variable
      ieee154e_vars.dataReceived = NULL;
      // abort
      endSlot();
      return;
   }

   // declare ownership over that packet
   ieee154e_vars.ackToSend->creator = COMPONENT_IEEE802154E;
   ieee154e_vars.ackToSend->owner   = COMPONENT_IEEE802154E;

   // add header IE header -- xv poipoi -- pkt is filled in reverse order..
   packetfunctions_reserveHeaderSize(ieee154e_vars.ackToSend,sizeof(header_IE_ht));
   //create the header for ack IE
   header_desc.length_elementid_type=(sizeof(timecorrection_IE_ht)<< IEEE802154E_DESC_LEN_HEADER_IE_SHIFT)|
                                     (IEEE802154E_ACK_NACK_TIMECORRECTION_ELEMENTID << IEEE802154E_DESC_ELEMENTID_HEADER_IE_SHIFT)|
                                     IEEE802154E_DESC_TYPE_SHORT;
   memcpy(ieee154e_vars.ackToSend->payload,&header_desc,sizeof(header_IE_ht));

   // prepend the IEEE802.15.4 header to the ACK
   ieee154e_vars.ackToSend->l2_frameType = IEEE154_TYPE_ACK;
   ieee154e_vars.ackToSend->l2_dsn       = ieee154e_vars.dataReceived->l2_dsn;
   ieee802154_prependHeader(ieee154e_vars.ackToSend,
                            ieee154e_vars.ackToSend->l2_frameType,
                            IEEE154_IELIST_YES,//ie in ack
                            IEEE154_FRAMEVERSION,//enhanced ack
                            IEEE154_SEC_NO_SECURITY,
                            ieee154e_vars.dataReceived->l2_dsn,
                            &(ieee154e_vars.dataReceived->l2_nextORpreviousHop)
                            );

   // space for 2-byte CRC
   packetfunctions_reserveFooterSize(ieee154e_vars.ackToSend,2);

	//Descubro o melhor canal do src deste frame para enviar o ack
	ieee154e_vars.freq = macneigh_getBChan(&ieee154e_vars.dataReceived->l2_nextORpreviousHop);

   // configure the radio for that frequency
   radio_setFrequency(ieee154e_vars.freq);

   // load the packet in the radio's Tx buffer
   radio_loadPacket(ieee154e_vars.ackToSend->payload,
                    ieee154e_vars.ackToSend->length);

   // enable the radio in Tx mode. This does not send that packet.
   radio_txEnable();

   //aqui eu considero que o radio continua ligado desde a recepcao ainda ate o final do ack
   //ieee154e_vars.radioOnInit=radio_getTimerValue();
   //ieee154e_vars.radioOnThisSlot=TRUE;

   //aqui devo aguardar um tempo para atrasar o tx do rx.
   //quando envio direto o tempo eh da ordem de 3ms. Coloco um atraso de 7 ms para dar 10ms entre Rx e TxAckv(328)
   //radiotimer_schedule(RX_RIT_DELAY_TX_TO_RX_MS); //este valor esta bom...mas quero melhorar...
   radiotimer_schedule_us(RX_RIT_DELAY_TX2RX_US);

}

/*
 * Livelist TX ack...
 * aqui eh usado quando a livelist pede um ack..neste caso o frame sera o mesmo da livelist porem com o comando ack desabilitado.
 *
 */
#if (ENABLE_LIVELIST_ACK == 1)
port_INLINE void RITactivity_rx61(void) {

	uint8_t frame[128];
	sRITelement psEle;

   radiotimer_cancel();

	// change state
	changeState(S_RIT_TXACK);
   incroute(0x81);

	psEle = activityrx_preparemultichannelhello(0,frame,14);
	updatedstaddr(frame,actualsrcaddr);

   // configure the radio for that frequency
   ieee154e_vars.freq = macneighbors_getMyBestChan();
   radio_setFrequency(ieee154e_vars.freq);

   // load the packet in the radio's Tx buffer
   radio_loadPacket(&frame[0],psEle.msglength-2);

#if 0  //((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
{
	uint8_t   pos=0;

	rffbuf[pos++]= RFF_IEEE802_RX;
	rffbuf[pos++]= 0x81;
	rffbuf[pos++]= ieee154e_vars.freq;
	rffbuf[pos++]= ieee154e_vars.targetaddr.type;
	pos = printaddress(actualsrcaddr,&rffbuf[0],pos);

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
}
#endif
    RECORD_RADIO_TX_ON

    // enable the radio in Tx mode. This does not send that packet.
	radio_txEnable();

	//ack does not use CSMA-CA - spec
	radio_txNow(TX_NOT_USE_CSMA_CA);

	radiotimer_schedule(RX_RIT_TXACKEND_MS);
}
#endif

port_INLINE void RITactivity_rx07(void) {

  radiotimer_cancel();

  changeState(S_RIT_TXACK);

  incroute(0x07);

  RECORD_RADIO_TX_ON

  //ack does not use CSMA-CA - spec
  radio_txNow(TX_NOT_USE_CSMA_CA);

  radiotimer_schedule(RX_RIT_TXACKEND_MS);
}

port_INLINE void RITactivity_rxe07() {

	uint32_t deltams;
	uint32_t timeelapsedms;

   incroute(0xE7);

   radiotimer_cancel();

   radio_rfOff();

   changeState(S_RIT_TXACKERROR);

   RECORD_RADIO_TX_OFF(ieee154e_vars.lastCapturedTime)

	//REPROGRAMAR A JANELA COM UM TEMPO MENOR...
	timeelapsedms = ieee154e_vars.lastCapturedTime + RX_RIT_TXACKEND_MS + 10;
	if (RX_RIT_TIMEOUT_MS > timeelapsedms){

		deltams = RX_RIT_TXACKEND_MS + 10;

		#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
		{
			uint8_t   pos=0;

			rffbuf[pos++]= RFF_IEEE802_RX;
			rffbuf[pos++]= 0xE7;
			pos = printvar((uint8_t *)&timeelapsedms,sizeof(uint32_t),rffbuf,pos);
			pos = printvar((uint8_t *)&deltams,sizeof(uint32_t),rffbuf,pos);

			openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);

			openserial_startOutput();
		}
		#endif

		//Volto ao estado anterior para indicar que estou novamente esperando um frame TX
		changeState(S_RIT_TXACK);

		//como desliguei o radio anteriormente devo reprograma-lo para TX
		radio_setFrequency(ieee154e_vars.freq);

		radio_txEnable();

		radio_txNow(TX_NOT_USE_CSMA_CA);

		radiotimer_schedule(deltams);
	}
	else{
		endSlot();
	}
}


port_INLINE void RITactivity_rx08(PORT_RADIOTIMER_WIDTH capturedTime) {
   // change state
   changeState(S_RIT_RXPROC);
   incroute(0x08);

   // cancel rt8
   radiotimer_cancel();

   // turn off the radio
   radio_rfOff();
   RECORD_RADIO_TX_OFF(ieee154e_vars.lastCapturedTime)

#if 0 //((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
{
	uint8_t   pos=0;
	uint8_t   *pauxframe;

	pauxframe = (uint8_t *) &ieee154e_vars.dataReceived->packet[0];

	rffbuf[pos++]= RFF_IEEE802_RX;
	rffbuf[pos++]= 0x08;

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	openserial_startOutput();
}
#endif

	if (macRITstate == S_RIT_RX_state) {
		// free the ack we just sent so corresponding RAM memory can be recycled
		openqueue_freePacketBuffer(ieee154e_vars.ackToSend);

		// clear local variable
		ieee154e_vars.ackToSend = NULL;

		// inform upper layer of reception (after ACK sent)
		notif_receive(ieee154e_vars.dataReceived);

		// clear local variable
		ieee154e_vars.dataReceived = NULL;
   }
   else {
		//descarto o frame recebido pois nao preciso mais dele
		if (macRITstate == S_RIT_RX_livelist)
			ritstat.rxola.countacktxrxok++;

		if (ieee154e_vars.dataReceived!=NULL) {
		   openqueue_freePacketBuffer(ieee154e_vars.dataReceived);
		   ieee154e_vars.dataReceived = NULL;
		}
   }
   // official end of Rx slot
   endSlot();

}

// ###########################################
// TX ROUTINES
// ###########################################
/*
 * Aqui eu estou com msg TX pendente...e acabei de receber um ola
 * Entao:
 *   - Verifico se a msg pendente eh um DIO (Broadcast) e neste caso devo ver se a msg pendente precisa ainda ser enviada
 *   - Verifico se a msg pendente eh um DAO entao devo enviar a msg para o endereco especifico
 *   - Verifico se a msg pendente eh um COAP entao devo enviar a msg para o endereco especifico
 */


port_INLINE void StartTxMultiChannelHello (uint8_t pending) {

	uint8_t frame[128];
	sRITelement psEle;
	uint8_t numTargetParents;
	open_addr_t addrdst;

	changeState(S_RIT_RXOLAOFFSET);

	livelistasn++;
	ritstat.txola.countdatatx++;
	actualrxolaasn = 0x00;

	leds_sync_toggle();

	//montar frame de Multi-Channel Hello - aqui estou forcando o frame ter 120 bytes (106+14 internos)
#if (ENABLE_LIVELIST_ACK == 1)
	psEle = activityrx_preparemultichannelhello(1,frame,127);
#else
	psEle = activityrx_preparemultichannelhello(0,frame,127);
#endif

#if (LIVELIST_SENDFIRSTNEIGH == 1)
	psEle.isBroadcastMulticast = 0;
	numTargetParents = macneighbors_setPendingOnlyFirst(&psEle.destaddr);
#else
	if (pending == 0){
		//descubro quantos vizinhos tenho na minha vizinhanca
		numTargetParents = macneighbors_setBroadCastPending(0);
	}
	else{
		numTargetParents = pending;
	}
#endif

	//coloco elemento na fila do RIT_Tx
    ieee154e_vars.RITQueue_ElementPending = RITQueue_Put(&psEle,0,numTargetParents);

    //preencho a variavel global ritvars
    sRIT_vars = RITQueue_Get_Element(ieee154e_vars.RITQueue_ElementPending);

#if ((ENABLE_DEBUG_RFF) && (DBG_IEEE802_TX == 1))
{
	uint8_t pos=0;
	if (pending == 0) {
		rffbuf[pos++]= 0xAA;
		rffbuf[pos++]= 0xAA;
		rffbuf[pos++]= 0xAA;
	}
	else {
		rffbuf[pos++]= 0xBB;
		rffbuf[pos++]= 0xBB;
		rffbuf[pos++]= 0xBB;
	}
	rffbuf[pos++]= ritstat.txola.countdatatx;
	rffbuf[pos++]= numTargetParents;
	rffbuf[pos++]= pending;
	rffbuf[pos++]= livelistretries;

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);

	openserial_startOutput();
}
#endif


    RITactivity_tx02();
}

#if (TEST_MEASURE_NEIGHBOR_RATE == 1)
port_INLINE void testeTxRITProcedure(void) {

	uint8_t macRIT_Pending_TX_frameType=0;
	//uint32_t dur_rt1;
	//uint32_t macRITTxPeriod;

	changeState(S_RIT_RXOLAOFFSET);
    macRITstate=S_RIT_TX_state;

    testTx++;

	sRIT_vars.destaddr.type = 0x02;
	sRIT_vars.destaddr.addr_64b[0] = 0x00;
	sRIT_vars.destaddr.addr_64b[1] = 0x00;
	sRIT_vars.destaddr.addr_64b[2] = 0x00;
	sRIT_vars.destaddr.addr_64b[3] = 0x00;
	sRIT_vars.destaddr.addr_64b[4] = 0x00;
	sRIT_vars.destaddr.addr_64b[5] = 0x00;
	sRIT_vars.destaddr.addr_64b[6] = 0x51;
	sRIT_vars.destaddr.addr_64b[7] = 0x52;
	ieee154e_vars.freq = 20;
	ieee154e_vars.targetaddr.type = sRIT_vars.destaddr.type;
	ieee154e_vars.targetaddr.addr_64b[0] = sRIT_vars.destaddr.addr_64b[0];

	RITactivity_tx02();

    openserial_startOutput();

}

uint8_t checkframeola(void){
    uint8_t ret=FALSE;

	if ((actualsrcaddr.type == 1) &&
		(actualsrcaddr.addr_16b[1] == MOTE1)) {
       mote1count++;
	}
	else if ((actualsrcaddr.type == 1) &&
			(actualsrcaddr.addr_16b[1] == MOTE2)) {
	       mote2count++;
	}

	//checa se bate com o que eu queria...senao volto a esperar...
	if ((actualsrcaddr.type == 1) &&
		(actualsrcaddr.addr_16b[1] == sRIT_vars.destaddr.addr_64b[7])) {
       moteOKcount++;
       ret = TRUE;
	}
	else {
       moteRetrycount++;
	}

	return ret;
}
#endif


port_INLINE void RITactivity_tx00(uint8_t txpending,uint8_t newtxframe) {

	uint8_t macRIT_Pending_TX_frameType=0;
    uint8_t notifyerror=0;

	changeState(S_RIT_RXOLAOFFSET);

    if (ieee154e_vars.dataToSend->l2_frameType == IEEE154_TYPE_DATA) {
		macRIT_Pending_TX_frameType = RITactivity_tx01(&element,txpending,newtxframe);
	}

    incroute(macRIT_Pending_TX_frameType);

    //TODO!!!!aqui tive problemas de frame invalido...nao sei por que...
    if (ieee154e_vars.dataToSend->l2_retriesLeft > 3)
    {
    	notifyerror = TRUE;
    }
    else if ((macRIT_Pending_TX_frameType > 0) || (txpending))
	{
		RITactivity_tx02();
	}
	else
	{
		notifyerror = TRUE;
	}

    if (notifyerror == TRUE){
    	incroute(0x81);

		if (ieee154e_vars.dataToSend != NULL) {
		  notif_sendDone(ieee154e_vars.dataToSend,E_FAIL);
		  ieee154e_vars.dataToSend = NULL;
     	}
     	endSlot();
    }
}


/*
 * checa qual o tipo de mensagem pendente
 * se existir um frame pendente anterior eu vou tratar ele primeiro
 * TODO!!!! AQUI TEM UMA PENDENCIA - TEM FRAME PENDENTE E TAMBEM FRAME NOVO
 * QUAL TEM PRIORIDADE ????
 * TRATAR O MAIS VELHO...OU SEJA O PENDENTE TEM PRIORIDADE
 * E DEPENDENTE DO TIPO DE FRAME NOVO EU IGNORO ELE...COMO IGNORAR
 * OU SENAO EU COLOCO ELE NA FILA...
 * OUTRA COISA EH QUE NA HORA DE COLOCAR ELE PENDNETE DEVE TER UMA PENDENCIA TAMBEM
 * NAS CAMADAS SUPERIORES...
 * return 0 - msg enviar diretamente
 * return >0 - msg colocar na fila - o valor corresponde ao frametype
 */
port_INLINE uint8_t RITactivity_tx01(sRITelement *pmsgout,uint8_t txpending,uint8_t newtxframe) {

	uint8_t ret=0,i;
	uint8_t testerff=0;
	uint8_t macRIT_Pending_TX_frameType=0;
	uint8_t flagpending=false;
	uint8_t icmpv6_type;
	uint8_t icmpv6_code;
	uint8_t iphc_header1;
	uint8_t iphc_nextheader;
	uint8_t numTargetParents;


	iphc_header1    = *(ieee154e_vars.dataToSend->payload+21);
	iphc_nextheader = *(ieee154e_vars.dataToSend->payload+23);

	if (ieee154e_vars.dataToSend->l4_protocol == IANA_ICMPv6) {
		testerff = 0x11;
	    //preparo elemento para ser colocado na fila do RIT
		pmsgout->msglength =  ieee154e_vars.dataToSend->length;
		pmsgout->timestamp = radio_getTimerValue();
		pmsgout->msg = (uint8_t *) ieee154e_vars.dataToSend->payload;

		if ((ieee154e_vars.dataToSend->l3_destinationAdd.type == ADDR_128B) &&
		   (((ieee154e_vars.dataToSend->l3_destinationAdd.addr_128b[0] == 0xFF) &&
		   (ieee154e_vars.dataToSend->l3_destinationAdd.addr_128b[1] == 0x02)) ||
		   ((ieee154e_vars.dataToSend->l3_destinationAdd.addr_128b[0] == 0xFE) &&
		   (ieee154e_vars.dataToSend->l3_destinationAdd.addr_128b[1] == 0x80))))
		{
			//FRAME DIO NOVO
		    //se existe frame pendente ele tem prioridade sobre um frame DIO
			if (txpending != true)
			{
				macRIT_Pending_TX_frameType = IANA_ICMPv6_RPL_DIO;
				//ritstat.txdio.countdata++;
				pmsgout->frameType = macRIT_Pending_TX_frameType;
				pmsgout->destaddr  = ieee154e_vars.dataToSend->l3_destinationAdd;
				testerff = 0x12;
			}
			else
			{
				//inclui na fila
				testerff = 0x13;
			}

		}
		else if (ieee154e_vars.dataToSend->l4_sourcePortORicmpv6Type == IANA_ICMPv6_RPL)
		{
			testerff = 0x14;
			//FRAME DAO
			macRIT_Pending_TX_frameType = IANA_ICMPv6_RPL_DAO;

            //rit statistics - conta novo produtor
			ritstat.txdao.countdatatx++;

			if (ieee154e_vars.dataToSend->creator == COMPONENT_FORWARDING)
			{
				testerff = 0x15;

				 //salvo o endereco do destino que pode estar na posicao final do frame (RPL Transition)
				 //address_1.type = ieee154e_vars.dataToSend->l3_destinationAdd.type;
				 RITQueue_copyaddress(&address_1,&ieee154e_vars.dataToSend->l3_destinationAdd);

				 if (ieee154e_vars.dataToSend->l3_destinationAdd.type == 3)
				 {
						testerff = 0x16;

					address_1 = ieee154e_vars.dataToSend->l2_nextORpreviousHop;
				 }
			}
			else
			{
				testerff = 0x17;

				address_1 = ieee154e_vars.dataToSend->l2_nextORpreviousHop;

			}

			pmsgout->frameType = macRIT_Pending_TX_frameType;
			pmsgout->destaddr  = address_1;

		}
	}
	else if (ieee154e_vars.dataToSend->l4_protocol == IANA_UDP) {
		testerff = 0x18;
		macRIT_Pending_TX_frameType = IANA_UDP;
		pmsgout->msglength =  ieee154e_vars.dataToSend->length;
		pmsgout->timestamp = radio_getTimerValue();
		pmsgout->frameType = macRIT_Pending_TX_frameType;
		pmsgout->destaddr = ieee154e_vars.dataToSend->l2_nextORpreviousHop;
		pmsgout->msg = (uint8_t *) ieee154e_vars.dataToSend->payload;
		flagpending = true;

        //rit statistics - conta novo produtor
		ritstat.txcoap.countdatatx++;
	}
	else if (ieee154e_vars.dataToSend->l4_protocol == IANA_UNDEFINED) {
		testerff = 0x19;

	    //preparo elemento para ser colocado na fila do RIT
		pmsgout->msglength =  ieee154e_vars.dataToSend->length;
		pmsgout->timestamp = radio_getTimerValue();
		pmsgout->msg = (uint8_t *) ieee154e_vars.dataToSend->payload;

		//algumas informacoes sao extraidas do frame para inferir sobre o seu tipo
		//atencao!!! a posicao destes campos variam de acordo com o tipo do frame.
		icmpv6_type = *(ieee154e_vars.dataToSend->payload+19);
		icmpv6_code = *(ieee154e_vars.dataToSend->payload+20);
		//o iphc_header2 pode variar com valor =0 quando nao eh forwarding e =0x13 quando eh forwarding
		iphc_header1 = *(ieee154e_vars.dataToSend->payload+21);
		iphc_nextheader = *(ieee154e_vars.dataToSend->payload+23);

		// DIO DA BRIDGE ANTIGA
		if ((ieee154e_vars.dataToSend->l3_destinationAdd.type == ADDR_NONE) &&
			(icmpv6_type == IANA_ICMPv6_RPL) &&
			(icmpv6_code == IANA_ICMPv6_RPL_DIO))
		{
			testerff = 0x1A;

			//ritstat.txdio.countdata++;

			//AQUI EH O CASO DA BRIDGE ANTIGA!!!!
			macRIT_Pending_TX_frameType = IANA_ICMPv6_RPL_DIO;
			//macRIT_Pending_TX_frameType = 0;  //send directly
			pmsgout->frameType = macRIT_Pending_TX_frameType;
			pmsgout->destaddr  = ieee154e_vars.dataToSend->l2_nextORpreviousHop;
		}
		else if ((iphc_header1 == 0x78) &&
				((iphc_nextheader == IANA_UDP) || (iphc_nextheader == IANA_IPv6ROUTE)))
		{ //verifica se o IPHC header eh 6LowPAN e o Next eh UDP ou IPV6 Route  (INFERENCIA DO COAP)
			testerff = 0x1B;
			macRIT_Pending_TX_frameType = IANA_UDP;
			pmsgout->frameType = macRIT_Pending_TX_frameType;
			pmsgout->destaddr  = ieee154e_vars.dataToSend->l2_nextORpreviousHop;

            //rit statistics - conta novo produtor
			ritstat.txcoap.countdatatx++;
		}
		else if (ieee154e_vars.dataToSend->creator == COMPONENT_OPENBRIDGE)
		{ //Se eh BRIDGE NOVA..EH O COMANDO COAP..
			testerff = 0x2B;
			macRIT_Pending_TX_frameType = IANA_UDP;
			pmsgout->frameType = macRIT_Pending_TX_frameType;
			pmsgout->destaddr  = ieee154e_vars.dataToSend->l2_nextORpreviousHop;

            //rit statistics - conta novo produtor
			ritstat.txcoap.countdatatx++;
		}
	}
	else if (ieee154e_vars.dataToSend->l4_protocol == IANA_IPv6ROUTE) {
		//AQUI EH QUANDO O COAP ENVIA UM FORWARDING FRAME...
		testerff = 0x1C;

	    //preparo elemento para ser colocado na fila do RIT
		pmsgout->msglength =  ieee154e_vars.dataToSend->length;
		pmsgout->timestamp = radio_getTimerValue();
		pmsgout->msg = (uint8_t *) ieee154e_vars.dataToSend->payload;

		//algumas informacoes sao extraidas do frame para inferir sobre o seu tipo
		//atencao!!! a posicao destes campos variam de acordo com o tipo do frame.
		//o iphc_header2 pode variar com valor =0 quando nao eh forwarding e =0x13 quando eh forwarding
		iphc_header1 = *(ieee154e_vars.dataToSend->payload+21);
		iphc_nextheader = *(ieee154e_vars.dataToSend->payload+23);

		if ((iphc_header1 == 0x78) && ((iphc_nextheader == IANA_UDP) || (iphc_nextheader == IANA_IPv6ROUTE)))
		{ //verifica se o IPHC header eh 6LowPAN e o Next eh UDP ou IPV6 Route  (INFERENCIA DO COAP)
			testerff = 0x1D;
			macRIT_Pending_TX_frameType = IANA_UDP;
			pmsgout->frameType = macRIT_Pending_TX_frameType;
			pmsgout->destaddr  = ieee154e_vars.dataToSend->l2_nextORpreviousHop;

            //rit statistics - conta novo produtor
			ritstat.txcoap.countdatatx++;
		}
	}


	if (macRIT_Pending_TX_frameType > 0)
	{
		pmsgout->isBroadcastMulticast = packetfunctions_isBroadcastMulticast(&ieee154e_vars.dataToSend->l2_nextORpreviousHop);

		if (pmsgout->isBroadcastMulticast){
			//descubro quantos vizinhos tenho na minha vizinhanca
		    numTargetParents = macneighbors_setBroadCastPending(1);
		}
		else{
		    numTargetParents = 1;
		}
		//coloco elemento na fila do RIT_Tx

		ieee154e_vars.RITQueue_ElementPending = RITQueue_Put(pmsgout,flagpending,numTargetParents);
		//preencho a variavel global ritvars
    	sRIT_vars = RITQueue_Get_Element(ieee154e_vars.RITQueue_ElementPending);
	}

	/* Aqui eh o caso onde tenho alguma excessao ou nao tenho nenhum vizinho vivo na livelist
	 * entao desconsidero o frame e notifico falha e termino o slot sem enviar nada.
	 */
    if ((sRIT_vars.destaddr.type == 0) || (numTargetParents == 0)) {
    	incroute(0xF5);
    	macRIT_Pending_TX_frameType = 0;
    }

	#if ((ENABLE_DEBUG_RFF) && (DBG_IEEE802_TX == 1))
	{
		uint8_t pos=0;

		if (testrff_isforwarding){
			testrff_isforwarding = 0;
		}

		rffbuf[pos++]= RFF_IEEE802_TX;
		rffbuf[pos++]= 0x01;
		rffbuf[pos++]= ieee154e_vars.slotOffset;
		rffbuf[pos++]= macRIT_Pending_TX_frameType;
		rffbuf[pos++]= testerff;
		//rffbuf[pos++]= ieee154e_vars.dataToSend->l4_protocol;
		//rffbuf[pos++]= ieee154e_vars.dataToSend->l2_numTxAttempts;
		//rffbuf[pos++]= ieee154e_vars.dataToSend->l2_retriesLeft;
		rffbuf[pos++]= pmsgout->isBroadcastMulticast;
		rffbuf[pos++]= numTargetParents;

		//rffbuf[pos++]= pmsgout->msglength;
		//rffbuf[pos++]= iphc_header1;
		//rffbuf[pos++]= iphc_nextheader;

		pos = printaddress(pmsgout->destaddr,rffbuf,pos);
		pos = printaddress(ieee154e_vars.dataToSend->l3_destinationAdd,rffbuf,pos);
		pos = printaddress(ieee154e_vars.dataToSend->l2_nextORpreviousHop,rffbuf,pos);

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
	#endif


   return (macRIT_Pending_TX_frameType);
}


/*======= TX
* esta rotina eh chamada logo apos o inicio do TxRITProcedure
* ela prepara o radio para recepcao de um ola
*/
port_INLINE void RITactivity_tx02(void) {

	uint8_t isBroadcast=0;
	uint8_t ret;

    changeState(S_RIT_RXOLAPREPARE);

	incroute(0x02);

    radiotimer_cancel();

	// turn off the radio
	radio_rfOff();

    if (sRIT_vars.destaddr.type == 0){
		openserial_printError(COMPONENT_IEEE802154E,ERR_WRONG_RIT_CHANNEL,
							(errorparameter_t)S_RIT_RXOLAPREPARE,0x02);
		endSlot();
		return;
    }

	//Descubro o melhor canal do destino do frame pendente
    isBroadcast = packetfunctions_isBroadcastMulticast(&sRIT_vars.destaddr);
	if (isBroadcast == TRUE){
		if (macRITstate == S_RIT_TX_state)  {
			ret = macneigh_getBChanMultiCast(&ieee154e_vars.freq,&ieee154e_vars.targetaddr, TRUE);
			if (ret == FALSE) {
			   //nao existe nenhum elemento vivo na livelist devo aguardar
			   incroute(0x28);
			   endSlot();
			   return;
			}
		}
		else{ //macRITstate == Livelist
			macneigh_getBChanMultiCast(&ieee154e_vars.freq,&ieee154e_vars.targetaddr, FALSE);
		}
	}
	else{
		ieee154e_vars.targetaddr = sRIT_vars.destaddr;
		ieee154e_vars.freq = macneigh_getBChan(&sRIT_vars.destaddr);
	}

	if ((ieee154e_vars.freq == 0) || (ieee154e_vars.freq > 26)){
		 // jump to the error code below this do-while loop
		openserial_printError(COMPONENT_IEEE802154E,ERR_WRONG_RIT_CHANNEL,
							(errorparameter_t)S_RIT_RXOLAPREPARE,
							ieee154e_vars.freq);
		 endSlot();
		 return;
	}

#if 0 //(DEBUG_LOG_RIT  == 1) && (DBG_IEEE802_TX == 1)
  {
	uint8_t pos=0;

	rffbuf[pos++]= RFF_IEEE802_TX;
	rffbuf[pos++]= 0x02;
	rffbuf[pos++]= isBroadcast;
	rffbuf[pos++]= ieee154e_vars.freq;
	pos = printaddress(ieee154e_vars.targetaddr,&rffbuf[0],pos);
	pos = printaddress(sRIT_vars.destaddr,&rffbuf[0],pos);
	//pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
  }
#endif

   // configure the radio for that frequency
   radio_setFrequency(ieee154e_vars.freq);

   // enable the radio in Rx mode. The radio does not actively listen yet.
	radio_rxEnable();

	//ieee154e_vars.radioOnInit=radio_getTimerValue();
	//ieee154e_vars.radioOnInit=ieee154e_vars.lastCapturedTime;

#if 0 // (ENABLE_CSMA_CA == 1)
  changeState(S_RIT_RXOLA);
  radio_rxNow();
  radiotimer_schedule(TX_RIT_TIMEOUT_MS);
#else
	radiotimer_schedule(TX_RIT_RXOLAPREPARE_MS);
#endif


}


port_INLINE void RITactivity_tx03() {

	radiotimer_cancel();

    changeState(S_RIT_RXOLA);

    RECORD_RADIO_RX_ON

	incroute(0x03);

    radio_rxNow();

	radiotimer_schedule(TX_RIT_TIMEOUT_MS);

#if 0 //((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_TX == 1))
  {
	uint8_t pos=0;

	rffbuf[pos++]= RFF_IEEE802_TX;
	rffbuf[pos++]= 0x03;
	rffbuf[pos++]= ieee154e_vars.freq;
	rffbuf[pos++]= 0xee;
	pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
  }
#endif

}



/* ACABOU O RIT AQUI E NAO RECEBI NENHUMA MENSAGEM (macRITstate == S_RIT_TX_state)
 * Como a janela de Tx é bem grande, é pressuposto que o nó esta ruim
 */

port_INLINE void RITactivity_txe03() {

	uint8_t nrPending=0;
	uint8_t nrDIOsent=0;

    radiotimer_cancel();

	// change state
    changeState(S_RIT_OLATIMEOUT);

    //RFF DEBUG
	incroute(0xE3);

    // turn off the radio
    radio_rfOff();

    leds_error_toggle();  //aqui nao recebi um ola de quem eu estava esperando...

    RECORD_RADIO_RX_OFF (ieee154e_vars.lastCapturedTime )

    #if 1// ((ENABLE_DEBUG_RFF ==1)  && (DBG_RADIO_POWER_CONS == 1))
	{
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_TX;
		rffbuf[pos++]= 0xE3;
		rffbuf[pos++]= livelistretries;
		pos = printaddress(ieee154e_vars.targetaddr,&rffbuf[0],pos);
		rffbuf[pos++]= 0xEE;
		pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
	#endif

	//update statistics
	macneighbors_updtstatistics(ieee154e_vars.targetaddr,sRIT_vars.frameType, E_FAIL);

	// TODO!!!! BUG4 - Aqui eu aguardei o tempo do RIT_TX para uma nó...entao sinalizo para todos que nao esta ok..
	// Isto faz sentido quando somente tenho um canal, mas em multi-canal eu somente escuto poucos ou somente um nó...
	// O ideal era nao falar em erro para todos e tentar nos proximos ciclos...mas por enquanto esta assim mesmo...pois ele no fim
	// cai por qtde de retry e somente sera tentado novamente quando voltar a livelist.
     if (sRIT_vars.frameType == IANA_ICMPv6_RA_PREFIX_INFORMATION) {
#if (BROADCAST_MAX_RETRIES > 0)
    	 livelistretries++;
    	 if (livelistretries > BROADCAST_MAX_RETRIES) {
    		 livelistretries = 0;
    		 macneighbors_clearlivelist(ieee154e_vars.targetaddr);
 			 livelistPending = 0;
    	 }
    	 else {
			livelistPending = RITQueue_getNrPendingParents(ieee154e_vars.RITQueue_ElementPending);
    	 }
#else
 		macneighbors_clearlivelist(ieee154e_vars.targetaddr);
#endif
    }
    else if (sRIT_vars.frameType == IANA_ICMPv6_RPL_DIO)  {
    	//Como o frame DIO eh brodcast ele pode ter ja enviado para alguns nós e falhado outros
  		//se foi enviado para algum nó eu devo indicar as camadas superiores sucesso no envio
        //caso contrario eu envio falha...

  		nrDIOsent = macneighbors_calcbroadcastsent();
  		if (ieee154e_vars.dataToSend != NULL) {
  			if (nrDIOsent > 0) {
  				notif_sendDone(ieee154e_vars.dataToSend,E_SUCCESS);
  			}
  			else{
  				notif_sendDone(ieee154e_vars.dataToSend,E_FAIL);
  			}

  			ieee154e_vars.dataToSend = NULL;
  		}

  		#if 1// ((ENABLE_DEBUG_RFF ==1)  && (DBG_RADIO_POWER_CONS == 1))
  		{
  			uint8_t pos=0;

  			rffbuf[pos++]= RFF_IEEE802_TX;
  			rffbuf[pos++]= 0x84;
  			rffbuf[pos++]= 0x84;
  			rffbuf[pos++]= nrDIOsent;

  			pos = printaddress(ieee154e_vars.targetaddr,&rffbuf[0],pos);
  			pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

  			openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
  		}
  		#endif
    }
    else {// sRIT_vars.frameType != IANA_ICMPv6_RPL_DIO
		incroute(0x83);

		// indicate transmit failed to schedule to keep stats
		schedule_indicateTx(&ieee154e_vars.asn,FALSE);

		// decrement transmits left counter
		ieee154e_vars.dataToSend->l2_retriesLeft--;

		if (ieee154e_vars.dataToSend->l2_retriesLeft==0) {

		  if (ieee154e_vars.dataToSend != NULL) {
			  // indicate tx fail if no more retries left
			  notif_sendDone(ieee154e_vars.dataToSend,E_FAIL);
			  // reset local variable
			  ieee154e_vars.dataToSend = NULL;
		  }
	   }
	   else {
		  // return packet to the virtual COMPONENT_SIXTOP_TO_IEEE802154E component
		  ieee154e_vars.dataToSend->owner = COMPONENT_SIXTOP_TO_IEEE802154E;
	   }
   }

   endSlot();

}


/*
 * Recebi um frame e estava esperando um OLA...
 * verifico se o frame eh um ola...senao volto a escutar a linha...
 *
 */
port_INLINE void RITactivity_tx04(PORT_RADIOTIMER_WIDTH capturedTime) {
    ieee802154_header_iht ieee802514_header;
    uint16_t lenIE=0;
    //uint8_t isFrameForMe=0;
    uint32_t auxlastCapturedTime;
	open_addr_t rxaddr_nexthop;
	open_addr_t rxaddr_dst;
	//open_addr_t rxaddr_src;
	//uint8_t elementpos;
	uint8_t   *pauxframe;
	uint8_t reopenwindow;

	radiotimer_cancel();

	radio_rfOff();

	RECORD_RADIO_RX_OFF (auxlastCapturedTime)

	//Salvo o tempo de espera pelo OLA...
	txwaitola = ieee154e_vars.radioRxOnTics;

	changeState(S_RIT_RXNEWOLA);

	incroute(0x04);

   // get a buffer to put the (received) data in
   ieee154e_vars.dataReceived = openqueue_getFreePacketBuffer(COMPONENT_IEEE802154E);

   if (ieee154e_vars.dataReceived==NULL) {
      // log the error
      openserial_printError(COMPONENT_IEEE802154E,ERR_NO_FREE_PACKET_BUFFER,
                            (errorparameter_t)0,
                            (errorparameter_t)0);
      // abort
      endSlot();

      //teste rff - clear msg pendente
      //openqueue_removeAllOwnedBy(COMPONENT_SIXTOP_TO_IEEE802154E);

      return;
   }

   // declare ownership over that packet
   ieee154e_vars.dataReceived->creator = COMPONENT_IEEE802154E;
   ieee154e_vars.dataReceived->owner   = COMPONENT_IEEE802154E;


   /*
   The do-while loop that follows is a little parsing trick.
   Because it contains a while(0) condition, it gets executed only once.
   The behavior is:
   - if a break occurs inside the do{} body, the error code below the loop
     gets executed. This indicates something is wrong with the packet being
     parsed.
   - if a return occurs inside the do{} body, the error code below the loop
     does not get executed. This indicates the received packet is correct.
   */
   do { // this "loop" is only executed once

      // retrieve the received data frame from the radio's Rx buffer
      ieee154e_vars.dataReceived->payload = &(ieee154e_vars.dataReceived->packet[FIRST_FRAME_BYTE]);

      radio_getReceivedFrame(       ieee154e_vars.dataReceived->payload,
                                   &ieee154e_vars.dataReceived->length,
                             sizeof(ieee154e_vars.dataReceived->packet),
                                   &ieee154e_vars.dataReceived->l1_rssi,
                                   &ieee154e_vars.dataReceived->l1_lqi,
                                   &ieee154e_vars.dataReceived->l1_crc);

      // break if wrong length
      if (ieee154e_vars.dataReceived->length<LENGTH_CRC || ieee154e_vars.dataReceived->length>LENGTH_IEEE154_MAX ) {
         // jump to the error code below this do-while loop
        openserial_printError(COMPONENT_IEEE802154E,ERR_INVALIDPACKETFROMRADIO,
                            (errorparameter_t)2,
                            ieee154e_vars.dataReceived->length);
         break;
      }

      // toss CRC (2 last bytes)
      packetfunctions_tossFooter(   ieee154e_vars.dataReceived, LENGTH_CRC);

      // if CRC doesn't check, stop
      if (ieee154e_vars.dataReceived->l1_crc==FALSE) {
  	    incroute(0x55);
         break;
      }

      // parse the IEEE802.15.4 header (RX DATA)
      // AQUI TAMBEM QUE ELE CHECA A TOPOLOGIA...SE O VIZINHO EH VALIDO...
      // Aqui se for invalido pode ser problema da topologia...entao volto a abrir a janela de RIT...
      ieee802154_retrieveHeader(ieee154e_vars.dataReceived,&ieee802514_header);

      if (ieee802514_header.valid==FALSE) {
   	    incroute(0x56);
        break;
      }

      // store header details in packet buffer
      ieee154e_vars.dataReceived->l2_frameType      = ieee802514_header.frameType;
      ieee154e_vars.dataReceived->l2_dsn            = ieee802514_header.dsn;
      ieee154e_vars.dataReceived->l2_IEListPresent  = ieee802514_header.ieListPresent;
      memcpy(&(ieee154e_vars.dataReceived->l2_nextORpreviousHop),&(ieee802514_header.src),sizeof(open_addr_t));

      // toss the IEEE802.15.4 header
      packetfunctions_tossHeader(ieee154e_vars.dataReceived,ieee802514_header.headerLength);

      // toss the IEs including Synch
      packetfunctions_tossHeader(ieee154e_vars.dataReceived,lenIE);

      // if I just received an invalid frame, stop
      // só eh valido beacon e data packet - ack nao sera valido!!!!!!
      if (isValidRxFrame(&ieee802514_header)==FALSE) {
  	    incroute(0x53);
      	  // jump to the error code below this do-while loop
         break;
      }

       memcpy(&rxaddr_nexthop, &(ieee154e_vars.dataReceived->l2_nextORpreviousHop),sizeof(open_addr_t));
       memcpy(&actualdstaddr, &(ieee802514_header.dest),sizeof(open_addr_t));
       memcpy(&actualsrcaddr, &(ieee802514_header.src),sizeof(open_addr_t));
       actualrxolaasn =  ieee154e_vars.dataReceived->packet[3];

		reopenwindow=0;

#if 0 // ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
{
	uint8_t   pos=0;

		pauxframe = (uint8_t *) &ieee154e_vars.dataReceived->packet[0];
		rffbuf[pos++]= RFF_IEEE802_TX;
		rffbuf[pos++]= 0x04;
		rffbuf[pos++]= pauxframe[0];   //len
		rffbuf[pos++]= pauxframe[1];   //802154.FCF [0]
		rffbuf[pos++]= pauxframe[2];   //802154.FCF [1]
		rffbuf[pos++]= pauxframe[3];   //asn [6]
		pos = printaddress(actualsrcaddr,rffbuf,pos);
		pos = printaddress(actualsrcaddr,rffbuf,pos);

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);

}
#endif

		//--------------------------------------------
		// TREAT TX FRAME
		if (ieee802514_header.frameType == IEEE154_TYPE_OLA) {
			ieee154e_vars.lastCapturedTime = auxlastCapturedTime;

			RITactivity_tx40(&actualsrcaddr);
		}
		else {  //aqui estava esperando um ola mas veio outro frame...devo voltar a esperar um novo frame.
			//TODO!!! AQUI NAO VAI DAR UM DEADLOCK ? O CARA TAMBEM ESTA ESPERANDO UM OLA...SE EU FICAR ESPERANDO ELE NAO VAI VIR...
			//MELHOR TALVEZ TERMINAR E ENVIAR UM OLA PARA ELE...E PERDER A MSG...
			//reopenwindow = TRUE;
			incroute(0x44);
			changeState(S_RIT_TX_CONTINUEWAIT);
			radiotimer_schedule(TX_RIT_DELAYCONTWAIT_MS);
		}

		//descarto o frame recebido pois nao preciso mais dele
		if (ieee154e_vars.dataReceived!=NULL) {
		   openqueue_freePacketBuffer(ieee154e_vars.dataReceived);
		   ieee154e_vars.dataReceived = NULL;
		}

		return;

   } while(0);


#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
{
	uint8_t   pos=0;

		pauxframe = (uint8_t *) &ieee154e_vars.dataReceived->packet[0];
		rffbuf[pos++]= RFF_IEEE802_TX;
		rffbuf[pos++]= 0x49;
		rffbuf[pos++]= pauxframe[1];   //802154.FCF [0]
		rffbuf[pos++]= pauxframe[2];   //802154.FCF [1]
		rffbuf[pos++]= pauxframe[3];   //dest addr [6]
		rffbuf[pos++]= pauxframe[8];   //dest addr [6]
		rffbuf[pos++]= pauxframe[9];   //dest addr [6]

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);

}
#endif
   // free the (invalid) received data so RAM memory can be recycled
   if (ieee154e_vars.dataReceived!=NULL) {
	   openqueue_freePacketBuffer(ieee154e_vars.dataReceived);
	   ieee154e_vars.dataReceived = NULL;
   }

   //Aqui ocorreu erro no frame recebido e volta a escutar a linha para RX.
   //Porem ele pode ficar travado aqui por algum motivo...entao ele somente vai fazer isso por tres tentativas
   nrErrorRetries++;
   if (nrErrorRetries < 5) {
	   incroute(0x49);
		changeState(S_RIT_TX_CONTINUEWAIT);
		radiotimer_schedule(TX_RIT_DELAYCONTWAIT_MS);
   }
   else {
	   // no servico hello foi esperado um ola do mote por 4 tentativas e sempre ocorreu a resposta de outros...
	   // considero entao que ele esta fora da livelist
	   if (macRITstate == S_RIT_multichnhello_state) {
		   macneighbors_clearlivelist(ieee154e_vars.targetaddr);
	   }

	   endSlot();
   }

}

port_INLINE void RITactivity_tx40(open_addr_t* address){
    uint8_t elementpos;

	if (sRIT_vars.isBroadcastMulticast){  //FRAME RPL.DIO ou MULTICHANNELHELLO

		if (macisThisAddressPendingBroadcast(address) == TRUE){
            ritstat.txdio.countdatatx++;
			incroute(0x40);
			changeState(S_RIT_TXDATAOFFSET);
			radiotimer_schedule(TX_RIT_DELAYRXTX_MS);
		}
		else {
			//Chegou msg mas de um endereco que nao vou enviar broadcast...devo aguardar novamente
			incroute(0x41);
        	//ritstat.txdio.countack++;

			changeState(S_RIT_TX_CONTINUEWAIT);
			radiotimer_schedule(TX_RIT_DELAYCONTWAIT_MS);
		}

	}
	else { //frame eh COAP OU RPL.DAO
		elementpos = RITQueue_Get_Pos(address);

		if (elementpos < maxElements)
		{ //OLA EH DA MSG PENDENTE...PREPARO PARA ENVIA-LA
			ieee154e_vars.RITQueue_ElementPending = elementpos;

#if (LIVELIST_FIXED_PACKETS == 1)
			livelistcount++;
#endif

			incroute(0x42);
			changeState(S_RIT_TXDATAOFFSET);
			radiotimer_schedule(TX_RIT_DELAYRXTX_MS);
		}
		else { //AQUI O ENDERECO DO OLA NAO EH O MESMO QUE EU ESTAVA ESPERANDO...VOLTO A ESPERAR...
			incroute(0x43);

			changeState(S_RIT_TX_CONTINUEWAIT);
			radiotimer_schedule(TX_RIT_DELAYCONTWAIT_MS);
		}
	}
}

port_INLINE uint8_t RITactivity_tx45(void) {

    uint8_t ret=FALSE;
    uint8_t nrDIOsent=0;
	uint32_t timeelapsedms;
	uint32_t deltams;

    radiotimer_cancel();

    changeState(S_RIT_TX_CONTINUEWAIT);

    incroute(0x45);

	pvObjList[ieee154e_vars.RITQueue_ElementPending].countretry++;

	deltams = calcdeltatime(TX_RIT_TIMEOUT_MS);

#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TX == 1))
{
	uint8_t pos=0;

	rffbuf[pos++]= RFF_IEEE802_TX;
	rffbuf[pos++]= 0x45;
	pos = printaddress(sRIT_vars.destaddr,rffbuf,pos);
	rffbuf[pos++]= 0xee;
	pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);
	pos = printvar((uint8_t *)&deltams,sizeof(uint32_t),rffbuf,pos);

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
}
#endif

    if (deltams > 0) {
		flagreschedule = TRUE;

		radio_setFrequency(ieee154e_vars.freq);

		radio_rxEnable();

		radio_rxNow();

		//Volto ao estado anterior para indicar que estou novamente esperando um frame TX
		changeState(S_RIT_RXOLA);

		radiotimer_schedule(deltams);
	}
	else {
		incroute(0x46);

		//TODO!!! AQUI ANTES EU TRATAVA O FRAME E ENVIAVA UM NOTIFY PARA INDICAR PARA O RPL OS FRAMES BROADCAST QUE
		//CONSEGUIRAM SER ENVIADOS...MAS ACHO QUE O IDEAL EH FAZER NO PROPRIO RPL TX...ou seja eu notifico falha
		//e se for broadcast ele analisa se houve algum frame que foi ok...ele acessando um servico do neighbors.
		// nrDIOsent = macneighbors_calcbroadcastsent();

		  if (txpending == FALSE)
			  ret = RITQueue_Free(ieee154e_vars.RITQueue_ElementPending);

          if  (macRITstate != S_RIT_multichnhello_state) {

			#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TX == 1))
			{
				uint8_t pos=0;

				rffbuf[pos++]= RFF_IEEE802_TX;
				rffbuf[pos++]= 0x45;
				rffbuf[pos++]= macRIT_Pending_TX_frameType;
				rffbuf[pos++]= ieee154e_vars.dataToSend->l2_retriesLeft;
				rffbuf[pos++]= pvObjList[ieee154e_vars.RITQueue_ElementPending].countretry;
				pos = printaddress(sRIT_vars.destaddr,rffbuf,pos);
				rffbuf[pos++]= 0xee;
				pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);
				pos = printvar((uint8_t *)&deltams,sizeof(uint32_t),rffbuf,pos);
				rffbuf[pos++]= 0xFe;
				rffbuf[pos++]= 0xFe;
				rffbuf[pos++]= 0xFe;
				rffbuf[pos++]= 0xFe;

				openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
			}
			#endif

			//Quando eh frame coap que nao enviei eu deixo ele terminar no endSlot pois vai ter retries
			if (macRIT_Pending_TX_frameType != IANA_UDP) {
				schedule_indicateTx(&ieee154e_vars.asn,TRUE);
				if (ieee154e_vars.dataToSend != NULL) {
					notif_sendDone(ieee154e_vars.dataToSend,E_FAIL);
					ieee154e_vars.dataToSend = NULL;
				}
			}
          }

   		  endSlot();
     }

	return ret;
}


port_INLINE void RITactivity_tx05(PORT_RADIOTIMER_WIDTH capturedTime) {

    radiotimer_cancel();


    changeState(S_RIT_TXDATAREADY);

    incroute(0x05);

	// record the captured time
    ieee154e_vars.lastCapturedTime = getdeltaslotperiod_ms();

	// change owner
	ieee154e_vars.dataToSend->owner = COMPONENT_IEEE802154E;
	// record that I attempt to transmit this packet
	ieee154e_vars.dataToSend->l2_numTxAttempts++;

	//Descubro o melhor canal do destino do frame pendente
	//sRIT_vars = RITQueue_Get_Element(ieee154e_vars.RITQueue_ElementPending);
	//ieee154e_vars.freq = macneigh_getBChan(sRIT_vars.destaddr);

    // configure the radio for that frequency - a frequencia ja estava setada
    radio_setFrequency(ieee154e_vars.freq);

	if (ieee154e_vars.RITQueue_ElementPending < maxElements) {

		//sRIT_vars = RITQueue_Get_Element(ieee154e_vars.RITQueue_ElementPending);

		//Se for livelist...incluo o endereco do DST..o Actualsrc para indicar que eh para ele...
	    if(macRITstate == S_RIT_multichnhello_state){
	    	updatedstaddr(sRIT_vars.msg,actualsrcaddr);
	    	//atualizo o ASN com o numero do ola...
	    	sRIT_vars.msg[2] = actualrxolaasn;

	    }
		radio_loadPacket(sRIT_vars.msg , sRIT_vars.msglength);

		radio_txEnable();
		radiotimer_schedule(TX_RIT_DELAYRXTX_MS);

		#if 0 //((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TX == 1))
		{
			uint8_t   pos=0;

			rffbuf[pos++]= RFF_IEEE802_TX;
			rffbuf[pos++]= 0x05;
			rffbuf[pos++]= ieee154e_vars.freq;
			pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

			openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);

			openserial_startOutput();
		}
		#endif

	}
	else
	{
		endSlot();
	}
}


port_INLINE void RITactivity_tx06(void) {

	radiotimer_cancel();

    changeState(S_RIT_TXDATA);

    incroute(0x06);

    RECORD_RADIO_TX_ON

    radio_txNow(TX_USE_CSMA_CA);

    radiotimer_schedule(TX_RIT_TXDATAECHO_MS);

}


port_INLINE void RITactivity_txe06(void) {

	uint32_t timeelapsedms;
	uint32_t deltams;
	uint8_t nrPending;
	uint8_t endslot=TRUE;

	radiotimer_cancel();

    radio_rfOff();

    RECORD_RADIO_TX_OFF ( ieee154e_vars.lastCapturedTime )

	changeState(S_RIT_TXDATAERROR);

	incroute(0xE6);

	//TODO!!!AQUI EU DEVO VOLTAR A ABRIR A JANELA POIS PODE TER COLIDIDO UM...MAS PODE HAVER MAIS...SENAO VOU PERDER PARA TODOS...
	//para os frames brodcast deve ser enviado um a um...sinalizo que ja enviei um frame...
	if (sRIT_vars.isBroadcastMulticast == TRUE)
	{
		RITQueue_Clear_Pending(ieee154e_vars.RITQueue_ElementPending,actualsrcaddr);

		nrPending = RITQueue_getNrPendingParents(ieee154e_vars.RITQueue_ElementPending);

		if (nrPending > 0) {

			changeState(S_RIT_RXOLAOFFSET);
			radiotimer_schedule(TX_RIT_DELAYCONTWAIT_MS);

		    #if ((ENABLE_DEBUG_RFF == 1)  && (DBG_IEEE802_TX == 1))
			{
				uint8_t pos=0;

				rffbuf[pos++]= RFF_IEEE802_TX;
				rffbuf[pos++]= 0xE6;
				rffbuf[pos++]= nrPending;

				pos = printaddress(actualsrcaddr,&rffbuf[0],pos);
				pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

				openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);

				//openserial_startOutput();
			}
			#endif

			endslot = FALSE;
		}
	}
	else{ //AQUI EH UM DAO OU COAP...NESTE CASO DEVO VOLTAR A RETRANSMITIR NOVAMENTE
		//REPROGRAMAR A JANELA
		timeelapsedms = ieee154e_vars.lastCapturedTime + TX_RIT_TXDATAECHO_MS + 10;
		if (TX_RIT_TIMEOUT_MS > timeelapsedms){

			deltams = TX_RIT_TXDATAECHO_MS + 10;

			#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
			{
				uint8_t   pos=0;

				rffbuf[pos++]= RFF_IEEE802_RX;
				rffbuf[pos++]= 0xE7;
				pos = printvar((uint8_t *)&timeelapsedms,sizeof(uint32_t),rffbuf,pos);
				pos = printvar((uint8_t *)&deltams,sizeof(uint32_t),rffbuf,pos);

				openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
			}
			#endif

		    //Volto ao estado anterior para indicar que estou novamente esperando um frame TX
			changeState(S_RIT_TXDATA);

			//como desliguei o radio anteriormente devo reprograma-lo para TX
			radio_setFrequency(ieee154e_vars.freq);

			radio_txEnable();

		    radio_txNow(TX_USE_CSMA_CA);

			radiotimer_schedule(deltams);

			endslot = FALSE;
		}
		else{
			endslot = TRUE;
		}

	}

	if (endslot == TRUE) {
		if (txpending == FALSE){
			  RITQueue_Free(ieee154e_vars.RITQueue_ElementPending);
		}

		if (ieee154e_vars.dataToSend != NULL) {
		  schedule_indicateTx(&ieee154e_vars.asn,TRUE);

		  notif_sendDone(ieee154e_vars.dataToSend,E_FAIL);

		  ieee154e_vars.dataToSend = NULL;
		}

		endSlot();
	}

}

/*
 * Aqui quando Tx significa que o frame foi enviado com sucesso (acontece apos ter recebido o echo do final do frame)
 */
port_INLINE void RITactivity_tx07(PORT_RADIOTIMER_WIDTH capturedTime) {
    //bool listenForAck;
	//open_addr_t address;
	uint8_t nrPending=0;
	uint8_t ret=0;
	uint8_t endslot=TRUE;
	uint8_t reenviobroadcast=0;

    radiotimer_cancel();

    changeState(S_RIT_RXACKOFFSET);

	incroute(0x07);

    radio_rfOff();

    RECORD_RADIO_TX_OFF ( ieee154e_vars.lastCapturedTime )

	if (sRIT_vars.frameType == IANA_ICMPv6_RA_PREFIX_INFORMATION)  {  //MULTI CHANNEL HELLO CMD
		// notifica o recebimento na tabela de vizinhos
		macneighbors_updtlivelist(ieee154e_vars.targetaddr);

		ritstat.txola.countdatatxok++;

#if (ENABLE_LIVELIST_ACK == 1)
	   RITactivity_tx08();
	   //radiotimer_schedule(RX_RIT_DELAY_TX2RX_MS);
	   endslot = FALSE;
#else
		//update statistics
		macneighbors_updtstatistics(ieee154e_vars.targetaddr,sRIT_vars.frameType, E_SUCCESS);

		RITQueue_Clear_Pending(ieee154e_vars.RITQueue_ElementPending,actualsrcaddr);

		nrPending = RITQueue_getNrPendingParents(ieee154e_vars.RITQueue_ElementPending);

		if (nrPending > 0) {

			incroute(0x72);

			changeState(S_RIT_RXOLAOFFSET);
			radiotimer_schedule(TX_RIT_DELAYCONTWAIT_MS);

		   endslot = FALSE;
		}

		#if 1// ((ENABLE_DEBUG_RFF == 1)  && (DBG_RADIO_POWER_CONS == 1))
		{
			uint8_t *pucAux = (uint8_t *) &ieee154e_vars.dataToSend;
			uint8_t pos=0;

			rffbuf[pos++]= RFF_IEEE802_TX;
			rffbuf[pos++]= 0x72;
			pos = printaddress(actualsrcaddr,&rffbuf[0],pos);
			pos = printaddress(ieee154e_vars.targetaddr,&rffbuf[0],pos);

			rffbuf[pos++]= nrPending;

			openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);

			//openserial_startOutput();
		}
		#endif
#endif

	}
	else if (sRIT_vars.frameType == IANA_ICMPv6_RPL_DIO) {

	    //QDO O RPL DIO -> CHECAR SE JA ENVIOU DIO PARA TODOS OS VIZINHOS
		//SENAO...REPROGRAMAR E TORNAR A ESPERAR UM NOVO OLA...ATE TERMINAR OS VIZINHOS OU A JANELA DE RIT_TX.
		RITQueue_Clear_Pending(ieee154e_vars.RITQueue_ElementPending,actualsrcaddr);
        ritstat.txdio.countdatatxok++;
		//update statistics
		macneighbors_updtstatistics(ieee154e_vars.targetaddr,sRIT_vars.frameType, E_SUCCESS);

		nrPending = RITQueue_getNrPendingParents(ieee154e_vars.RITQueue_ElementPending);

		if (nrPending > 0) {

			incroute(0x72);

			changeState(S_RIT_RXOLAOFFSET);
			radiotimer_schedule(TX_RIT_DELAYCONTWAIT_MS);

			#if ((ENABLE_DEBUG_RFF == 1)  && (RFF_IEEE802_TX == 1))
			{
				uint8_t *pucAux = (uint8_t *) &ieee154e_vars.dataToSend;
				uint8_t pos=0;

				rffbuf[pos++]= RFF_IEEE802_TX;
				rffbuf[pos++]= 0x72;
				rffbuf[pos++]= nrPending;
				pos = printaddress(ieee154e_vars.targetaddr,&rffbuf[0],pos);

				openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);

				openserial_startOutput();
			}
			#endif
            endslot = FALSE;
		}

	}
	else  { //frameType  == (DAO OU COAP)
	   if (ieee154e_vars.dataToSend != NULL)
	   {
		   // decides whether to listen for an ACK
		   //if (packetfunctions_isBroadcastMulticast(&ieee154e_vars.dataToSend->l2_nextORpreviousHop) !=TRUE) {
		   if (sRIT_vars.isBroadcastMulticast == FALSE) {

               incroute(0x71);

               //estatisticas
               if (sRIT_vars.frameType == IANA_UDP) {
                   ritstat.txcoap.countdatatxok++;
               }

			   RITactivity_tx08();
			   endslot = FALSE;
		   }
	   }
	}

	if (endslot == TRUE) {

		if (txpending == FALSE){
		  RITQueue_Free(ieee154e_vars.RITQueue_ElementPending);
		}

		if (macRITstate != S_RIT_multichnhello_state) {

			#if 1 //TESTE RFF 250716
				//TODO!!! PROVISORIO!!! Aqui foi implementado uma forma de enviar um notify no DIO para o RPL porem ele espera que o frame nao seja
				//broadcast e tambem tenho o endereco no formato 64bits. Entao é convertido o endereco para este formato e enviado como nao se fosse
				//broadcast pois no RIT o broadcast eh por vizinho e desta forma aumentara o nro de frames recebidos pelo nó.
				//TODO!!! Aqui tambem pode ter mais de um vizinho que foi enviado o broadcast e aqui somente esta notificando o primeiro vizinho
			    macneighbors_getaddrbroadcastsent(&ieee154e_vars.dataToSend->l2_nextORpreviousHop);
			#endif

			if (ieee154e_vars.dataToSend != NULL) {
				notif_sendDone(ieee154e_vars.dataToSend,E_SUCCESS);
				ieee154e_vars.dataToSend = NULL;
			}
		}

		endSlot();
	}


}




/*
 * Aqui quando eh Tx e eu preciso de um ack...abro o radio como RX esperando o ack.
 */

port_INLINE void RITactivity_tx08(void) {

	radio_rfOff();

   // change state
   changeState(S_RIT_RXACKPREPARE);

   incroute(0x08);

   //ieee154e_vars.lastCapturedTime = capturedTime;

	// o Ack eh enviado no meu melhor canal
	ieee154e_vars.freq = macneighbors_getMyBestChan();

   // configure the radio for that frequency
   radio_setFrequency(ieee154e_vars.freq);

   // enable the radio in Rx mode. The radio is not actively listening yet.
   radio_rxEnable();

//aqui eu nao sei por que nao consigo dar um atraso para comecar a receber o ack...se eu atrasar 100us ele nao recebe o ack...
//porem se chamar direto ele funciona
#if 1
   // change state
   changeState(S_RIT_RXACK);
   RECORD_RADIO_RX_ON
   radio_rxNow();
   radiotimer_schedule(TX_RIT_ACK_TIMEOUT_MS);
#else
   //radiotimer_schedule_us(RX_RIT_DELAY_TX2RX_US);
   //radiotimer_schedule(RX_RIT_DELAY_TX_TO_RX_MS);
#endif
}

port_INLINE void RITactivity_tx81(void) {

   // change state
   changeState(S_RIT_RXACK);

   incroute(0x81);

   RECORD_RADIO_RX_ON

   radio_rxNow();

   radiotimer_schedule(TX_RIT_ACK_TIMEOUT_MS);

	#if 1 // ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TIMER == 1))
	{
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_TX;
		rffbuf[pos++]= 0x81;

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
	#endif

}

port_INLINE void RITactivity_txe08() {

   radiotimer_cancel();

   // turn off the radio
   radio_rfOff();

	RECORD_RADIO_RX_OFF(ieee154e_vars.lastCapturedTime)

   // change state
   changeState(S_RIT_RXNOACK);

	incroute(0xE8);

   //update statistics
   if (ieee154e_vars.RITQueue_ElementPending < maxElements) {
		//decrementa elemento pendente...para broadcast
		RITQueue_update_element(ieee154e_vars.RITQueue_ElementPending);

		//sRIT_vars = RITQueue_Get_Element(ieee154e_vars.RITQueue_ElementPending);

	    macneighbors_updtstatistics(ieee154e_vars.targetaddr,sRIT_vars.frameType, E_FAIL);
   }


#if 1 //((ENABLE_DEBUG_RFF ==1)  && (DBG_RADIO_POWER_CONS == 1))
	{
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_TX;
		rffbuf[pos++]= 0xE8;

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
#endif

	// SINALIZO ERRO E GUARDO A MENSAGEM PARA TENTAR NO PROXIMO CICLO...activity_tie5(); (copie aqui embaixo)
    if (ieee154e_vars.dataToSend != NULL) {
		// indicate transmit failed to schedule to keep stats
		schedule_indicateTx(&ieee154e_vars.asn,FALSE);

		// decrement transmits left counter
		ieee154e_vars.dataToSend->l2_retriesLeft--;

		if (ieee154e_vars.dataToSend->l2_retriesLeft==0) {
		  // indicate tx fail if no more retries left
		  notif_sendDone(ieee154e_vars.dataToSend,E_FAIL);
		} else {
		  // return packet to the virtual COMPONENT_SIXTOP_TO_IEEE802154E component
		  ieee154e_vars.dataToSend->owner = COMPONENT_SIXTOP_TO_IEEE802154E;
		}

		// reset local variable
		ieee154e_vars.dataToSend = NULL;
    }
	// abort
    endSlot();
}

/*
 * Aqui indica que quando TX eu estava esperando um ack e ele chegou.
 * aviso as camadas de cima do sucesso.
 */
port_INLINE void RITactivity_tx09(PORT_RADIOTIMER_WIDTH capturedTime) {

   ieee802154_header_iht     ieee802514_header;
   open_addr_t auxdstaddr;
   open_addr_t auxsrcaddr;
   uint8_t flagerr=0;

   // change state
   changeState(S_RIT_TXPROC);

   // cancel tt8
   radiotimer_cancel();

   // turn off the radio
   radio_rfOff();

   incroute(0x09);

   RECORD_RADIO_RX_OFF(ieee154e_vars.lastCapturedTime)

   // get a buffer to put the (received) ACK in
   ieee154e_vars.ackReceived = openqueue_getFreePacketBuffer(COMPONENT_IEEE802154E);
   if (ieee154e_vars.ackReceived==NULL) {
      // log the error
      openserial_printError(COMPONENT_IEEE802154E,ERR_NO_FREE_PACKET_BUFFER,
                            (errorparameter_t)0,
                            (errorparameter_t)0);
      // abort
      endSlot();
      return;
   }

   // declare ownership over that packet
   ieee154e_vars.ackReceived->creator = COMPONENT_IEEE802154E;
   ieee154e_vars.ackReceived->owner   = COMPONENT_IEEE802154E;

   /*
   The do-while loop that follows is a little parsing trick.
   Because it contains a while(0) condition, it gets executed only once.
   Below the do-while loop is some code to cleans up the ack variable.
   Anywhere in the do-while loop, a break statement can be called to jump to
   the clean up code early. If the loop ends without a break, the received
   packet was correct. If it got aborted early (through a break), the packet
   was faulty.
   */
   do { // this "loop" is only executed once

      // retrieve the received ack frame from the radio's Rx buffer
      ieee154e_vars.ackReceived->payload = &(ieee154e_vars.ackReceived->packet[FIRST_FRAME_BYTE]);
      radio_getReceivedFrame(       ieee154e_vars.ackReceived->payload,
                                   &ieee154e_vars.ackReceived->length,
                             sizeof(ieee154e_vars.ackReceived->packet),
                                   &ieee154e_vars.ackReceived->l1_rssi,
                                   &ieee154e_vars.ackReceived->l1_lqi,
                                   &ieee154e_vars.ackReceived->l1_crc);

      // break if wrong length
      if (ieee154e_vars.ackReceived->length<LENGTH_CRC || ieee154e_vars.ackReceived->length>LENGTH_IEEE154_MAX) {
         // break from the do-while loop and execute the clean-up code below
        openserial_printError(COMPONENT_IEEE802154E,ERR_INVALIDPACKETFROMRADIO,
                            (errorparameter_t)1,
                            ieee154e_vars.ackReceived->length);
         flagerr = TRUE;
         break;
      }

#if 0 //((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_TX == 1))
  {
	uint8_t   pos=0;

	if (ieee154e_vars.RITQueue_ElementPending < maxElements)
	{
		sRIT_vars = RITQueue_Get_Element(ieee154e_vars.RITQueue_ElementPending);

		rffbuf[pos++]= RFF_IEEE802_TX;
		rffbuf[pos++]= 0x09;
		rffbuf[pos++]= ieee154e_vars.ackReceived->packet[1];
		rffbuf[pos++]= ieee154e_vars.ackReceived->packet[2];
		rffbuf[pos++]= ieee154e_vars.ackReceived->packet[3];
		rffbuf[pos++]= ieee154e_vars.ackReceived->packet[6];
		rffbuf[pos++]= ieee154e_vars.ackReceived->packet[7];
		rffbuf[pos++]= ieee154e_vars.ackReceived->packet[8];
		rffbuf[pos++]= ieee154e_vars.ackReceived->packet[9];
		rffbuf[pos++]= ieee154e_vars.ackReceived->packet[10];

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
  }
#endif
      // toss CRC (2 last bytes)
      packetfunctions_tossFooter(   ieee154e_vars.ackReceived, LENGTH_CRC);

      // break if invalid CRC
      if (ieee154e_vars.ackReceived->l1_crc==FALSE) {
         // break from the do-while loop and execute the clean-up code below
         flagerr = TRUE;
         break;
      }

      // parse the IEEE802.15.4 header (RX ACK)
      ieee802154_retrieveHeader(ieee154e_vars.ackReceived,&ieee802514_header);

		#if 0
			  // break if invalid IEEE802.15.4 header
			  if (ieee802514_header.valid==FALSE) {
				 // break from the do-while loop and execute the clean-up code below
				 flagerr = TRUE;
				 break;
			  }
		#endif

      // store header details in packet buffer
      ieee154e_vars.ackReceived->l2_frameType  = ieee802514_header.frameType;
      ieee154e_vars.ackReceived->l2_dsn        = ieee802514_header.dsn;
      memcpy(&(ieee154e_vars.ackReceived->l2_nextORpreviousHop),&(ieee802514_header.src),sizeof(open_addr_t));
      memcpy(&auxdstaddr, &(ieee802514_header.dest),sizeof(open_addr_t));
      memcpy(&auxsrcaddr, &(ieee802514_header.src),sizeof(open_addr_t));

	   macneighbors_updtstatistics(ieee154e_vars.targetaddr,sRIT_vars.frameType, E_SUCCESS);

		if ((ieee802514_header.frameType == IEEE154_TYPE_CMD) &&
			(ieee154e_vars.ackReceived->packet[10] == CMDLIVELIST) &&
			(idmanager_isMyAddress(&auxdstaddr))) {
			incroute(0x91);
			ritstat.txola.countacktxrxok++;
		}
		else{
			incroute(0x92);
			   //libera areas alocadas
		       RITQueue_Free(ieee154e_vars.RITQueue_ElementPending);

		       if (ieee154e_vars.dataToSend != NULL) {
				  // inform schedule of successful transmission
				  schedule_indicateTx(&ieee154e_vars.asn,TRUE);

				  // inform upper layer
				  notif_sendDone(ieee154e_vars.dataToSend,E_SUCCESS);
				  ieee154e_vars.dataToSend = NULL;
		       }
		}


      // in any case, execute the clean-up code below (processing of ACK done)
   } while (0);

   if (flagerr) {
	   incroute(0x99);
	   //update statistics
	   macneighbors_updtstatistics(ieee154e_vars.targetaddr,sRIT_vars.frameType, E_FAIL);

       RITQueue_Free(ieee154e_vars.RITQueue_ElementPending);
   }

   // free the received ack so corresponding RAM memory can be recycled
   openqueue_freePacketBuffer(ieee154e_vars.ackReceived);

   // clear local variable
   ieee154e_vars.ackReceived = NULL;

   // official end of Tx slot
   endSlot();
}



//======= frame validity check

/**
\brief Decides whether the packet I just received is valid received frame.

A valid Rx frame satisfies the following constraints: (PARA O RIT)  TODO!!! PARA O AMAC OU RITMC DEVE ACEITAR O ACK TAMBEM
- its IEEE802.15.4 header is well formatted
- it's a DATA or COMMAND (BEACON DO RIT)
- it's sent on the same PANid as mine
- it's for me (unicast or broadcast)

\param[in] ieee802514_header IEEE802.15.4 header of the packet I just received

\returns TRUE if packet is valid received frame, FALSE otherwise
*/
port_INLINE bool isValidRxFrame(ieee802154_header_iht* ieee802514_header) {


   return ieee802514_header->valid==TRUE                                                           && \
          (
             ieee802514_header->frameType==IEEE154_TYPE_DATA                   ||
             ieee802514_header->frameType==IEEE154_TYPE_CMD
          )                                                                                        && \
          packetfunctions_sameAddress(&ieee802514_header->panid,idmanager_getMyID(ADDR_PANID))     && \
          (
             idmanager_isMyAddress(&ieee802514_header->dest)                   ||
             packetfunctions_isBroadcastMulticast(&ieee802514_header->dest)
          );
}

/**
\brief Decides whether the packet I just received is a valid ACK.

A packet is a valid ACK if it satisfies the following conditions:
- the IEEE802.15.4 header is valid
- the frame type is 'ACK'
- the sequence number in the ACK matches the sequence number of the packet sent
- the ACK contains my PANid
- the packet is unicast to me
- the packet comes from the neighbor I sent the data to

\param[in] ieee802514_header IEEE802.15.4 header of the packet I just received
\param[in] packetSent points to the packet I just sent

\returns TRUE if packet is a valid ACK, FALSE otherwise.
*/
port_INLINE bool isValidAck(ieee802154_header_iht* ieee802514_header, OpenQueueEntry_t* packetSent) {
   /*
   return ieee802514_header->valid==TRUE                                                           && \
          ieee802514_header->frameType==IEEE154_TYPE_ACK                                           && \
          ieee802514_header->dsn==packetSent->l2_dsn                                               && \
          packetfunctions_sameAddress(&ieee802514_header->panid,idmanager_getMyID(ADDR_PANID))     && \
          idmanager_isMyAddress(&ieee802514_header->dest)                                          && \
          packetfunctions_sameAddress(&ieee802514_header->src,&packetSent->l2_nextORpreviousHop);
   */
   // poipoi don't check for seq num
   return ieee802514_header->valid==TRUE                                                           && \
          ieee802514_header->frameType==IEEE154_TYPE_ACK                                           && \
          packetfunctions_sameAddress(&ieee802514_header->panid,idmanager_getMyID(ADDR_PANID))     && \
          idmanager_isMyAddress(&ieee802514_header->dest)                                          && \
          packetfunctions_sameAddress(&ieee802514_header->src,&packetSent->l2_nextORpreviousHop);
}

//======= ASN handling

port_INLINE void incrementAsnOffset() {
   // increment the asn
   ieee154e_vars.asn.bytes0and1++;
   if (ieee154e_vars.asn.bytes0and1==0) {
      ieee154e_vars.asn.bytes2and3++;
      if (ieee154e_vars.asn.bytes2and3==0) {
         ieee154e_vars.asn.byte4++;
      }
   }
   // increment the offsets
   ieee154e_vars.slotOffset  = (ieee154e_vars.slotOffset+1)%schedule_getFrameLength();
   ieee154e_vars.asnOffset   = (ieee154e_vars.asnOffset+1)%16;
}

//from upper layer that want to send the ASN to compute timing or latency
port_INLINE void ieee154e_getAsn(uint8_t* array) {
   array[0]         = (ieee154e_vars.asn.bytes0and1     & 0xff);
   array[1]         = (ieee154e_vars.asn.bytes0and1/256 & 0xff);
   array[2]         = (ieee154e_vars.asn.bytes2and3     & 0xff);
   array[3]         = (ieee154e_vars.asn.bytes2and3/256 & 0xff);
   array[4]         =  ieee154e_vars.asn.byte4;
}

port_INLINE void joinPriorityStoreFromAdv(uint8_t jp){
  ieee154e_vars.dataReceived->l2_joinPriority = jp;
  ieee154e_vars.dataReceived->l2_joinPriorityPresent = TRUE;
}


port_INLINE void asnStoreFromAdv(uint8_t* asn) {

   // store the ASN
   ieee154e_vars.asn.bytes0and1   =     asn[0]+
                                    256*asn[1];
   ieee154e_vars.asn.bytes2and3   =     asn[2]+
                                    256*asn[3];
   ieee154e_vars.asn.byte4        =     asn[4];

   // determine the current slotOffset
   /*
   Note: this is a bit of a hack. Normally, slotOffset=ASN%slotlength. But since
   the ADV is exchanged in slot 0, we know that we're currently at slotOffset==0
   */
   ieee154e_vars.slotOffset       = 0;
   schedule_syncSlotOffset(ieee154e_vars.slotOffset);
   ieee154e_vars.nextActiveSlotOffset = schedule_getNextActiveSlotOffset();

   /*
   infer the asnOffset based on the fact that
   ieee154e_vars.freq = 11 + (asnOffset + channelOffset)%16
   */
   ieee154e_vars.asnOffset = ieee154e_vars.freq - 11 - schedule_getChannelOffset();
}

//======= synchronization
void changeIsSync(bool newIsSync) {
   ieee154e_vars.isSync = newIsSync;

   if (ieee154e_vars.isSync==TRUE) {
      resetStats();
   } else {
      schedule_resetBackoff();
   }
}

//
/*======= notifying upper layer
 * Agora considero no calculdo do Rank as mensagens do DIO..que antes nao era considerada por causa de ser broadcast...
 * Como eh necessario enviar um broadcast para cada mote da minha tabela...isso ajuda a aumentar o numero de pacotes enviados...
 * Para isso eu engano o RPL colocando o endereco do Broadcast, o endereco do endereco que foi enviado.
 */
void notif_sendDone(OpenQueueEntry_t* packetSent, owerror_t error) {

   // record the outcome of the trasmission attempt
   packetSent->l2_sendDoneError   = error;

   // record the current ASN
   memcpy(&packetSent->l2_asn,&ieee154e_vars.asn,sizeof(asn_t));
   // associate this packet with the virtual component
   // COMPONENT_IEEE802154E_TO_RES so RES can knows it's for it
   packetSent->owner              = COMPONENT_IEEE802154E_TO_SIXTOP;

   // post RES's sendDone task
   scheduler_push_task(task_sixtopNotifSendDone,TASKPRIO_SIXTOP_NOTIF_TXDONE);
   // wake up the scheduler
   SCHEDULER_WAKEUP();
}

void notif_receive(OpenQueueEntry_t* packetReceived) {
   // record the current ASN
   memcpy(&packetReceived->l2_asn, &ieee154e_vars.asn, sizeof(asn_t));
   // indicate reception to the schedule, to keep statistics
   schedule_indicateRx(&packetReceived->l2_asn);
   // associate this packet with the virtual component
   // COMPONENT_IEEE802154E_TO_SIXTOP so sixtop can knows it's for it
   packetReceived->owner          = COMPONENT_IEEE802154E_TO_SIXTOP;

   // post RES's Receive task
   scheduler_push_task(task_sixtopNotifReceive,TASKPRIO_SIXTOP_NOTIF_RX);
   // wake up the scheduler
   SCHEDULER_WAKEUP();
}

//======= stats

port_INLINE void resetStats() {
   ieee154e_stats.numSyncPkt      =    0;
   ieee154e_stats.numSyncAck      =    0;
   ieee154e_stats.minCorrection   =  127;
   ieee154e_stats.maxCorrection   = -127;
   ieee154e_stats.numTicsOn       =    0;
   ieee154e_stats.numTicsTotal    =    0;
   // do not reset the number of de-synchronizations
}

void updateStats(PORT_SIGNED_INT_WIDTH timeCorrection) {
   // update minCorrection
   if (timeCorrection<ieee154e_stats.minCorrection) {
     ieee154e_stats.minCorrection = timeCorrection;
   }
   // update maxConnection
   if(timeCorrection>ieee154e_stats.maxCorrection) {
     ieee154e_stats.maxCorrection = timeCorrection;
   }
}

//======= misc

/**
\brief Calculates the frequency channel to transmit on, based on the
absolute slot number and the channel offset of the requested slot.

During normal operation, the frequency used is a function of the
channelOffset indicating in the schedule, and of the ASN of the
slot. This ensures channel hopping, consecutive packets sent in the same slot
in the schedule are done on a difference frequency channel.

During development, you can force single channel operation by having this
function return a constant channel number (between 11 and 26). This allows you
to use a single-channel sniffer; but you can not schedule two links on two
different channel offsets in the same slot.

\param[in] channelOffset channel offset for the current slot

\returns The calculated frequency channel, an integer between 11 and 26.
*/
port_INLINE uint8_t calculateFrequency(uint8_t channelOffset) {
   // comment the following line out to disable channel hopping
   return SYNCHRONIZING_CHANNEL; // single channel
   //return 11+(ieee154e_vars.asnOffset+channelOffset)%16; //channel hopping
}

/**
\brief Changes the state of the IEEE802.15.4e FSM.

Besides simply updating the state global variable,
this function toggles the FSM debug pin.

\param[in] newstate The state the IEEE802.15.4e FSM is now in.
*/
void changeState(ieee154e_state_t newstate) {
   // update the state
   ieee154e_vars.state = newstate;
}

/**
\brief Housekeeping tasks to do at the end of each slot.

This functions is called once in each slot, when there is nothing more
to do. This might be when an error occured, or when everything went well.
This function resets the state of the FSM so it is ready for the next slot.

Note that by the time this function is called, any received packet should already
have been sent to the upper layer. Similarly, in a Tx slot, the sendDone
function should already have been done. If this is not the case, this function
will do that for you, but assume that something went wrong.
*/
void endSlot() {

	// change state
	changeState(S_RIT_SLEEP);

	radiotimer_cancel();

    // turn off the radio
    radio_rfOff();

    radio_flushfifos();

	// record the captured time
    ieee154e_vars.lastCapturedTime = getdeltaslotperiod_ms();   //getdeltaTimerValue();

	#if (SINK == 1)
		//TODO CICLO ABRO A SERIAL PARA ENTRADA
		openserial_stop();
		openserial_startInput();
	#endif

#if (RIT_RX_SNIFFER == 0)
	//imprimir estatisticas e debugs
	treatdebug(macRITstate);
	clearritroute();

	//Limpo a fila de mensagens pendentes
	RITQueue_cleanupoldmsg();
	sRIT_vars = RITQueue_Get_Element(ieee154e_vars.RITQueue_ElementPending);
#endif

   //clear vars for duty cycle on this slot

	RECORD_RADIO_TX_CLEAR
	RECORD_RADIO_RX_CLEAR

	frameDioPending = 0;
    macRITstate = 0;

   // clean up dataToSend
   if (ieee154e_vars.dataToSend!=NULL) {

	   //Aqui quando eu estou com erro eu nao retransmito o pacote...espero a proxima vez
		if (sRIT_vars.frameType == IANA_ICMPv6_RPL_DIO)
		{
			  if (txpending == FALSE){
				  RITQueue_Free(ieee154e_vars.RITQueue_ElementPending);
			  }

			  schedule_indicateTx(&ieee154e_vars.asn,TRUE);
			  notif_sendDone(ieee154e_vars.dataToSend,E_SUCCESS);
		}
		else {

		  // if everything went well, dataToSend was set to NULL in ti9
		  // getting here means transmit failed

		  // indicate Tx fail to schedule to update stats
		  schedule_indicateTx(&ieee154e_vars.asn,FALSE);

		  //decrement transmits left counter
		  ieee154e_vars.dataToSend->l2_retriesLeft--;

			#if 0
				  if (ieee154e_vars.dataToSend->l2_retriesLeft==0) {
					 // indicate tx fail if no more retries left
					 notif_sendDone(ieee154e_vars.dataToSend,E_FAIL);
				  } else {
					 // return packet to the virtual COMPONENT_SIXTOP_TO_IEEE802154E component
					 ieee154e_vars.dataToSend->owner = COMPONENT_SIXTOP_TO_IEEE802154E;
				  }
			#else
				  ieee154e_vars.dataToSend->owner = COMPONENT_SIXTOP_TO_IEEE802154E;
			#endif
		}

		ieee154e_vars.dataToSend = NULL;
   }


   // clean up dataReceived
   if (ieee154e_vars.dataReceived!=NULL) {
      // assume something went wrong. If everything went well, dataReceived
      // would have been set to NULL in ri9.
      // indicate  "received packet" to upper layer since we don't want to loose packets
      notif_receive(ieee154e_vars.dataReceived);

      // reset local variable
      ieee154e_vars.dataReceived = NULL;
   }

   // clean up ackToSend
   if (ieee154e_vars.ackToSend!=NULL) {
      // free ackToSend so corresponding RAM memory can be recycled
      openqueue_freePacketBuffer(ieee154e_vars.ackToSend);
      // reset local variable
      ieee154e_vars.ackToSend = NULL;
   }

   // clean up ackReceived
   if (ieee154e_vars.ackReceived!=NULL) {
      // free ackReceived so corresponding RAM memory can be recycled
      openqueue_freePacketBuffer(ieee154e_vars.ackReceived);
      // reset local variable
      ieee154e_vars.ackReceived = NULL;
   }

   openserial_startOutput();
}


bool ieee154e_isSynch(){
   return ieee154e_vars.isSync;
}
/**
\brief Indicates the FSM timer has fired.

This function executes in ISR mode, when the FSM timer fires.
*/
void isr_ieee154e_timer(void) {

   switch (ieee154e_vars.state) {
//RX
   case S_RIT_TXOLAPREPARE:
		  RITactivity_rx01();
		 break;
	  case S_RIT_TXOLA:
		  RITactivity_rxe01();
		 break;
	  case S_RIT_RXDATAPREPARE:
		 RITactivity_rx03();
	   	 break;
	  case S_RIT_RXDATA:
		  RITactivity_rxe03();
		  break;
      case S_RIT_TXACKPREPARE:
    	  RITactivity_rx07();
         break;
      case S_RIT_TXACK:
    	 RITactivity_rxe07();
         break;
#if (ENABLE_LIVELIST_ACK == 1)
	  case S_RIT_LLTXACKOFFSET:
		  RITactivity_rx61();
		 break;
#endif

//TX
  	  case S_RIT_RXOLAOFFSET:
  		 RITactivity_tx02();
  		 break;
  	  case S_RIT_RXOLAPREPARE:
		 RITactivity_tx03();
		 break;
#if (RIT_RX_SNIFFER == 1)
	  case S_RIT_RXPROC:
		  RITactivity_rxsniffer_timeout();
 		  break;
#endif
 	  case S_RIT_RXOLA:
 		  RITactivity_txe03();
 		  break;
 	  case S_RIT_TXDATAOFFSET:
		  RITactivity_tx05(0);
		  break;
 	  case S_RIT_TX_CONTINUEWAIT:
 		  RITactivity_tx45();
		  break;
	  case S_RIT_TXDATAREADY:
	     RITactivity_tx06();
		 break;
	  case S_RIT_TXDATA:
		 RITactivity_txe06();
		 break;
	  case S_RIT_RXACKOFFSET:
		 RITactivity_tx08();
		 break;
	  case S_RIT_RXACKPREPARE:
		 RITactivity_tx81();
		 break;
	  case S_RIT_RXACK:
		 RITactivity_txe08();
		 break;

	  default:
		 // log the error
		 openserial_printError(COMPONENT_IEEE802154E,ERR_WRONG_STATE_IN_TIMERFIRES,
							   (errorparameter_t)ieee154e_vars.state,
							   (errorparameter_t)ieee154e_vars.slotOffset);
		 // abort
		 endSlot();
		 break;
   }

   ieee154e_dbg.num_timer++;
}

/**
\brief Indicates the radio just received the first byte of a packet.

This function executes in ISR mode.
*/


void ieee154e_startOfFrame(PORT_RADIOTIMER_WIDTH capturedTime) {
   ieee154e_dbg.num_startOfFrame++;
}

/**
\brief Indicates the radio just received the last byte of a packet.

This function executes in ISR mode.
*/
void ieee154e_endOfFrame(PORT_RADIOTIMER_WIDTH capturedTime) {

	capturedTime = getdeltaslotperiod_ms();

    switch (ieee154e_vars.state) {
     case S_RIT_TXOLA:
		 RITactivity_rx02(capturedTime);
		 break;
	 case S_RIT_RXDATA:
		 RITactivity_rx04(capturedTime);
		break;
	 case S_RIT_TXACK:
		 RITactivity_rx08(capturedTime);
		break;
//TX
	 case S_RIT_RXOLA:
		 RITactivity_tx04(capturedTime);
		break;
#if (RIT_RX_SNIFFER == 1)
	  case S_RIT_RXPROC:
		 RITactivity_rxsniffer_newdata(capturedTime);
		 break;
#endif

	 case S_RIT_TXDATA:
		 RITactivity_tx07(capturedTime);
		break;
	 case S_RIT_RXACK:
		 RITactivity_tx09(capturedTime);
		break;
	 default:
		// log the error
		openserial_printError(COMPONENT_IEEE802154E,ERR_WRONG_STATE_IN_ENDOFFRAME,
							  (errorparameter_t)ieee154e_vars.state,
							  (errorparameter_t)macRITstate /*ieee154e_vars.slotOffset */);
		// abort
		endSlot();
		break;
   }

   ieee154e_dbg.num_endOfFrame++;
}

#endif // (IEEE802154E_RIT == 1)

