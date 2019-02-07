/*
 * Protocolo ARM da camada MAC baseado no mecanismo RIT assincrono
 * Proposto por:
 * Esta implementacao é uma versao com os principais componentes baseado no artigo 2010.
 * A base do artigo é um mecanismo com OLA, RTS e CTS para envio do dado entre transmissor e receptor
 * O mecanismo de multicanal é baseado no canal de controle CC para publicacao do OLA e a MSG tem o melhor canal do vizinho.
 */

#include "leds.h"
#include "IEEE802154E.h"
#if (IEEE802154E_ARM == 1)
#include "IEEE802154ARM.h"
#include "leds.h"
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

#define IGNORE_CW 0



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

uint8_t livelistPending;
uint8_t livelistretries;
extern uint8_t livelistasn;

extern uint8_t olaasn;

uint16_t counttxrts;
uint16_t countrxrts;
uint16_t counttxcts;
uint16_t countrxcts;

uint32_t radiotxdutycycle1;
uint32_t radiotxdutycycle2;
uint32_t radiorxdutycycle1;
uint32_t radiorxdutycycle2;
uint32_t radiotxola;
extern uint32_t txwaitola;

uint8_t macRIT_Pending_TX_frameType;  //PROVISORIO..>TESTE ONLY
uint8_t macRIT_Pending_RX_frameType;  //PROVISORIO..>TESTE ONLY
uint8_t nrErrorRetries;
uint8_t COAPSimCount;
uint8_t RxFlagImprimeRoute = 0;
uint8_t rx_cwstate = 0;

uint32_t tempo1;
uint32_t livelistcount;
uint8_t livelistfirsttime;

uint8_t isrmultichannelhello;
uint32_t slottimeref;
uint8_t flagSerialTx;
uint8_t lastslotwastx;
uint8_t testrff_isforwarding;
uint8_t ucFlagTxReOpen;
uint8_t lastRITstatewastx;

open_addr_t        actualsrcaddr;         //usado quando recebe um frame...para identificar quem enviou...
uint8_t            actualbestchannel;     //este bestchannel eh obtido do ultimo ola.
open_addr_t        actualdstaddr;
uint8_t            actualolaasn;

ieee154e_vars_t    ieee154e_vars;
sRITqueue          sRIT_vars;

ieee154e_stats_t   ieee154e_stats;
extern RIT_stats_t ritstat;
extern uint8_t maxElements;

open_addr_t        address_1;
uint8_t            flagreschedule=0;

bool                txpending=FALSE;
OpenQueueEntry_t  sPendingDataPacket;

uint8_t macRITstate;

static uint16_t macRITDataWaitDuration;
static uint16_t macRITRXforTxPeriod;
static uint16_t macAckWaitPeriod;
volatile uint32_t actualtimer0;

#if 1 //ENABLE_DEBUG_RFF
#define DBG_802154E_TX_DATA 1
#define DBG_802154E_RX_DATA 0
uint8_t rffnewsendmsg;
#define RFF_LOG_DEBUG_DAO 1
#endif

extern OpenQueueEntry_t advRIT;

#define DEBUG_ACK  0
uint8_t MsgNeedAck;

//=========================== prototypes ======================================
void ieee154e_init(void);
// interrupts
void     isr_ieee154e_newSlot(void);
void     isr_ieee154e_timer(void);
void     isr_multichannelhello(void);

//Packetfunctions
port_INLINE uint8_t activityrx_prepareritdatareq(uint8_t hello, uint16_t *destaddr);
port_INLINE void updatedstaddr(uint8_t* packet, open_addr_t  address);
port_INLINE sRITelement activityrx_preparemultichannelhello(uint8_t hellospec, uint8_t *frame,uint8_t len);

// SYNCHRONIZING
void     activity_synchronize_startOfFrame(PORT_RADIOTIMER_WIDTH capturedTime);
void     activity_synchronize_endOfFrame(PORT_RADIOTIMER_WIDTH capturedTime);

void     activity_ti1ORri1(void);
void     activity_rxwindowend(void);
void     activity_ritrxlistening(void);
void     activity_rxnewframe(PORT_RADIOTIMER_WIDTH capturedTime);

port_INLINE uint8_t checkframeok(uint8_t frametype, OpenQueueEntry_t*  dtReceived,ieee802154_header_iht *ieee802514_header);
port_INLINE uint8_t activityrx_reopenrxwindow(void);
bool RITQueue_ExistFramePending(void);
port_INLINE void ARMactivity_preparedata(PORT_RADIOTIMER_WIDTH capturedTime);
owerror_t openqueue_freePacketRITBuffer(OpenQueueEntry_t* pkt);


// TX
port_INLINE void StartTxMultiChannelHello (uint8_t pending);
port_INLINE void ARMactivity_tx00(uint8_t elepending, uint8_t newtxframe);
port_INLINE void ARMactivity_txe01(void);
port_INLINE void ARMactivity_tx02(void);
port_INLINE void ARMactivity_tx03(void);
port_INLINE uint8_t ARMactivity_tx45(void);
port_INLINE void ARMactivity_tx05(void);
port_INLINE void ARMactivity_tx06(PORT_RADIOTIMER_WIDTH capturedTime);
port_INLINE void ARMactivity_tx09(void);
port_INLINE void ARMactivity_tx0a(PORT_RADIOTIMER_WIDTH capturedTime);
port_INLINE void ARMactivity_tx0b(void);
port_INLINE void ARMactivity_tx0c(PORT_RADIOTIMER_WIDTH capturedTime);
port_INLINE void ARMactivity_txe03(void);
port_INLINE void ARMactivity_txe0b(void);

//RX
port_INLINE void ARMactivity_rx00(void);
port_INLINE void ARMactivity_rx01(void);
port_INLINE void ARMactivity_rx03(void);
port_INLINE uint8_t ARMactivity_rx05(void);
port_INLINE void ARMactivity_rx08(PORT_RADIOTIMER_WIDTH capturedTime);
port_INLINE void ARMactivity_rx09(void);
port_INLINE void ARMactivity_rx0a(void);
port_INLINE void ARMactivity_rx0b(PORT_RADIOTIMER_WIDTH capturedTime);
port_INLINE void ARMactivity_rxe01(void);
port_INLINE void ARMactivity_rxe0a(void);
port_INLINE void ARMactivity_rxe03(void);
port_INLINE void ARMactivity_rxe05(void);
port_INLINE uint8_t ARMactivity_rx85 (uint8_t ackRequested,uint8_t frameType, open_addr_t *rxaddr_dst);

uint32_t calcdeltatime(uint32_t timebase);
uint8_t toogleTxRxSerial(void);

owerror_t sixtop_send_internal(OpenQueueEntry_t* msg,uint8_t iePresent, uint8_t frameVersion);


// frame validity check
bool     isValidRxFrame(ieee802154_header_iht* ieee802514_header);
bool     isValidAck(ieee802154_header_iht*     ieee802514_header,
                    OpenQueueEntry_t*          packetSent);
// IEs Handling
bool     ieee154e_processIEs(OpenQueueEntry_t* pkt, uint16_t* lenIE);
void     ieee154e_processSlotframeLinkIE(OpenQueueEntry_t* pkt,uint8_t * ptr);
// ASN handling
void     incrementAsnOffset(void);
//void     asnStoreFromAdv(uint8_t* asn);
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

port_INLINE uint8_t checkframeok(uint8_t frametype, OpenQueueEntry_t*  dtReceived,ieee802154_header_iht *ieee802514_header){

    uint16_t lenIE=0;

	dtReceived->creator = COMPONENT_IEEE802154E;
	dtReceived->owner   = COMPONENT_IEEE802154E;

	dtReceived->payload = &(dtReceived->packet[FIRST_FRAME_BYTE]);

	radio_getReceivedFrame(    dtReceived->payload,
							   &dtReceived->length,
						 sizeof(dtReceived->packet),
							   &dtReceived->l1_rssi,
							   &dtReceived->l1_lqi,
							   &dtReceived->l1_crc);

	// break if wrong length
	if (dtReceived->length<LENGTH_CRC || dtReceived->length>LENGTH_IEEE154_MAX ) {
		openserial_printError(COMPONENT_IEEE802154E,ERR_INVALIDPACKETFROMRADIO,
							 (errorparameter_t)1,
							  dtReceived->length);
		return(FALSE);
	}

	// toss CRC (2 last bytes)
	packetfunctions_tossFooter(   dtReceived, LENGTH_CRC);

	// if CRC doesn't check, stop
	if (dtReceived->l1_crc==FALSE) {
	   incroute(0x45);
	   return(FALSE);
	}

	// parse the IEEE802.15.4 header (RX DATA)
	// AQUI TAMBEM QUE ELE CHECA A TOPOLOGIA...SE O VIZINHO EH VALIDO...
	ieee802154_retrieveHeader(dtReceived,ieee802514_header);

	if (frametype != IEEE154_TYPE_ACK){
		if (ieee802514_header->valid==FALSE) {
			incroute(0x46);
			return(FALSE);
		}
	}

	// store header details in packet buffer
	dtReceived->l2_frameType      = ieee802514_header->frameType;
	dtReceived->l2_dsn            = ieee802514_header->dsn;
	dtReceived->l2_IEListPresent  = ieee802514_header->ieListPresent;

	//checa se frame eh de livelist
    if (ieee802514_header->frameType == IEEE154_TYPE_CMD) {
    	return (TRUE);
    }

	memcpy(&(dtReceived->l2_nextORpreviousHop),&(ieee802514_header->src),sizeof(open_addr_t));

	if (frametype != IEEE154_TYPE_ACK){
		// toss the IEEE802.15.4 header
		packetfunctions_tossHeader(dtReceived,ieee802514_header->headerLength);

		// toss the IEs including Synch
		packetfunctions_tossHeader(dtReceived,lenIE);

		// if I just received an invalid frame, stop
		// só eh valido beacon e data packet - ack nao sera valido!!!!!!
		if (isValidRxFrame(ieee802514_header)==FALSE) {
			incroute(0x43);
		   return(FALSE);
		}
	}

	//memcpy(&rxaddr_nexthop, &(ieee154e_vars.dataReceived->l2_nextORpreviousHop),sizeof(open_addr_t));
	//memcpy(&rxaddr_dst, &(ieee802514_header.dest),sizeof(open_addr_t));
	//memcpy(&actualsrcaddr, &(ieee802514_header.src),sizeof(open_addr_t));
	return TRUE;
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
 

  flagSerialTx = 0;
   flagreschedule = 0;
   isrmultichannelhello = 0;
   macRITstate = 0;
   //sRIT_vars.livelistasn=0;
   livelistasn = 0;
   livelistPending = 0;
   olaasn=0;

   // switch radio on
   radio_rfOn();

	//set the period constant
	macAckWaitPeriod       = TICK_RIT_ACK_WAIT_PERIOD;
	macRITRXforTxPeriod    = TICK_MAC_RIT_RX_TO_TX_PERIOD;
	macRITDataWaitDuration = TICK_MAC_RIT_RX_WIND_PERIOD;
#if (ENABLE_CSMA_CA == 0)
	macRITPeriod           = TICK_MAC_RIT_PERIOD;
	macRITTxPeriod         = TICK_MAC_RIT_TX_PERIOD;

    macRITsleepPeriod = (uint16_t) ((uint16_t)macRITPeriod-(uint16_t)macRITDataWaitDuration);
#endif
	//initiate RIT Queue
	RITQueue_Init();
#if  (ENABLE_DEBUG_RFF ==1)
	clearstatistics();
#endif
	livelistfirsttime = 0;
	livelistcount=0;
	testrff_isforwarding = 0;
	lastRITstatewastx = 0;

	counttxcts=0;
	counttxrts=0;
	countrxcts=0;
	countrxrts=0;

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

   opentimers_start(ARM_MULTICHANNELHELLO_MS, TIMER_PERIODIC, TIME_MS, (opentimers_cbt) isr_multichannelhello);

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

/*
 * checa se estou esperando por mensagens deste endereço
 *
 */
uint8_t chkwaitthisaddress(uint8_t frameType, uint8_t isBroadcastMulticast, open_addr_t* address){

    uint8_t ret=FALSE;
	uint8_t elementpos;

	if (isBroadcastMulticast){
		if (macisThisAddressPendingBroadcast(address) == TRUE){
			ret = TRUE;
		}
	}
	else {
		elementpos = RITQueue_Get_Pos(address);

		if (elementpos < maxElements)
		{
			ret = TRUE;
		}
	}
    return ret;
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

#if 1
  activity_ti1ORri1();
#else
  //openserial_stop();
  //openserial_startOutput();

   radio_setTimerPeriod(RX_ARM_PERIOD_MS);
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
 //  uint8_t elePending;
   bool newtxframe=FALSE;
 //  uint32_t macRITPeriod;
 //  uint32_t macRITTxPeriod;
   slotOffset_t targetSlotOffset;
   volatile uint8_t	random;
   volatile uint32_t	rxritperiod;

   slotvarsInitialization();

	ieee154e_dbg.num_txslot++;

   radiotimer_cancel();

    //verifico se existe mensagem pendente para ser enviada
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

#if (ARM_RX_ONLY == 0)
   if ((txpending == TRUE) || (ieee154e_vars.dataToSend != NULL)) {   // I have a packet to send
		macRITstate=S_RIT_TX_state;
		ARMactivity_tx00(txpending,newtxframe);

		radio_setTimerPeriod(TX_ARM_PERIOD_MS);
        lastRITstatewastx = TRUE;
  }
#if  (BROADCAST_MAX_RETRIES > 0)
   else if ((lastRITstatewastx == FALSE) && (livelistPending > 0)) {
		macRITstate = S_RIT_multichnhello_state;
		StartTxMultiChannelHello(livelistPending);

		//Quando Livelist e nao eh mais a primeira vez...entao aumento o tempo de espera...
		if (livelistretries > 0) {
			radio_setTimerPeriod(TX_ARM_PERIOD_MS*1.5);
		}
		else{
			radio_setTimerPeriod(TX_ARM_PERIOD_MS);
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
			radio_setTimerPeriod(TX_ARM_PERIOD_MS);
			lastRITstatewastx = TRUE;
		}
		else {
			macRITstate=S_RIT_RX_state;
			ieee154e_dbg.num_rxslot++;

			openserial_stop();
			openserial_startOutput();

			ARMactivity_rx00();

			radio_setTimerPeriod(RX_ARM_PERIOD_MS);
			lastRITstatewastx = FALSE;
		}
#else
		StartTxMultiChannelHello(0);
		radio_setTimerPeriod(TX_ARM_PERIOD_MS);
		lastRITstatewastx = TRUE;
#endif

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
			  ARMactivity_rx00();
		  }
		  else {
			  openserial_startInput();
			  ARMactivity_rx00();
		  }
		#else
	      openserial_stop();
		  openserial_startOutput();
		  ARMactivity_rx00();
		#endif

		random = 1; //macRadioRandomByte() & ((1 << 4) - 1);
		rxritperiod = (RX_ARM_PERIOD_MS + random);
		radio_setTimerPeriod(rxritperiod);

		lastRITstatewastx = FALSE;

  }
#else
    openserial_stop();
    openserial_startOutput();
    ARMactivity_rx00();

	radio_setTimerPeriod(RX_ARM_PERIOD_MS);
#endif

  ieee154e_vars.slotOffset = ieee154e_vars.nextActiveSlotOffset;
//  targetSlotOffset = targetSlotOffset;
}

/*
 * Esta rotina é utilizada para reprogramar o timer e voltar a ouvir na janela do RX_RIT
 * Neste caso será atualizado a livelist com este ola recebido
 * e sera reprogramado a maquina de estado para esperar um novo frame ainda dentro da janela do ola.
 *
 */
port_INLINE uint8_t activityrx_reopenrxwindow(void) {
   uint32_t deltams=0;

#if (MULTI_CHANNEL_HELLO_ENABLE == 0)
   // notifica o recebimento na tabela de vizinhos
   macneighbors_updtlivelist(actualsrcaddr);
#endif

    //REPROGRAMAR A JANELA COM UM TEMPO MENOR...
   deltams = calcdeltatime(RX_ARM_RTSTIMEOUT_MS);

    if (deltams > 0){

		incroute(0x47);

		//como desliguei o radio anteriormente devo reprograma-lo para RX
		ieee154e_vars.freq = macneighbors_getMyBestChan();

		radio_setFrequency(ieee154e_vars.freq);

		RECORD_RADIO_RX_ON

		radio_rxEnable();

		radio_rxNow();

		//Volto ao estado anterior para indicar que estou novamente esperando um OLA ACK
		changeState(S_ARM_RXRTS);

		radiotimer_schedule(deltams);

		return DISCARD_YES_ENDSLOT_NO;
    }
    else
    {
		return DISCARD_YES_ENDSLOT_YES;
    }


}

/*
// Aqui eh o caso onde ja enviei o rit e ja programei o timer
// Este start eh do proprio RIT entao devo somente aguardar o estouro do timer...
void activity_RITDoNothing(void){
	changeState(S_ARM_RXOLAREADY);
	macRITstate=S_RIT_RX_state;
}
*/

/* Rx Rit procedure - Aqui ele vai enviar o ola e abre a janela do RIT
 *  Calcula Frequencia
 *  prepara o frame do RIT request
 *  carrega o frame
 *  envia ele
 *  Proximo evento eh esperar um tempo para ligar o Rx
 */

port_INLINE void ARMactivity_rx00(void) {

	uint32_t random;
	uint32_t rxritperiod;
	uint16_t destaddr;

	RxFlagImprimeRoute = 0;

	//incremento o olaasn...
	olaasn++;
	RECORD_RADIO_TX_INIT
	RECORD_RADIO_RX_INIT

	changeState(S_ARM_TXOLAPREPARE);

	if (idmanager_getIsDAGroot())
		incroute(0xDA);
	else
		incroute(0x00);

	// o ola eh sempre enviado para o canal de controle
	ieee154e_vars.freq = macneighbors_getControlChan();

	// configure the radio for that frequency
	radio_setFrequency(ieee154e_vars.freq);

	destaddr = 0xFFFF; //Broadcast
	activityrx_prepareritdatareq(HELLOTYPE_1,&destaddr);

	radio_txEnable();

    random = macRadioRandomByte() & ((1 << 3) - 1);
	rxritperiod = (RX_ARM_TXOLAPREPARE_MS);  // + random);
	radiotimer_schedule(rxritperiod);

	//avoid warning
//    random = random;
}

port_INLINE void ARMactivity_rx01(void) {

  changeState(S_ARM_TXOLA);

  incroute(0x01);

  radiotimer_cancel();

  RECORD_RADIO_TX_ON

  radio_txNow(TX_USE_CSMA_CA);

  radiotimer_schedule(RX_ARM_TXOLAECHO_MS);

}


/*
 *  aqui esta no inicio do slot RX e nao recebeu OLA ECHO...
 *  Reprograma o timer para a janela do RIT e volta a enviar o pacote.
 *  Problema aqui eh o radio estar travado e ele vai terminar o slot quando o timer estourar a janela.
 */
port_INLINE void ARMactivity_rxe01(void) {

	uint32_t deltams;
	uint32_t timeelapsedms;

	radiotimer_cancel();

	radio_rfOff();

	changeState(S_ARM_OLANOECHO);

	incroute(0xE1);

    RECORD_RADIO_TX_OFF (ieee154e_vars.lastCapturedTime)

	deltams = calcdeltatime(RX_ARM_OLATIMEOUT_MS);

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

		endSlot();
	}
	else{
		endSlot();
	}

}


/*
 * Aqui ele recebeu um ola echo e vai esperar para receber um RTS no meu melhor canal.
 * Se nao chegar significa que ninguem quer falar comigo.
 */

port_INLINE void ARMactivity_rx02(PORT_RADIOTIMER_WIDTH capturedTime) {

	radiotimer_cancel();

	radio_rfOff();

	changeState(S_ARM_RXRTSPREPARE);

	incroute(0x02);

    RECORD_RADIO_TX_OFF (ieee154e_vars.lastCapturedTime)
    radiotxola = ieee154e_vars.radioTxOnTics;

	// recepcao da mensagem é no meu melhor canal
	ieee154e_vars.freq = macneighbors_getMyBestChan();

    // configure the radio for that frequency
    radio_setFrequency(ieee154e_vars.freq);

    // enable the radio in Rx mode. The radio does not actively listen yet.
	radio_rxEnable();

	radiotimer_schedule(RX_ARM_DELAYTX2RX_MS);
}

/*
 * Aqui eu estou habilitando o RX para esperar pelo frame de RTS de alguem querendo comunicar comigo
 */

port_INLINE void ARMactivity_rx03() {

	radiotimer_cancel();

    changeState(S_ARM_RXRTS);

   RECORD_RADIO_RX_ON

	//leds_radio_toggle();

   incroute(0x03);

    radio_rxNow();

    radiotimer_schedule(RX_ARM_RTSTIMEOUT_MS);

}

/* ACABOU O RIT AQUI E NAO RECEBI NENHUMA MENSAGEM (macRITstate == S_RIT_RX_state)
 * Quando RX significa que abri a janela do RIT e nao teve nenhum transmissor querem comunicar comigo.
 * ESTE CASO É O MAIS COMUM DE OCORRER!!!!
 */
port_INLINE void ARMactivity_rxe03() {

    radiotimer_cancel();

    radio_rfOff();

    RECORD_RADIO_RX_OFF (ieee154e_vars.lastCapturedTime)

	incroute(0xE3);

    changeState(S_ARM_RTSTIMEOUT);

#if 0 //((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
{
	uint8_t   pos=0;

	rffbuf[pos++]= RFF_IEEE802_RX;
	rffbuf[pos++]= 0xE3;
	pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);
	pos = printvar((uint8_t *)&ieee154e_vars.radioRxOnTics,sizeof(uint32_t),rffbuf,pos);
	pos = printvar((uint8_t *)&ieee154e_vars.radioTxOnTics,sizeof(uint32_t),rffbuf,pos);

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
}
#endif

	endSlot();
}


/*
 * recebi um frame e estava esperando por um RTS
 */

port_INLINE void ARMactivity_rx04(PORT_RADIOTIMER_WIDTH capturedTime) {

	uint32_t timeelapsedms;
	uint8_t frameOK=0;
    ieee802154_header_iht ieee802514_header;
	uint8_t discardframe;
	open_addr_t rxaddr_dst;
	//uint8_t ucCWcount;

	radiotimer_cancel();

	changeState(S_ARM_TXCTSOFFSET);

	radio_rfOff();

	incroute(0x04);

	//Atencao...aqui nao uso o lastcapturedtime para deixar o ultimo valor caso tenha que reprogramar o timer...
    RECORD_RADIO_RX_OFF (timeelapsedms)
	radiorxdutycycle1 = ieee154e_vars.radioRxOnTics;

	ieee154e_vars.ackReceived = openqueue_getFreePacketBuffer(COMPONENT_IEEE802154E);
	if (ieee154e_vars.ackReceived==NULL) {

	  openserial_printError(COMPONENT_IEEE802154E,ERR_NO_FREE_PACKET_BUFFER,
							(errorparameter_t)0,
							(errorparameter_t)0);
	   frameOK = FALSE;
	}
	else {
		frameOK = checkframeok(IEEE154_TYPE_ACK, ieee154e_vars.ackReceived,&ieee802514_header);
		memcpy(&rxaddr_dst, &(ieee802514_header.dest),sizeof(open_addr_t));
		memcpy(&actualsrcaddr, &(ieee802514_header.src),sizeof(open_addr_t));
	}

   if ((frameOK == TRUE) ) {

		#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
		{
			uint8_t pos=0;
			uint8_t *pauxframe;

			pauxframe = (uint8_t *) &ieee154e_vars.ackReceived->packet[0];
			rffbuf[pos++]= RFF_IEEE802_RX;
			rffbuf[pos++]= 0x04;
			rffbuf[pos++]= ieee154e_vars.ackReceived->l2_frameType;
			//rffbuf[pos++]= frameOK;
			rffbuf[pos++]= pauxframe[1];   //802154.FCF [0]
			rffbuf[pos++]= pauxframe[2];   //802154.FCF [1]
			rffbuf[pos++]= pauxframe[3];   //dest addr [7]
			pos = printaddress(actualsrcaddr,&rffbuf[0],pos);
			pos = printaddress(actualdstaddr,&rffbuf[0],pos);

			openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
		}
		#endif

       //TODO!!! EU ACHO QUE O FRAME NUNCA VAI SER BROADCAST...POIS ESTOU RECEBENDO UM RTS...
	   //CHECAR SE EH UM RTS...byte 10 do frame.
  	  if ((ieee154e_vars.ackReceived->l2_frameType == IEEE154_TYPE_ACK) &&
  		  (idmanager_isMyAddress(&rxaddr_dst) || packetfunctions_isBroadcastMulticast(&rxaddr_dst))) {
   		 incroute(0x41);

   		 countrxrts++;

   		 ieee154e_vars.lastCapturedTime = timeelapsedms;

		 radiotimer_schedule(RX_ARM_DELAYRTS2CTS_MS);
  		 discardframe = DISCARD_YES_ENDSLOT_NO;

	  }
  	  else{  //Continuo esperando!!!!
 		 incroute(0x42);
		 discardframe = activityrx_reopenrxwindow();
  	  }

   }
   else {
		 incroute(0x43);
		 discardframe = activityrx_reopenrxwindow();
   }

   if (discardframe > 0) {
	   // clean up ackReceived
	   if ((discardframe == DISCARD_YES_ENDSLOT_YES) || (discardframe == DISCARD_YES_ENDSLOT_NO)){
		   if (ieee154e_vars.ackReceived!=NULL) {
			  openqueue_freePacketBuffer(ieee154e_vars.ackReceived);
			  ieee154e_vars.ackReceived = NULL;
			}
	   }

	   if (discardframe == DISCARD_YES_ENDSLOT_YES) {
		   endSlot();
	   }
  }

}


/*
 * Aqui recebi um frame de RTS...indicando que existe alguem querendo comunicar comigo.
 * Entao devo enviar um CTS no meu BC confirmando a comunicacao
 *
 */
port_INLINE uint8_t ARMactivity_rx05(void) {

	//	header_IE_ht header_desc;
	//	OpenQueueEntry_t adv;
	//	uint8_t freq;
	//	uint32_t duration;
		uint8_t p=0;
		uint8_t frame[30];
		open_addr_t myaddr;
		open_addr_t *pmyaddr=(open_addr_t *)&myaddr;

		radiotimer_cancel();

	    changeState(S_ARM_TXCTS);

		incroute(0x05);

		// record the captured time
	    ieee154e_vars.lastCapturedTime = getdeltaslotperiod_ms();

	    // configure the radio for that frequency
		ieee154e_vars.freq = macneighbors_getMyBestChan();
	    radio_setFrequency(ieee154e_vars.freq);

		// MONTA O FRAME DO CTS
		frame[p++] = 0x40 | IEEE154_TYPE_ACK;
		frame[p++] = 0xaa;
		frame[p++] = 0xb1;
		frame[p++] = 0xfe;
		frame[p++] = 0xca;
		//address dest
		frame[p++] = actualsrcaddr.addr_16b[1];
		frame[p++] = actualsrcaddr.addr_16b[0];
		//address source
		pmyaddr = idmanager_getMyID(ADDR_16B);
		frame[p++] = pmyaddr->addr_16b[1];
		frame[p++] = pmyaddr->addr_16b[0];
		//RTS OU CTS
		frame[p++] = ARM_CTS_CMD;
		//crc
		frame[p++] = 0x00;
		frame[p++] = 0x00;

		// space for 2-byte CRC
		//packetfunctions_reserveFooterSize(&adv,2);

		radio_loadPacket((uint8_t *)&frame[0],p);

		radio_txEnable();

		RECORD_RADIO_TX_ON

	    radio_txNow(TX_USE_CSMA_CA);

		counttxcts++;

	    radiotimer_schedule(RX_ARM_CTSECHO_MS);

		#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TX == 1))
		{
			uint8_t   pos=0;

			rffbuf[pos++]= RFF_IEEE802_RX;
			rffbuf[pos++]= 0x05;
			rffbuf[pos++]= ieee154e_vars.freq;
			rffbuf[pos++]= 0xee;

			pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

			openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
		}
		#endif

		return 0;
}

/* Nao recebi o echo do CTS (macRITstate == S_RIT_RX_state)
 */
port_INLINE void ARMactivity_rxe05(void) {

    radiotimer_cancel();

    radio_rfOff();

	RECORD_RADIO_TX_OFF (ieee154e_vars.lastCapturedTime)

    changeState(S_ARM_CTSNOECHO);

	incroute(0xE5);

	endSlot();
}

/*
 * Aqui eu ja Recebi um RTS e enviei um CTS...agora o canal de comunicacao com o TX esta estabelecido
 * Agora devo esperar pelo dado.
 */
port_INLINE uint8_t ARMactivity_rx06(PORT_RADIOTIMER_WIDTH capturedTime) {

	radiotimer_cancel();

	radio_rfOff();

    changeState(S_ARM_RXDATAPREPARE);

	//como desliguei o radio anteriormente devo reprograma-lo para RX
	ieee154e_vars.freq = macneighbors_getMyBestChan();
	radio_setFrequency(ieee154e_vars.freq);

    incroute(0x06);

    radio_rxEnable();

	RECORD_RADIO_TX_OFF (ieee154e_vars.lastCapturedTime)
    radiotxdutycycle1 = ieee154e_vars.radioTxOnTics;

	radiotimer_schedule(RX_ARM_DELAYCTS2DATA_MS);

#if 0//((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TX == 1))
{
	uint8_t   pos=0;

	rffbuf[pos++]= RFF_IEEE802_RX;
	rffbuf[pos++]= 0x06;
	rffbuf[pos++]= ieee154e_vars.freq;

	//pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
}
#endif
	return TRUE;
}

port_INLINE void ARMactivity_rx07(void) {

	radiotimer_cancel();

    changeState(S_ARM_RXDATA);

    radio_rxNow();

    RECORD_RADIO_RX_ON

	incroute(0x07);

    radiotimer_schedule(RX_ARM_DATATIMEOUT_MS);

#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TX == 1))
{
	uint8_t   pos=0;

	rffbuf[pos++]= RFF_IEEE802_RX;
	rffbuf[pos++]= 0x07;
	rffbuf[pos++]= ieee154e_vars.freq;

	//pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
}
#endif
}

/*
 * Ocorreu o Timeout do dado (macRITstate == S_RIT_RX_state)
 */
port_INLINE void ARMactivity_rxe07(void) {

    radiotimer_cancel();

    radio_rfOff();

    changeState(S_ARM_DATATIMEOUT);

    RECORD_RADIO_RX_OFF(ieee154e_vars.lastCapturedTime)

	incroute(0xE7);

	#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
	{
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_RX;
		rffbuf[pos++]= 0xE7;
		pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);
		//pos = printvar((uint8_t *)&freerunning,sizeof(uint32_t),rffbuf,pos);

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);

	}
	#endif

	endSlot();
}

/*
 * Aqui chegou um novo frame e eu estava esperando um frame de dados.
 * Aqui ja passei pelo RTS e CTS portanto a conexao é com o Transmissor...porem pode haver outros tipos de frames.
 */

port_INLINE void ARMactivity_rx08(PORT_RADIOTIMER_WIDTH capturedTime) {
    ieee802154_header_iht ieee802514_header;
    uint8_t frameOK=0;
    uint8_t discardframe = FALSE;
	open_addr_t rxaddr_dst;
	open_addr_t newsrcaddr;
	uint8_t IsTheSameAddress;
	uint8_t   *pauxframe;
//	uint8_t reopenwindow=0;
    uint32_t timeelapsedms;

	radiotimer_cancel();

	radio_rfOff();

    RECORD_RADIO_RX_OFF(timeelapsedms)
    radiorxdutycycle2 = ieee154e_vars.radioRxOnTics;

	changeState(S_ARM_RXNEWDATA);

	incroute(0x08);

#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
{
	uint8_t pos=0;

	pauxframe = (uint8_t *) &ieee154e_vars.dataReceived->packet[0];
	rffbuf[pos++]= RFF_IEEE802_RX;
	rffbuf[pos++]= 0x08;
	rffbuf[pos++]= ieee802514_header.frameType;
	rffbuf[pos++]= pauxframe[1];   //802154.FCF [0]
	rffbuf[pos++]= pauxframe[2];   //802154.FCF [1]
	pos = printaddress(newsrcaddr,&rffbuf[0],pos);

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
}
#endif


   // get a buffer to put the (received) data in
   ieee154e_vars.dataReceived = openqueue_getFreePacketBuffer(COMPONENT_IEEE802154E);

   if (ieee154e_vars.dataReceived==NULL) {
      // log the error
      openserial_printError(COMPONENT_IEEE802154E,ERR_NO_FREE_PACKET_BUFFER,
                            (errorparameter_t)0,
                            (errorparameter_t)0);
      endSlot();

      //teste rff - clear msg pendente
      //openqueue_removeAllOwnedBy(COMPONENT_SIXTOP_TO_IEEE802154E);
      return;
   }

   frameOK = checkframeok(IEEE154_TYPE_DATA, ieee154e_vars.dataReceived,&ieee802514_header);

   memcpy(&actualdstaddr, &(ieee802514_header.dest),sizeof(open_addr_t));
   memcpy(&actualsrcaddr, &(ieee802514_header.src),sizeof(open_addr_t));

   if (frameOK == TRUE) {
	   actualsrcaddr = packetfunctions_convert2AddressType(&actualsrcaddr,actualsrcaddr.type);
	   IsTheSameAddress = packetfunctions_sameAddress(&actualsrcaddr,&actualsrcaddr);

	   //Aqui como ja recebi um ack do actualsrcaddr entao eu somente vou receber frame de dados deste src...
	   if ((IsTheSameAddress == TRUE) && (ieee802514_header.frameType == IEEE154_TYPE_DATA)) {
			incroute(0x81);

			//Aqui eu retiro o frame type (DIO,DAO, COAP) para propositos de debug apenas...PROVISORIO!!!!!
			macRIT_Pending_RX_frameType = parserframerx(ieee154e_vars.dataReceived);

			discardframe = ARMactivity_rx85(ieee802514_header.ackRequested,ieee802514_header.frameType, &actualdstaddr);
			ieee154e_vars.lastCapturedTime = timeelapsedms;
	   }
	   else {
		   if ((ieee154e_vars.dataReceived->packet[10] == CMDLIVELIST)
				/*   &&
	  			  (ieee154e_vars.dataReceived->packet[3] == olaasn)  && (idmanager_isMyAddress(&actualdstaddr))*/) {

                macRIT_Pending_RX_frameType = IANA_ICMPv6_RA_PREFIX_INFORMATION; //only  for debug....
				ritstat.rxola.countdatatxok++;
			#if (ENABLE_LIVELIST_ACK == 1)
				//descobre se o tx programou esperar um ack na livelist...
				if (ieee154e_vars.dataReceived->packet[11] == 0) {
					incroute(0x83);
					discardframe = DISCARD_YES_ENDSLOT_YES;
				}
				else {
					incroute(0x84);
					macRITstate = S_RIT_RX_livelist;
					changeState(S_ARM_LLTXACKOFFSET);
					radiotimer_schedule(TX_ARM_DELAYRX2TX_MS);
					discardframe = DISCARD_NO_ENDSLOT_NO;
				}
			#else
				incroute(0x83);
				discardframe = DISCARD_YES_ENDSLOT_YES;
			#endif
		   }
		   else { //é um ola...volta a abrir a janela esperando pelo dado...
			   incroute(0x86);
			   //discardframe = activityrx_reopenrxwindow();
				discardframe = DISCARD_YES_ENDSLOT_YES;
		   }
	   }

	   if (discardframe > 0) {
		   // clean up dataReceived
		   if (discardframe == DISCARD_YES_ENDSLOT_YES){
				incroute(0x87);
			   if (ieee154e_vars.dataReceived!=NULL) {
				  openqueue_freePacketBuffer(ieee154e_vars.dataReceived);
				  ieee154e_vars.dataReceived = NULL;
				}
		   }

		   endSlot();
	  }

	  return;

   }

   // free the (invalid) received data so RAM memory can be recycled
   if (ieee154e_vars.dataReceived!=NULL) {
	   openqueue_freePacketBuffer(ieee154e_vars.dataReceived);
	   ieee154e_vars.dataReceived = NULL;
   }

   //Aqui ocorreu erro no frame recebido e volta a escutar a linha para RX.
   //Porem ele pode ficar travado aqui por algum motivo...entao ele somente vai fazer isso por tres tentativas
   nrErrorRetries++;
   if (nrErrorRetries < 5) {
	   incroute(0x89);
	   if (macRITstate == S_RIT_RX_state)
		   activityrx_reopenrxwindow();
	   else { // (macRITstate == S_RIT_TX_state ou hello
			changeState(S_ARM_TX_CONTINUEWAIT);
			radiotimer_schedule(TX_ARM_DELAYCONTWAIT_MS);
	   }
		ieee154e_vars.lastCapturedTime = timeelapsedms;
   }
   else {
		ieee154e_vars.lastCapturedTime = timeelapsedms;
	   // no servico hello foi esperado um ola do mote por 4 tentativas e sempre ocorreu a resposta de outros...
	   // considero entao que ele esta fora da livelist
	   if (macRITstate == S_RIT_multichnhello_state) {
		   macneighbors_clearlivelist(ieee154e_vars.targetaddr);
	   }

	   endSlot();
   }

}


/* rx08 - aqui eu devo checar se o frame eh para mim e eh um AP
 * return - 0 - nao descarta o frame e nao termina o slot
 *          1 - termina o slot e nao descarta o frame (pois ele ja foi descartado)
 *          2 - termina o slot e descarta o frame
 */

port_INLINE uint8_t ARMactivity_rx85 (uint8_t ackRequested,uint8_t frameType, open_addr_t *rxaddr_dst) {
   uint8_t DiscardAndEndSlot = 0;
//   uint32_t capturedTime=0;

	changeState(S_ARM_TXACKOFFSET);

    incroute(0x85);

    //checa se frame é para o mote (mesmo endereco destino ou eh um frame broadcast)
	if (frameType == IEEE154_TYPE_UNDEFINED) {
		incroute(0x86);
		DiscardAndEndSlot = activityrx_reopenrxwindow();
	}
	else if (packetfunctions_isBroadcastMulticast(rxaddr_dst)){

		incroute(0x87);
		// indicate reception to upper layer (no ACK asked)
		if (ieee154e_vars.dataReceived!=NULL) {
			macRIT_Pending_RX_frameType = IANA_ICMPv6_RPL_DIO;
			//AQUI É UM DIO...ENTAO SOMENTO NOTIFICO AS CAMADAS SUPERIORES
			ritstat.rxdio++;

		   //ritstat.rxola.countdatatxok++;
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

		incroute(0x88);

		// check if ack requested
		if (ackRequested==1) {
			#if 0 //((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
			{
				uint8_t pos=0;

				rffbuf[pos++]= RFF_IEEE802_RX;
				rffbuf[pos++]= 0x88;
				rffbuf[pos++]= 0xee;
				pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

				openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
			}
			#endif
			radiotimer_schedule(TX_ARM_DELAYRX2TX_MS);
			DiscardAndEndSlot = DISCARD_NO_ENDSLOT_NO;
		}
		else{
			//Aqui ele eh para mim mas nao precisa de ACK...nao sei que frame eh??????
			DiscardAndEndSlot = DISCARD_YES_ENDSLOT_YES;
		}
	}
	else { //aqui recebeu um frame que nao era ola e nao era para este mote....reprogramo a janela
		//Este caso nao eh para acontecer normalmente
		incroute(0x89);
		DiscardAndEndSlot = activityrx_reopenrxwindow();
	}

	return DiscardAndEndSlot;
}

/*
 * Livelist TX ack...
 * aqui eh usado quando a livelist pede um ack..neste caso o frame sera o mesmo da livelist porem com o comando ack desabilitado.
 *
 */
#if (ENABLE_LIVELIST_ACK == 1)
port_INLINE void ARMactivity_rx81(void) {

	uint8_t frame[128];
	sRITelement psEle;

   radiotimer_cancel();

	// change state
    changeState(S_ARM_TXACK);
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

	radiotimer_schedule(RX_ARM_ACKECHO_MS);
}
#endif


port_INLINE void ARMactivity_rx09(void) {
   //PORT_SIGNED_INT_WIDTH timeCorrection;
   //uint32_t duration;
   header_IE_ht header_desc;

   radiotimer_cancel();

   // change state
   changeState(S_ARM_TXACKPREPARE);

   incroute(0x09);

   // get a buffer to put the ack to send in
   ieee154e_vars.ackToSend = openqueue_getFreePacketBuffer(COMPONENT_IEEE802154E);

   ieee154e_vars.lastCapturedTime = getdeltaslotperiod_ms();

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

   // configure the radio for that frequency
   ieee154e_vars.freq = macneighbors_getMyBestChan();
   radio_setFrequency(ieee154e_vars.freq);

   // load the packet in the radio's Tx buffer
   radio_loadPacket(ieee154e_vars.ackToSend->payload,
                    ieee154e_vars.ackToSend->length);

	// enable the radio in Tx mode. This does not send that packet.
	radio_txEnable();

	RECORD_RADIO_TX_ON

#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
{
	uint8_t   pos=0;

	rffbuf[pos++]= RFF_IEEE802_RX;
	rffbuf[pos++]= 0x09;
	rffbuf[pos++]= ieee154e_vars.ackToSend->length;
	rffbuf[pos++]= ieee154e_vars.freq;

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
}
#endif

   //aqui devo aguardar um tempo para atrasar o tx do rx.
   //quando envio direto o tempo eh da ordem de 3ms. Coloco um atraso de 7 ms para dar 10ms entre Rx e TxAckv(328)
   radiotimer_schedule(RX_ARM_DELAYDATA2ACK_MS);

}

port_INLINE void ARMactivity_rx0a(void) {
//  uint32_t duration;

  radiotimer_cancel();

  changeState(S_ARM_TXACK);

  incroute(0x0a);

  radio_txNow(TX_NOT_USE_CSMA_CA);

  radiotimer_schedule(RX_ARM_ACKECHO_MS);
}


port_INLINE void ARMactivity_rxe0a() {

	//uint32_t deltams;
	//uint32_t timeelapsedms;

   radiotimer_cancel();

   radio_rfOff();

	RECORD_RADIO_TX_OFF(ieee154e_vars.lastCapturedTime)

   changeState(S_ARM_ACKNOECHO);

   incroute(0xEA);

#if ((ENABLE_DEBUG_RFF == 1)  && (DBG_IEEE802_TX == 1))
{
	uint8_t pos=0;

	rffbuf[pos++]= RFF_IEEE802_RX;
	rffbuf[pos++]= 0xEA;
	rffbuf[pos++]= macRITstate;
	rffbuf[pos++]= macRIT_Pending_RX_frameType;
	pos = printaddress(ieee154e_vars.targetaddr,&rffbuf[0],pos);
	rffbuf[pos++]= 0xee;
	pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
}
#endif

	// log the error
	openserial_printError(COMPONENT_IEEE802154E,ERR_WDACKDURATION_OVERFLOWS,
						  (errorparameter_t)ieee154e_vars.state,
						  (errorparameter_t)ieee154e_vars.slotOffset);

	endSlot();

}

port_INLINE void ARMactivity_rx0b(PORT_RADIOTIMER_WIDTH capturedTime) {

	radiotimer_cancel();

	radio_rfOff();

	changeState(S_ARM_RXPROC);

	RECORD_RADIO_TX_OFF(ieee154e_vars.lastCapturedTime)
    radiotxdutycycle2 = ieee154e_vars.radioTxOnTics;

	incroute(0x0b);
	RxFlagImprimeRoute = 0;

	#if 0 //((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
	{
		uint8_t   pos=0;

		rffbuf[pos++]= RFF_IEEE802_RX;
		rffbuf[pos++]= 0x0b;
		rffbuf[pos++]= macRIT_Pending_RX_frameType;

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
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
   else{
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




port_INLINE void StartTxMultiChannelHello (uint8_t pending) {

	uint8_t frame[128];
	sRITelement psEle;
	uint8_t numTargetParents;
	open_addr_t addrdst;

	changeState(S_ARM_RXOLAOFFSET);

	livelistasn++;
	ritstat.txola.countdatatx++;

	leds_sync_toggle();

#if (ENABLE_LIVELIST_ACK == 1)
	psEle = activityrx_preparemultichannelhello(1,frame,LIVELIST_FRAME_LEN);
#else
	psEle = activityrx_preparemultichannelhello(0,frame,LIVELIST_FRAME_LEN);
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

    //radiotimer_schedule_us(TX_ARM_RXOLAPREPARE_US);
    ARMactivity_tx02();

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
uint8_t ARMactivity_tx01(uint8_t txpending,uint8_t newtxframe) {

	sRITelement element;
	sRITelement *pmsgout=&element;
	open_addr_t address;
	uint8_t testerff=0;
	uint8_t flagpending=false;
	uint8_t icmpv6_type;
	uint8_t icmpv6_code;
	uint8_t iphc_header1;
	uint8_t iphc_nextheader;
	uint8_t numTargetParents;

    changeState(S_ARM_TXFRAME);

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
			//if (txpending != true){
				macRIT_Pending_TX_frameType = IANA_ICMPv6_RPL_DIO;
				pmsgout->frameType = macRIT_Pending_TX_frameType;
				pmsgout->destaddr  = ieee154e_vars.dataToSend->l3_destinationAdd;
				testerff = 0x12;
			//}
		}
		else if (ieee154e_vars.dataToSend->l4_sourcePortORicmpv6Type == IANA_ICMPv6_RPL)
		{
			testerff = 0x14;
			//FRAME DAO
			macRIT_Pending_TX_frameType = IANA_ICMPv6_RPL_DAO;

			if (ieee154e_vars.dataToSend->creator == COMPONENT_FORWARDING) {
				testerff = 0x15;
				 //salvo o endereco do destino que pode estar na posicao final do frame (RPL Transition)
				 //address_1.type = ieee154e_vars.dataToSend->l3_destinationAdd.type;
				 RITQueue_copyaddress(&address_1,&ieee154e_vars.dataToSend->l3_destinationAdd);

				 if (ieee154e_vars.dataToSend->l3_destinationAdd.type == 3){
					testerff = 0x16;
					address_1 = ieee154e_vars.dataToSend->l2_nextORpreviousHop;
				 }
			}
			else {
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

		//teste rff  -- PROVISORIO
		//ieee154e_vars.dataToSend->l2_nextORpreviousHop.addr_64b[6] = 0xCC;
		//ieee154e_vars.dataToSend->l2_nextORpreviousHop.addr_64b[7] = 0xCC;
		//teste rff -- PROVISORIO

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

			//AQUI EH O CASO DA BRIDGE ANTIGA!!!!
			macRIT_Pending_TX_frameType = IANA_ICMPv6_RPL_DIO;
			//macRIT_Pending_TX_frameType = 0;  //send directly
			pmsgout->frameType = macRIT_Pending_TX_frameType;
			pmsgout->destaddr  = ieee154e_vars.dataToSend->l2_nextORpreviousHop;
		}
		else if (((iphc_header1 == 0x78) && ((iphc_nextheader == IANA_UDP) || (iphc_nextheader == IANA_IPv6ROUTE))) ||
				 (ieee154e_vars.dataToSend->creator == COMPONENT_OPENBRIDGE))
		{ //verifica se o IPHC header eh 6LowPAN e o Next eh UDP ou IPV6 Route  (INFERENCIA DO COAP)
			testerff = 0x1B;
			macRIT_Pending_TX_frameType = IANA_UDP;
			pmsgout->frameType = macRIT_Pending_TX_frameType;

			//teste rff  -- PROVISORIO
			//ieee154e_vars.dataToSend->l2_nextORpreviousHop.addr_64b[6] = 0xAA;
			//ieee154e_vars.dataToSend->l2_nextORpreviousHop.addr_64b[7] = 0xCC;
			//teste rff -- PROVISORIO

			if (txpending){
				neighbors_getPreferredParentEui64(&address);
			}

			pmsgout->destaddr  = ieee154e_vars.dataToSend->l2_nextORpreviousHop;

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
    	incroute(0xFA);
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

		if (macRIT_Pending_TX_frameType == IANA_ICMPv6_RPL_DIO) {
			rffbuf[pos++]= 0xD1;
			rffbuf[pos++]= 0xD1;
			rffbuf[pos++]= 0xD1;
			rffbuf[pos++]= 0xD1;
		}
		else if (macRIT_Pending_TX_frameType == IANA_ICMPv6_RPL_DAO) {
			rffbuf[pos++]= 0xDA;
			rffbuf[pos++]= 0xDA;
			rffbuf[pos++]= 0xDA;
			rffbuf[pos++]= 0xDA;
		}
		else if (macRIT_Pending_TX_frameType == IANA_UDP) {
			rffbuf[pos++]= 0xCA;
			rffbuf[pos++]= 0xCA;
			rffbuf[pos++]= 0xCA;
			rffbuf[pos++]= 0xCA;

			//rit statistics - conta novo produtor
			ritstat.txcoap.countdatatx++;
		}
		else  {
			rffbuf[pos++]= 0xBB;
			rffbuf[pos++]= 0xBB;
			rffbuf[pos++]= 0xBB;
			rffbuf[pos++]= 0xBB;

		}

		//rffbuf[pos++]= ieee154e_vars.slotOffset;
		rffbuf[pos++]= macRIT_Pending_TX_frameType;
		rffbuf[pos++]= testerff;
		//rffbuf[pos++]= ieee154e_vars.dataToSend->l4_protocol;
		//rffbuf[pos++]= ieee154e_vars.dataToSend->l2_numTxAttempts;
		rffbuf[pos++]= ieee154e_vars.dataToSend->l2_retriesLeft;
		//rffbuf[pos++]= pmsgout->isBroadcastMulticast;
		//rffbuf[pos++]= numTargetParents;

		//rffbuf[pos++]= pmsgout->msglength;
		//rffbuf[pos++]= iphc_header1;
		//rffbuf[pos++]= iphc_nextheader;

	    //pos = printaddress(pmsgout->destaddr,rffbuf,pos);
		pos = printaddress(ieee154e_vars.dataToSend->l3_destinationAdd,rffbuf,pos);
		pos = printaddress(ieee154e_vars.dataToSend->l2_nextORpreviousHop,rffbuf,pos);

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
	#endif


   return (macRIT_Pending_TX_frameType);
}

port_INLINE void ARMactivity_tx00(uint8_t txpending,uint8_t newtxframe) {

	//uint8_t macRIT_Pending_TX_frameType=0;

	changeState(S_ARM_RXOLAOFFSET);

    if (ieee154e_vars.dataToSend->l2_frameType == IEEE154_TYPE_DATA) {
		macRIT_Pending_TX_frameType = ARMactivity_tx01(txpending,newtxframe);
	}

    incroute(macRIT_Pending_TX_frameType);

	openserial_startOutput();

    if (macRIT_Pending_TX_frameType > 0){
		incroute(0x01);
		ARMactivity_tx02();
	}
	else {
        incroute(0x13);
		ARMactivity_txe01();
	}

}

/*
 * Somente entra aqui quando ocorreu algum erro na interpretacao do frame pendente.
 */
port_INLINE void ARMactivity_txe01(void) {

	radiotimer_cancel();

    changeState(S_ARM_TXFRMERROR);

	incroute(0xE1);

	#if ((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_TX == 1))
	  {
		uint8_t   pos=0;

		rffbuf[pos++]= RFF_IEEE802_TX;
		rffbuf[pos++]= 0xE1;
		rffbuf[pos++]= macRITstate;
		rffbuf[pos++]= sRIT_vars.frameType;
		rffbuf[pos++]= ieee154e_vars.dataToSend->l2_retriesLeft;
		pos = printaddress(ieee154e_vars.targetaddr,&rffbuf[0],pos);
		rffbuf[pos++]= 0xee;
		pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	  }
	#endif

	//Quando existe um COAP pendente entao eu termino no EndSlot pois ele vai gerar Retries
  	if (!((macRITstate == S_RIT_TX_state) && (sRIT_vars.frameType == IANA_UDP))) {

  		if (ieee154e_vars.dataToSend != NULL) {
  		  notif_sendDone(ieee154e_vars.dataToSend,E_FAIL);
  		  ieee154e_vars.dataToSend = NULL;
  	 	}
  	}

	endSlot();

}


/*======= TX
* esta rotina eh chamada logo apos o inicio do TxRITProcedure
* ela prepara o radio para recepcao de um ola
*/
port_INLINE void ARMactivity_tx02(void) {

	uint8_t isBroadcast=0;
//	uint8_t ret;

    changeState(S_ARM_RXOLAPREPARE);

	incroute(0x02);

    radiotimer_cancel();

	// turn off the radio
	radio_rfOff();


    if (sRIT_vars.destaddr.type == 0){
		openserial_printError(COMPONENT_IEEE802154E,ERR_WRONG_RIT_CHANNEL,
							(errorparameter_t)S_ARM_RXOLAPREPARE,0x02);
		endSlot();
		return;
    }

	// o ola eh sempre enviado para o canal de controle
    ieee154e_vars.targetaddr = sRIT_vars.destaddr;
	ieee154e_vars.freq = macneighbors_getControlChan();

/*
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
							(errorparameter_t)S_ARM_RXOLAPREPARE,
							ieee154e_vars.freq);
		 endSlot();
		 return;
	}
*/

#if (DEBUG_LOG_RIT  == 1) && (DBG_IEEE802_TX == 1)
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

	RECORD_RADIO_RX_ON

	radiotimer_schedule(TX_ARM_RXOLAPREPARE_MS);
}



port_INLINE void ARMactivity_tx03(void) {

	radiotimer_cancel();

    changeState(S_ARM_RXOLA);

	incroute(0x03);

    radio_rxNow();

	radiotimer_schedule(TX_ARM_OLATIMEOUT_MS);

#if ((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_TX == 1))
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

/* Aqui estava esperando um OLA mas nao veio (macRITstate == S_RIT_TX_state)
 * Como a janela de Tx é bem grande, é pressuposto que o nó esta ruim
 */
port_INLINE void ARMactivity_txe03() {

	uint8_t nrDIOsent=0;

    radiotimer_cancel();

	// change state
    changeState(S_ARM_OLATIMEOUT);

    // turn off the radio
    radio_rfOff();

	RECORD_RADIO_RX_OFF(ieee154e_vars.lastCapturedTime)

	incroute(0xE3);

	//Pego o primeiro vizinho ainda pendente msg e atualizo estatisticas para ele
	//TODO!!! Como tratar isso quando tiver mais de um vizinho ???
	ieee154e_vars.targetaddr = getAddressPendingBroadcast();

    #if 0
	{
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_TX;
		rffbuf[pos++]= 0xE3;
		rffbuf[pos++]= macRITstate;
		rffbuf[pos++]= sRIT_vars.frameType;
		rffbuf[pos++]= ieee154e_vars.dataToSend->l2_retriesLeft;
		pos = printaddress(ieee154e_vars.targetaddr,&rffbuf[0],pos);
		rffbuf[pos++]= 0xee;
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
		macneighbors_clearlivelist(ieee154e_vars.targetaddr);

		ieee154e_vars.dataToSend = NULL;
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

    }
    else {// sRIT_vars.frameType != IANA_ICMPv6_RPL_DIO

    	//Quando existe um COAP pendente entao eu termino no EndSlot pois ele vai gerar Retries
      	if (!((macRITstate == S_RIT_TX_state) && (sRIT_vars.frameType == IANA_UDP))) {
			// indicate transmit failed to schedule to keep stats
			schedule_indicateTx(&ieee154e_vars.asn,FALSE);

			if (ieee154e_vars.dataToSend != NULL) {
			  // indicate tx fail if no more retries left
			  notif_sendDone(ieee154e_vars.dataToSend,E_FAIL);
			  // reset local variable
			  ieee154e_vars.dataToSend = NULL;
			}
	   }

   }

   endSlot();
}

/*
 * Aqui eu recebi um novo frame. estava esperando um Ola
 */

port_INLINE void ARMactivity_tx04(PORT_RADIOTIMER_WIDTH capturedTime) {
    ieee802154_header_iht ieee802514_header;
	open_addr_t rxaddr_dst;
	uint8_t   *pauxframe;
	uint8_t frameOK=0;

	radiotimer_cancel();

	radio_rfOff();

	RECORD_RADIO_RX_OFF(tempo1)

	//Salvo o tempo de espera pelo OLA...
	txwaitola = ieee154e_vars.radioRxOnTics;

	changeState(S_ARM_RXNEWOLA);

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

      return;
   }

   frameOK = checkframeok(IEEE154_TYPE_CMD, ieee154e_vars.dataReceived,&ieee802514_header);
   memcpy(&actualdstaddr, &(ieee802514_header.dest),sizeof(open_addr_t));
   memcpy(&actualsrcaddr, &(ieee802514_header.src),sizeof(open_addr_t));
	//Atualiza bestchannel
	actualbestchannel = ieee154e_vars.dataReceived->packet[11];

	//PROVISORIO!!!! DESCOBRIR O PROBLEMA...E REJEITAR O FRAME
    if ((actualbestchannel < 11) || (actualbestchannel > 25)) {
		openserial_printError(COMPONENT_IEEE802154E,ERR_WRONG_RIT_CHANNEL,
							(errorparameter_t)S_ARM_RXNEWOLA,actualbestchannel);
		frameOK = FALSE;
    }
    else{

		#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
		{
			uint8_t pos=0;

			pauxframe = (uint8_t *) &ieee154e_vars.dataReceived->packet[0];
			rffbuf[pos++]= RFF_IEEE802_TX;
			rffbuf[pos++]= 0x04;
			rffbuf[pos++]= ieee154e_vars.dataReceived->l2_frameType;
			rffbuf[pos++]= (macRIT_Pending_TX_frameType);
			rffbuf[pos++]= pauxframe[1];   //802154.FCF [0]
			rffbuf[pos++]= pauxframe[2];   //802154.FCF [1]
			//rffbuf[pos++]= pauxframe[7];   //dest addr [7]
			pos = printaddress(actualsrcaddr,&rffbuf[0],pos);
			rffbuf[pos++]= 0xBC;
			rffbuf[pos++]= actualbestchannel;

			//rffbuf[pos++]= 0xee;   //src addr [7]
			//pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

			openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
		}
		#endif
    }

	if (frameOK == TRUE){

		if (ieee802514_header.frameType == IEEE154_TYPE_OLA) {

			if (sRIT_vars.isBroadcastMulticast){  //FRAME RPL.DIO ou MULTICHANNELHELLO

				if (macisThisAddressPendingBroadcast(&actualsrcaddr) == TRUE){
				    ieee154e_vars.lastCapturedTime = tempo1;
					ritstat.txdio.countdatatx++;
					incroute(0x40);
					changeState(S_ARM_TXRTSOFFSET);
					radiotimer_schedule(RX_ARM_DELAYTX2RX_MS);
				}
				else {
					//Chegou msg mas de um endereco que nao vou enviar broadcast...devo aguardar novamente
					incroute(0x41);

					changeState(S_ARM_TX_CONTINUEWAIT);
					radiotimer_schedule(TX_ARM_DELAYCONTWAIT_MS);
				}

			}
			else { //frame eh COAP OU RPL.DAO or Livelist
				uint8_t elementpos;

				elementpos = RITQueue_Get_Pos(&ieee802514_header.src);
				leds_debug_toggle();

			    if (elementpos < maxElements)
				{ //OLA EH DA MSG PENDENTE...PREPARO PARA ENVIA-LA
					ieee154e_vars.RITQueue_ElementPending = elementpos;
					incroute(0x42);

					#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
					{
						uint8_t pos=0;

						pauxframe = (uint8_t *) &ieee154e_vars.dataReceived->packet[0];
						rffbuf[pos++]= RFF_IEEE802_TX;
						rffbuf[pos++]= 0x04;
						rffbuf[pos++]= ieee154e_vars.dataReceived->l2_frameType;
						rffbuf[pos++]= (macRIT_Pending_TX_frameType);
						rffbuf[pos++]= pauxframe[1];   //802154.FCF [0]
						rffbuf[pos++]= pauxframe[2];   //802154.FCF [1]
						//rffbuf[pos++]= pauxframe[7];   //dest addr [7]
						pos = printaddress(actualsrcaddr,&rffbuf[0],pos);
						rffbuf[pos++]= 0xBC;
						rffbuf[pos++]= actualbestchannel;

						//rffbuf[pos++]= 0xee;   //src addr [7]
						//pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

						openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
					}
					#endif


					#if (LIVELIST_FIXED_PACKETS == 1)
						livelistcount++;
					#endif

					changeState(S_ARM_TXRTSOFFSET);
					radiotimer_schedule(TX_ARM_DELAYRX2TX_MS);
				    ieee154e_vars.lastCapturedTime = tempo1;
				}
				else { //AQUI O ENDERECO DO OLA NAO EH O MESMO QUE EU ESTAVA ESPERANDO...VOLTO A ESPERAR...
					incroute(0x43);
					changeState(S_ARM_TX_CONTINUEWAIT);
					radiotimer_schedule(TX_ARM_DELAYCONTWAIT_MS);
				}
			}

		}
		else {  //aqui estava esperando um ola mas veio outro frame...devo voltar a esperar um novo frame.
			//TODO!!! AQUI NAO VAI DAR UM DEADLOCK ? O CARA TAMBEM ESTA ESPERANDO UM OLA...SE EU FICAR ESPERANDO ELE NAO VAI VIR...
			//MELHOR TALVEZ TERMINAR E ENVIAR UM OLA PARA ELE...E PERDER A MSG...
			incroute(0x44);
			changeState(S_ARM_TX_CONTINUEWAIT);
			radiotimer_schedule(TX_ARM_DELAYCONTWAIT_MS);

			#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
			{
				uint8_t pos=0;

				pauxframe = (uint8_t *) &ieee154e_vars.dataReceived->packet[0];
				rffbuf[pos++]= RFF_IEEE802_TX;
				rffbuf[pos++]= 0x44;
				rffbuf[pos++]= ieee154e_vars.dataReceived->l2_frameType;
				rffbuf[pos++]= frameOK;
				rffbuf[pos++]= pauxframe[1];   //802154.FCF [0]
				rffbuf[pos++]= pauxframe[2];   //802154.FCF [1]
				//rffbuf[pos++]= pauxframe[7];   //dest addr [7]
				rffbuf[pos++]= 0xaa;  //inverti os bytes abaixo do endereco para ficar mais facil
				pos = printaddress(actualsrcaddr,&rffbuf[0],pos);
				//rffbuf[pos++]= 0xee;   //src addr [7]
				//pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

				openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
			}
			#endif
		}

		//descarto o frame recebido pois nao preciso mais dele
		if (ieee154e_vars.dataReceived!=NULL) {
		   openqueue_freePacketBuffer(ieee154e_vars.dataReceived);
		   ieee154e_vars.dataReceived = NULL;
		}

		return;
   }

   // free the (invalid) received data so RAM memory can be recycled
   if (ieee154e_vars.dataReceived!=NULL) {
	   openqueue_freePacketBuffer(ieee154e_vars.dataReceived);
	   ieee154e_vars.dataReceived = NULL;
   }

   //Aqui ocorreu erro no frame recebido e volta a escutar a linha para RX.
   //Porem ele pode ficar travado aqui por algum motivo...entao ele somente vai fazer isso por tres tentativas
   nrErrorRetries++;
   if (nrErrorRetries < 3) {
	   incroute(0x49);
	   if (macRITstate == S_RIT_RX_state)
		   activityrx_reopenrxwindow();

	   else { // (macRITstate == S_RIT_TX_state ou hello
			changeState(S_ARM_TX_CONTINUEWAIT);
			radiotimer_schedule(TX_ARM_DELAYCONTWAIT_MS);
	   }
   }
   else {

	   ieee154e_vars.lastCapturedTime = tempo1;
	   // no servico hello foi esperado um ola do mote por 4 tentativas e sempre ocorreu a resposta de outros...
	   // considero entao que ele esta fora da livelist
	   if (macRITstate == S_RIT_multichnhello_state) {
		   macneighbors_clearlivelist(ieee154e_vars.targetaddr);
	   }

	   endSlot();
   }

}


port_INLINE uint8_t ARMactivity_tx45(void) {

    uint8_t ret=FALSE;
	uint32_t deltams;

    radiotimer_cancel();

    incroute(0x45);

    changeState(S_ARM_TX_CONTINUEWAIT);

	pvObjList[ieee154e_vars.RITQueue_ElementPending].countretry++;

	deltams = calcdeltatime(TX_ARM_OLATIMEOUT_MS);

    if (deltams > 0) {
		flagreschedule = TRUE;

		//radio_rfOff();

		radio_setFrequency(ieee154e_vars.freq);

		radio_rxEnable();

		radio_rxNow();

		//Volto ao estado anterior para indicar que estou novamente esperando um frame TX
		changeState(S_ARM_RXOLA);

		radiotimer_schedule(deltams);

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

			openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
		}
		#endif
	}
	else {
		incroute(0x46);

		//TODO!!! AQUI ANTES EU TRATAVA O FRAME E ENVIAVA UM NOTIFY PARA INDICAR PARA O RPL OS FRAMES BROADCAST QUE
		//CONSEGUIRAM SER ENVIADOS...MAS ACHO QUE O IDEAL EH FAZER NO PROPRIO RPL TX...ou seja eu notifico falha
		//e se for broadcast ele analisa se houve algum frame que foi ok...ele acessando um servico do neighbors.
		// nrDIOsent = macneighbors_calcbroadcastsent();

		  //ieee154e_vars.radioOnTics+=(radio_getTimerValue()-ieee154e_vars.radioOnInit);

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

port_INLINE void ARMactivity_tx05(void) {

//	header_IE_ht header_desc;
//	OpenQueueEntry_t adv;
//	uint8_t freq;
//	uint32_t duration;
	uint8_t p=0;
	uint8_t frame[30];
	open_addr_t myaddr;
	open_addr_t *pmyaddr=(open_addr_t *)&myaddr;

    changeState(S_ARM_TXRTSPREPARE);

	radiotimer_cancel();

	incroute(0x05);

	// record the captured time
    ieee154e_vars.lastCapturedTime = getdeltaslotperiod_ms();

    if ((actualbestchannel > 11) && (actualbestchannel <= 25)) {
		ieee154e_vars.freq = actualbestchannel;
		radio_setFrequency(ieee154e_vars.freq);
    }
    else{
		openserial_printError(COMPONENT_IEEE802154E,ERR_WRONG_RIT_CHANNEL,
							(errorparameter_t)S_ARM_TXRTS,0x01);
		endSlot();
		return;
    }

	// MONTA O FRAME DO RTS
	frame[p++] = 0x40 | IEEE154_TYPE_ACK;
	frame[p++] = 0xaa;
	frame[p++] = 0xb1;
	frame[p++] = 0xfe;
	frame[p++] = 0xca;
	//address dest
	frame[p++] = actualsrcaddr.addr_16b[1];
	frame[p++] = actualsrcaddr.addr_16b[0];
	//address source
	pmyaddr = idmanager_getMyID(ADDR_16B);
	frame[p++] = pmyaddr->addr_16b[1];
	frame[p++] = pmyaddr->addr_16b[0];
	//RTS OU CTS
	frame[p++] = ARM_RTS_CMD;
	//crc
	frame[p++] = 0x00;
	frame[p++] = 0x00;

	// space for 2-byte CRC
	//packetfunctions_reserveFooterSize(&adv,2);

	radio_loadPacket((uint8_t *)&frame[0],p);

	radio_txEnable();

	RECORD_RADIO_TX_ON
	counttxrts++;

    changeState(S_ARM_TXRTS);

    radio_txNow(TX_USE_CSMA_CA);

    radiotimer_schedule(TX_ARM_RTSECHO_MS);

	#if 0//((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TX == 1))
	{
		uint8_t   pos=0;

		rffbuf[pos++]= RFF_IEEE802_TX;
		rffbuf[pos++]= 0x05;
		rffbuf[pos++]= ieee154e_vars.freq;
		rffbuf[pos++]= 0xee;

		pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
	#endif

}

port_INLINE void ARMactivity_txe05(void) {
	//uint32_t timeelapsedms;
	//uint32_t deltams;
	uint8_t nrPending;
	//uint8_t endslot=TRUE;

	radiotimer_cancel();

    radio_rfOff();

	changeState(S_ARM_RTSNOECHO);

    RECORD_RADIO_TX_OFF(ieee154e_vars.lastCapturedTime)

	incroute(0xE5);

#if ((ENABLE_DEBUG_RFF == 1)  && (DBG_IEEE802_TX == 1))
{
	uint8_t pos=0;

	rffbuf[pos++]= RFF_IEEE802_TX;
	rffbuf[pos++]= 0xE5;
	rffbuf[pos++]= macRITstate;
	rffbuf[pos++]= sRIT_vars.frameType;
	rffbuf[pos++]= ieee154e_vars.dataToSend->l2_retriesLeft;
	pos = printaddress(ieee154e_vars.targetaddr,&rffbuf[0],pos);
	rffbuf[pos++]= 0xee;
	pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
}
#endif

	//TODO!!!AQUI EU DEVO VOLTAR A ABRIR A JANELA POIS PODE TER COLIDIDO UM...MAS PODE HAVER MAIS...SENAO VOU PERDER PARA TODOS...
	//para os frames brodcast deve ser enviado um a um...sinalizo que ja enviei um frame...
	if (sRIT_vars.isBroadcastMulticast == TRUE)
	{
		RITQueue_Clear_Pending(ieee154e_vars.RITQueue_ElementPending,actualsrcaddr);

		nrPending = RITQueue_getNrPendingParents(ieee154e_vars.RITQueue_ElementPending);

		if (nrPending > 0) {

			changeState(S_ARM_RXOLAOFFSET);
			radiotimer_schedule(TX_ARM_DELAYCONTWAIT_MS);
		}
		else {
			if (ieee154e_vars.dataToSend != NULL) {
			  schedule_indicateTx(&ieee154e_vars.asn,TRUE);

			  notif_sendDone(ieee154e_vars.dataToSend,E_FAIL);

			  ieee154e_vars.dataToSend = NULL;
			}

			endSlot();
		}
	}
	else{ //AQUI EH UM DAO OU COAP...

    	//Quando existe um COAP pendente entao eu termino no EndSlot pois ele vai gerar Retries
      	if (!((macRITstate == S_RIT_TX_state) && (sRIT_vars.frameType == IANA_UDP))) {
			// indicate transmit failed to schedule to keep stats
			schedule_indicateTx(&ieee154e_vars.asn,FALSE);

			if (ieee154e_vars.dataToSend != NULL) {
			  // indicate tx fail if no more retries left
			  notif_sendDone(ieee154e_vars.dataToSend,E_FAIL);
			}
			// reset local variable
			ieee154e_vars.dataToSend = NULL;
	   }

		endSlot();
	}

}


/* Aqui eu enviei o RTS e recebi o echo da msg.
 * Agora devo esperar por um RTS
 */
port_INLINE void ARMactivity_tx06(PORT_RADIOTIMER_WIDTH capturedTime) {

	radiotimer_cancel();

	radio_rfOff();

	changeState(S_ARM_RXCTSPREPARE);

	incroute(0x06);

    RECORD_RADIO_TX_OFF(ieee154e_vars.lastCapturedTime)
	radiotxola = ieee154e_vars.radioTxOnTics;

	//O canal eu devo obter do frame do ola
    if ((actualbestchannel > 0) && (actualbestchannel <= 25)) {
		ieee154e_vars.freq = actualbestchannel;
		radio_setFrequency(ieee154e_vars.freq);
    }
    else{
		openserial_printError(COMPONENT_IEEE802154E,ERR_WRONG_RIT_CHANNEL,
							(errorparameter_t)S_ARM_RXCTSPREPARE,0x01);
		endSlot();
		return;
    }

	// enable the radio in Rx mode. The radio is not actively listening yet.
	radio_rxEnable();

#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
{
	uint8_t   pos=0;

	rffbuf[pos++]= RFF_IEEE802_TX;
	rffbuf[pos++]= 0x06;
	pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
}
#endif

   radiotimer_schedule(TX_ARM_DELAYRTS2CTS_MS);
}


port_INLINE void ARMactivity_tx07(void) {

	radiotimer_cancel();

    changeState(S_ARM_RXCTS);

    incroute(0x07);

    RECORD_RADIO_RX_ON

    radio_rxNow();

    radiotimer_schedule(TX_ARM_CTSTIMEOUT_MS);

}



/*
 * Aqui eu enviei um RTS e estava esperando um CTS mas nao chegou...
 *
*/
port_INLINE void ARMactivity_txe07(void) {
	uint8_t nrPending;

	changeState(S_ARM_CTSTIMEOUT);

	radiotimer_cancel();

	radio_rfOff();

    RECORD_RADIO_RX_OFF(ieee154e_vars.lastCapturedTime)

	incroute(0xE7);

	#if ((ENABLE_DEBUG_RFF == 1)  && (DBG_IEEE802_TX == 1))
	{
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_TX;
		rffbuf[pos++]= 0xE7;
		rffbuf[pos++]= macRITstate;
		rffbuf[pos++]= sRIT_vars.frameType;
		rffbuf[pos++]= ieee154e_vars.dataToSend->l2_retriesLeft;
		pos = printaddress(ieee154e_vars.targetaddr,&rffbuf[0],pos);
		rffbuf[pos++]= 0xee;
		pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
	#endif

	//TODO!!!AQUI EU DEVO VOLTAR A ABRIR A JANELA POIS PODE TER COLIDIDO UM...MAS PODE HAVER MAIS...SENAO VOU PERDER PARA TODOS...
	//para os frames brodcast deve ser enviado um a um...sinalizo que ja enviei um frame...
	if (sRIT_vars.isBroadcastMulticast == TRUE)
	{
		RITQueue_Clear_Pending(ieee154e_vars.RITQueue_ElementPending,actualsrcaddr);

		nrPending = RITQueue_getNrPendingParents(ieee154e_vars.RITQueue_ElementPending);

		if (nrPending > 0) {

			changeState(S_ARM_RXOLAOFFSET);
			radiotimer_schedule(TX_ARM_DELAYCONTWAIT_MS);
		}
		else {
			if (ieee154e_vars.dataToSend != NULL) {
			  schedule_indicateTx(&ieee154e_vars.asn,TRUE);

			  notif_sendDone(ieee154e_vars.dataToSend,E_FAIL);

			  ieee154e_vars.dataToSend = NULL;
			}

			endSlot();
		}
	}
	else{ //AQUI EH UM DAO OU COAP...

		//Quando existe um COAP pendente entao eu termino no EndSlot pois ele vai gerar Retries
		if (!((macRITstate == S_RIT_TX_state) && (sRIT_vars.frameType == IANA_UDP))) {
			// indicate transmit failed to schedule to keep stats
			schedule_indicateTx(&ieee154e_vars.asn,FALSE);

			if (ieee154e_vars.dataToSend != NULL) {
			  // indicate tx fail if no more retries left
			  notif_sendDone(ieee154e_vars.dataToSend,E_FAIL);
			}
			// reset local variable
			ieee154e_vars.dataToSend = NULL;
	   }

		endSlot();
	}

}

/* Aqui eu acabei de receber um CTS confirmando o canal de comunicacao com o RX.
 * vou comecar a enviar a msg de dados
 * */
port_INLINE void ARMactivity_tx08(PORT_RADIOTIMER_WIDTH capturedTime) {

//	uint8_t ret = false;
//	uint8_t *msg;
//	uint8_t msglen=0;
	uint32_t duration=0;

    radiotimer_cancel();

    radio_rfOff();

    changeState(S_ARM_TXDATAREADY);

    incroute(0x08);

    RECORD_RADIO_RX_OFF(ieee154e_vars.lastCapturedTime)
    radiorxdutycycle1 = ieee154e_vars.radioRxOnTics;

	//O canal eu devo obter do frame do ola
    if ((actualbestchannel > 0) && (actualbestchannel <= 25)) {
		ieee154e_vars.freq = actualbestchannel;
		radio_setFrequency(ieee154e_vars.freq);
    }
    else{
		openserial_printError(COMPONENT_IEEE802154E,ERR_WRONG_RIT_CHANNEL,
							(errorparameter_t)S_ARM_TXDATAREADY,0x01);
		endSlot();
		return;
    }

	countrxcts++;

#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
{
	uint8_t   pos=0;

	rffbuf[pos++]= RFF_IEEE802_TX;
	rffbuf[pos++]= 0x08;
	rffbuf[pos++]= ieee154e_vars.freq;
	rffbuf[pos++]= 0xee;
	pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
}
#endif

    radio_setFrequency(ieee154e_vars.freq);

	if (ieee154e_vars.RITQueue_ElementPending < maxElements) {

		//sRIT_vars = RITQueue_Get_Element(ieee154e_vars.RITQueue_ElementPending);
		if (macRITstate == S_RIT_multichnhello_state)
			duration++;

		radio_loadPacket(sRIT_vars.msg , sRIT_vars.msglength);

		radio_txEnable();

		radiotimer_schedule(TX_ARM_DELAYRX2TX_MS);
	}
	else
	{
		endSlot();
	}
}

port_INLINE void ARMactivity_tx09(void) {

	radiotimer_cancel();

    changeState(S_ARM_TXDATA);

    RECORD_RADIO_TX_ON

    radio_txNow(TX_USE_CSMA_CA);

    incroute(0x09);

    radiotimer_schedule(TX_ARM_DATAECHO_MS);


#if 0 //((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TX == 1))
{
	uint8_t   pos=0;

	rffbuf[pos++]= RFF_IEEE802_TX;
	rffbuf[pos++]= 0x09;
	rffbuf[pos++]= ieee154e_vars.freq;
	rffbuf[pos++]= 0xee;
	pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
}
#endif
}



port_INLINE void ARMactivity_txe09(void) {

	//uint32_t timeelapsedms;
	//uint32_t deltams;
	uint8_t nrPending;
	//uint8_t endslot=TRUE;

	radiotimer_cancel();

    radio_rfOff();

	incroute(0xE9);
	changeState(S_ARM_DATANOECHO);

	RECORD_RADIO_TX_OFF(ieee154e_vars.lastCapturedTime)

#if ((ENABLE_DEBUG_RFF == 1)  && (DBG_IEEE802_TX == 1))
{
	uint8_t pos=0;

	rffbuf[pos++]= RFF_IEEE802_TX;
	rffbuf[pos++]= 0xE9;
	rffbuf[pos++]= macRITstate;
	rffbuf[pos++]= sRIT_vars.frameType;
	rffbuf[pos++]= ieee154e_vars.dataToSend->l2_retriesLeft;
	pos = printaddress(ieee154e_vars.targetaddr,&rffbuf[0],pos);
	rffbuf[pos++]= 0xee;
	pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);
	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);

}
#endif
	//TODO!!!AQUI EU DEVO VOLTAR A ABRIR A JANELA POIS PODE TER COLIDIDO UM...MAS PODE HAVER MAIS...SENAO VOU PERDER PARA TODOS...
	//para os frames brodcast deve ser enviado um a um...sinalizo que ja enviei um frame...
	if (sRIT_vars.isBroadcastMulticast == TRUE)
	{
		RITQueue_Clear_Pending(ieee154e_vars.RITQueue_ElementPending,actualsrcaddr);

		nrPending = RITQueue_getNrPendingParents(ieee154e_vars.RITQueue_ElementPending);

		if (nrPending > 0) {

			changeState(S_ARM_RXOLAOFFSET);
			radiotimer_schedule(TX_ARM_DELAYCONTWAIT_MS);
		}
		else {
			if (ieee154e_vars.dataToSend != NULL) {
			  schedule_indicateTx(&ieee154e_vars.asn,TRUE);

			  notif_sendDone(ieee154e_vars.dataToSend,E_FAIL);

			}
		    ieee154e_vars.dataToSend = NULL;

		    endSlot();
		}
	}
	else{ //AQUI EH UM DAO OU COAP...
    	//Quando existe um COAP pendente entao eu termino no EndSlot pois ele vai gerar Retries
      	if (!((macRITstate == S_RIT_TX_state) && (sRIT_vars.frameType == IANA_UDP))) {
			// indicate transmit failed to schedule to keep stats
			schedule_indicateTx(&ieee154e_vars.asn,FALSE);

			if (ieee154e_vars.dataToSend != NULL) {
			  // indicate tx fail if no more retries left
			  notif_sendDone(ieee154e_vars.dataToSend,E_FAIL);
			}
			// reset local variable
			ieee154e_vars.dataToSend = NULL;
	   }

		endSlot();

	}


}



/*
 * Aqui o Txdata foi enviado OK.
 * Se o frame eh um DIO aqui eh a finalizacao OK (normal) do TX.
 * Se o frame eh um DAO ou COAP ele deve preparar para o ACK
 */
port_INLINE void ARMactivity_tx0a(PORT_RADIOTIMER_WIDTH capturedTime) {
    //bool listenForAck;
	//open_addr_t address;
	uint8_t nrPending=0;
//	uint8_t ret=0;
	uint8_t endslot=TRUE;
//	uint8_t reenviobroadcast=0;

    radiotimer_cancel();

    changeState(S_ARM_RXACKOFFSET);

	incroute(0x0a);

    radio_rfOff();

	RECORD_RADIO_TX_OFF(ieee154e_vars.lastCapturedTime)
    radiotxdutycycle1 = ieee154e_vars.radioTxOnTics;

#if ((ENABLE_DEBUG_RFF == 1)  && (DBG_IEEE802_TX == 1))
{
	uint8_t pos=0;

	rffbuf[pos++]= RFF_IEEE802_TX;
	rffbuf[pos++]= 0x0a;
	rffbuf[pos++]= sRIT_vars.frameType;
	rffbuf[pos++]= 0xaa;
	//pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);
	pos = printaddress(actualsrcaddr,&rffbuf[0],pos);
	pos = printaddress(ieee154e_vars.targetaddr,&rffbuf[0],pos);

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
}
#endif

	if (sRIT_vars.frameType == IANA_ICMPv6_RA_PREFIX_INFORMATION)  {  //MULTI CHANNEL HELLO CMD
		// notifica o recebimento na tabela de vizinhos
		macneighbors_updtlivelist(actualsrcaddr);

		incroute(0xA1);
		ritstat.txola.countdatatxok++;

#if (ENABLE_LIVELIST_ACK == 1)
	   ARMactivity_tx0b();
	   //radiotimer_schedule(RX_ARM_DELAY_TX2RX_MS);
	   endslot = FALSE;
#else
		//update statistics
		macneighbors_updtstatistics(actualsrcaddr,sRIT_vars.frameType, E_SUCCESS);

		RITQueue_Clear_Pending(ieee154e_vars.RITQueue_ElementPending,actualsrcaddr);

		nrPending = RITQueue_getNrPendingParents(ieee154e_vars.RITQueue_ElementPending);

		if (nrPending > 0) {
			incroute(0xA4);
			changeState(S_ARM_RXOLAOFFSET);
			radiotimer_schedule(TX_ARM_DELAYCONTWAIT_MS);

		   endslot = FALSE;
		}

#endif

	}
	else if (sRIT_vars.frameType == IANA_ICMPv6_RPL_DIO) {

	    //QDO O RPL DIO -> CHECAR SE JA ENVIOU DIO PARA TODOS OS VIZINHOS
		//SENAO...REPROGRAMAR E TORNAR A ESPERAR UM NOVO OLA...ATE TERMINAR OS VIZINHOS OU A JANELA DE RIT_TX.
		RITQueue_Clear_Pending(ieee154e_vars.RITQueue_ElementPending,actualsrcaddr);

		incroute(0xA2);

		//update statistics
		macneighbors_updtstatistics(ieee154e_vars.targetaddr,sRIT_vars.frameType, E_SUCCESS);

		nrPending = RITQueue_getNrPendingParents(ieee154e_vars.RITQueue_ElementPending);

		if (nrPending > 0) {

			incroute(0xA5);
			changeState(S_ARM_RXOLAOFFSET);
			radiotimer_schedule(TX_ARM_DELAYCONTWAIT_MS);

            endslot = FALSE;
		}
		else{
			incroute(0xA6);

			ritstat.txdio.countdatatxok++;
		}


	}
	else  { //frameType  == (DAO OU COAP)
        incroute(0xA3);

	   if (ieee154e_vars.dataToSend != NULL)
	   {
		   // decides whether to listen for an ACK
		   //if (packetfunctions_isBroadcastMulticast(&ieee154e_vars.dataToSend->l2_nextORpreviousHop) !=TRUE) {
		   if (sRIT_vars.isBroadcastMulticast == FALSE) {

               //estatisticas COAP
               if (sRIT_vars.frameType == IANA_UDP) {
                   ritstat.txcoap.countdatatxok++;
               }

			   ARMactivity_tx0b();
			   endslot = FALSE;
		   }
	   }
	}

	if (endslot == TRUE) {
		//calcula o tempo decorrido do radio ligado
		//ieee154e_vars.radioOnTics+=(radio_getTimerValue()-ieee154e_vars.radioOnInit);

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
port_INLINE void ARMactivity_tx0b(void) {

	changeState(S_ARM_RXACK);

	incroute(0x0b);

	// o Ack eh enviado no canal do RX...ja esta setado
	//ieee154e_vars.freq = macneighbors_getMyBestChan();

	// configure the radio for that frequency
	radio_setFrequency(ieee154e_vars.freq);

	// enable the radio in Rx mode. The radio is not actively listening yet.
	radio_rxEnable();

	RECORD_RADIO_RX_ON

	radio_rxNow();

	radiotimer_schedule(TX_ARM_ACKTIMEOUT_MS);

	#if 0 // ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TIMER == 1))
	{
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_TX;
		rffbuf[pos++]= 0x0b;

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
	#endif

}


port_INLINE void ARMactivity_txe0b() {

	radiotimer_cancel();

	radio_rfOff();

	changeState(S_ARM_ACKTIMEOUT);

	RECORD_RADIO_RX_OFF (ieee154e_vars.lastCapturedTime)

	incroute(0xEb);

	//update statistics
	if (ieee154e_vars.RITQueue_ElementPending < maxElements) {
		//decrementa elemento pendente...para broadcast
		RITQueue_update_element(ieee154e_vars.RITQueue_ElementPending);

		//sRIT_vars = RITQueue_Get_Element(ieee154e_vars.RITQueue_ElementPending);

	    macneighbors_updtstatistics(ieee154e_vars.targetaddr,sRIT_vars.frameType, E_FAIL);
    }


#if ((ENABLE_DEBUG_RFF == 1)  && (DBG_IEEE802_TX == 1))
{
	uint8_t pos=0;

	rffbuf[pos++]= RFF_IEEE802_TX;
	rffbuf[pos++]= 0xEb;
	rffbuf[pos++]= macRITstate;
	rffbuf[pos++]= sRIT_vars.frameType;
	rffbuf[pos++]= ieee154e_vars.dataToSend->l2_retriesLeft;
	pos = printaddress(ieee154e_vars.targetaddr,&rffbuf[0],pos);
	rffbuf[pos++]= 0xee;
	pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);
	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);

}
#endif

	//Quando existe um COAP pendente entao eu termino no EndSlot pois ele vai gerar Retries
		if (!((macRITstate == S_RIT_TX_state) && (sRIT_vars.frameType == IANA_UDP))) {
		// indicate transmit failed to schedule to keep stats
		schedule_indicateTx(&ieee154e_vars.asn,FALSE);

		if (ieee154e_vars.dataToSend != NULL) {
		  // indicate tx fail if no more retries left
		  notif_sendDone(ieee154e_vars.dataToSend,E_FAIL);
		}
		// reset local variable
		ieee154e_vars.dataToSend = NULL;
	}

	endSlot();
}



/*
 * Aqui indica que quando TX eu estava esperando um ack e ele chegou.
 * Esta é a finalizacao OK de um frame com COAP ou DAO.
 *
 */
port_INLINE void ARMactivity_tx0c(PORT_RADIOTIMER_WIDTH capturedTime) {

	ieee802154_header_iht ieee802514_header;
	uint8_t frameOK=0;
	open_addr_t rxaddr_dst;

   radiotimer_cancel();

   changeState(S_ARM_TXPROC);

   radio_rfOff();

   RECORD_RADIO_RX_OFF (ieee154e_vars.lastCapturedTime)
   radiorxdutycycle2 = ieee154e_vars.radioRxOnTics;

   incroute(0x0c);

   ieee154e_vars.ackReceived = openqueue_getFreePacketBuffer(COMPONENT_IEEE802154E);
   	if (ieee154e_vars.ackReceived==NULL) {
   	  // log the error
   	  openserial_printError(COMPONENT_IEEE802154E,ERR_NO_FREE_PACKET_BUFFER,
   							(errorparameter_t)0,
   							(errorparameter_t)0);
   	   frameOK = FALSE;
   	}
   	else {

   	   frameOK = checkframeok(IEEE154_TYPE_ACK, ieee154e_vars.ackReceived,&ieee802514_header);

   	   memcpy(&rxaddr_dst, &(ieee802514_header.dest),sizeof(open_addr_t));
   	   memcpy(&actualsrcaddr, &(ieee802514_header.src),sizeof(open_addr_t));


		#if 0// ((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_TX == 1))
		  {
			uint8_t   pos=0;

			if (ieee154e_vars.RITQueue_ElementPending < maxElements)
			{
				sRIT_vars = RITQueue_Get_Element(ieee154e_vars.RITQueue_ElementPending);

				rffbuf[pos++]= RFF_IEEE802_TX;
				rffbuf[pos++]= 0x09;
				rffbuf[pos++]= sRIT_vars.frameType;
				rffbuf[pos++]= sRIT_vars.msglength;

				pos = printaddress(ieee154e_vars.targetaddr,&rffbuf[0],pos);

				openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
			}
		  }
		#endif
  }

	if (frameOK  == TRUE) {

		macneighbors_updtstatistics(ieee154e_vars.targetaddr,sRIT_vars.frameType, E_SUCCESS);

		if ((ieee802514_header.frameType == IEEE154_TYPE_CMD) &&
			(ieee154e_vars.ackReceived->packet[10] == CMDLIVELIST) ) {
			/*(idmanager_isMyAddress(&auxdstaddr)) ) {*/
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
	}
	else {
	   incroute(0x89);
	   //update statistics
	   macneighbors_updtstatistics(ieee154e_vars.targetaddr,sRIT_vars.frameType, E_FAIL);

       RITQueue_Free(ieee154e_vars.RITQueue_ElementPending);
   }

   if (ieee154e_vars.ackReceived != NULL) {
	   // free the received ack so corresponding RAM memory can be recycled
	   openqueue_freePacketBuffer(ieee154e_vars.ackReceived);
	   // clear local variable
	   ieee154e_vars.ackReceived = NULL;
   }

   endSlot();
}





//======= frame validity check

/**
\brief Decides whether the packet I just received is valid received frame.

A valid Rx frame satisfies the following constraints: (PARA O ARM)  TODO!!! PARA O AMAC OU ARM DEVE ACEITAR O ACK TAMBEM
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

#if 0
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
#endif

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

   ieee154e_vars.laststate = ieee154e_vars.state;
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

    // turn off the radio
    radio_rfOff();

	radiotimer_cancel();

    radio_flushfifos();

	// record the captured time
    ieee154e_vars.lastCapturedTime = getdeltaslotperiod_ms();

    //imprimir estatisticas e debugs
#if (ENABLE_DEBUG_RFF == 1)
    if ((macRITstate > S_RIT_sleep_state) && (macRITstate <= S_RIT_RX_livelist)) {
	   treatdebug(macRITstate);
    }
#endif
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

			  txpending = FALSE;
		}
		else {

		  if (macRITstate == S_RIT_TX_state){

			  // if everything went well, dataToSend was set to NULL in ti9
			  // getting here means transmit failed

			  // indicate Tx fail to schedule to update stats
			  schedule_indicateTx(&ieee154e_vars.asn,FALSE);

			  //decrement transmits left counter
			  ieee154e_vars.dataToSend->l2_retriesLeft--;

			  if (ieee154e_vars.dataToSend->l2_retriesLeft==0) {
				 // indicate tx fail if no more retries left
				 notif_sendDone(ieee154e_vars.dataToSend,E_FAIL);
				 txpending = FALSE;
			  } else {
				 // return packet to the virtual COMPONENT_SIXTOP_TO_IEEE802154E component
				 ieee154e_vars.dataToSend->owner = COMPONENT_SIXTOP_TO_IEEE802154E;

				 //ligo o flag de txpendig para ser utilizado no proximo ciclo do RIT
				 txpending=TRUE;
			  }
			}
		  else
		  {
			txpending = FALSE;
		  }

		}

		// reset local variable
		ieee154e_vars.dataToSend = NULL;
   }
   else{
	   txpending = FALSE;
   }


   //clean up RIT vars
	clearritroute();
	actualbestchannel = 0;
	frameDioPending = 0;
	macRITstate = 0;
	macRIT_Pending_RX_frameType = 0;
	macRIT_Pending_TX_frameType = 0;
	RITQueue_cleanupoldmsg();
	sRIT_vars = RITQueue_Get_Element(ieee154e_vars.RITQueue_ElementPending);

	//clear vars for duty cycle on this slot
	ieee154e_vars.radioOnThisSlot=FALSE;

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

	// change state
	changeState(S_ARM_SLEEP);
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
//RX -------------------

      case S_ARM_TXOLAPREPARE:
		  ARMactivity_rx01();
		 break;
	  case S_ARM_TXOLA:
		  ARMactivity_rxe01();
		 break;
	  case S_ARM_RXRTSPREPARE:
		  ARMactivity_rx03();
	   	 break;
	  case S_ARM_RXRTS:
		  ARMactivity_rxe03();
		  break;
	  case S_ARM_TXCTSOFFSET:
		  ARMactivity_rx05();
		  break;
	  case S_ARM_TXCTS:
		  ARMactivity_rxe05();
		  break;
	  case S_ARM_RXDATAPREPARE:
		  ARMactivity_rx07();
		  break;
	  case S_ARM_RXDATA:
		  ARMactivity_rxe07();
		  break;
#if (ENABLE_LIVELIST_ACK == 1)
	  case S_ARM_LLTXACKOFFSET:
		 ARMactivity_rx81();
		 break;
#endif
	  case S_ARM_TXACKOFFSET:
		  ARMactivity_rx09();
		  break;
      case S_ARM_TXACKPREPARE:
    	  ARMactivity_rx0a();
         break;
      case S_ARM_TXACK:
    	  ARMactivity_rxe0a();
         break;

//TX -------------------
  	  case S_ARM_RXOLAOFFSET:
  		 ARMactivity_tx02();
  		 break;
  	  case S_ARM_RXOLAPREPARE:
		 ARMactivity_tx03();
		 break;
 	  case S_ARM_RXOLA:
 		 ARMactivity_txe03();
 		  break;
	  case S_ARM_TXRTSOFFSET:
		  ARMactivity_tx05();
		 break;
	  case S_ARM_TXRTS:
		  ARMactivity_txe05();
		  break;
     case S_ARM_RXCTSPREPARE:
    	  ARMactivity_tx07();
		  break;
	  case S_ARM_RXCTS:
		 ARMactivity_txe07();
		 break;
 	  case S_ARM_TX_CONTINUEWAIT:
 		  ARMactivity_tx45();
		  break;
	  case S_ARM_TXDATAREADY:
	     ARMactivity_tx09();
		 break;
	  case S_ARM_TXDATA:
		 ARMactivity_txe09();
		 break;
	  case S_ARM_RXACK:
		 ARMactivity_txe0b();
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
//RX -------------------

     case S_ARM_TXOLA:
    	ARMactivity_rx02(capturedTime);
		break;
     case S_ARM_RXRTS:
    	ARMactivity_rx04(capturedTime);
		break;
	  case S_ARM_TXCTS:
		ARMactivity_rx06(capturedTime);
		break;
	 case S_ARM_RXDATA:
		 ARMactivity_rx08(capturedTime);
		break;
	 case S_ARM_TXACK:
		 ARMactivity_rx0b(capturedTime);
		break;

//TX----------------
	 case S_ARM_RXOLA:
		 ARMactivity_tx04(capturedTime);
		break;
     case S_ARM_TXRTS:
		 ARMactivity_tx06(capturedTime);
		break;
     case S_ARM_RXCTS:
		  ARMactivity_tx08(capturedTime);
		break;
	 case S_ARM_TXDATA:
		 ARMactivity_tx0a(capturedTime);
		break;
	 case S_ARM_RXACK:
		 ARMactivity_tx0c(capturedTime);
		break;
	 default:
		// log the error
		openserial_printError(COMPONENT_IEEE802154E,ERR_WRONG_STATE_IN_ENDOFFRAME,
							  (errorparameter_t)ieee154e_vars.state,
							  (errorparameter_t)macRITstate /*ieee154e_vars.slotOffset */);
		endSlot();
		break;
   }

   ieee154e_dbg.num_endOfFrame++;
}




#endif // (IEEE802154E_RIT == 1)

