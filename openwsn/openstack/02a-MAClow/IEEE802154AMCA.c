#include "opendefs.h"
#include "IEEE802154E.h"
#if (IEEE802154E_AMCA == 1)
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
#include "IEEE802154RIT.h"
#include "SysTick.h"
#include "mac_csp_tx.h"
#include "topology.h"
#include "openbridge.h"
#if (WATCHDOG_CONF_ENABLE == 1)
#include "watchdog.h"
#endif
#include "stdlib.h"

#define TESTE_TIMER 0
#define TEST_MEASURE_NEIGHBOR_RATE 0

#define TRATA_ACK 1

//=========================== variables =======================================
extern uint8_t rffpasshere;
extern radio_csma_vars_t radio_csma_vars;
extern uint32_t ritTimingclock;
extern uint32_t systickcount;
extern uint32_t lastradiotimer_freerunnig;
extern uint8_t ucFlagForwarding;
extern scheduler_vars_t scheduler_vars;
extern scheduler_dbg_t  scheduler_dbg;
extern sRITqueue pvObjList[MAX_RIT_LIST_ELEM];
extern uint8_t frameDioPending;

uint8_t nrErrorRetries;
uint8_t COAPSimCount;

uint8_t msghello[30];
uint8_t isrmultichannelhello;
uint32_t slottimeref;
uint8_t flagSerialTx;
uint8_t lastslotwastx;
uint8_t testrff_isforwarding;
uint8_t ucFlagTxReOpen;
uint8_t rfftooglelastype=0;
uint8_t printstattoggle=0;
open_addr_t actualsrcaddr;         //usado quando recebe um frame...para identificar quem enviou...
ieee154e_vars_t    ieee154e_vars;
ieee154e_stats_t   ieee154e_stats;
ieee154e_dbg_t     ieee154e_dbg;
open_addr_t        address_1;
uint8_t            flagreschedule=0;
sRITelement        element;

uint8_t            coappending;
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

//RIT period value
static uint16_t macRITPeriod;
static uint16_t macRITTxPeriod;
//On in RIT period
static uint16_t macRITDataWaitDuration;
//Off in RIT period
static uint16_t macRITsleepPeriod;
//RIT period interrupted: On waiting for Olà
static uint16_t macRITRXforTxPeriod;
static uint16_t macAckWaitPeriod;
volatile uint32_t actualtimer0;

//teste rff
uint8_t rffslotOffset;
uint8_t rffframelen;
uint8_t rffstate;

#if 1 //ENABLE_DEBUG_RFF
uint8_t rffbuf[MAX_ELEM_ROUTE];
uint8_t ritroute[MAX_ELEM_ROUTE];
uint8_t lastritroutepos = 0;

#define DBG_802154E_TX_DATA 1
#define DBG_802154E_RX_DATA 0
uint8_t rffnewsendmsg;
#define RFF_LOG_DEBUG_DAO 1
#endif

uint8_t RFFcount=0;
uint8_t rffcountdao=0;
uint8_t rffcountdao_tx=0;
uint8_t rffcountdao_txok=0;
uint8_t rffcountdao_txack=0;
uint8_t rffcountdao_rx=0;
uint8_t u8rffcounterror=0;
uint8_t rffcountdio=0;

uint16_t rffcountolatx;
uint16_t rffcountolarx;

extern OpenQueueEntry_t advRIT;

#define DEBUG_ACK  0
uint8_t MsgNeedAck;

uint32_t moteerr1count=0;
uint32_t moteerr2count=0;
uint32_t moteOKcount=0;
uint32_t moteRetrycount=0;
uint32_t testTx=0;

enum discardfrm {
  DISCARD_NO_ENDSLOT_NO=0,
  DISCARD_NO_ENDSLOT_YES,
  DISCARD_YES_ENDSLOT_YES,
};
//=========================== prototypes ======================================
void ieee154e_init(void);
// interrupts
void     isr_ieee154e_newSlot(void);
void     isr_ieee154e_timer(void);
void     isr_multichannelhello(void);
port_INLINE void getRitRequest(void);
void clearstatistics(void);

port_INLINE uint8_t activityrx_reopenrxwindow(open_addr_t addr);
port_INLINE uint8_t activitytx_reopenrxwindow(void);
port_INLINE void activitytx_olaackprepare(PORT_RADIOTIMER_WIDTH capturedTime);
bool RITQueue_ExistFramePending(void);
port_INLINE void activitytx_preparedata(PORT_RADIOTIMER_WIDTH capturedTime);

owerror_t openqueue_freePacketRITBuffer(OpenQueueEntry_t* pkt);
port_INLINE void activitytx_senddata(void);

// SYNCHRONIZING
void     activity_synchronize_startOfFrame(PORT_RADIOTIMER_WIDTH capturedTime);
void     activity_synchronize_endOfFrame(PORT_RADIOTIMER_WIDTH capturedTime);

extern macneighbors_vars_t macneighbors_vars;

// TX
void     activity_ti1ORri1(void);
port_INLINE void StartTxMultiChannelHello (void);
port_INLINE void StartTxRITProcedure(uint8_t elepending, uint8_t newtxframe);
port_INLINE void StartRxRITProcedure(void);
port_INLINE void activityrx_rxolanoecho(void);
port_INLINE void activityrx_waitolaecho(void);
port_INLINE void activityrx_sendola(void);
port_INLINE void activityrx_dataprepare(PORT_RADIOTIMER_WIDTH capturedTime);
void     activity_rxwindowend(void);
port_INLINE void activitytx_rxolaprepare(void);
port_INLINE void activityrx_senddataack(void);

void     activitytx_senddone(PORT_RADIOTIMER_WIDTH capturedTime);
port_INLINE void activitytx_rxwaitforack(PORT_RADIOTIMER_WIDTH capturedTime);
void     activityrx_noacktx(void);
void     activitytx_rxnoack(void);
void     activitytx_rxackok(PORT_RADIOTIMER_WIDTH capturedTime);
// RX
port_INLINE uint8_t activityrx_prepareritdatareq(uint8_t hello);
void     activitytx_rxwindowopen(void);
void     activity_ritrxlistening(void);
void     activity_rxritwindowend(void);
void     activity_txritwindowend(void);
void     activity_rxnewframe(PORT_RADIOTIMER_WIDTH capturedTime);
port_INLINE void activityrx_preparetxack(PORT_RADIOTIMER_WIDTH capturedTime);

void     activityrx_txackok(PORT_RADIOTIMER_WIDTH capturedTime);

void activity_RITDoNothing(void);
uint8_t toogleTxRxSerial(void);
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


uint32_t getdeltaTimerValue(void) {
	volatile uint32_t value;
	volatile uint32_t delta;

	value = radio_getTimerValue();
	if (value >= actualtimer0)
		delta = value - actualtimer0;
	else
		delta = ((0xFFFFFFFF) - actualtimer0) + value;


	actualtimer0 = value;

 return delta;
}

/*
 * Retorna o tempo (em ms) decorrido desde o inicio do slot time
 */
uint32_t getdeltaslotperiod_ms(void) {
	volatile uint32_t value;
	volatile uint32_t delta;

	value = radio_getTimerValue();
	if (value >= slottimeref)
		delta = value - slottimeref;
	else
		delta = ((0xFFFFFFFF) - slottimeref) + value;

	value = delta*RADIOTIMER_TICS_MS+1;
 return (value);
}

//=========================== public ==========================================

void clearstatistics(void)
{
    //rit statistics - zera as variaveis
	ritstat.txola.countdata=0;
	ritstat.txola.countdatatx=0;
	ritstat.txola.countdatatxok=0;
	ritstat.txola.countdatatxerr=0;
	ritstat.txola.countack=0;
	ritstat.txola.countacktxrx=0;
	ritstat.txola.countacktxrxok=0;
	ritstat.txola.countacktxrxerr=0;

	ritstat.rxola.countdata=0;
	ritstat.rxola.countdatatx=0;
	ritstat.rxola.countdatatxok=0;
	ritstat.rxola.countdatatxerr=0;
	ritstat.rxola.countack=0;
	ritstat.rxola.countacktxrx=0;
	ritstat.rxola.countacktxrxok=0;
	ritstat.rxola.countacktxrxerr=0;

	ritstat.txdio.countdata=0;
	ritstat.txdio.countdatatx=0;
	ritstat.txdio.countdatatxok=0;
	ritstat.txdio.countdatatxerr=0;
	ritstat.txdio.countack=0;
	ritstat.txdio.countacktxrx=0;
	ritstat.txdio.countacktxrxok=0;
	ritstat.txdio.countacktxrxerr=0;
	ritstat.rxdio=0;
	ritstat.txdao.countdata=0;
	ritstat.txdao.countdatatx=0;
	ritstat.txdao.countdatatxok=0;
	ritstat.txdao.countdatatxerr=0;
	ritstat.txdao.countack=0;
	ritstat.txdao.countacktxrx=0;
	ritstat.txdao.countacktxrxok=0;
	ritstat.txdao.countacktxrxerr=0;
	ritstat.rxdao=0;

	ritstat.txcoap.countdata=0;
	ritstat.txcoap.countdatatx=0;
	ritstat.txcoap.countdatatxok=0;
	ritstat.txcoap.countdatatxerr=0;
	ritstat.txcoap.countack=0;
	ritstat.txcoap.countacktxrx=0;
	ritstat.txcoap.countacktxrxok=0;
	ritstat.txcoap.countacktxrxerr=0;

	coappending = 0;

}

void clearritroute(void)
{
#if ENABLE_DEBUG_RFF
	int i;

	lastritroutepos = 0;
	for (i=0;i<20;i++)
	ritroute[i] = 0;
#endif
}

#if (ENABLE_DEBUG_RFF == 1)

uint8_t printvar(uint8_t *var,uint8_t size, uint8_t *buf,uint8_t pos){

	//buf[pos++]= 0xBB;

	switch(size)
	{
		case 1:
			buf[pos++]= *var;
			break;
		case 2:
			buf[pos++]= *var++;
			buf[pos++]= *var;
			break;
		case 4:
			buf[pos++]= *var++;
			buf[pos++]= *var++;
			buf[pos++]= *var++;
			buf[pos++]= *var;
			break;
		default:
			break;
	}

    return pos;
}

uint8_t printaddress(open_addr_t addr,uint8_t *buf,uint8_t pos){

	buf[pos++]= 0xAA;
	//buf[pos++]= addr.type;
    if (addr.type == 0x01)
    {
		buf[pos++]= addr.addr_16b[0];
		buf[pos++]= addr.addr_16b[1];
    }
    else if (addr.type == 0x02)	{
		buf[pos++]= addr.addr_64b[6];
		buf[pos++]= addr.addr_64b[7];
    }
	else if (addr.type == 0x03) {
		buf[pos++]= addr.addr_128b[14];
		buf[pos++]= addr.addr_128b[15];
	}

    return pos;

}


/* ignorar imprimir RX quando entrar nos seguintes casos:
   0x85 - rxwindowend (acontece toda hora)
   0x68 - quando ele receber um frame porem é um ola.
*/
const uint8_t noprint[]={0x84,0x85,0x68};
uint8_t checkimprimir(void)
{
   uint8_t j,i;

#if 1
    for(j=0;j<sizeof(noprint);j++){
    	for(i=0;i<lastritroutepos;i++){
    		if (ritroute[i] == noprint[j])
    			return 0;
    	}
    }
#endif
	return TRUE;
}

uint8_t incroute(uint8_t element)
{
#if ENABLE_DEBUG_RFF
    //if (macRITstate == S_RIT_TX_state)
    {
		if (lastritroutepos < MAX_ELEM_ROUTE)
		{
			ritroute[lastritroutepos] = element;
			lastritroutepos++;
		}
		else
		{
			lastritroutepos = 0;
			ritroute[lastritroutepos] = 0xCC;
			ritroute[lastritroutepos] = element;
		}
    }

    return lastritroutepos;
#else
    return 0;
#endif

}
//imprime as estatisticas
void printstat(void){
	uint8_t pos=0;
	uint8_t i=0;

	if (1) { // (printstattoggle == TRUE) {
		printstattoggle = FALSE;
		rffbuf[pos++]  = 0x96;   //ESTATISTICAS LIVELIST ATUAL DA VIZINHANÇA
		rffbuf[pos++]= idmanager_getIsDAGroot();   //Mostra se é SINK =1  ou MOTE=0

		for (i=0;i<MAXNUMMACNEIGHBORS;i++) {
		  if (macneighbors_vars.neighbors[i].used==TRUE) {
			pos = printaddress(macneighbors_vars.neighbors[i].addr_16b,&rffbuf[0],pos);
			rffbuf[pos++]  = macneighbors_vars.neighbors[i].bestchan;
			rffbuf[pos++]  = macneighbors_vars.neighbors[i].stableNeighbor;
			//rffbuf[pos++]  = macneighbors_vars.neighbors[i].switchStabilityCounter;
			pos = printvar((uint8_t *)&macneighbors_vars.neighbors[i].numTxDIO,sizeof(uint8_t),rffbuf,pos);
			//pos = printvar((uint8_t *)&macneighbors_vars.neighbors[i].numTxDIOErr,sizeof(uint8_t),rffbuf,pos);
			pos = printvar((uint8_t *)&macneighbors_vars.neighbors[i].numTxDAO,sizeof(uint8_t),rffbuf,pos);
			//pos = printvar((uint8_t *)&macneighbors_vars.neighbors[i].numTxDAOErr,sizeof(uint8_t),rffbuf,pos);
		  }
	    }
	    //Nro de COAP
#if (SINK == 1)
		rffbuf[pos++] = 0xee;

		pos = printvar((uint8_t *)&ritstat.txcoap.countdata     ,sizeof(uint16_t),rffbuf,pos);
		pos = printvar((uint8_t *)&ritstat.txcoap.countdatatxok ,sizeof(uint16_t),rffbuf,pos);
		pos = printvar((uint8_t *)&ritstat.txcoap.countacktxrxok,sizeof(uint16_t),rffbuf,pos);
#endif
	}
	else {
		printstattoggle = TRUE;
#if 0
		rffbuf[pos++]= 0x94; //ESTATISTICAS EXTERNA DO NR DE FRAMES ENVIADOS E RECEBIDOS
	    pos = printvar((uint8_t *)&ritstat.txdio.countdata,sizeof(uint16_t),rffbuf,pos);
	    pos = printvar((uint8_t *)&ritstat.txdio.countdatatx,sizeof(uint16_t),rffbuf,pos);
	    pos = printvar((uint8_t *)&ritstat.txdio.countdatatxok,sizeof(uint16_t),rffbuf,pos);
	    pos = printvar((uint8_t *)&ritstat.rxdio,sizeof(uint16_t),rffbuf,pos);

	    rffbuf[pos++]= 0x93;   //ESTATISTICAS INTERNA DOS SLOTS PRODUZIDOS E FINALIZADOS
		rffbuf[pos++]= idmanager_getIsDAGroot();

		//pos = printvar((uint8_t *)&ieee154e_dbg.num_newSlot,sizeof(uint32_t),rffbuf,pos);
		pos = printvar((uint8_t *)&ieee154e_dbg.num_txslot,sizeof(uint32_t),rffbuf,pos);
		pos = printvar((uint8_t *)&ieee154e_dbg.num_txend,sizeof(uint32_t),rffbuf,pos);
		pos = printvar((uint8_t *)&ieee154e_dbg.num_rxslot,sizeof(uint32_t),rffbuf,pos);
		pos = printvar((uint8_t *)&ieee154e_dbg.num_rxend,sizeof(uint32_t),rffbuf,pos);
#endif
	}

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
}

void printroute(void)
{
	uint8_t i=0;
	uint8_t pos=0;
	uint8_t imprimir=0;
    uint8_t lastpos=0;

	lastpos = incroute(0xCC);

	if (macRITstate == S_RIT_RX_state){
		imprimir = checkimprimir();
		rffbuf[pos++]= 0x98;  //DEGUB - MOSTRA A ROTA DO SLOT RX
	}
	else if (macRITstate == S_RIT_multichnhello_state){
		imprimir = 1;
		rffbuf[pos++]= 0x97; //DEGUB - MOSTRA A ROTA DO SLOT LIVELIST
	}
	else{
		imprimir = 1;     //DEGUB - MOSTRA A ROTA DO SLOT TX
		rffbuf[pos++]= 0x99;
	}


	if  (imprimir) {

		for(i=0;i<lastpos;i++)
		{
			rffbuf[pos++]=ritroute[i];
		}

		if ( pos < 40) {
			if (macRITstate == S_RIT_RX_state) {
				pos = printvar((uint8_t *)&ieee154e_dbg.num_txslot,sizeof(uint32_t),rffbuf,pos);
				pos = printvar((uint8_t *)&ieee154e_dbg.num_txend,sizeof(uint32_t),rffbuf,pos);
				pos = printvar((uint8_t *)&ieee154e_dbg.num_rxslot,sizeof(uint32_t),rffbuf,pos);
				pos = printvar((uint8_t *)&ieee154e_dbg.num_rxend,sizeof(uint32_t),rffbuf,pos);
			}
			else if (macRITstate == S_RIT_multichnhello_state) {
				pos = printvar((uint8_t *)&ieee154e_dbg.num_txslot,sizeof(uint16_t),rffbuf,pos);
				pos = printvar((uint8_t *)&ieee154e_dbg.num_txend,sizeof(uint16_t),rffbuf,pos);
			}
		}

		//pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);
	    //pos = printvar((uint8_t *)&ieee154e_dbg.num_rxslot,sizeof(uint16_t),rffbuf,pos);
	    //pos = printvar((uint8_t *)&ieee154e_dbg.num_rxend,sizeof(uint16_t),rffbuf,pos);

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
   }

}


#endif

void ieee154e_init(void) {

   // initialize variables
   memset(&ieee154e_vars,0,sizeof(ieee154e_vars_t));
   memset(&ieee154e_dbg,0,sizeof(ieee154e_dbg_t));

   changeIsSync(TRUE);
   ucFlagForwarding = FALSE;
   ucFlagTxReOpen = FALSE;

   resetStats();
   ieee154e_stats.numDeSync  = 0;
   rfftooglelastype = 0;
   COAPSimCount=0;
   flagSerialTx = 0;
   flagreschedule = 0;
   moteerr1count=0;
   moteerr2count=0;
   moteOKcount=0;
   moteRetrycount=0;
   testTx=0;
   isrmultichannelhello = 0;
   macRITstate = 0;
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
	clearstatistics();
	testrff_isforwarding = 0;

	leds_all_off();
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

   opentimers_start(RIT_MULTICHANNELHELLO_MS, TIMER_PERIODIC, TIME_MS, (opentimers_cbt) isr_multichannelhello);

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
  uint8_t ret;

#if (WATCHDOG_CONF_ENABLE == 1)
  WatchdogClear();
#endif
  ieee154e_dbg.num_newSlot++;
  RFFcount = 0;
  nrErrorRetries = 0;
  rffpasshere = 0;

#if 0 //(SINK == 1)
  ret = checkinterrupt();

  if (ret)
  {
	#if 1 //((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_TX == 1))
	  {
		uint8_t   pos=0;
		uint8_t *pucAux = (uint8_t *) &ieee154e_dbg.num_newSlot;

		rffbuf[pos++]= 0xFE;
		rffbuf[pos++]= 0xFE;
		rffbuf[pos++]= macRITstate;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	  }
	#endif
  }
  //radiotimer_clearValue();
#endif

  clearritroute();

  activity_ti1ORri1();

}


port_INLINE void activitytx_datanoecho(void) {

	uint32_t timeelapsedms;
	uint32_t deltams;
	uint8_t nrPending;
	uint8_t endslot=TRUE;

	radiotimer_cancel();

    radio_rfOff();

	incroute(0x87);
	changeState(S_RIT_TXDATANOECHO);

    ieee154e_vars.lastCapturedTime = getdeltaslotperiod_ms();
    //ieee154e_vars.radioOnTics +=(radio_getTimerValue()-ieee154e_vars.radioOnInit);

	//TODO!!!AQUI EU DEVO VOLTAR A ABRIR A JANELA POIS PODE TER COLIDIDO UM...MAS PODE HAVER MAIS...SENAO VOU PERDER PARA TODOS...
	//para os frames brodcast deve ser enviado um a um...sinalizo que ja enviei um frame...
	if (sRIT_vars.isBroadcastMulticast == TRUE)
	{
		RITQueue_Clear_Pending(ieee154e_vars.RITQueue_ElementPending,actualsrcaddr);

		nrPending = RITQueue_getNrPendingParents(ieee154e_vars.RITQueue_ElementPending);

		if (nrPending > 0) {

			changeState(S_RIT_TXDATAOFFSET);
			radiotimer_schedule(TX_RIT_DELAYCONTWAIT_MS);

			#if 1 // ((ENABLE_DEBUG_RFF == 1)  && (DBG_RADIO_POWER_CONS == 1))
			{
				uint8_t pos=0;

				rffbuf[pos++]= RFF_IEEE802_TX;
				rffbuf[pos++]= 0x87;
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

			#if 1 // ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
			{
				uint8_t   pos=0;

				rffbuf[pos++]= RFF_IEEE802_RX;
				rffbuf[pos++]= 0x87;
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



//======= misc

/**
\brief Trigger this module to print status information, over serial.

debugPrint_* functions are used by the openserial module to continuously print
status information about several modules in the OpenWSN stack.

\returns TRUE if this function printed something, FALSE otherwise.
*/
bool debugPrint_asn() {
   asn_t output;
   output.byte4         =  ieee154e_vars.asn.byte4;
   output.bytes2and3    =  ieee154e_vars.asn.bytes2and3;
   output.bytes0and1    =  ieee154e_vars.asn.bytes0and1;
   openserial_printStatus(STATUS_ASN,(uint8_t*)&output,sizeof(output));
   return TRUE;
}

/**
\brief Trigger this module to print status information, over serial.

debugPrint_* functions are used by the openserial module to continuously print
status information about several modules in the OpenWSN stack.

\returns TRUE if this function printed something, FALSE otherwise.
*/
bool debugPrint_isSync() {
   uint8_t output=0;
   output = ieee154e_vars.isSync;
   openserial_printStatus(STATUS_ISSYNC,(uint8_t*)&output,sizeof(uint8_t));
   return TRUE;
}

/**
\brief Trigger this module to print status information, over serial.

debugPrint_* functions are used by the openserial module to continuously print
status information about several modules in the OpenWSN stack.

\returns TRUE if this function printed something, FALSE otherwise.
*/
bool debugPrint_macStats() {
   // send current stats over serial
   openserial_printStatus(STATUS_MACSTATS,(uint8_t*)&ieee154e_stats,sizeof(ieee154e_stats_t));
   return TRUE;
}

//=========================== private =========================================
slotOffset_t RIT_checkpendingmsginsomeslot(open_addr_t *neighbor){

	slotOffset_t slotOffset;
	open_addr_t auxneighbor;
    OpenQueueEntry_t* dataToSend;

	//encontro um vizinho valido rodando os varios slots
	for (slotOffset=ieee154e_vars.slotOffset ;slotOffset < 4;slotOffset++)
	{
		schedule_syncSlotOffset(slotOffset);
	    ieee154e_vars.nextActiveSlotOffset = schedule_getNextActiveSlotOffset();

	    schedule_getNeighbor(&auxneighbor);

  	    //verifico se ele tem mensagem pendende
		dataToSend = openqueue_macGetDataPacket(&auxneighbor);
		if (dataToSend != NULL)
		{
		   break;
		}

	}

	memcpy (neighbor, &auxneighbor,sizeof(open_addr_t));
    return slotOffset;
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

   ieee154e_vars.lastCapturedTime = 0;
   actualtimer0 = radio_getTimerValue();
   slottimeref = actualtimer0;

   radiotimer_cancel();

    //verifico se existe mensagem pendente para ser enviada
    ieee154e_vars.dataToSend = NULL;
    targetSlotOffset = ieee154e_vars.slotOffset;

#if (TESTRIT_ONLY_OLA == 0)
    if (1)  //(txpending == FALSE)
	{
		schedule_syncSlotOffset(targetSlotOffset);
	    ieee154e_vars.nextActiveSlotOffset = schedule_getNextActiveSlotOffset();

	    schedule_getNeighbor(&neighbor);

	    if (neighbor.type > 0) {
    	    //verifico se ele tem mensagem pendende
    	    ieee154e_vars.dataToSend  = openqueue_macGetDataPacket(&neighbor);
        }
	}
    else {
    	ieee154e_vars.dataToSend  = &sPendingDataPacket;
    }

#else
	 txpending = FALSE;
#endif

   if ((txpending == TRUE) || (ieee154e_vars.dataToSend != NULL)) {   // I have a packet to send
		macRITstate=S_RIT_TX_state;

		ieee154e_dbg.num_txslot++;
		ritstat.txola.countdatatx++;

		StartTxRITProcedure(txpending,newtxframe);

		radio_setTimerPeriod(TX_RIT_PERIOD_MS);
  }
  else if (isrmultichannelhello == TRUE)  {

		isrmultichannelhello = FALSE;
		macRITstate = S_RIT_multichnhello_state;

		ieee154e_dbg.num_txslot++;

		StartTxMultiChannelHello();

		radio_setTimerPeriod(TX_RIT_PERIOD_MS);
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
			  StartRxRITProcedure();
		  }
		  else {
			  openserial_startInput();
			  StartRxRITProcedure();
		  }
		#else
	      openserial_stop();
		  openserial_startOutput();
		  StartRxRITProcedure();
		#endif


		#if (ENABLE_CSMA_CA == 1)
			  random = 1; //macRadioRandomByte() & ((1 << 4) - 1);
			  rxritperiod = (RX_RIT_PERIOD_MS + random);
			  radio_setTimerPeriod(rxritperiod);
		#else
			  radio_setTimerPeriod(RX_RIT_PERIOD_TICKS);
		#endif
  }

  ieee154e_vars.slotOffset = ieee154e_vars.nextActiveSlotOffset;

}

/* O RIT Procedure consiste de
 * escolher o canal ; ligar o radio ;
 * enviar o frame de rit
 * programa o radio para recepcao e
 * programa programar o timer por um tempo
 */
port_INLINE void getRitRequest(void) {
	   uint8_t len;
	   OpenQueueEntry_t* adv = &advRIT;
	   sync_IE_ht  sync_IE;

	   //Clear the Area of the getRitRequest
	   openqueue_freePacketRITBuffer(adv);

	   len = 0;

	   // declare ownership over that packet
	   adv->creator = COMPONENT_SIXTOP;     //?????????????????
	   adv->owner   = COMPONENT_IEEE802154E;
	   ieee154e_vars.dataToSend = adv;

	   // reserve space for ADV-specific header
	   // reserving for IEs.
	   len += processIE_prependSlotframeLinkIE(adv);
	   len += processIE_prependSyncIE(adv);

	   //add IE header
	   processIE_prependMLMEIE(adv,len);

	   // some l2 information about this packet

	   adv->l2_frameType                     = IEEE154_TYPE_OLA;
	   //adv->l2_frameType                     = IEEE154_TYPE_BEACON;
	   adv->l2_nextORpreviousHop.type        = ADDR_16B;
	   adv->l2_nextORpreviousHop.addr_16b[0] = 0xff;
	   adv->l2_nextORpreviousHop.addr_16b[1] = 0xff;

	   //I has an IE in my payload
	   adv->l2_IEListPresent = IEEE154_IELIST_YES;


		ieee154e_vars.dataToSend->owner = COMPONENT_IEEE802154E;
		sync_IE.join_priority = neighbors_getMyDAGrank()/(2*MINHOPRANKINCREASE); //poipoi -- use dagrank(rank)
		// fill in the ASN field of the ADV
		ieee154e_getAsn(sync_IE.asn);
		// record that I attempt to transmit this packet
		ieee154e_vars.dataToSend->l2_numTxAttempts++;

		memcpy(ieee154e_vars.dataToSend->l2_ASNpayload,&sync_IE,sizeof(sync_IE_ht));

	   // put in queue for MAC to handle
	   sixtop_send_internal(adv,IEEE154_IELIST_YES,IEEE154_FRAMEVERSION);

	   // I'm now busy sending an ADV
	   //sixtop_vars.busySendingEB = TRUE;
}

//hellospec 0=Dont need ACK ; 1=Need ACK
port_INLINE sRITelement activityrx_preparemultichannelhello(uint8_t hellospec, uint8_t *frame) {

	uint8_t pos=0;
	open_addr_t myaddr;
	open_addr_t *pmyaddr=(open_addr_t *)&myaddr;
	sRITelement psEle;

	psEle.timestamp = 0;
	psEle.msg = frame;

	psEle.isBroadcastMulticast = TRUE;
	psEle.destaddr.type = 1;
	psEle.destaddr.addr_16b[0] = 0xFF;
	psEle.destaddr.addr_16b[1] = 0xFF;

	psEle.frameType = IANA_ICMPv6_RA_PREFIX_INFORMATION;  // 3 = MAC COMMAND

	// MONTA O FRAME DE RIT Data Request
	// Se addr.source 16bits e addr.dest 64bits = 0xE840

	// bits 0-2   - FrameType      - 011 - MAC Command
	// bit  3     - SecurityEnable - 0 = false
	// bit  4     - Frame Pending  - 0 = false
	// bit  5     - Ack Request    - 0 = false
	// bit  6     - Intra Pan      - 1 = true
	// bit  7-9   - Reserved       - Reserved
	// bit  10-11 - Dest Addr Mode - 2 = short 16 bits
	// bit  12-13 - Frame Ver      - 2 = Version 2
	// bit  14-15 - Src Addr Mode  - 2 = short 16 bits

	frame[pos++] = 0x43;  //FCS[0] LSB
	frame[pos++] = 0xA8;  //FCS[1] MSB
	frame[pos++] = 0xb1;  //Seq Number
	frame[pos++] = 0xfe;
	frame[pos++] = 0xca;
	//address dest
	frame[pos++] = psEle.destaddr.addr_16b[0];
	frame[pos++] = psEle.destaddr.addr_16b[1];
	//address source
#if 0
	pmyaddr = idmanager_getMyID(ADDR_64B);
	frame[pos++] = pmyaddr->addr_16b[7];
	frame[pos++] = pmyaddr->addr_16b[6];
	frame[pos++] = pmyaddr->addr_16b[5];
	frame[pos++] = pmyaddr->addr_16b[4];
	frame[pos++] = pmyaddr->addr_16b[3];
	frame[pos++] = pmyaddr->addr_16b[2];
	frame[pos++] = pmyaddr->addr_16b[1];
	frame[pos++] = pmyaddr->addr_16b[0];
#else
	pmyaddr = idmanager_getMyID(ADDR_16B);
	frame[pos++] = pmyaddr->addr_16b[1];
	frame[pos++] = pmyaddr->addr_16b[0];
#endif
	//Command Frame Identifier
	frame[pos++] = CMDFRMID;
	frame[pos++] = hellospec;   //hellospec 0=Dont need ACK ; 1=Need ACK
	//crc
	frame[pos++] = 0x00;
	frame[pos++] = 0x00;

	// space for 2-byte CRC
	//packetfunctions_reserveFooterSize(&adv,2);

    psEle.msglength = pos;

	return psEle;
}
port_INLINE uint8_t activityrx_prepareritdatareq(uint8_t hellospec) {

	header_IE_ht header_desc;
	OpenQueueEntry_t adv;
	uint8_t freq;
	uint8_t pos=0;
	uint8_t frame[30];
	uint32_t duration;
	open_addr_t myaddr;
	open_addr_t *pmyaddr=(open_addr_t *)&myaddr;

	// MONTA O FRAME DE RIT Data Request
	// Se addr.source 16bits e addr.dest 64bits = 0xE840

	// bits 0-2   - FrameType      - 011 - MAC Command
	// bit  3     - SecurityEnable - 0 = false
	// bit  4     - Frame Pending  - 0 = false
	// bit  5     - Ack Request    - 0 = false
	// bit  6     - Intra Pan      - 1 = true
	// bit  7-9   - Reserved       - Reserved
	// bit  10-11 - Dest Addr Mode - 2 = short 16 bits
	// bit  12-13 - Frame Ver      - 2 = Version 2
	// bit  14-15 - Src Addr Mode  - 2 = short 16 bits


	frame[pos++] = 0x43;  //FCS[0] LSB
	frame[pos++] = 0xA8;  //FCS[1] MSB
	frame[pos++] = 0xb1;  //Seq Number
	frame[pos++] = 0xfe;
	frame[pos++] = 0xca;
	//address dest
	frame[pos++] = 0xff;
	frame[pos++] = 0xff;
	//address source
#if 0
	pmyaddr = idmanager_getMyID(ADDR_64B);
	frame[pos++] = pmyaddr->addr_16b[7];
	frame[pos++] = pmyaddr->addr_16b[6];
	frame[pos++] = pmyaddr->addr_16b[5];
	frame[pos++] = pmyaddr->addr_16b[4];
	frame[pos++] = pmyaddr->addr_16b[3];
	frame[pos++] = pmyaddr->addr_16b[2];
	frame[pos++] = pmyaddr->addr_16b[1];
	frame[pos++] = pmyaddr->addr_16b[0];
#else
	pmyaddr = idmanager_getMyID(ADDR_16B);
	frame[pos++] = pmyaddr->addr_16b[1];
	frame[pos++] = pmyaddr->addr_16b[0];
#endif
	//Command Frame Identifier
	frame[pos++] = CMDFRMID;
	frame[pos++] = hellospec;   //hellospec 0=Dont need ACK ; 1=Need ACK
	//crc
	frame[pos++] = 0x00;
	frame[pos++] = 0x00;

	// space for 2-byte CRC
	//packetfunctions_reserveFooterSize(&adv,2);

	radio_loadPacket((uint8_t *)&frame[0],pos);

	return pos;
}



/*
// Aqui eh o caso onde ja enviei o rit e ja programei o timer
// Este start eh do proprio RIT entao devo somente aguardar o estouro do timer...
void activity_RITDoNothing(void){
	changeState(S_RIT_RXOLAREADY);
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
 
port_INLINE void StartRxRITProcedure(void) {

	uint32_t random,rxritperiod;

	changeState(S_RIT_TXOLAPREPARE);

	incroute(idmanager_getIsDAGroot());

	// o ola eh sempre enviado para o meu melhor canal
	ieee154e_vars.freq = macneighbors_getMyBestChan();

	// configure the radio for that frequency
	radio_setFrequency(ieee154e_vars.freq);

#if (USE_RITREQ_SHORT == 1)
	//frame reduzido...12 bytes
	activityrx_prepareritdatareq(0);
#else
	//frame do Beacon do OpenWSN...64bytes
	//################ load the packet in the radio's Tx buffer
	//pega o frame do rit da camada sixtop
	getRitRequest();

	radio_loadPacket(ieee154e_vars.dataToSend->payload,
					ieee154e_vars.dataToSend->length);

#endif
	//################ enable the radio in Tx mode. Transmit the packet
	radio_txEnable();

	//ieee154e_vars.radioOnInit=radio_getTimerValue();
	ieee154e_vars.radioOnInit=ieee154e_vars.lastCapturedTime;
	ieee154e_vars.radioOnThisSlot=TRUE;

	ritstat.txola.countdata++;

    random = macRadioRandomByte() & ((1 << 3) - 1);
	rxritperiod = (RX_RIT_TXOLAPREPARE_MS);  // + random);
	radiotimer_schedule(rxritperiod);

	#if 0// ((ENABLE_DEBUG_RFF == 1)  && (DBG_IEEE802_TIMER == 1))
		{
			uint8_t pos=0;

			rffbuf[pos++]= RFF_IEEE802_RX;
			rffbuf[pos++]= 0x01;
			rffbuf[pos++]= ieee154e_vars.slotOffset;
			rffbuf[pos++]= ieee154e_vars.freq;
			pos = printvar((uint8_t *)&rxritperiod,sizeof(uint32_t),rffbuf,pos);

			openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);

			openserial_startOutput();
		}
	#endif
}

port_INLINE void activityrx_sendola(void) {
  uint32_t dur_rt2;

  changeState(S_RIT_TXOLA);
  incroute(0x02);

  radiotimer_cancel();

#if (ENABLE_CSMA_CA == 1)

  ieee154e_vars.lastCapturedTime = getdeltaslotperiod_ms();   // getdeltaTimerValue();

  radio_txNow(TX_USE_CSMA_CA);

  radiotimer_schedule(RX_RIT_TXOLAECHO_MS);

#else
  //ieee154e_vars.radioOnInit=radio_getTimerValue();
  //ieee154e_vars.radioOnThisSlot=TRUE;

  radio_txNow();

  //dur_rt2 = ieee154e_vars.lastCapturedTime+200;
  radiotimer_schedule(RX_RIT_TXOLAECHO_TICKS);
#endif

  ritstat.txola.countdatatx++;

#if 0 //((ENABLE_DEBUG_RFF == 1)  && (DBG_IEEE802_TIMER == 1))
	{
		uint8_t *pucAux = (uint8_t *) &ieee154e_vars.lastCapturedTime;
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_RX;
		rffbuf[pos++]= 0x02;
		rffbuf[pos++]= ieee154e_vars.slotOffset;
/*
		rffbuf[pos++]= radio_csma_vars.countok;
		rffbuf[pos++]= radio_csma_vars.rfftxbusy;
		rffbuf[pos++]= radio_csma_vars.rfftxstop2;
		rffbuf[pos++]= radio_csma_vars.rfftxwait;
		rffbuf[pos++]= radio_csma_vars.nb;
		rffbuf[pos++]= radio_csma_vars.countBusy;
		rffbuf[pos++]= radio_csma_vars.counterr;
*/
		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
#endif

}


/*
 *  aqui esta no inicio do slot RX e nao recebeu OLA ECHO...
 *  Reprograma o timer para a janela do RIT e volta a enviar o pacote.
 *  Problema aqui eh o radio estar travado e ele vai terminar o slot quando o timer estourar a janela.
 */
port_INLINE void activityrx_rxolanoecho(void) {

	uint32_t deltams;
	uint32_t timeelapsedms;

	radiotimer_cancel();

	radio_rfOff();

	changeState(S_RIT_RXOLANOECHO);

	incroute(0x83);

	ieee154e_vars.lastCapturedTime = getdeltaslotperiod_ms();  //getdeltaTimerValue();

	//ieee154e_vars.radioOnTics+=(radio_getTimerValue()-ieee154e_vars.radioOnInit);
	//ieee154e_vars.radioOnTics+=(ieee154e_vars.lastCapturedTime - ieee154e_vars.radioOnInit);

	//REPROGRAMAR A JANELA COM UM TEMPO MENOR...
	timeelapsedms = ieee154e_vars.lastCapturedTime;
	if (RX_RIT_TIMEOUT_MS > timeelapsedms){

		deltams = RX_RIT_TIMEOUT_MS - timeelapsedms;

		if (deltams < 5)
			deltams = 5;

		#if 1 // ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
		{
			uint8_t   pos=0;

			rffbuf[pos++]= RFF_IEEE802_RX;
			rffbuf[pos++]= 0x83;
			pos = printvar((uint8_t *)&timeelapsedms,sizeof(uint32_t),rffbuf,pos);
			pos = printvar((uint8_t *)&deltams,sizeof(uint32_t),rffbuf,pos);

			openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
		}
		#endif


	    //Volto ao estado anterior para indicar que estou novamente esperando um frame TX
		changeState(S_RIT_TXOLA);

		//como desliguei o radio anteriormente devo reprograma-lo para TX
		radio_setFrequency(ieee154e_vars.freq);

		radio_txEnable();

	    radio_txNow(TX_USE_CSMA_CA);

		radiotimer_schedule(deltams);
	}
	else{
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
uint8_t checkmsgtype(sRITelement *pmsgout,uint8_t txpending,uint8_t newtxframe) {

	uint8_t *pu8Address;
	uint8_t ret=0,i;
	uint8_t testerff=0;
	uint8_t macRIT_Pending_TX_frameType=0;
	uint8_t flagpending=false;
	uint8_t icmpv6_type;
	uint8_t icmpv6_code;
	uint8_t iphc_header1;
	uint8_t iphc_nextheader;
	uint8_t numTargetParents;
	uint8_t nbrIdx;
	open_addr_t address;


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
				ritstat.txdio.countdata++;
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
                    /*
					 pu8Address = (uint8_t *) &(ieee154e_vars.dataToSend->l3_destinationAdd.addr_128b[15]);
					 address_1.addr_64b[7] = *pu8Address--;
					 address_1.addr_64b[6] = *pu8Address--;
					 address_1.addr_64b[5] = *pu8Address--;
					 address_1.addr_64b[4] = *pu8Address--;
					 address_1.addr_64b[3] = *pu8Address--;
					 address_1.addr_64b[2] = *pu8Address--;
					 address_1.addr_64b[1] = *pu8Address--;
					 address_1.addr_64b[0] = *pu8Address;
					 //senddirectly = TRUE;
                    */

				 }
			}
			else
			{
				testerff = 0x17;


				address_1 = ieee154e_vars.dataToSend->l2_nextORpreviousHop;
				 //salvo o endereco do destino que pode estar na posicao final do frame (RPL Transition)
				 /*
				 address_1.type = 2;
				 pu8Address = (uint8_t *) (ieee154e_vars.dataToSend->payload + ieee154e_vars.dataToSend->length-3);
				 address_1.addr_64b[7] = *pu8Address--;
				 address_1.addr_64b[6] = *pu8Address--;
				 address_1.addr_64b[5] = *pu8Address--;
				 address_1.addr_64b[4] = *pu8Address--;
				 address_1.addr_64b[3] = *pu8Address--;
				 address_1.addr_64b[2] = *pu8Address--;
				 address_1.addr_64b[1] = *pu8Address--;
				 address_1.addr_64b[0] = *pu8Address;
				 */
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

			ritstat.txdio.countdata++;

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
		uint8_t *pucAux;

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

port_INLINE void StartTxMultiChannelHello (void) {

	uint8_t frame[30];
	sRITelement psEle;
	uint8_t numTargetParents;

	changeState(S_RIT_TXDATAOFFSET);

	//montar frame de Multi-Channel Hello
	psEle = activityrx_preparemultichannelhello(0,frame);

	//descubro quantos vizinhos tenho na minha vizinhanca
	numTargetParents = macneighbors_setBroadCastPending(0);

	//coloco elemento na fila do RIT_Tx
    ieee154e_vars.RITQueue_ElementPending = RITQueue_Put(&psEle,0,numTargetParents);

    //preencho a variavel global ritvars
    sRIT_vars = RITQueue_Get_Element(ieee154e_vars.RITQueue_ElementPending);

#if 1 //((ENABLE_DEBUG_RFF) && (DBG_IEEE802_TX == 1))
{
	uint8_t pos=0;

	rffbuf[pos++]= RFF_IEEE802_TX;
	rffbuf[pos++]= 0xAA;
	rffbuf[pos++]= numTargetParents;

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);

	openserial_startOutput();
}
#endif


    activitytx_rxolaprepare();
}

#if (TEST_MEASURE_NEIGHBOR_RATE == 1)
port_INLINE void testeTxRITProcedure(void) {

	uint8_t macRIT_Pending_TX_frameType=0;
	//uint32_t dur_rt1;
	//uint32_t macRITTxPeriod;

	changeState(S_RIT_TXDATAOFFSET);
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

	activitytx_rxolaprepare();

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


port_INLINE void StartTxRITProcedure(uint8_t txpending,uint8_t newtxframe) {

	uint8_t macRIT_Pending_TX_frameType=0;
    uint8_t notifyerror=0;

	changeState(S_RIT_TXDATAOFFSET);

    if (ieee154e_vars.dataToSend->l2_frameType == IEEE154_TYPE_DATA) {
		macRIT_Pending_TX_frameType = checkmsgtype(&element,txpending,newtxframe);
	}

    incroute(macRIT_Pending_TX_frameType);

    //TODO!!!!aqui tive problemas de frame invalido...nao sei por que...
    if (ieee154e_vars.dataToSend->l2_retriesLeft > 3)
    {
    	notifyerror = TRUE;
    }
    else if ((macRIT_Pending_TX_frameType > 0) || (txpending))
	{
		activitytx_rxolaprepare();
	}
	else
	{
		notifyerror = TRUE;
	}

    if (notifyerror == TRUE){
    	incroute(0x81);

		#if 1// ((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_TX == 1))
		  {
			uint8_t   pos=0;

			rffbuf[pos++]= RFF_IEEE802_TX;
			rffbuf[pos++]= 0x81;
			rffbuf[pos++]= macRIT_Pending_TX_frameType;
			rffbuf[pos++]= ieee154e_vars.dataToSend->l2_retriesLeft;

			openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
		  }
		#endif

		if (ieee154e_vars.dataToSend != NULL) {
		  notif_sendDone(ieee154e_vars.dataToSend,E_FAIL);
		  ieee154e_vars.dataToSend = NULL;
     	}
     	endSlot();
    }
}

port_INLINE void activitytx_senddata(void) {

	radiotimer_cancel();

    changeState(S_RIT_TXDATA);

    incroute(0x06);

#if (ENABLE_CSMA_CA == 1)
    radio_txNow(TX_USE_CSMA_CA);
#else
    radio_txNow();
#endif

    radiotimer_schedule(TX_RIT_TXDATAECHO_MS);

}

/*
 * Aqui quando Tx significa que o frame foi enviado com sucesso (acontece apos ter recebido o echo do final do frame)
 */
port_INLINE void activitytx_senddone(PORT_RADIOTIMER_WIDTH capturedTime) {
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

	if (sRIT_vars.frameType == IANA_ICMPv6_RA_PREFIX_INFORMATION)  {  //MULTI CHANNEL HELLO CMD
		// notifica o recebimento na tabela de vizinhos
		macneighbors_updtlivelist(ieee154e_vars.targetaddr);

		//update statistics
		macneighbors_updtstatistics(ieee154e_vars.targetaddr,sRIT_vars.frameType, E_SUCCESS);

		RITQueue_Clear_Pending(ieee154e_vars.RITQueue_ElementPending,actualsrcaddr);

		nrPending = RITQueue_getNrPendingParents(ieee154e_vars.RITQueue_ElementPending);

		if (nrPending > 0) {

			incroute(0x72);

			changeState(S_RIT_TXDATAOFFSET);
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
	}
	else if (sRIT_vars.frameType == IANA_ICMPv6_RPL_DIO) {

	    //QDO O RPL DIO -> CHECAR SE JA ENVIOU DIO PARA TODOS OS VIZINHOS
		//SENAO...REPROGRAMAR E TORNAR A ESPERAR UM NOVO OLA...ATE TERMINAR OS VIZINHOS OU A JANELA DE RIT_TX.
		RITQueue_Clear_Pending(ieee154e_vars.RITQueue_ElementPending,actualsrcaddr);

		//update statistics
		macneighbors_updtstatistics(ieee154e_vars.targetaddr,sRIT_vars.frameType, E_SUCCESS);

		nrPending = RITQueue_getNrPendingParents(ieee154e_vars.RITQueue_ElementPending);

		if (nrPending > 0) {

			incroute(0x72);

			changeState(S_RIT_TXDATAOFFSET);
			radiotimer_schedule(TX_RIT_DELAYCONTWAIT_MS);

			#if 1 // ((ENABLE_DEBUG_RFF == 1)  && (DBG_RADIO_POWER_CONS == 1))
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

			   activitytx_rxwaitforack(capturedTime);
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

port_INLINE void activitytx_rxwaitforack(PORT_RADIOTIMER_WIDTH capturedTime) {

	uint32_t duration;
   // change state
   changeState(S_RIT_RXACK);

   incroute(0x08);

   ieee154e_vars.lastCapturedTime = capturedTime;

	// o Ack eh enviado no meu melhor canal
	ieee154e_vars.freq = macneighbors_getMyBestChan();

   // configure the radio for that frequency
   radio_setFrequency(ieee154e_vars.freq);

   // enable the radio in Rx mode. The radio is not actively listening yet.
   radio_rxEnable();

   //caputre init of radio for duty cycle calculation
   //ieee154e_vars.radioOnInit=radio_getTimerValue();
   //ieee154e_vars.radioOnThisSlot=TRUE;
   //ieee154e_vars.lastCapturedTime=ieee154e_vars.radioOnInit;

   radio_rxNow();

   radiotimer_schedule(TX_RIT_ACK_TIMEOUT_MS);

	#if 1 // ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TIMER == 1))
		{
			uint8_t pos=0;

			rffbuf[pos++]= RFF_IEEE802_TX;
			rffbuf[pos++]= 0x08;

			openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
		}
	#endif

}


port_INLINE void activityrx_noacktx() {

	uint32_t deltams;
	uint32_t timeelapsedms;

   incroute(0x88);

   radiotimer_cancel();

   radio_rfOff();

   changeState(S_RIT_NOECHOTXACK);

   ieee154e_vars.lastCapturedTime = getdeltaslotperiod_ms();  //getdeltaTimerValue();

   // compute the duty cycle if radio has been turned on
   // if (ieee154e_vars.radioOnThisSlot==TRUE){
   //	  ieee154e_vars.radioOnTics+=(radio_getTimerValue()-ieee154e_vars.radioOnInit);
   // }

	//REPROGRAMAR A JANELA COM UM TEMPO MENOR...
	timeelapsedms = ieee154e_vars.lastCapturedTime + RX_RIT_ACKECHO_TIMEOUT_MS + 10;
	if (RX_RIT_TIMEOUT_MS > timeelapsedms){

		deltams = RX_RIT_ACKECHO_TIMEOUT_MS + 10;

		#if 1 // ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
		{
			uint8_t   pos=0;

			rffbuf[pos++]= RFF_IEEE802_RX;
			rffbuf[pos++]= 0x88;
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


	// SINALIZO ERRO E GUARDO A MENSAGEM PARA TENTAR NO PROXIMO CICLO...activity_tie5(); (copie aqui embaixo)
	#if 0
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
	#endif
}


port_INLINE void activitytx_rxnoack() {

	incroute(0x89);

   radiotimer_cancel();

   // turn off the radio
   radio_rfOff();

   // change state
   changeState(S_RIT_RXNOACK);

   // compute the duty cycle if radio has been turned on
   //if (ieee154e_vars.radioOnThisSlot==TRUE){
   //	  ieee154e_vars.radioOnTics+=(radio_getTimerValue()-ieee154e_vars.radioOnInit);
   //}

   //update statistics
   if (ieee154e_vars.RITQueue_ElementPending < maxElements) {
		//decrementa elemento pendente...para broadcast
		RITQueue_update_element(ieee154e_vars.RITQueue_ElementPending);

		//sRIT_vars = RITQueue_Get_Element(ieee154e_vars.RITQueue_ElementPending);

	    macneighbors_updtstatistics(ieee154e_vars.targetaddr,sRIT_vars.frameType, E_FAIL);
   }


#if 1 //((ENABLE_DEBUG_RFF ==1)  && (DBG_RADIO_POWER_CONS == 1))
	{
		uint8_t *pucAux = (uint8_t *) &ieee154e_vars.radioOnTics;
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_TX;
		rffbuf[pos++]= 0x89;

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
port_INLINE void activitytx_rxackok(PORT_RADIOTIMER_WIDTH capturedTime) {
   ieee802154_header_iht     ieee802514_header;
//  uint16_t                  lenIE;
   uint8_t ret;
   uint8_t flagerr=0;
   uint8_t frameType;

   incroute(0x09);

   // change state
   changeState(S_RIT_TXPROC);

   // cancel tt8
   radiotimer_cancel();

   // turn off the radio
   radio_rfOff();

   // record the captured time
   ieee154e_vars.lastCapturedTime = getdeltaslotperiod_ms();   //getdeltaTimerValue();


   //ieee154e_vars.radioOnTics+=(radio_getTimerValue()-ieee154e_vars.radioOnInit);


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

	#if 0
		  // toss the IEEE802.15.4 header
		  packetfunctions_tossHeader(ieee154e_vars.ackReceived,ieee802514_header.headerLength);

		  // break if invalid ACK
		  if (isValidAck(&ieee802514_header,ieee154e_vars.dataToSend)==FALSE) {
			 // break from the do-while loop and execute the clean-up code below
			 flagerr = TRUE;
			 break;
		  }
		  //hanlde IEs --xv poipoi
		  if (ieee802514_header.ieListPresent==FALSE){
			 flagerr = TRUE;
			 break; //ack should contain IEs.
		  }

		  if (ieee154e_processIEs(ieee154e_vars.ackReceived,&lenIE)==FALSE){
			// invalid IEs in ACK
			flagerr = TRUE;
			break;
		  }

		  // toss the IEs
		  packetfunctions_tossHeader(ieee154e_vars.ackReceived,lenIE);
	#endif

		#if 1 //((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_TX == 1))
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

	   macneighbors_updtstatistics(ieee154e_vars.targetaddr,sRIT_vars.frameType, E_SUCCESS);

	   //libera areas alocadas
       RITQueue_Free(ieee154e_vars.RITQueue_ElementPending);

       if (ieee154e_vars.dataToSend != NULL) {
		  // inform schedule of successful transmission
		  schedule_indicateTx(&ieee154e_vars.asn,TRUE);

		  // inform upper layer
		  notif_sendDone(ieee154e_vars.dataToSend,E_SUCCESS);
		  ieee154e_vars.dataToSend = NULL;
       }

      // in any case, execute the clean-up code below (processing of ACK done)
   } while (0);

   if (flagerr) {
	   incroute(0x89);
	   //update statistics
	   macneighbors_updtstatistics(ieee154e_vars.targetaddr,sRIT_vars.frameType, E_FAIL);

       ret = RITQueue_Free(ieee154e_vars.RITQueue_ElementPending);
   }

   // free the received ack so corresponding RAM memory can be recycled
   openqueue_freePacketBuffer(ieee154e_vars.ackReceived);

   // clear local variable
   ieee154e_vars.ackReceived = NULL;

   // official end of Tx slot
   endSlot();
}


/*======= TX
* esta rotina eh chamada logo apos o inicio do TxRITProcedure
* ela prepara o radio para recepcao de um ola
*/
port_INLINE void activitytx_rxolaprepare(void) {

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

#if 1 //(DEBUG_LOG_RIT  == 1) && (DBG_IEEE802_TX == 1)
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

port_INLINE void activityrx_dataprepare(PORT_RADIOTIMER_WIDTH capturedTime) {

	radiotimer_cancel();

	radio_rfOff();

	changeState(S_RIT_RXDATAPREPARE);

	incroute(0x03);
	ritstat.txola.countdatatxok++;

    ieee154e_vars.lastCapturedTime = getdeltaslotperiod_ms();   //getdeltaTimerValue();

	#if 0//((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_RX == 1))
	  {
		uint8_t pos=0;
		uint32_t freerunning = radiotimer_getfreerunning();

		rffbuf[pos++]= RFF_IEEE802_RX;
		rffbuf[pos++]= 0x03;
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

	changeState(S_RIT_RXDATA);
	radiotimer_cancel();
	radio_rxNow();
	radiotimer_schedule(RX_RIT_TIMEOUT_MS);
}


port_INLINE void activitytx_rxwindowopen() {

	uint32_t period;

	radiotimer_cancel();

    changeState(S_RIT_RXOLA);

    ieee154e_vars.lastCapturedTime = getdeltaslotperiod_ms();

	incroute(0x03);

    radio_rxNow();

	radiotimer_schedule(TX_RIT_TIMEOUT_MS);

#if 1//((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_RX == 1))
  {
	uint8_t pos=0;

	rffbuf[pos++]= RFF_IEEE802_TX;
	rffbuf[pos++]= 0x03;
	pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
  }
#endif

}

//Abre a janela para recepcao...
port_INLINE void activity_rxwindowopen() {

	uint32_t duration=0;

	radiotimer_cancel();

	ieee154e_vars.lastCapturedTime += getdeltaTimerValue();

    changeState(S_RIT_RXDATA);

	incroute(0x33);

    radio_rxNow();

    //radiotimer_schedule(macRITDataWaitDuration);
	//duration = ieee154e_vars.lastCapturedTime + TICK_MAC_RIT_RX_WIND_PERIOD;
	//radiotimer_schedule(duration);
    radiotimer_schedule(RX_RIT_TIMEOUT_MS);

#if 0 //((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TIMER == 1))
	{
		uint8_t *pucAux = (uint8_t *)  &ieee154e_vars.lastCapturedTime;
		uint8_t *pucAux1 = (uint8_t *)  &ieee154e_dbg.num_newSlot;
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_RX;
		rffbuf[pos++]= 0x33;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;
		rffbuf[pos++]= *pucAux1++;
		rffbuf[pos++]= *pucAux1++;
		rffbuf[pos++]= *pucAux1++;
		rffbuf[pos++]= *pucAux1;
		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);

	}
#endif
}

/* ACABOU O RIT AQUI E NAO RECEBI NENHUMA MENSAGEM (macRITstate == S_RIT_RX_state)
 * Quando RX significa que abri a janela do RIT e nao teve nenhuma mensagem - caso normal.
 */

port_INLINE void activity_rxritwindowend() {

	uint8_t nrPending=0;

    radiotimer_cancel();

    ieee154e_vars.lastCapturedTime = getdeltaslotperiod_ms();   //getdeltaTimerValue();

    // turn off the radio
    radio_rfOff();

    // change state
    changeState(S_RIT_RXSLEEP);

    //RFF DEBUG
	incroute(0x85);

   // compute the duty cycle if radio has been turned on
/*
   if (ieee154e_vars.radioOnThisSlot==TRUE){
	  //ieee154e_vars.radioOnTics+=(radio_getTimerValue()-ieee154e_vars.radioOnInit);
	   ieee154e_vars.radioOnTics+=(ieee154e_vars.lastCapturedTime-ieee154e_vars.radioOnInit);
   }
*/
#if 0// ((ENABLE_DEBUG_RFF ==1)  && (DBG_RADIO_POWER_CONS == 1))
{
	uint8_t pos=0;
	uint32_t freerunning = radiotimer_getfreerunning();

	rffbuf[pos++]= RFF_IEEE802_RX;
	rffbuf[pos++]= 0x85;
	pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);
	pos = printvar((uint8_t *)&freerunning,sizeof(uint32_t),rffbuf,pos);

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);

	openserial_startOutput();
}
#endif

#if 1 //teste rff 050716
 	//aqui eu nao recebi nenhuma msg na janela...
	if (macRITstate == S_RIT_TX_state) {
		if (ieee154e_vars.dataToSend != NULL) {
			incroute(0x40);
			notif_sendDone(ieee154e_vars.dataToSend,E_SUCCESS);
			ieee154e_vars.dataToSend = NULL;
		}
	}
#endif
	// abort
	endSlot();
}


/* ACABOU O RIT AQUI E NAO RECEBI NENHUMA MENSAGEM (macRITstate == S_RIT_TX_state)
 * Como a janela de Tx é bem grande, é pressuposto que o nó esta ruim
 */

port_INLINE void activity_txritwindowend() {

	uint8_t nrPending=0;
	uint8_t nrDIOsent=0;

    radiotimer_cancel();

    ieee154e_vars.lastCapturedTime = getdeltaTimerValue();

	// change state
    changeState(S_RIT_TXSLEEP);

    //RFF DEBUG
	incroute(0x84);

    // turn off the radio
    radio_rfOff();

	//verifico se o frame atual é um RPL.DAO ou COAP
	//sRIT_vars = RITQueue_Get_Element(ieee154e_vars.RITQueue_ElementPending);

    #if 1// ((ENABLE_DEBUG_RFF ==1)  && (DBG_RADIO_POWER_CONS == 1))
	{
		uint32_t period;
		//uint8_t *pucAux = (uint8_t *)  &period;
		uint8_t pos=0;

		period = radiotimer_getPeriod();

		rffbuf[pos++]= RFF_IEEE802_TX;
		rffbuf[pos++]= 0x84;
		rffbuf[pos++]= 0x83;
		rffbuf[pos++]= macRITstate;
		rffbuf[pos++]= sRIT_vars.frameType;

		pos = printaddress(ieee154e_vars.targetaddr,&rffbuf[0],pos);
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


port_INLINE uint8_t activitytx_reopenrxwindow(void) {

    uint8_t ret=FALSE;
    uint8_t nrDIOsent=0;
	uint32_t timeelapsedms;
	uint32_t deltams;

    radiotimer_cancel();

	//pego o elemento atual pendente
	//sRIT_vars = RITQueue_Get_Element(ieee154e_vars.RITQueue_ElementPending);

	pvObjList[ieee154e_vars.RITQueue_ElementPending].countretry++;

	// record the captured time
	ieee154e_vars.lastCapturedTime = getdeltaslotperiod_ms();

	//REPROGRAMAR A JANELA COM UM TEMPO MENOR...
	timeelapsedms = ieee154e_vars.lastCapturedTime;
	if (TX_RIT_TIMEOUT_MS > timeelapsedms){
		deltams = TX_RIT_TIMEOUT_MS - timeelapsedms;

		incroute(0x45);

		if (deltams < 5)
			deltams = 5;

		flagreschedule = TRUE;

		//radio_rfOff();

		radio_setFrequency(ieee154e_vars.freq);

		radio_rxEnable();

		radio_rxNow();

		//Volto ao estado anterior para indicar que estou novamente esperando um frame TX
		changeState(S_RIT_RXOLA);

		radiotimer_schedule(deltams);

		#if 1 //((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TIMER == 1))
		{
			uint8_t pos=0;

			rffbuf[pos++]= RFF_IEEE802_TX;
			rffbuf[pos++]= 0x45;
			rffbuf[pos++]= pvObjList[ieee154e_vars.RITQueue_ElementPending].countretry;
			rffbuf[pos++]= ieee154e_vars.freq;
			//pos = printaddress(sRIT_vars.destaddr,rffbuf,pos);

			pos = printvar((uint8_t *)&timeelapsedms,sizeof(uint32_t),rffbuf,pos);
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
			  //schedule_indicateTx(&ieee154e_vars.asn,TRUE);

			  if (ieee154e_vars.dataToSend != NULL) {
					notif_sendDone(ieee154e_vars.dataToSend,E_FAIL);

				ieee154e_vars.dataToSend = NULL;
			  }
          }

   		  endSlot();
     }

	return ret;
}

/*
 * Aqui eu estou com msg TX pendente...e acabei de receber um ola
 * Entao:
 *   - Verifico se a msg pendente eh um DIO (Broadcast) e neste caso devo ver se a msg pendente precisa ainda ser enviada
 *   - Verifico se a msg pendente eh um DAO entao devo enviar a msg para o endereco especifico
 *   - Verifico se a msg pendente eh um COAP entao devo enviar a msg para o endereco especifico
 */
void tratatxstate(open_addr_t* address){
    uint8_t elementpos;

	if (sRIT_vars.isBroadcastMulticast){  //FRAME RPL.DIO ou MULTICHANNELHELLO

		if (macisThisAddressPendingBroadcast(address) == TRUE){
            ritstat.txdio.countdatatx++;
			incroute(0x40);
			changeState(S_RIT_TXACKOFFSET);
			radiotimer_schedule(TX_RIT_DELAYRXTX_MS);
		}
		else {
			//Chegou msg mas de um endereco que nao vou enviar broadcast...devo aguardar novamente
			incroute(0x41);
        	ritstat.txdio.countack++;

			changeState(S_RIT_TX_CONTINUEWAIT);
			radiotimer_schedule(TX_RIT_DELAYCONTWAIT_MS);
		}

	}
	else { //frame eh COAP OU RPL.DAO
		elementpos = RITQueue_Get_Pos(address);

		if (elementpos < maxElements)
		{ //OLA EH DA MSG PENDENTE...PREPARO PARA ENVIA-LA
			ieee154e_vars.RITQueue_ElementPending = elementpos;
			incroute(0x42);
			changeState(S_RIT_TXACKOFFSET);
			radiotimer_schedule(TX_RIT_DELAYRXTX_MS);
		}
		else { //AQUI O ENDERECO DO OLA NAO EH O MESMO QUE EU ESTAVA ESPERANDO...VOLTO A ESPERAR...
			incroute(0x43);

			changeState(S_RIT_TX_CONTINUEWAIT);
			radiotimer_schedule(TX_RIT_DELAYCONTWAIT_MS);
		}
	}
}

/* tratarxstate - trata msg recebida diferente de ola
 * return - 0 - nao descarta o frame e nao termina o slot
 *          1 - termina o slot e nao descarta o frame (pois ele ja foi descartado)
 *          2 - termina o slot e descarta o frame
 */

uint8_t tratarxstate (uint8_t ackRequested,uint8_t frameType, open_addr_t *rxaddr_dst) {
   uint8_t DiscardAndEndSlot = 0;
   uint8_t isFrameForMe=0;
   uint32_t capturedTime=0;

    incroute(0x05);
	#if 1 // ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
	{
		uint8_t   pos=0;

		rffbuf[pos++]= RFF_IEEE802_RX;
		rffbuf[pos++]= 0x05;
		rffbuf[pos++]= frameType;
		pos = printaddress(actualsrcaddr,&rffbuf[0],pos);
		pos = printaddress(*rxaddr_dst,&rffbuf[0],pos);
		//pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
	#endif

	 //checa se frame é para o mote (mesmo endereco destino ou eh um frame broadcast)
	if (frameType == IEEE154_TYPE_UNDEFINED) {
		incroute(0x53);
		DiscardAndEndSlot = activityrx_reopenrxwindow(actualsrcaddr);
	}
	else if (packetfunctions_isBroadcastMulticast(rxaddr_dst)){

		incroute(0x50);

		//AQUI É UM DIO...ENTAO SOMENTO NOTIFICO AS CAMADAS SUPERIORES
		ritstat.rxdio++;

		//ieee154e_vars.radioOnTics+=radio_getTimerValue()-ieee154e_vars.radioOnInit;

		// indicate reception to upper layer (no ACK asked)
		if (ieee154e_vars.dataReceived!=NULL) {
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

		// check if ack requested
		if (ackRequested==1) {
			incroute(0x51);

			activityrx_preparetxack(capturedTime);
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
    timeelapsedms = ieee154e_vars.lastCapturedTime;
    if (RX_RIT_TIMEOUT_MS > timeelapsedms){

		incroute(0x47);

		deltams = RX_RIT_TIMEOUT_MS - timeelapsedms;

		if (deltams < 5)
			deltams = 5;

		#if 0 // ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
		{
			uint8_t   pos=0;

			rffbuf[pos++]= RFF_IEEE802_RX;
			rffbuf[pos++]= 0x47;
			pos = printvar((uint8_t *)&timeelapsedms,sizeof(uint32_t),rffbuf,pos);
			pos = printvar((uint8_t *)&deltams,sizeof(uint32_t),rffbuf,pos);

			openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
		}
		#endif

		//como desliguei o radio anteriormente devo reprograma-lo para RX
		ieee154e_vars.freq = macneighbors_getMyBestChan();

		radio_setFrequency(ieee154e_vars.freq);

		radio_rxEnable();

		radio_rxNow();

		//descarto o frame recebido
		if (ieee154e_vars.dataReceived!=NULL) {
		  openqueue_freePacketBuffer(ieee154e_vars.dataReceived);
		  ieee154e_vars.dataReceived = NULL;
		}

		//Volto ao estado anterior para indicar que estou novamente esperando um frame TX
		changeState(S_RIT_RXDATA);

		radiotimer_schedule(deltams);

		return DISCARD_NO_ENDSLOT_NO;
    }
    else
    {
		return DISCARD_YES_ENDSLOT_YES;
    }


}
/*
 * Aqui o frame foi recebido com sucesso...
 * se somente processo um frame por vez...fecho o radio e processo a resposta.
 *
 */
port_INLINE void activity_rxnewframe(PORT_RADIOTIMER_WIDTH capturedTime) {
    ieee802154_header_iht ieee802514_header;
    uint16_t lenIE=0;
    uint8_t isFrameForMe=0;
    uint8_t discardframe = FALSE;
	open_addr_t rxaddr_nexthop;
	open_addr_t rxaddr_dst;
	open_addr_t rxaddr_src;
	uint8_t elementpos;
	uint8_t   *pauxframe;
	uint8_t reopenwindow=0;

	radiotimer_cancel();

	radio_rfOff();

	ieee154e_vars.lastCapturedTime = getdeltaslotperiod_ms();    //getdeltaTimerValue();

	changeState(S_RIT_RXNEWFRM);

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
      openqueue_removeAllOwnedBy(COMPONENT_SIXTOP_TO_IEEE802154E);

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

	#if 1 //((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
	{
		uint8_t   pos=0;

		if ((macRITstate == S_RIT_TX_state) || (macRITstate == S_RIT_multichnhello_state)) {
			pauxframe = (uint8_t *) &ieee154e_vars.dataReceived->packet[0];
			rffbuf[pos++]= RFF_IEEE802_RX;
			rffbuf[pos++]= 0x04;
			//rffbuf[pos++]= macRITstate;
			//rffbuf[pos++]= ieee154e_vars.dataReceived->length;
			//rffbuf[pos++]= ieee154e_vars.freq;
			//rffbuf[pos++]= ieee802514_header.valid;
			//rffbuf[pos++]= pauxframe[1];   //802154.FCF [0]
			//rffbuf[pos++]= pauxframe[2];   //802154.FCF [1]
			//rffbuf[pos++]= pauxframe[6];   //dest addr [6]
			//rffbuf[pos++]= pauxframe[7];   //dest addr [7]
			rffbuf[pos++]= 0xaa;  //inverti os bytes abaixo do endereco para ficar mais facil
			rffbuf[pos++]= pauxframe[9];   //src addr [7]
			rffbuf[pos++]= pauxframe[8];   //src addr [6]
			//rffbuf[pos++]= pauxframe[10];  //IPHC header [0]
			rffbuf[pos++]= 0xee;   //src addr [7]
			pos = printvar((uint8_t *)&ieee154e_vars.lastCapturedTime,sizeof(uint32_t),rffbuf,pos);

			openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);

			openserial_startOutput();
		}
	}
	#endif
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

      // record the captured time
#if 0
      if (ieee154e_vars.lastCapturedTime > capturedTime)
      { //ocorreu overflow
    	  uint32_t aux;

    	  aux = 0xFFFFFFFF - ieee154e_vars.lastCapturedTime;
    	  ieee154e_vars.lastCapturedTime = aux + capturedTime;
      }
      else
    	  ieee154e_vars.lastCapturedTime -= capturedTime;
#endif

      // if I just received an invalid frame, stop
      // só eh valido beacon e data packet - ack nao sera valido!!!!!!
      if (isValidRxFrame(&ieee802514_header)==FALSE) {
  	    incroute(0x53);
      	  // jump to the error code below this do-while loop
         break;
      }

       memcpy(&rxaddr_nexthop, &(ieee154e_vars.dataReceived->l2_nextORpreviousHop),sizeof(open_addr_t));
       memcpy(&rxaddr_dst, &(ieee802514_header.dest),sizeof(open_addr_t));
       memcpy(&actualsrcaddr, &(ieee802514_header.src),sizeof(open_addr_t));

		if ((macRITstate == S_RIT_TX_state) || (macRITstate == S_RIT_multichnhello_state))
		{
			reopenwindow=0;

			//descarto o frame recebido pois nao preciso mais dele
			if (ieee154e_vars.dataReceived!=NULL) {
			   openqueue_freePacketBuffer(ieee154e_vars.dataReceived);
			   ieee154e_vars.dataReceived = NULL;
			}

			if (ieee802514_header.frameType == IEEE154_TYPE_OLA) {
				tratatxstate(&actualsrcaddr);
			}
			else {  //aqui estava esperando um ola mas veio outro frame...devo voltar a esperar um novo frame.
				//TODO!!! AQUI NAO VAI DAR UM DEADLOCK ? O CARA TAMBEM ESTA ESPERANDO UM OLA...SE EU FICAR ESPERANDO ELE NAO VAI VIR...
				//MELHOR TALVEZ TERMINAR E ENVIAR UM OLA PARA ELE...E PERDER A MSG...
				//reopenwindow = TRUE;
				incroute(0x44);
				changeState(S_RIT_TX_CONTINUEWAIT);
				radiotimer_schedule(TX_RIT_DELAYCONTWAIT_MS);
			}

			return;
		}
	   else { // (macRITstate == S_RIT_RX_state)

           //incroute(ieee802514_header.frameType);
           if (ieee802514_header.frameType != IEEE154_TYPE_OLA) {
        	   discardframe = tratarxstate(ieee802514_header.ackRequested,ieee802514_header.frameType, &rxaddr_dst);
           }
		   else {
			   discardframe = activityrx_reopenrxwindow(actualsrcaddr);
		   }

    	   if (discardframe > 0) {
    			incroute(0x48);

    		   // clean up dataReceived
    		   if (discardframe == DISCARD_YES_ENDSLOT_YES){
				   if (ieee154e_vars.dataReceived!=NULL) {
					  openqueue_freePacketBuffer(ieee154e_vars.dataReceived);
					  ieee154e_vars.dataReceived = NULL;
					}
    		   }

			   endSlot();
    	  }

    	  // everything went well, return here not to execute the error code below
    	  return;
	   }

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
	   if (macRITstate == S_RIT_RX_state)
		   activityrx_reopenrxwindow(actualsrcaddr);
	   else { // (macRITstate == S_RIT_TX_state ou hello
			changeState(S_RIT_TX_CONTINUEWAIT);
			radiotimer_schedule(TX_RIT_DELAYCONTWAIT_MS);
	   }
   }
   else {
	   // no servico hello foi esperado um ola do mote por 4 tentativas e sempre ocorreu a resposta de outros...
	   // considero entao que ele esta fora da livelist
	   if (macRITstate == S_RIT_multichnhello_state) {
		   macneighbors_clearlivelist(ieee154e_vars.targetaddr);
	   }

	   endSlot();
   }


   openserial_startOutput();
}

port_INLINE void prepare_tx_msg(PORT_RADIOTIMER_WIDTH capturedTime) {

	uint8_t ret = false;
	uint8_t *msg;
	uint8_t msglen=0;
	uint32_t duration=0;

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

		radio_loadPacket(sRIT_vars.msg , sRIT_vars.msglength);

		radio_txEnable();

		radiotimer_schedule(TX_RIT_DELAYRXTX_MS);

		#if 1 //((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
		{
			uint8_t   pos=0;
			uint8_t *pucAux = (uint8_t *) &capturedTime;

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

port_INLINE void activityrx_preparetxack(PORT_RADIOTIMER_WIDTH capturedTime) {
   //PORT_SIGNED_INT_WIDTH timeCorrection;
   header_IE_ht header_desc;
   uint32_t duration;

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

#if 0
   // calculate the time timeCorrection (this is the time when the packet arrive w.r.t the time it should be.
   timeCorrection = (PORT_SIGNED_INT_WIDTH)((PORT_SIGNED_INT_WIDTH)ieee154e_vars.syncCapturedTime-(PORT_SIGNED_INT_WIDTH)TsTxOffset);

   // add the payload to the ACK (i.e. the timeCorrection)
   packetfunctions_reserveHeaderSize(ieee154e_vars.ackToSend,sizeof(timecorrection_IE_ht));
   timeCorrection  = -timeCorrection;
   timeCorrection *= US_PER_TICK;
   ieee154e_vars.ackToSend->payload[0] = (uint8_t)((((uint16_t)timeCorrection)   ) & 0xff);
   ieee154e_vars.ackToSend->payload[1] = (uint8_t)((((uint16_t)timeCorrection)>>8) & 0xff);
#endif
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
#if (ENABLE_CSMA_CA == 1)
   radiotimer_schedule(RX_RIT_DELAY_TX_TO_RX_MS);
#else
   duration = ieee154e_vars.lastCapturedTime+328;
   radiotimer_schedule(duration);
#endif

}

port_INLINE void activityrx_senddataack(void) {
  uint32_t duration;

  radiotimer_cancel();

  changeState(S_RIT_TXACK);
  incroute(0x07);

#if 0 //((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
{
	uint8_t   pos=0;
	uint8_t   *pauxframe;

	pauxframe = (uint8_t *) &ieee154e_vars.dataReceived->packet[0];

	rffbuf[pos++]= RFF_IEEE802_RX;
	rffbuf[pos++]= 0x07;
/*
	rffbuf[pos++]= pauxframe[75];
	rffbuf[pos++]= pauxframe[76];
	rffbuf[pos++]= pauxframe[77];
	rffbuf[pos++]= pauxframe[78];
	rffbuf[pos++]= pauxframe[79];
	rffbuf[pos++]= pauxframe[80];
	rffbuf[pos++]= pauxframe[81];
	rffbuf[pos++]= pauxframe[82];
	rffbuf[pos++]= pauxframe[83];
	rffbuf[pos++]= pauxframe[84];
*/
	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	openserial_startOutput();
}
#endif
#if (ENABLE_CSMA_CA == 1)
  //ack does not use CSMA-CA - spec
  radio_txNow(TX_NOT_USE_CSMA_CA);

  radiotimer_schedule(RX_RIT_ACKECHO_TIMEOUT_MS);
#else
  radio_txNow();

  duration = DURATION_rt7;
  radiotimer_schedule(DURATION_rt7);
#endif
}

port_INLINE void activityrx_txackok(PORT_RADIOTIMER_WIDTH capturedTime) {
   // change state
   changeState(S_RIT_RXPROC);
   incroute(0x08);

   // cancel rt8
   radiotimer_cancel();

   // record the captured time
   ieee154e_vars.lastCapturedTime = capturedTime;

   // turn off the radio
   radio_rfOff();

   // compute the duty cycle if radio has been turned on
   //if (ieee154e_vars.radioOnThisSlot==TRUE){
   //  ieee154e_vars.radioOnTics+=(radio_getTimerValue()-ieee154e_vars.radioOnInit);
   //}

	#if 0 //((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
	{
		uint8_t   pos=0;
		uint8_t   *pauxframe;

		pauxframe = (uint8_t *) &ieee154e_vars.dataReceived->packet[0];

		rffbuf[pos++]= RFF_IEEE802_RX;
		rffbuf[pos++]= 0x08;
/*
		rffbuf[pos++]= pauxframe[84];
		rffbuf[pos++]= pauxframe[85];
		rffbuf[pos++]= pauxframe[86];
		rffbuf[pos++]= pauxframe[87];
		rffbuf[pos++]= pauxframe[88];
		rffbuf[pos++]= pauxframe[89];
		rffbuf[pos++]= pauxframe[90];
*/
		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
		openserial_startOutput();
	}
	#endif

   // free the ack we just sent so corresponding RAM memory can be recycled
   openqueue_freePacketBuffer(ieee154e_vars.ackToSend);

   // clear local variable
   ieee154e_vars.ackToSend = NULL;

#if 0
   // synchronize to the received packet
   if (idmanager_getIsDAGroot()==FALSE && neighbors_isPreferredParent(&(ieee154e_vars.dataReceived->l2_nextORpreviousHop))) {
      synchronizePacket(ieee154e_vars.syncCapturedTime);
   }
#endif

   // inform upper layer of reception (after ACK sent)
   notif_receive(ieee154e_vars.dataReceived);

   // clear local variable
   ieee154e_vars.dataReceived = NULL;

   // official end of Rx slot
   endSlot();

}

//======= frame validity check

/**
\brief Decides whether the packet I just received is valid received frame.

A valid Rx frame satisfies the following constraints: (PARA O AMCA)  TODO!!! PARA O AMAC OU RITMC DEVE ACEITAR O ACK TAMBEM
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

	open_addr_t srcaddr;

   // record the outcome of the trasmission attempt
   packetSent->l2_sendDoneError   = error;

   // record the current ASN
   memcpy(&packetSent->l2_asn,&ieee154e_vars.asn,sizeof(asn_t));
   // associate this packet with the virtual component
   // COMPONENT_IEEE802154E_TO_RES so RES can knows it's for it
   packetSent->owner              = COMPONENT_IEEE802154E_TO_SIXTOP;

#if 0 //ENABLE_DEBUG_RFF
   {
	//uint8_t *pucAux01 = (uint8_t *) &ieee154e_vars.asn.bytes0and1;
	//uint8_t *pucAux23 = (uint8_t *) &ieee154e_vars.asn.bytes2and3;
    uint8_t pos=0;

	rffbuf[pos++]= RFF_IEEE802_TX;
	rffbuf[pos++]= 0x11;
	rffbuf[pos++]= packetSent->l4_protocol;
	rffbuf[pos++]= error;
	rffbuf[pos++]= packetSent->creator;
	rffbuf[pos++]= packetSent->l2_sendDoneError;
	rffbuf[pos++]= scheduler_dbg.numTasksCur;
	rffbuf[pos++]= scheduler_dbg.numTasksMax;

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
   }
#endif

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
	//bool listenForAck;
	uint32_t   resto=0;

	// change state
	changeState(S_RIT_SLEEP);

	radiotimer_cancel();

    // turn off the radio
    radio_rfOff();

    radio_flushfifos();

    if (macRITstate == S_RIT_TX_state){
		//leds_debug_off();
		//leds_sync_off();
    }

	// record the captured time
    ieee154e_vars.lastCapturedTime = getdeltaslotperiod_ms();   //getdeltaTimerValue();

	#if (SINK == 1)
		//TODO CICLO ABRO A SERIAL PARA ENTRADA
		openserial_stop();
		openserial_startInput();
	#endif

    //imprimir estatisticas
	if (macRITstate == S_RIT_RX_state)
	    ieee154e_dbg.num_rxend++;
	else
	    ieee154e_dbg.num_txend++;

	if ((ieee154e_dbg.num_newSlot % 100) == 0) {
		printstat();
	}
	else{
		printroute();
	}

	//Limpo a fila de mensagens pendentes
	RITQueue_cleanupoldmsg();
	sRIT_vars = RITQueue_Get_Element(ieee154e_vars.RITQueue_ElementPending);

   //clear vars for duty cycle on this slot
	ieee154e_vars.radioOnTics=0;
	ieee154e_vars.radioOnThisSlot=FALSE;

	frameDioPending = 0;

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
   macRITstate = 0;
   //leds_error_off();
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
	  case S_RIT_TXOLAPREPARE:               //Rx02
		  activityrx_sendola();
		 break;
	  case S_RIT_TXOLA:                      //Rx83
		  activityrx_rxolanoecho();
		 break;
	  case S_RIT_RXDATAPREPARE:              //Rx04
		 activity_rxwindowopen();
	   	 break;
	  case S_RIT_RXDATA:                     //Rx84
		  activity_rxritwindowend();
		  break;
      case S_RIT_TXACKPREPARE:               //Rx07
    	  activityrx_senddataack();
         break;
      case S_RIT_TXACK:                      //Rx88
    	 activityrx_noacktx();
         break;
  	  case S_RIT_TXDATAOFFSET:               //Tx02
  		 activitytx_rxolaprepare();
  		 break;
  	  case S_RIT_RXOLAPREPARE:               //Tx03
		 activitytx_rxwindowopen();
		 break;
  	  case S_RIT_TXMULTICASTTIMEOUT:
 	  case S_RIT_RXOLA:                      //tx84
 		  activity_txritwindowend();
 		  break;
 	  case S_RIT_TXACKOFFSET:                //tx05
		  prepare_tx_msg(0);
		  break;
 	  case S_RIT_TX_CONTINUEWAIT:            //tx45
 		  activitytx_reopenrxwindow();
		  break;
	  case S_RIT_TXDATAREADY:                //tx06
	     activitytx_senddata();
		 break;
	  case S_RIT_TXDATA:
		 activitytx_datanoecho();            //Tx87
		 break;
	  case S_RIT_RXACK:                      //Tx89
		 activitytx_rxnoack();
		 break;
/*
	  case S_RIT_SLEEP:
		 endSlot();
		 break;
*/
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
     case S_RIT_TXOLA:                             //Rx03
		  activityrx_dataprepare(capturedTime);
		 break;
	 case S_RIT_RXDATA:                            //Rx05
		 activity_rxnewframe(capturedTime);
		break;
	 case S_RIT_TXACK:                             //Rx08
		 activityrx_txackok(capturedTime);
		break;
	 case S_RIT_RXOLA:                             //Tx04
		 activity_rxnewframe(capturedTime);
		break;
	 case S_RIT_TXDATA:                            //Tx07
		 activitytx_senddone(capturedTime);
		break;
	 case S_RIT_RXACK:                             //Tx09
		 activitytx_rxackok(capturedTime);
		break;
/*
	 case S_RIT_SLEEP:
		 endSlot();
		break;
*/
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

