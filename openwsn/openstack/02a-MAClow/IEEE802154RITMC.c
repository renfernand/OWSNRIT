#include "opendefs.h"
#include "IEEE802154E.h"
#if (IEEE802154E_RITMC == 1)
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

//=========================== variables =======================================
extern radio_csma_vars_t radio_csma_vars;

uint8_t flagSerialTx;
uint8_t lastslotwastx;
#define TRATA_ACK 1
uint8_t testrff_isforwarding;
open_addr_t actualsrcaddr;         //usado quando recebe um frame...para identificar quem enviou...

extern uint8_t ucFlagForwarding;
uint8_t ucFlagTxReOpen;
uint8_t  rfftooglelastype=0;
extern scheduler_vars_t scheduler_vars;
extern scheduler_dbg_t  scheduler_dbg;
extern sRITqueue pvObjList[MAX_RIT_LIST_ELEM];
ieee154e_vars_t    ieee154e_vars;
ieee154e_stats_t   ieee154e_stats;
ieee154e_dbg_t     ieee154e_dbg;
open_addr_t        address_1;
uint8_t flagreschedule=0;
uint8_t     coappending;
sRITelement element;


//uint8_t u8NrMsgQueue;          // numero de posicoes utilizadas na queue de RIT Pending Msg to TX
//uint8_t numAvailableElements;  // numero de lugares ocupados no pool de mensagem
extern uint8_t RITQueue_ElementPending;   //salvo o elemento que acabou de enviar uma msg - para retornar apos o envio (apos o ola)
extern uint8_t maxElements;
extern uint8_t macRITActualPos;           //tem a informacao da posicao do atual Tx Msg Pending (antes do ola)
//uint32_t numOfQueueInsertionErrors;


RIT_stats_t ritstat;

uint8_t macRITstate;


//On in RIT period
static uint16_t macRITDataWaitDuration;
//Off in RIT period
static uint16_t macRITsleepPeriod;
//RIT period interrupted: On waiting for Ol�
static uint16_t macRITRXforTxPeriod;
static uint16_t macAckWaitPeriod;


//teste rff
uint8_t rffslotOffset;
uint8_t rffframetype;
uint8_t rffframelen;
uint8_t rffstate;

#if 1 //ENABLE_DEBUG_RFF
#define MAX_ELEM_ROUTE 100
volatile uint8_t ritroute[MAX_ELEM_ROUTE];
volatile uint8_t lastritroutepos = 0;

#define DBG_802154E_TX_DATA 1
#define DBG_802154E_RX_DATA 0
uint8_t rffnewsendmsg;
#define RFF_LOG_DEBUG_DAO 1
#endif


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

#define TESTE_TIMER 0
//=========================== prototypes ======================================
// interrupts
void     isr_ieee154e_newSlot(void);
void     isr_ieee154e_timer(void);

port_INLINE void getRitRequest(void);
void clearstatistics(void);

port_INLINE uint8_t activitytx_reopenrxwindow(PORT_RADIOTIMER_WIDTH capturedTime);
port_INLINE void activitytx_olaackprepare(PORT_RADIOTIMER_WIDTH capturedTime);
bool RITQueue_ExistFramePending(void);
port_INLINE void activitytx_preparedata(PORT_RADIOTIMER_WIDTH capturedTime);

owerror_t openqueue_freePacketRITBuffer(OpenQueueEntry_t* pkt);
port_INLINE void activitytx_senddata(void);

// SYNCHRONIZING
void     activity_synchronize_startOfFrame(PORT_RADIOTIMER_WIDTH capturedTime);
void     activity_synchronize_endOfFrame(PORT_RADIOTIMER_WIDTH capturedTime);

// TX
void     activity_ti1ORri1(void);
port_INLINE void StartTxRITProcedure(uint8_t elepending, uint8_t newtxframe);
port_INLINE void StartRxRITProcedure(void);
port_INLINE void activityrx_rxolanoecho(void);
port_INLINE void activityrx_waitolaecho(void);
port_INLINE void activityrx_sendola(void);
port_INLINE void activityrx_dataprepare(PORT_RADIOTIMER_WIDTH capturedTime);
void     activity_ti2(void);
void     activity_rxwindowend(void);
port_INLINE void activitytx_rxolaprepare(void);
port_INLINE void activityrx_senddataack(void);

void     activity_tie3(void);
void     activitytx_senddone(PORT_RADIOTIMER_WIDTH capturedTime);
port_INLINE void activitytx_rxwaitforack(PORT_RADIOTIMER_WIDTH capturedTime);
//void     activity_tie4(void);
void     activity_ti7(void);
void     activity_tie5(void);
//void     activity_ti8(PORT_RADIOTIMER_WIDTH capturedTime);
void     activityrx_noacktx(void);
void     activitytx_rxnoack(void);
void     activitytx_rxackok(PORT_RADIOTIMER_WIDTH capturedTime);
// RX
void     activitytx_rxwindowopen(void);
//void     activity_rie1(void);
void     activity_ritrxlistening(void);
void     activity_ritwindowend(void);
//void     activity_ri4(PORT_RADIOTIMER_WIDTH capturedTime);
void     activity_rie3(void);
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


#if (TESTE_CSMA == 1)

#if SINK
#define TX_RIT_WIN1_MS        100
#define TX_RIT_WIN1_TICKS     (32768 * TX_RIT_WIN1_MS)/1000

void teste3(void){
	changeState(S_RIT_TXOLAPREPARE);
#if (ENABLE_CSMA_CA == 1)
	radiotimer_schedule(TX_RIT_WIN1_MS);
#else
	radiotimer_schedule(TX_RIT_WIN1_TICKS);
#endif
}

//SOMENTE ENVIA FRAME
void teste2(void){

	//radio_setTimerPeriod(TX_RIT_PERIOD_MS);
#if 0
	radio_rfOff();

    clearritroute();

    //leds_sync_toggle();
    getRitRequest();
    //ieee154e_vars.dataToSend = openqueue_macGetDataPacket(&neighbor);
    ieee154e_vars.dataToSend->l4_protocol  = IANA_UNDEFINED;
	//ieee154e_vars.dataToSend->payload;
	ieee154e_vars.dataToSend->length+=36;

   StartTxRITProcedure(0,0);
   //openserial_stop();
   //openserial_startOutput();
#else
  // leds_sync_toggle();
   teste3();
#endif
   ieee154e_dbg.num_newSlot++;

}
#else



//SOMENTE RECEBE FRAME
void teste2(void){

   radio_setTimerPeriod(RX_RIT_PERIOD_MS);
   radio_rfOff();
   //leds_sync_toggle();
   StartRxRITProcedure();

   //openserial_stop();
   //openserial_startOutput();

   ieee154e_dbg.num_newSlot++;

}
#endif
#endif
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

void ieee154e_init(void) {

   // initialize variables
   memset(&ieee154e_vars,0,sizeof(ieee154e_vars_t));
   memset(&ieee154e_dbg,0,sizeof(ieee154e_dbg_t));

   changeIsSync(TRUE);
   ucFlagForwarding = FALSE;
   ucFlagTxReOpen = FALSE;

   resetStats();
   ieee154e_stats.numDeSync                 = 0;
   rfftooglelastype = 0;

   flagSerialTx = 0;
   flagreschedule = 0;

   macRITstate =S_RIT_sleep_state;

   // switch radio on
   radio_rfOn();

	//set the period constant
	macAckWaitPeriod       = TICK_RIT_ACK_WAIT_PERIOD;
	macRITRXforTxPeriod    = TICK_MAC_RIT_RX_TO_TX_PERIOD;
	macRITDataWaitDuration = TICK_MAC_RIT_RX_WIND_PERIOD;
	//macRITsleepPeriod = (uint16_t) ((uint16_t)macRITPeriod-(uint16_t)macRITDataWaitDuration);

	//initiate RIT Queue
	RITQueue_Init();
	clearstatistics();
	testrff_isforwarding = 0;

	leds_all_off();
	lastslotwastx=0;

   // set callback functions for the radio
   radio_setOverflowCb(isr_ieee154e_newSlot);  //timer indicando o inicio do slot
   radio_setCompareCb(isr_ieee154e_timer);     //timer diversos dentro do slot
   radio_setStartFrameCb(ieee154e_startOfFrame); //indica inicio do pacote
   radio_setEndFrameCb(ieee154e_endOfFrame);     //indica fim do um pacote

   // have the radio start its timer
   RITTimer_Init();

   clearritroute();
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

	ritstat.txdao.countdata=0;
	ritstat.txdao.countdatatx=0;
	ritstat.txdao.countdatatxok=0;
	ritstat.txdao.countdatatxerr=0;
	ritstat.txdao.countack=0;
	ritstat.txdao.countacktxrx=0;
	ritstat.txdao.countacktxrxok=0;
	ritstat.txdao.countacktxrxerr=0;

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
#if 1 //ENABLE_DEBUG_RFF
	int i;

	lastritroutepos = 0;
	for (i=0;i<100;i++)
	ritroute[i] = 0;
#endif
}

uint8_t incroute(uint8_t element)
{
#if 1 //ENABLE_DEBUG_RFF
	if (lastritroutepos >= MAX_ELEM_ROUTE)
		lastritroutepos = 0;

	ritroute[lastritroutepos] = element;
	lastritroutepos++;

    return lastritroutepos;
#else
    return 0;
#endif

}

/* ignorar imprimir RX quando entrar nos seguintes casos:
   0x85 - rxwindowend (acontece toda hora)
   0x68 - quando ele receber um frame porem � um ola.
*/
#if ENABLE_DEBUG_RFF
const uint8_t noprint[]={0x85,0x68};
uint8_t checkimprimir(void)
{
   uint8_t j,i;


    for(j=0;j<sizeof(noprint);j++){
    	for(i=0;i<lastritroutepos;i++){
    		if (ritroute[i] == noprint[j])
    			return 0;
    	}
    }
	return TRUE;
}
#endif

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
\brief Indicates a new slot has just started.

This function executes in ISR mode, when the new slot timer fires.
*/
/*
 * Network_StartRITProcedure - send Ola and open window for data
 *  */

void isr_ieee154e_newSlot() {

    //leds_sync_toggle();
#if (TESTE_CSMA == 0)
    activity_ti1ORri1();
#else

#if (SINK == 1)

#if (ENABLE_CSMA_CA == 1)
   radio_setTimerPeriod(TX_RIT_PERIOD_MS);
#else
   radio_setTimerPeriod(TX_RIT_PERIOD_TICKS);
#endif

   radio_rfOff();

   // clearritroute();

    getRitRequest();
    //ieee154e_vars.dataToSend = openqueue_macGetDataPacket(&neighbor);
    ieee154e_vars.dataToSend->l4_protocol  = IANA_UNDEFINED;
	//ieee154e_vars.dataToSend->payload;
	ieee154e_vars.dataToSend->length+=30;

    StartTxRITProcedure(0,0);

#else // SINK == 0
#if (ENABLE_CSMA_CA == 1)
    radio_setTimerPeriod(RX_RIT_PERIOD_MS);
#else
    radio_setTimerPeriod(RX_RIT_PERIOD_TICKS);
#endif
    radio_rfOff();
    //leds_sync_toggle();
    StartRxRITProcedure();
#endif


#endif



   ieee154e_dbg.num_newSlot++;
}


port_INLINE void activitytx_datanoecho(void) {

	//open_addr_t address;
	//uint8_t elementpos=0;
	//uint8_t ret=0;
	sRITqueue elequeue;
	//uint8_t endslot=false;

	changeState(S_RIT_TXDATANOECHO);
	incroute(0x87);
#if (TESTE_CSMA == 1)
	//leds_debug_off();
    //leds_sync_toggle();
	ritstat.txola.countdatatxerr++;
#endif
	// cancel tt4
	radiotimer_cancel();

   // turn off the radio
   radio_rfOff();
   ieee154e_vars.radioOnTics+=(radio_getTimerValue()-ieee154e_vars.radioOnInit);
   ieee154e_vars.radioOnThisSlot = 0;

    if (RITQueue_ElementPending < maxElements) {
    	elequeue = RITQueue_Get_Element(RITQueue_ElementPending);

    	/*
    	if (elequeue.frameType == IANA_ICMPv6_RPL_DIO)
    	{
			dio_stat.countsenderror++;
    	}
    	else if (elequeue.frameType == IANA_ICMPv6_RPL_DAO)
    	{
			dao_stat.countsenderror++;
    	}
        */
    	RITQueue_Free(RITQueue_ElementPending);
	}


#if 0 //((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_TX == 1))
  {
	uint8_t   pos=0;

	rffbuf[pos++]= RFF_IEEE802_TX;
	rffbuf[pos++]= 0x87;
	rffbuf[pos++]= RITQueue_ElementPending;
	rffbuf[pos++]= elequeue.frameType;
	rffbuf[pos++]= elequeue.msglength;
	rffbuf[pos++]= elequeue.isBroadcastMulticast;
	rffbuf[pos++]= elequeue.destaddr.type;
    if (element.destaddr.type == 0x01)	{
		rffbuf[pos++]= elequeue.destaddr.addr_16b[0];
		rffbuf[pos++]= elequeue.destaddr.addr_16b[1];
    }
    if (element.destaddr.type == 0x02)	{
		rffbuf[pos++]= elequeue.destaddr.addr_64b[6];
		rffbuf[pos++]= elequeue.destaddr.addr_64b[7];
    }
	else if (element.destaddr.type == 0x03) {
		rffbuf[pos++]= elequeue.destaddr.addr_128b[14];
		rffbuf[pos++]= elequeue.destaddr.addr_128b[15];
	}

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
  }
#endif

#if ((ENABLE_DEBUG_RFF == 1)  && (DBG_RADIO_POWER_CONS == 1))
	{
		uint8_t pos=0;
		uint8_t *pucAux = (uint8_t *) &ieee154e_vars.radioOnTics;

		rffbuf[pos++]= RFF_IEEE802_RADIO;
		rffbuf[pos++]= 0x87;
		//rffbuf[pos++]= (uint8_t) ieee154e_vars.dataToSend->length;
		//rffbuf[pos++]= (uint8_t) ieee154e_vars.dataReceived->length;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
#endif
#if (TESTE_CSMA == 0)
	   // indicate succesful Tx to schedule to keep statistics
	  schedule_indicateTx(&ieee154e_vars.asn,TRUE);
	  // indicate to upper later the packet was sent successfully
	  notif_sendDone(ieee154e_vars.dataToSend,E_FAIL);
#endif
	  // reset local variable
	  ieee154e_vars.dataToSend = NULL;
	  // abort
	  endSlot();
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
   bool txpending=FALSE;
   bool newtxframe=FALSE;
   uint32_t macRITPeriod;
   //uint32_t macRITTxPeriod;

   ieee154e_vars.lastCapturedTime = 0;
   radiotimer_cancel();

   //aqui eu estou posicionando o slot para o slot 1.
   if (ieee154e_vars.slotOffset > 3) {
	   ieee154e_vars.slotOffset       = 0;
	   schedule_syncSlotOffset(ieee154e_vars.slotOffset);
	   ieee154e_vars.nextActiveSlotOffset = schedule_getNextActiveSlotOffset();
   }

   // check whether we can send
  if (schedule_getOkToSend())
  {
      //last_slotoffset = ieee154e_vars.slotOffset;
      //last_nextActiveSlotOffset = ieee154e_vars.nextActiveSlotOffset;

       ieee154e_vars.slotOffset = RIT_checkpendingmsginsomeslot(&neighbor);
       ieee154e_vars.dataToSend = openqueue_macGetDataPacket(&neighbor);
       newtxframe = TRUE;
  }
  else
  {
     ieee154e_vars.dataToSend = NULL;
  }

  //teste rff - registra rota do TX
  clearritroute();

  //check if there is message pending from the last cycle
  //txpending = RITQueue_ExistFramePending();
  txpending = 0;

  if ((lastslotwastx == 0) && (ieee154e_vars.dataToSend != NULL)) {   // I have a packet to send

	//TODO!!! AQUI EU NAO ESTOU SALVANDO EM UMA FILA...NAO SERIA NECESSARIO CASO EU TENHA MULTIPLAS MSGS PARADAS...
    //se o estado anterior eh RIT_RX pode ser que estou esperando ainda um evento dele...
    StartTxRITProcedure(txpending,newtxframe);

	lastslotwastx = TRUE;

#if 0 //((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_TX == 1))
  {
	uint8_t   pos=0;
	uint8_t *pucAux = (uint8_t *) &ieee154e_dbg.num_newSlot;
	uint32_t period =  radiotimer_getPeriod();
	uint8_t *pucAux1 = (uint8_t *) &period;

	rffbuf[pos++]= RFF_IEEE802_TX;
	rffbuf[pos++]= 0x01;
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
    //Programo um slot de Tx que eh geralmente maior que o de rx
	//macRITTxPeriod = TICK_MAC_RIT_TX_PERIOD;
    //radio_setTimerPeriod(macRITTxPeriod);
  }
  else  {

		lastslotwastx = 0;
#if 0 //((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_TX == 1))
  {
	uint8_t   pos=0;
	uint8_t *pucAux = (uint8_t *) &ieee154e_dbg.num_newSlot;
	uint32_t period =  radiotimer_getPeriod();
	uint8_t *pucAux1 = (uint8_t *) &period;

	rffbuf[pos++]= RFF_IEEE802_RX;
	rffbuf[pos++]= 0x02;
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
	   if (ieee154e_vars.dataReceived != NULL) {
		   // free the (invalid) received data buffer so RAM memory can be recycled
		   openqueue_freePacketBuffer(ieee154e_vars.dataReceived);

		   // clear local variable
		   ieee154e_vars.dataReceived = NULL;
	   }

		#if SINK
			 //start inputting serial data
			  if (toogleTxRxSerial()){
				  openserial_stop();
				  openserial_startOutput();
				  StartRxRITProcedure();
			  }
			  else {
				  openserial_stop();
				  openserial_startInput();
				#if (SINK_SIMULA_COAP == 1)
				  //simula coap
				  openbridge_simucoap();
				#endif
			  }

	          macRITPeriod = TICK_MAC_RIT_PERIOD;
			  radio_setTimerPeriod(macRITPeriod);
		#else
		  openserial_stop();
		  openserial_startOutput();

			  StartRxRITProcedure();

          macRITPeriod = TICK_MAC_RIT_PERIOD;
		  radio_setTimerPeriod(macRITPeriod);
		#endif
  }
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
#if (TESTE_CSMA == 1) && (SINK == 1)
	   adv->l2_frameType                     = IEEE154_TYPE_DATA;
	   adv->l2_nextORpreviousHop.type        = ADDR_64B;
	   adv->l2_nextORpreviousHop.addr_16b[0] = 0x00;
	   adv->l2_nextORpreviousHop.addr_16b[1] = 0x12;
	   adv->l2_nextORpreviousHop.addr_16b[2] = 0x4B;
	   adv->l2_nextORpreviousHop.addr_16b[3] = 0x00;
	   adv->l2_nextORpreviousHop.addr_16b[4] = 0x02;
	   adv->l2_nextORpreviousHop.addr_16b[5] = 0xf4;
	   adv->l2_nextORpreviousHop.addr_16b[6] = 0xAF;
	   adv->l2_nextORpreviousHop.addr_16b[7] = 0xC0;
#else
	   adv->l2_frameType                     = IEEE154_TYPE_OLA;
	   //adv->l2_frameType                     = IEEE154_TYPE_BEACON;
	   adv->l2_nextORpreviousHop.type        = ADDR_16B;
	   adv->l2_nextORpreviousHop.addr_16b[0] = 0xff;
	   adv->l2_nextORpreviousHop.addr_16b[1] = 0xff;
#endif

	   //I has an IE in my payload
	   adv->l2_IEListPresent = IEEE154_IELIST_NO;


		ieee154e_vars.dataToSend->owner = COMPONENT_IEEE802154E;
		sync_IE.join_priority = neighbors_getMyDAGrank()/(2*MINHOPRANKINCREASE); //poipoi -- use dagrank(rank)
		// fill in the ASN field of the ADV
		//ieee154e_getAsn(sync_IE.asn);
		// record that I attempt to transmit this packet
		ieee154e_vars.dataToSend->l2_numTxAttempts++;

		//memcpy(ieee154e_vars.dataToSend->l2_ASNpayload,&sync_IE,sizeof(sync_IE_ht));

	   // put in queue for MAC to handle
	   sixtop_send_internal(adv,IEEE154_IELIST_NO,IEEE154_FRAMEVERSION);

	   // I'm now busy sending an ADV
	   //sixtop_vars.busySendingEB = TRUE;
}


port_INLINE void activityrx_prepareritdatareq(void) {

	header_IE_ht header_desc;
	OpenQueueEntry_t adv;
	uint8_t freq;
	uint8_t pos=0;
	uint8_t frame[30];
	uint32_t duration;
	open_addr_t myaddr;
	open_addr_t *pmyaddr=(open_addr_t *)&myaddr;

	// MONTA O FRAME DE RIT Data Request
	frame[pos++] = 0x40;  //FCS
	frame[pos++] = 0xe8;  //FCS
	frame[pos++] = 0xb1;  //Seq Number
	frame[pos++] = 0xfe;
	frame[pos++] = 0xca;
	//address dest
	frame[pos++] = 0xff;
	frame[pos++] = 0xff;
	//address source
	pmyaddr = idmanager_getMyID(ADDR_64B);
	frame[pos++] = pmyaddr->addr_16b[7];
	frame[pos++] = pmyaddr->addr_16b[6];
	frame[pos++] = pmyaddr->addr_16b[5];
	frame[pos++] = pmyaddr->addr_16b[4];
	frame[pos++] = pmyaddr->addr_16b[3];
	frame[pos++] = pmyaddr->addr_16b[2];
	frame[pos++] = pmyaddr->addr_16b[1];
	frame[pos++] = pmyaddr->addr_16b[0];
	frame[pos++] = 0x20;  //Command Frame Identifier
	//crc
	frame[pos++] = 0x00;
	frame[pos++] = 0x00;

	// space for 2-byte CRC
	//packetfunctions_reserveFooterSize(&adv,2);

	radio_loadPacket((uint8_t *)&frame[0],pos);
}
/*
// Aqui eh o caso onde ja enviei o rit e ja programei o timer
// Este start eh do proprio RIT entao devo somente aguardar o estouro do timer...
void activity_RITDoNothing(void){
	changeState(S_RIT_RXOLAREADY);
	macRITstate=S_RIT_RX_window_state;
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

	uint32_t dur_rt1;

	changeState(S_RIT_TXOLAPREPARE);
	macRITstate=S_RIT_RX_window_state;

	//#################    escolhe o canal
    //radiotimer_cancel();

    incroute(0x01);

	// calculate the frequency to transmit on
	ieee154e_vars.freq = calculateFrequency(schedule_getChannelOffset());

	// configure the radio for that frequency
	radio_setFrequency(ieee154e_vars.freq);

#if 0
	//frame do Beacon do OpenWSN...64bytes
	//################ load the packet in the radio's Tx buffer
	//pega o frame do rit da camada sixtop
	getRitRequest();

	radio_loadPacket(ieee154e_vars.dataToSend->payload,
					ieee154e_vars.dataToSend->length);

#else
	//frame reduzido...12 bytes
	activityrx_prepareritdatareq();
#endif
	//################ enable the radio in Tx mode. Transmit the packet
	radio_txEnable();
	ieee154e_vars.radioOnInit=radio_getTimerValue();
	ieee154e_vars.radioOnThisSlot=TRUE;

#if (TESTE_CSMA == 1)
    //leds_debug_on();
 	changeState(S_RIT_TXOLA);
 	ritstat.txola.countdatatx++;
    radio_txNow(TX_USE_CSMA_CA);
	radiotimer_schedule(RX_RIT_TXOLAECHO_MS);
#endif

	//leds_sync_on();
	//leds_sync_toggle();
}

port_INLINE void activityrx_sendola(void) {
  uint32_t dur_rt2;

  changeState(S_RIT_TXOLA);
  incroute(0x02);

  //radiotimer_cancel();

  //ieee154e_vars.radioOnInit=radio_getTimerValue();
  //ieee154e_vars.radioOnThisSlot=TRUE;

  radio_txNow(TX_USE_CSMA_CA);

//aqui devo aguardar um tempo para ligar o radio como RX...
  //dur_rt1 = ieee154e_vars.lastCapturedTime+TsTxOffset-TsLongGT-delayRx-maxRxDataPrepare;
  //dur_rt2 = ieee154e_vars.lastCapturedTime+70;
  //dur_rt2 = ieee154e_vars.lastCapturedTime+200;
  //radiotimer_schedule(dur_rt2);

#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TIMER == 1))
	{
		uint8_t *pucAux = (uint8_t *) &dur_rt2;
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_RX;
		rffbuf[pos++]= 0x02;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
#endif
}



#if 0
port_INLINE void activityrx_waitolaecho(void) {
  uint32_t dur_rt3;

  changeState(S_RIT_RXOLAECHO);
  incroute(0x03);

  //radiotimer_cancel();
  // turn off the radio
  //radio_rfOff();
  //ieee154e_vars.radioOnTics+=(radio_getTimerValue()-ieee154e_vars.radioOnInit);
  //ieee154e_vars.radioOnThisSlot = 0;

 // calculate the frequency to Rx on
 ieee154e_vars.freq = calculateFrequency(schedule_getChannelOffset());

 // configure the radio for that frequency
 radio_setFrequency(ieee154e_vars.freq);

 // enable the radio in Rx mode. The radio does not actively listen yet.
  radio_rxEnable();

  radio_rxNow();

  //dur_rt2 = ieee154e_vars.lastCapturedTime+TsTxOffset-TsLongGT-delayRx;
  dur_rt3 = ieee154e_vars.lastCapturedTime+200;
  radiotimer_schedule(dur_rt3);

#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TIMER == 1))
	{
		uint8_t *pucAux = (uint8_t *) &dur_rt3;
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_RX;
		rffbuf[pos++]= 0x03;
		rffbuf[pos++]= (uint8_t) ieee154e_vars.dataToSend->length;
		rffbuf[pos++]= (uint8_t) ieee154e_vars.dataReceived->length;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
#endif
}
#endif
port_INLINE void activityrx_rxolanoecho(void) {

  changeState(S_RIT_RXOLANOECHO);
  incroute(0x83);
  ritstat.txola.countdatatxerr++;

  // turn off the radio - aqui eu nao recebi o ola...ignoro o slot
  radiotimer_cancel();
  radio_rfOff();
  ieee154e_vars.radioOnTics+=(radio_getTimerValue()-ieee154e_vars.radioOnInit);
  ieee154e_vars.radioOnThisSlot = 0;
  //sinalizo erro pois o frame nao foi enviado...e termino o slot.
  //openqueue_freePacketBuffer(ieee154e_vars.dataReceived);
  endSlot();
  ieee154e_vars.dataReceived = NULL;
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
		testerff = 0x01;
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
				pmsgout->frameType = macRIT_Pending_TX_frameType;
				pmsgout->destaddr  = ieee154e_vars.dataToSend->l3_destinationAdd;
				testerff = 0x12;
                //rit statistics - conta novo produtor
				ritstat.txdio.countdata++;
			}
			else
			{
				//inclui na fila
				testerff = 0x13;
			}

		}
		else if (ieee154e_vars.dataToSend->l4_sourcePortORicmpv6Type == IANA_ICMPv6_RPL)
		{
			testerff = 0x02;
			//FRAME DAO
			macRIT_Pending_TX_frameType = IANA_ICMPv6_RPL_DAO;

            //rit statistics - conta novo produtor
			ritstat.txdao.countdata++;

			if (ieee154e_vars.dataToSend->creator == COMPONENT_FORWARDING)
			{
				testerff = 0x03;

				 //salvo o endereco do destino que pode estar na posicao final do frame (RPL Transition)
				 //address_1.type = ieee154e_vars.dataToSend->l3_destinationAdd.type;
				 RITQueue_copyaddress(&address_1,&ieee154e_vars.dataToSend->l3_destinationAdd);

				 if (ieee154e_vars.dataToSend->l3_destinationAdd.type == 3)
				 {
						testerff = 0x04;


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
				testerff = 0x05;
				 //salvo o endereco do destino que pode estar na posicao final do frame (RPL Transition)
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
			}

			pmsgout->frameType = macRIT_Pending_TX_frameType;
			pmsgout->destaddr  = address_1;

		}
	}
	else if (ieee154e_vars.dataToSend->l4_protocol == IANA_UDP) {
		testerff = 0x09;
		macRIT_Pending_TX_frameType = IANA_UDP;
		pmsgout->msglength =  ieee154e_vars.dataToSend->length;
		pmsgout->timestamp = radio_getTimerValue();
		pmsgout->frameType = macRIT_Pending_TX_frameType;
		pmsgout->destaddr = ieee154e_vars.dataToSend->l2_nextORpreviousHop;
		pmsgout->msg = (uint8_t *) ieee154e_vars.dataToSend->payload;
		flagpending = true;

        //rit statistics - conta novo produtor
		ritstat.txcoap.countdata++;
	}
	else if (ieee154e_vars.dataToSend->l4_protocol == IANA_UNDEFINED) {
		testerff = 0x06;

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
#if (TESTE_CSMA == 1)
		if (1)
#else
		if ((ieee154e_vars.dataToSend->l3_destinationAdd.type == ADDR_NONE) &&
			(icmpv6_type == IANA_ICMPv6_RPL) &&
			(icmpv6_code == IANA_ICMPv6_RPL_DIO))
#endif
		{
			testerff = 0x07;

            //rit statistics - conta novo produtor
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
			testerff = 0x08;
			macRIT_Pending_TX_frameType = IANA_UDP;
			pmsgout->frameType = macRIT_Pending_TX_frameType;
			pmsgout->destaddr  = ieee154e_vars.dataToSend->l2_nextORpreviousHop;

            //rit statistics - conta novo produtor
			ritstat.txcoap.countdata++;
		}
	}
	else if (ieee154e_vars.dataToSend->l4_protocol == IANA_IPv6ROUTE) {
		//AQUI EH QUANDO O COAP ENVIA UM FORWARDING FRAME...
		testerff = 0x0A;

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
			testerff = 0x0B;
			macRIT_Pending_TX_frameType = IANA_UDP;
			pmsgout->frameType = macRIT_Pending_TX_frameType;
			pmsgout->destaddr  = ieee154e_vars.dataToSend->l2_nextORpreviousHop;

            //rit statistics - conta novo produtor
			ritstat.txcoap.countdata++;
		}
	}


	if (macRIT_Pending_TX_frameType > 0)
	{
		pmsgout->isBroadcastMulticast = packetfunctions_isBroadcastMulticast(&ieee154e_vars.dataToSend->l2_nextORpreviousHop);

		if (pmsgout->isBroadcastMulticast){
			//descubro quantos vizinhos tenho na minha vizinhanca
		    numTargetParents = neighbors_howmanyIhave();
		}
		else{
		    numTargetParents = 1;
		}
		//coloco elemento na fila do RIT_Tx
		macRITActualPos = RITQueue_Put(pmsgout,flagpending,numTargetParents);
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
		rffbuf[pos++]= macRIT_Pending_TX_frameType;
		rffbuf[pos++]= testerff;
		rffbuf[pos++]= ieee154e_vars.dataToSend->l4_protocol;
		rffbuf[pos++]= ieee154e_vars.dataToSend->l2_numTxAttempts;
		rffbuf[pos++]= ieee154e_vars.dataToSend->l2_retriesLeft;
		rffbuf[pos++]= pmsgout->isBroadcastMulticast;
		rffbuf[pos++]= numTargetParents;

		//rffbuf[pos++]= pmsgout->msglength;
		//rffbuf[pos++]= iphc_header1;
		//rffbuf[pos++]= iphc_nextheader;

		rffbuf[pos++]= pmsgout->destaddr.type;
		if (element.destaddr.type == 0x01)	{
			rffbuf[pos++]= pmsgout->destaddr.addr_16b[0];
			rffbuf[pos++]= pmsgout->destaddr.addr_16b[1];
		}
		if (element.destaddr.type == 0x02)	{
			rffbuf[pos++]= pmsgout->destaddr.addr_64b[6];
			rffbuf[pos++]= pmsgout->destaddr.addr_64b[7];
		}
		else if (element.destaddr.type == 0x03) {
			rffbuf[pos++]= pmsgout->destaddr.addr_128b[14];
			rffbuf[pos++]= pmsgout->destaddr.addr_128b[15];
		}


		/*
		rffbuf[pos++]= 0xDD;
		rffbuf[pos++]= dio_stat.countdata;
		rffbuf[pos++]= dio_stat.countdatatxok;
		rffbuf[pos++]= dio_stat.countsenderror;
		rffbuf[pos++]= 0xEE;
		rffbuf[pos++]= dao_stat.countdata;
		rffbuf[pos++]= dao_stat.countdatatxok;
		rffbuf[pos++]= dao_stat.countsenderror;

		rffbuf[pos++]= 0xEE;
		rffbuf[pos++]= numAvailableElements;
		for (i=0; i<maxElements ;i++)
			rffbuf[pos++]= pvObjList[i].frameType;

		rffbuf[pos++]= 0xCC;
		pucAux = (uint8_t *) &ieee154e_vars.asn;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= 0xcc;
		pucAux = (uint8_t *) &ieee154e_vars.dataToSend;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= 0xcc;
	*/
		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
	#endif


   return (macRIT_Pending_TX_frameType);
}


//como tem mensagem pendente ele stop o RIT_OLA e start RIT RX_ waiting for TX  ola
port_INLINE void StartTxRITProcedure(uint8_t txpending,uint8_t newtxframe) {

	uint8_t macRIT_Pending_TX_frameType=0;
	//uint8_t i;
	uint32_t dur_rt1;
	uint32_t macRITTxPeriod;
	//sRITelement element;

	changeState(S_RIT_TXDATAOFFSET);
    macRITstate=S_RIT_TX_state;

    incroute(0x01);


/*
 * TODO!!!! EXISTE OUTRO TIPO DE DADO SEM SER DO TIPO DATA OU BEACON ?
 */
	if (ieee154e_vars.dataToSend->l2_frameType == IEEE154_TYPE_DATA) {
		macRIT_Pending_TX_frameType = checkmsgtype(&element,txpending,newtxframe);
	}

    //incroute(macRIT_Pending_TX_frameType);
	incroute((uint8_t) ieee154e_dbg.num_newSlot);
    //Programo um slot de Tx que eh geralmente maior que o de rx
#if (TESTE_CSMA == 0)
    macRITTxPeriod = TICK_MAC_RIT_TX_PERIOD;
    radio_setTimerPeriod(macRITTxPeriod);
#else
    ritstat.txola.countdata++;
#endif
    //TODO!!!!aqui tive problemas de frame invalido...nao sei por que...
    //neste caso eu desprezo o frame.
    if (ieee154e_vars.dataToSend->l2_retriesLeft > 3)
    {
    	  incroute(0x81);
		  // indicate to upper later the packet was sent successfully
		  notif_sendDone(ieee154e_vars.dataToSend,E_FAIL);
		  // reset local variable
		  ieee154e_vars.dataToSend = NULL;
		  // abort
		  endSlot();

    }
    else if ((macRIT_Pending_TX_frameType > 0) || (txpending))
	{
		//radiotimer_schedule(DURATION_rt2);
		//dur_rt1 = 22;
		//dur_rt1 = ieee154e_vars.lastCapturedTime+TsTxOffset-TsLongGT-delayRx-maxRxDataPrepare;
		//dur_rt1 = ieee154e_vars.lastCapturedTime+22;
		//radiotimer_schedule(dur_rt1);

    	//leds_debug_on();

    	activitytx_rxolaprepare();

	#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TIMER == 1))
		{
			uint8_t *pucAux = (uint8_t *) &dur_rt1;
			uint8_t pos=0;

			rffbuf[pos++]= RFF_IEEE802_TX;
			rffbuf[pos++]= 0x01;
			rffbuf[pos++]= (uint8_t) ieee154e_vars.dataToSend->length;
			rffbuf[pos++]= (uint8_t) ieee154e_vars.dataReceived->length;
			rffbuf[pos++]= *pucAux++;
			rffbuf[pos++]= *pucAux++;
			rffbuf[pos++]= *pucAux++;
			rffbuf[pos++]= *pucAux;
			openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
		}
	#endif
	}

}

/* Tx Rit procedure - Aqui ele vai ter de esperar receber um frame de RIT para enviar
 *  Calcula Frequencia
 *  prepara o frame do RIT request
 *  carrega o frame
 */
port_INLINE void activitytx_senddata(void) {

	uint32_t duration;

    changeState(S_RIT_TXDATA);
    incroute(0x06);

    radio_txNow(1);


    //duration = ieee154e_vars.lastCapturedTime + RIT_DURATION_tt4;   tt4=500
    duration = ieee154e_vars.lastCapturedTime + 700;
   radiotimer_schedule(duration);

#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TIMER == 1))
	{
		uint8_t *pucAux = (uint8_t *) &duration;
		uint8_t *pucAux1 = (uint8_t *) &ieee154e_vars.lastCapturedTime;
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_TX;
		rffbuf[pos++]= 0x06;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;
		rffbuf[pos++]= 0xcc;
		rffbuf[pos++]= *pucAux1++;
		rffbuf[pos++]= *pucAux1++;
		rffbuf[pos++]= *pucAux1++;
		rffbuf[pos++]= *pucAux1;
		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
#endif
}



port_INLINE void activity_tie3() {
   // log the error
   openserial_printError(COMPONENT_IEEE802154E,ERR_WDDATADURATION_OVERFLOWS,
                         (errorparameter_t)ieee154e_vars.state,
                         (errorparameter_t)ieee154e_vars.slotOffset);

   // abort
   endSlot();
}

/*
 * Aqui quando Tx significa que o frame foi enviado com sucesso (acontece apos ter recebido o echo do final do frame)
 */
port_INLINE void activitytx_senddone(PORT_RADIOTIMER_WIDTH capturedTime) {
//   bool listenForAck;
	open_addr_t address;
	uint8_t elementpos=0;
	uint8_t ret=0;
	sRITqueue elequeue;
	uint8_t endslot=false;

   // change state
   changeState(S_RIT_RXACKOFFSET);
   incroute(0x07);

   // cancel tt4
   radiotimer_cancel();

   // turn off the radio
   radio_rfOff();

#if (TESTE_CSMA == 1)
    ritstat.txola.countdatatxok++;
    //leds_debug_off();
#endif
	//verifica se existe msg pendente para este endereco de destino
    //   RITQueue_copyaddress(&address,&element.destaddr);
    if (RITQueue_ElementPending < maxElements) {

    	//decrementa elemento pendente...para broadcast
    	RITQueue_update_element(RITQueue_ElementPending);

    	elequeue = RITQueue_Get_Element(RITQueue_ElementPending);

    	//update statistics
    	switch (elequeue.frameType)
    	{
    		case  IANA_ICMPv6_RPL_DIO:
    			ritstat.txdio.countdatatxok++;
    			break;
    		case  IANA_ICMPv6_RPL_DAO:
    			ritstat.txdao.countdatatxok++;
    			break;
    		case  IANA_UDP:
    			ritstat.txcoap.countdatatxok++;
    			break;

    	}
	}

	if (elequeue.frameType  != IANA_ICMPv6_RPL_DIO)  {
       if (ieee154e_vars.dataToSend != NULL)
       {
			#if 0//((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_TX == 1))
			  {
				uint8_t   pos=0;
				uint8_t  *pucAux;

				rffbuf[pos++]= RFF_IEEE802_TX;
				rffbuf[pos++]= 0x43;
				rffbuf[pos++]= RITQueue_ElementPending;
				rffbuf[pos++]= elequeue.frameType;
				rffbuf[pos++]= elequeue.msglength;
				rffbuf[pos++]= elequeue.isBroadcastMulticast;
				rffbuf[pos++]= 0xcc;
				pucAux = (uint8_t *) &ieee154e_vars.dataToSend;
				rffbuf[pos++]= *pucAux++;
				rffbuf[pos++]= *pucAux++;
				rffbuf[pos++]= *pucAux++;
				rffbuf[pos++]= *pucAux++;
				rffbuf[pos++]= 0xcc;

				openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
			  }
			#endif


    	   // decides whether to listen for an ACK
    	   //if (packetfunctions_isBroadcastMulticast(&ieee154e_vars.dataToSend->l2_nextORpreviousHop) !=TRUE) {
		   if (elequeue.isBroadcastMulticast == FALSE) {
			   activitytx_rxwaitforack(capturedTime);
		   }
		   else {
			   endslot=TRUE;
		   }


	   }
       else { //(ieee154e_vars.dataToSend == NULL
			#if ((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_TX == 1))
			  {
				uint8_t   pos=0;
				uint8_t  *pucAux;

				rffbuf[pos++]= RFF_IEEE802_TX;
				rffbuf[pos++]= 0x55;
				rffbuf[pos++]= RITQueue_ElementPending;
				rffbuf[pos++]= elequeue.frameType;
				rffbuf[pos++]= elequeue.msglength;
				rffbuf[pos++]= elequeue.isBroadcastMulticast;
				rffbuf[pos++]= 0xcc;
				pucAux = (uint8_t *) &ieee154e_vars.dataToSend;
				rffbuf[pos++]= *pucAux++;
				rffbuf[pos++]= *pucAux++;
				rffbuf[pos++]= *pucAux++;
				rffbuf[pos++]= *pucAux++;
				rffbuf[pos++]= 0xcc;

				openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
			  }
			#endif

    	   endslot=TRUE;
       }

	}
	else
	{ //frameType  == IANA_ICMPv6_RPL_DIO


		#if ((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_TX == 1))
		  {
			uint8_t   pos=0;
			uint8_t  *pucAux;

			rffbuf[pos++]= RFF_IEEE802_TX;
			rffbuf[pos++]= 0x72;
			rffbuf[pos++]= RITQueue_ElementPending;
			rffbuf[pos++]= elequeue.frameType;
			rffbuf[pos++]= elequeue.msglength;
			rffbuf[pos++]= elequeue.isBroadcastMulticast;
			rffbuf[pos++]= elequeue.numTargetParents;
			rffbuf[pos++]= actualsrcaddr.type;
			switch (actualsrcaddr.type)
			{
				case 0x01:
					rffbuf[pos++]= actualsrcaddr.addr_16b[0];
					rffbuf[pos++]= actualsrcaddr.addr_16b[1];
					break;
				case 0x02:
					rffbuf[pos++]= actualsrcaddr.addr_64b[6];
					rffbuf[pos++]= actualsrcaddr.addr_64b[7];
					break;
				case 0x03:
					rffbuf[pos++]= actualsrcaddr.addr_128b[14];
					rffbuf[pos++]= actualsrcaddr.addr_128b[15];
					break;
				default:
					break;
			}
			openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
		  }
		#endif
		//O RPL DIO EH BROADCAST...
		//ENTAO EU DEVO CHECAR SE EU JA ENVIEI O DIO PARA TODOS OS MEUS VIZINHOS...
		//E SE AINDA NAO DEVO ESPERAR UM NOVO OLA...
		//FACO ISSO ATE TERMINAR A MINHA JANELA DE RIT_TX.
        //TODO!!!! NAO FICOU BOM>>>ACHO MELHOR MANDAR A CADA TEMPO UM BROADCAST MAIS ESPACADO...DIRETAMENTE ELE ESTA PERDENDO MSG...
		/*
		  if (elequeue.numTargetParents > 0) {
			incroute(0x71);
		    if (activitytx_reopenrxwindow(capturedTime) == 0) {
				endslot=TRUE;
		    }
		}
		else {
		*/
#if (TESTE_CSMA == 0)

		endslot=TRUE;
		//}
#else
		ritstat.txola.countack++;
		activitytx_rxwaitforack(capturedTime);
		endslot=0;
#endif

	}

	if (endslot)
	{
		incroute(0x72);
		//calcula o tempo decorrido do radio ligado
		  ieee154e_vars.radioOnTics+=(radio_getTimerValue()-ieee154e_vars.radioOnInit);

		  ret = RITQueue_Free(RITQueue_ElementPending);

#if (TESTE_CSMA == 0)
		   // indicate succesful Tx to schedule to keep statistics
		  schedule_indicateTx(&ieee154e_vars.asn,TRUE);
		  // indicate to upper later the packet was sent successfully
		  notif_sendDone(ieee154e_vars.dataToSend,E_SUCCESS);
#endif
		  // reset local variable
		  ieee154e_vars.dataToSend = NULL;
		  // abort
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

	// calculate the frequency to transmit on
   ieee154e_vars.freq = calculateFrequency(schedule_getChannelOffset());

   ieee154e_vars.lastCapturedTime = capturedTime;

   // configure the radio for that frequency
   radio_setFrequency(ieee154e_vars.freq);

   // enable the radio in Rx mode. The radio is not actively listening yet.
   radio_rxEnable();

   radio_rxNow();
#if (TESTE_CSMA == 0)
   duration = ieee154e_vars.lastCapturedTime+TICK_RIT_ACK_WAIT_PERIOD;
   radiotimer_schedule(duration);
#else
   radiotimer_schedule(TX_RIT_ACK_TIMEOUT_MS);
#endif

}



/* TX_ACK_RESPONSE_TIMEOUT
 * Aqui se TX eu enviei o dado e esperava um ack...Entao ocorreu timeout e eu nao recebi o ack.
 */
port_INLINE void activity_tie5() {

#if ENABLE_DEBUG_RFF
   {
     uint32_t capturetime;
     uint8_t *pucAux = (uint8_t *) &capturetime;
     uint8_t pos=0;
     capturetime=radio_getTimerValue();

	 rffbuf[pos++]= RFF_IEEE802_TX;
	 rffbuf[pos++]= 0x13;
	 rffbuf[pos++]= *pucAux++;
	 rffbuf[pos++]= *pucAux++;
	 rffbuf[pos++]= *pucAux++;
	 rffbuf[pos++]= *pucAux;

	 openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
   }
#endif //teste rff


#if 1
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

	// abort
	endSlot();
#else
	// indicate transmit failed to schedule to keep stats
	schedule_indicateTx(&ieee154e_vars.asn,TRUE);

	// decrement transmits left counter
	ieee154e_vars.dataToSend->l2_retriesLeft=0;

	if (ieee154e_vars.dataToSend->l2_retriesLeft==0) {
	  // indicate tx fail if no more retries left
	  notif_sendDone(ieee154e_vars.dataToSend,E_SUCCESS);
	} else {
	  // return packet to the virtual COMPONENT_SIXTOP_TO_IEEE802154E component
	  ieee154e_vars.dataToSend->owner = COMPONENT_SIXTOP_TO_IEEE802154E;
	}

	// reset local variable
	ieee154e_vars.dataToSend = NULL;

	// abort
	endSlot();
#endif
}


port_INLINE void activityrx_noacktx() {

   incroute(0x88);
   // change state
   changeState(S_RIT_NOECHOTXACK);

   // turn off the radio
   radio_rfOff();

   // compute the duty cycle if radio has been turned on
   if (ieee154e_vars.radioOnThisSlot==TRUE){
	  ieee154e_vars.radioOnTics+=(radio_getTimerValue()-ieee154e_vars.radioOnInit);
   }

#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_RADIO_POWER_CONS == 1))
	{
		uint8_t *pucAux = (uint8_t *) &ieee154e_vars.radioOnTics;
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_RADIO;
		rffbuf[pos++]= 0x88;
		//rffbuf[pos++]= (uint8_t) ieee154e_vars.dataToSend->length;
		//rffbuf[pos++]= (uint8_t) ieee154e_vars.dataReceived->length;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;
		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
#endif
   //clear vars for duty cycle on this slot
   //ieee154e_vars.radioOnTics=0;
   //ieee154e_vars.radioOnThisSlot=FALSE;

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

#if (TESTE_CSMA == 1)
   ritstat.rxola.countacktxrxerr++;
#endif
    // abort
    endSlot();
}


port_INLINE void activitytx_rxnoack() {

	incroute(0x89);

	leds_debug_toggle();
   // change state
   changeState(S_RIT_RXNOACK);

   // turn off the radio
   radio_rfOff();

#if (TESTE_CSMA == 1)
    ritstat.txola.countacktxrxerr++;
#endif
   // compute the duty cycle if radio has been turned on
   if (ieee154e_vars.radioOnThisSlot==TRUE){
	  ieee154e_vars.radioOnTics+=(radio_getTimerValue()-ieee154e_vars.radioOnInit);
   }

#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_RADIO_POWER_CONS == 1))
	{
		uint8_t *pucAux = (uint8_t *) &ieee154e_vars.radioOnTics;
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_RADIO;
		rffbuf[pos++]= 0x89;
		//rffbuf[pos++]= (uint8_t) ieee154e_vars.dataToSend->length;
		//rffbuf[pos++]= (uint8_t) ieee154e_vars.dataReceived->length;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;
		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
#endif

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
   sRITqueue elequeue;

   incroute(0x09);

   //leds_debug_toggle();
   // change state
   changeState(S_RIT_TXPROC);

   // cancel tt8
#if (TESTE_CSMA == 1)
   radiotimer_cancel();
    ritstat.txola.countacktxrxok++;
#else
    radiotimer_cancel();
#endif

   // turn off the radio
   radio_rfOff();
   ieee154e_vars.radioOnTics+=(radio_getTimerValue()-ieee154e_vars.radioOnInit);

   // record the captured time
   ieee154e_vars.lastCapturedTime = capturedTime;

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

         break;
      }

      // toss CRC (2 last bytes)
      packetfunctions_tossFooter(   ieee154e_vars.ackReceived, LENGTH_CRC);

      // break if invalid CRC
      if (ieee154e_vars.ackReceived->l1_crc==FALSE) {
         // break from the do-while loop and execute the clean-up code below
         break;
      }

      // parse the IEEE802.15.4 header (RX ACK)
      ieee802154_retrieveHeader(ieee154e_vars.ackReceived,&ieee802514_header);

#if 0
      // break if invalid IEEE802.15.4 header
      if (ieee802514_header.valid==FALSE) {
         // break from the do-while loop and execute the clean-up code below
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
         break;
      }
      //hanlde IEs --xv poipoi
      if (ieee802514_header.ieListPresent==FALSE){
         break; //ack should contain IEs.
      }

      if (ieee154e_processIEs(ieee154e_vars.ackReceived,&lenIE)==FALSE){
        // invalid IEs in ACK
        break;
      }

      // toss the IEs
      packetfunctions_tossHeader(ieee154e_vars.ackReceived,lenIE);
#endif

#if 0 //((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_TX == 1))
  {
		uint8_t   pos=0;
		uint8_t *pucAux;

		if (RITQueue_ElementPending < maxElements)
		{
			elequeue = RITQueue_Get_Element(RITQueue_ElementPending);

			rffbuf[pos++]= RFF_IEEE802_TX;
			rffbuf[pos++]= 0x07;
			rffbuf[pos++]= RITQueue_ElementPending;
			rffbuf[pos++]= coappending;
			rffbuf[pos++]= elequeue.frameType;
			rffbuf[pos++]= elequeue.msglength;
			pucAux = (uint8_t *) &ieee154e_vars.dataToSend;
			rffbuf[pos++]= *pucAux++;
			rffbuf[pos++]= *pucAux++;
			rffbuf[pos++]= *pucAux++;
			rffbuf[pos++]= *pucAux++;
			rffbuf[pos++]= elequeue.isBroadcastMulticast;
			rffbuf[pos++]= elequeue.destaddr.type;
/*
			if (element.destaddr.type == 0x01)	{
				rffbuf[pos++]= elequeue.destaddr.addr_16b[0];
				rffbuf[pos++]= elequeue.destaddr.addr_16b[1];
			}
			if (element.destaddr.type == 0x02)	{
				rffbuf[pos++]= elequeue.destaddr.addr_64b[6];
				rffbuf[pos++]= elequeue.destaddr.addr_64b[7];
			}
			else if (element.destaddr.type == 0x03) {
				rffbuf[pos++]= elequeue.destaddr.addr_128b[14];
				rffbuf[pos++]= elequeue.destaddr.addr_128b[15];
			}
*/
			openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
		}
  }
#endif

#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_RADIO_POWER_CONS == 1))
	{
		uint8_t *pucAux = (uint8_t *) &ieee154e_vars.radioOnTics;
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_RADIO;
		rffbuf[pos++]= 0x09;
		//rffbuf[pos++]= (uint8_t) ieee154e_vars.dataToSend->length;
		//rffbuf[pos++]= (uint8_t) ieee154e_vars.dataReceived->length;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;
		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
#endif

       //RFF - se o frame eh COAP eu removo o flag de coap pending para liberar novos RPLs
	   elequeue = RITQueue_Get_Element(RITQueue_ElementPending);

	   //update statistics
	   switch (elequeue.frameType)
	   {
	   	   case IANA_UDP:
			   ritstat.txcoap.countacktxrxok++;
			   incroute(0x0A);
			   coappending = 0;
			   break;
	   	   case IANA_ICMPv6_RPL_DAO:
			   ritstat.txdao.countacktxrxok++;
			   coappending = 0;
			   break;
   	   }

       ret = RITQueue_Free(RITQueue_ElementPending);

#if (TESTE_CSMA == 0)
      // inform schedule of successful transmission
      schedule_indicateTx(&ieee154e_vars.asn,TRUE);

      // inform upper layer
      notif_sendDone(ieee154e_vars.dataToSend,E_SUCCESS);
#endif
      ieee154e_vars.dataToSend = NULL;

      // in any case, execute the clean-up code below (processing of ACK done)
   } while (0);

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
*
* */
port_INLINE void activitytx_rxolaprepare(void) {
	uint32_t dur_rt2;

    changeState(S_RIT_RXOLAPREPARE);

	incroute(0x02);

	radiotimer_cancel();

	// turn off the radio
	radio_rfOff();

   // calculate the frequency to transmit on
   ieee154e_vars.freq = calculateFrequency(schedule_getChannelOffset());

   // configure the radio for that frequency
   radio_setFrequency(ieee154e_vars.freq);

   // enable the radio in Rx mode. The radio does not actively listen yet.
    radio_rxEnable();
    ieee154e_vars.radioOnInit=radio_getTimerValue();
    ieee154e_vars.radioOnThisSlot=TRUE;

#if 0 //(DEBUG_LOG_RIT  == 1) && (DBG_IEEE802_TX == 1)
  {
	uint32_t capturetime;
	uint8_t *pucAux = (uint8_t *) &capturetime;
	uint8_t pos=0;

	//leds_sync_on();
    capturetime=radio_getTimerValue();

	rffbuf[pos++]= RFF_IEEE802_TX;
	rffbuf[pos++]= 0x12;
	rffbuf[pos++]= macRITstate;
	rffbuf[pos++]= *pucAux++;
	rffbuf[pos++]= *pucAux++;
	rffbuf[pos++]= *pucAux++;
	rffbuf[pos++]= *pucAux;
	//rffbuf[pos++]= ieee154e_vars.dataToSend->creator;
	//rffbuf[pos++]= ieee154e_vars.dataToSend->owner;
	//rffbuf[pos++]= ieee154e_vars.dataToSend->l2_frameType;

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
  }
#endif

#if (TESTE_CSMA == 0)
	dur_rt2 = ieee154e_vars.lastCapturedTime+55;
	radiotimer_schedule(dur_rt2);
    //radiotimer_schedule(DURATION_rt3);
#else
    //activitytx_rxwindowopen();
    changeState(S_RIT_RXOLA);
    radio_rxNow();
    radiotimer_schedule(TX_RIT_TIMEOUT_MS);
#endif

#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TIMER == 1))
	{
		uint8_t *pucAux = (uint8_t *) &dur_rt2;
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_TX;
		rffbuf[pos++]= 0x02;
		rffbuf[pos++]= (uint8_t) ieee154e_vars.dataToSend->length;
		rffbuf[pos++]= (uint8_t) ieee154e_vars.dataReceived->length;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;
		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
#endif

}

//======= RX
//Abre a janela para recepcao...
port_INLINE void activityrx_dataprepare(PORT_RADIOTIMER_WIDTH capturedTime) {
	uint32_t dur_rt3;

    changeState(S_RIT_RXDATAPREPARE);
	incroute(0x03);

    ritstat.txola.countdatatxok++;

	radiotimer_cancel();
	// turn off the radio
	radio_rfOff();

   ieee154e_vars.lastCapturedTime = capturedTime;

   // calculate the frequency to transmit on
   ieee154e_vars.freq = calculateFrequency(schedule_getChannelOffset());

   // configure the radio for that frequency
   radio_setFrequency(ieee154e_vars.freq);

   // enable the radio in Rx mode. The radio does not actively listen yet.
    radio_rxEnable();
    //ieee154e_vars.radioOnInit=radio_getTimerValue();
    //ieee154e_vars.radioOnThisSlot=TRUE;

   //radiotimer_schedule(DURATION_rt3);
	//dur_rt3 = ieee154e_vars.lastCapturedTime+TsTxOffset+TsLongGT;
	//radiotimer_schedule(dur_rt3);
    activity_rxwindowopen();

	#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TIMER == 1))
		{
			uint8_t *pucAux = (uint8_t *) &dur_rt3;
			uint8_t pos=0;

			rffbuf[pos++]= RFF_IEEE802_RX;
			rffbuf[pos++]= 0x03;
			rffbuf[pos++]= *pucAux++;
			rffbuf[pos++]= *pucAux++;
			rffbuf[pos++]= *pucAux++;
			rffbuf[pos++]= *pucAux;

			openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
		}
	#endif
}


port_INLINE void activitytx_rxwindowopen() {

	sRITqueue elequeue;
	uint32_t duration;

    changeState(S_RIT_RXOLA);

    //RFF DEBUG
	if (macRITstate == S_RIT_TX_state)
		incroute(0x03);
	else
		incroute(0x55);

	//RFF DEBUG END

	radiotimer_cancel();

    radio_rxNow();


  //Schedule Timer TX Windows - Esta janela deve ser no minimo de 1 periodo de RIT.
  //radiotimer_schedule(macRITRXforTxPeriod);
  duration = ieee154e_vars.lastCapturedTime + TICK_MAC_RIT_RX_TO_TX_PERIOD;
  if (duration > 16500)
	  duration = 16500;
  //update the Tx duration
  RITQueue_updateactualtxtime(duration);

  radiotimer_schedule(duration);


#if 0 //((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_RX == 1))
  {
    uint32_t capturetime;
	uint8_t *pucAux = (uint8_t *) &duration;
	uint8_t pos=0;
	if (macRITstate == S_RIT_TX_state)
	{
	    capturetime=radio_getTimerValue();

		rffbuf[pos++]= RFF_IEEE802_TX;
		rffbuf[pos++]= 0x03;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}

  }
#endif

}

//Abre a janela para recepcao...
port_INLINE void activity_rxwindowopen() {

	uint32_t duration=0;

    changeState(S_RIT_RXDATA);

	incroute(0x04);

	radiotimer_cancel();

    radio_rxNow();

   //Programa os timers do RIT...
   //radiotimer_schedule(macRITDataWaitDuration);
  //duration = ieee154e_vars.lastCapturedTime + TICK_MAC_RIT_RX_WIND_PERIOD;
  //radiotimer_schedule(duration);

#if (TESTE_CSMA == 1) && (SINK ==1)
  duration = 500;
  radiotimer_schedule(duration);
#else
  radiotimer_schedule(RX_RIT_TIMEOUT_MS);
#endif

#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TIMER == 1))
	{
		uint8_t *pucAux = (uint8_t *) &duration;
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_RX;
		rffbuf[pos++]= 0x04;
		rffbuf[pos++]= (uint8_t) ieee154e_vars.dataToSend->length;
		rffbuf[pos++]= (uint8_t) ieee154e_vars.dataReceived->length;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;
		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
#endif
}

/* ACABOU O RIT AQUI E NAO RECEBI NENHUMA MENSAGEM
 * Quando TX significa que eu estou com uma mensagem pendente e fiquei esperando o ola do vizinho que nao veio...
 * Quando RX significa que abri a janela do RIT e nao teve nenhuma mensagem - caso normal.
 */

port_INLINE void activity_ritwindowend() {

	sRITqueue elequeue;

	// change state
    changeState(S_RIT_SLEEP_WINDOW);

    //RFF DEBUG
	if (macRITstate == S_RIT_TX_state)
		incroute(0x84);
	else
		incroute(0x85);

   // turn off the radio
   radio_rfOff();

#if (TESTE_CSMA == 1)
   radiotimer_cancel();
#endif

   // compute the duty cycle if radio has been turned on
   if (ieee154e_vars.radioOnThisSlot==TRUE){
	  ieee154e_vars.radioOnTics+=(radio_getTimerValue()-ieee154e_vars.radioOnInit);
   }

#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_RADIO_POWER_CONS == 1))
	{
		uint8_t *pucAux = (uint8_t *) &ieee154e_vars.radioOnTics;
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_RADIO;
		rffbuf[pos++]= 0x86;
		//rffbuf[pos++]= (uint8_t) ieee154e_vars.dataToSend->length;
		//rffbuf[pos++]= (uint8_t) ieee154e_vars.dataReceived->length;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;
		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
#endif

    if (macRITstate == S_RIT_TX_state)
    {
		//verifico se o frame atual � um RPL.DAO ou COAP
		elequeue = RITQueue_Get_Element(macRITActualPos);

		ieee154e_vars.lastCapturedTime = radio_getTimerValue();
#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TIMER == 1))
	{
		uint8_t *pucAux = (uint8_t *) &ieee154e_vars.lastCapturedTime;
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_RX;
		rffbuf[pos++]= 0x84;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;
		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
#endif
#if (TESTE_CSMA == 0)
 	    // SINALIZO ERRO E GUARDO A MENSAGEM PARA TENTAR NO PROXIMO CICLO...activity_tie5(); (copie aqui embaixo)
	    if (elequeue.frameType != IANA_ICMPv6_RPL_DIO)
	    {
	 		incroute(0x43);
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
	   }
	   else
	   {
		   //leds_debug_off();
			incroute(0x44);
		   // indicate succesful Tx to schedule to keep statistics
	      schedule_indicateTx(&ieee154e_vars.asn,TRUE);
	      // indicate to upper later the packet was sent successfully
	      notif_sendDone(ieee154e_vars.dataToSend,E_SUCCESS);

	   }
#endif
	   // reset local variable
	   ieee154e_vars.dataToSend = NULL;
   }
   else // if (macRITstate == RIT_RX_window_state)
   {
	   //ESTADO NORMAL...SOMENTE ENTRO EM SLEEP E ESPERO PROXIMO RIT.
#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TIMER == 1))
	{
		uint8_t *pucAux = (uint8_t *) &ieee154e_vars.lastCapturedTime;
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_RX;
		rffbuf[pos++]= 0x85;
		rffbuf[pos++]= (uint8_t) ieee154e_vars.dataToSend->length;
		rffbuf[pos++]= (uint8_t) ieee154e_vars.dataReceived->length;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;
		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
#endif

   }

	// abort
   endSlot();


}

port_INLINE void activity_rie3() {

   // log the error
   openserial_printError(COMPONENT_IEEE802154E,ERR_WDDATADURATION_OVERFLOWS,
                         (errorparameter_t)ieee154e_vars.state,
                         (errorparameter_t)ieee154e_vars.slotOffset);

   // abort
   endSlot();
}


port_INLINE uint8_t activitytx_reopenrxwindow(PORT_RADIOTIMER_WIDTH capturedTime) {

  uint8_t ret=0;
  uint32_t duration;
  sRITqueue elequeue;

  changeState(S_RIT_RXOLA);
  incroute(0x43);
  leds_sync_toggle();

  radiotimer_cancel();

  ritstat.txdao.countdatatx++;
  flagreschedule = TRUE;

  // turn off the radio
  radio_rfOff();
  //ieee154e_vars.radioOnTics+=(radio_getTimerValue()-ieee154e_vars.radioOnInit);
  //ieee154e_vars.radioOnThisSlot = 0;

 // calculate the frequency to Rx on
 ieee154e_vars.freq = calculateFrequency(schedule_getChannelOffset());

 // configure the radio for that frequency
 radio_setFrequency(ieee154e_vars.freq);

 // enable the radio in Rx mode. The radio does not actively listen yet.
  radio_rxEnable();

  radio_rxNow();

  //pego o elemento atual pendente
  elequeue = RITQueue_Get_Element(macRITActualPos);

  if (pvObjList[macRITActualPos].countretry < 3)
  {
	  ritstat.txdao.countdatatxerr++;

	  pvObjList[macRITActualPos].countretry++;
#if (ENABLE_CSMA_CA == 0)
	  duration = ieee154e_vars.lastCapturedTime+7000;
	  radiotimer_schedule(duration);
#else
     radiotimer_reschedule(TX_RIT_TIMEOUT_MS);
#endif

	  ret=TRUE;
  }


  #if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TIMER == 1))
	{
		uint8_t *pucAux  = (uint8_t *) &elequeue.lasttxduration;
		uint8_t *pucAux1 = (uint8_t *) &capturedTime;
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_TIMER;
		rffbuf[pos++]= 0x43;
		rffbuf[pos++]= macRITstate;
		rffbuf[pos++]= ret;
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

	return ret;
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

   changeState(S_RIT_TXACKOFFSET);

   // turn off the radio
   radio_rfOff();

   //RFF DEBUG
   //leds_sync_off();
	if (macRITstate == S_RIT_TX_state)
		incroute(0x04);
	else
 	    incroute(0x05);
	//RFF DEBUG END

	ritstat.rxola.countdata++;

   // cancel rt4
   radiotimer_cancel();

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
  	    incroute(0x51);
  	    // jump to the error code below this do-while loop
         break;
      }

#if 0 //((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
  {
	uint8_t   pos=0;
	uint8_t *pucAux = (uint8_t *) &capturedTime;

	pauxframe = (uint8_t *) &ieee154e_vars.dataReceived->packet[0];

	rffbuf[pos++]= RFF_IEEE802_RX;
	rffbuf[pos++]= 0x04;
	rffbuf[pos++]= ieee802514_header.valid;
	rffbuf[pos++]= ieee154e_vars.dataReceived->length;
	rffbuf[pos++]= pauxframe[1];   //802154.FCF [0]
	rffbuf[pos++]= pauxframe[2];   //802154.FCF [1]
	rffbuf[pos++]= pauxframe[6];   //dest addr [6]
	rffbuf[pos++]= pauxframe[7];   //dest addr [7]
	rffbuf[pos++]= pauxframe[14];  //src addr [6]
	rffbuf[pos++]= pauxframe[15];  //src addr [7]
	rffbuf[pos++]= pauxframe[22];  //IPHC header [0]
	rffbuf[pos++]= pauxframe[23];  //IPHC header [1]
	rffbuf[pos++]= pauxframe[24];  //Next Header

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
  }
#endif

      // parse the IEEE802.15.4 header (RX DATA)
      // AQUI TAMBEM QUE ELE CHECA A TOPOLOGIA...SE O VIZINHO EH VALIDO...
      // break if invalid IEEE802.15.4 header
      ieee802154_retrieveHeader(ieee154e_vars.dataReceived,&ieee802514_header);
#if (TESTE_CSMA == 0)
      if (ieee802514_header.valid==FALSE) {
  	    incroute(0x52);
    	  // break from the do-while loop and execute the clean-up code below
         break;
      }
#endif
      // store header details in packet buffer
      ieee154e_vars.dataReceived->l2_frameType      = ieee802514_header.frameType;
      ieee154e_vars.dataReceived->l2_dsn            = ieee802514_header.dsn;
      ieee154e_vars.dataReceived->l2_IEListPresent  = ieee802514_header.ieListPresent;
      memcpy(&(ieee154e_vars.dataReceived->l2_nextORpreviousHop),&(ieee802514_header.src),sizeof(open_addr_t));

      // toss the IEEE802.15.4 header
      packetfunctions_tossHeader(ieee154e_vars.dataReceived,ieee802514_header.headerLength);

      if (ieee802514_header.frameType != IEEE154_TYPE_OLA){

          // toss the IEs including Synch
          packetfunctions_tossHeader(ieee154e_vars.dataReceived,lenIE);

          // record the captured time
          //ieee154e_vars.lastCapturedTime = capturedTime;
          // ieee154e_vars.lastCapturedTime = 0;

          // if I just received an invalid frame, stop
          // s� eh valido beacon e data packet - ack nao sera valido!!!!!!
    #if (TESTE_CSMA == 0)
          if (isValidRxFrame(&ieee802514_header)==FALSE) {
      	    incroute(0x53);
        	  // jump to the error code below this do-while loop
             break;
          }
    #endif
           memcpy(&rxaddr_nexthop, &(ieee154e_vars.dataReceived->l2_nextORpreviousHop),sizeof(open_addr_t));
      }

      memcpy(&rxaddr_dst, &(ieee802514_header.dest),sizeof(open_addr_t));
      memcpy(&actualsrcaddr, &(ieee802514_header.src),sizeof(open_addr_t));

       // --------------
       // -------- TX

	   if (macRITstate == S_RIT_TX_state) {
			//sendPending = FALSE;
		    ritstat.rxola.countdata++;
    	   if (ieee802514_header.frameType == IEEE154_TYPE_OLA) {

				#if (DEBUG_CSMA == 1)
					//leds_sync_toggle();
					radio_csma_vars.capturedTime0 = (uint32_t) radiotimer_getCapturedTime();
				#endif

				//verifica se existe msg pendente para este endereco de destino ou broadcast
				elementpos = RITQueue_Get_Pos(&actualsrcaddr);

				if (elementpos < maxElements)
				{
					RITQueue_ElementPending = elementpos;
					//descarto o frame recebido pois nao preciso mais dele
					openqueue_freePacketBuffer(ieee154e_vars.dataReceived);
					ieee154e_vars.dataReceived = NULL;

				    ritstat.rxola.countdatatx++;
					//envio mensagem pendente
					//activity_ti2();
				    //incroute(0x61);
				    //leds_debug_on();
					prepare_tx_msg(capturedTime);

				}
				else
				{
				   if (flagreschedule)
					 flagreschedule = 0;

				    ritstat.rxola.countdatatxerr++;

			         incroute(0x42);

				    if (activitytx_reopenrxwindow(capturedTime) == 0) {
					   break;
				    }

				   // clean up dataReceived
				   if (ieee154e_vars.dataReceived!=NULL) {
					  openqueue_freePacketBuffer(ieee154e_vars.dataReceived);
					  ieee154e_vars.dataReceived = NULL;
					}
				}

			}
			else
			{  //aqui estava esperando um ola mas veio outro frame...
				//devo recalcular o delta de tempo restante para RITTXWindon
				//e voltar a esperar um novo frame.
				   if (flagreschedule)
					 flagreschedule = 0;

				#if 0 //((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
			      {
					uint8_t   pos=0;
					uint8_t *pucAux = (uint8_t *) &capturedTime;

					rffbuf[pos++]= RFF_IEEE802_TX;
					rffbuf[pos++]= 0x43;
					rffbuf[pos++]= ieee154e_vars.dataReceived->length;
					rffbuf[pos++]= ieee802514_header.frameType;

					openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
				  }
				#endif

				    ritstat.rxola.countdatatxerr++;
			   if (activitytx_reopenrxwindow(capturedTime) == 0) {
				   break;
			   }

			   // clean up dataReceived
			   if (ieee154e_vars.dataReceived!=NULL) {
	  			  openqueue_freePacketBuffer(ieee154e_vars.dataReceived);
				  ieee154e_vars.dataReceived = NULL;
	            }
			}
	   }

	   //-------------------
	   //--------RX

	   else if (macRITstate == S_RIT_RX_window_state) {

           //incroute(ieee802514_header.frameType);

#if (TESTE_CSMA == 0)
           if (ieee802514_header.frameType != IEEE154_TYPE_OLA) {

   			 //checa se frame � para o mote (mesmo endereco destino ou eh um frame broadcast)
        	isFrameForMe = 0;
			if (packetfunctions_isBroadcastMulticast(&rxaddr_dst))
				isFrameForMe = TRUE;
			else if  (idmanager_isMyAddress(&rxaddr_dst))
				isFrameForMe = TRUE;

			if (isFrameForMe) {
				// check if ack requested
				if (ieee802514_header.ackRequested==1)
				{
					incroute(0x65);
					activityrx_preparetxack(capturedTime);
				}
				else
				{
				   incroute(0x66);

				   radio_rfOff();
				   ieee154e_vars.radioOnTics+=radio_getTimerValue()-ieee154e_vars.radioOnInit;

				   // indicate reception to upper layer (no ACK asked)
				   if (ieee154e_vars.dataReceived!=NULL) {
					   notif_receive(ieee154e_vars.dataReceived);
				       ieee154e_vars.dataReceived = NULL;
	               }
				   endSlot();
				}
			}
			else
			{
				//TODO!!!!! aqui eu abri minha janela mas nao era para mim o frame...devo abrir novamente ????
				//estou ignorando e espero no proximo ciclo.
				incroute(0x67);
				discardframe = TRUE;
			}

		}
		else {
		   // indicate reception to upper layer (no ACK asked)
		   //notif_receive(ieee154e_vars.dataReceived);
		   //TODO!!! AQUI TENHO DUVIDAS SE DEVO OU NAO DESPREZAR O FRAME DO RIT...
		   //Do jeito que esta a tabela de vizinhos nao esta sendo incrementada pelo nr de anuncios do RIT
		   //Porem se eu nunca comunicar com ele ele nao vai incrementar...mesmo se ele for um bom vizinho...
		  incroute(0x68);
		  discardframe = TRUE;
		}
#else //TESTE_CSMA
		if (ieee802514_header.frameType != IEEE154_TYPE_OLA) {
			//leds_sync_toggle();
			if ((rxaddr_dst.type ==2) && (rxaddr_dst.addr_16b[7] == MOTE2)) {
				ritstat.rxola.countack++;
				activityrx_preparetxack(capturedTime);
				discardframe = 0;
			}
			else{
				//leds_sync_toggle();
				discardframe = TRUE;
			}
		}
		else {
			discardframe = TRUE;
		}
#endif
	   }

	   if (discardframe) {
			incroute(0x69);

		   // turn off the radio calcula o tempo do radio ligado e volto a dormir
		   radio_rfOff();
		   ieee154e_vars.radioOnTics+=radio_getTimerValue()-ieee154e_vars.radioOnInit;

		   // clean up dataReceived
		   if (ieee154e_vars.dataReceived!=NULL) {
  			  openqueue_freePacketBuffer(ieee154e_vars.dataReceived);
			  ieee154e_vars.dataReceived = NULL;
            }


			//descarto o frame recebido pois nao preciso mais dele
			endSlot();
	  }

	  //openserial_startOutput();

	  // everything went well, return here not to execute the error code below
	  return;

   } while(0);

   ritstat.rxola.countdatatxerr++;

   // free the (invalid) received data so RAM memory can be recycled
   if (ieee154e_vars.dataReceived!=NULL) {
	   openqueue_freePacketBuffer(ieee154e_vars.dataReceived);

	   // clear local variable
	   ieee154e_vars.dataReceived = NULL;
   }

    //se existe um frame TX pendente..apesar da mensagem ser invalida eu reabro a janela esperando outra msg valida
    if (macRITstate == S_RIT_TX_state) {
        activitytx_reopenrxwindow(capturedTime);
    }

#if 0 // ((ENABLE_DEBUG_RFF ==1)  && (DBG_RADIO_POWER_CONS == 1))
	{
		uint8_t *pucAux = (uint8_t *) &ieee154e_vars.radioOnTics;
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_RX;
		rffbuf[pos++]= 0x69;
		rffbuf[pos++]= ieee154e_vars.dataReceived->length;
		rffbuf[pos++]= pauxframe[1];
		rffbuf[pos++]= pauxframe[2];
		rffbuf[pos++]= pauxframe[3];
		rffbuf[pos++]= pauxframe[4];
		rffbuf[pos++]= pauxframe[5];
		rffbuf[pos++]= pauxframe[6];
		rffbuf[pos++]= pauxframe[7];
		rffbuf[pos++]= pauxframe[8];
		rffbuf[pos++]= pauxframe[9];
		rffbuf[pos++]= pauxframe[10];
		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
#endif

   // free the (invalid) received data so RAM memory can be recycled
   if (ieee154e_vars.dataReceived!=NULL) {
	   openqueue_freePacketBuffer(ieee154e_vars.dataReceived);

	   // clear local variable
	   ieee154e_vars.dataReceived = NULL;
   }

   // abort
   endSlot();

   openserial_startOutput();
}


//mesma rotina do activity_ti2
//somente alterado o buffer para ler da fila ao inves da variavel global

port_INLINE void prepare_tx_msg(PORT_RADIOTIMER_WIDTH capturedTime) {

	uint8_t ret = false;
	uint8_t *msg;
	uint8_t msglen=0;
	sRITqueue elequeue;
	//uint8_t elementpos=0;
	uint32_t duration=0;

   // change state
   changeState(S_RIT_TXDATAREADY);
   incroute(0x05);

	// change owner
	ieee154e_vars.dataToSend->owner = COMPONENT_IEEE802154E;
	// record that I attempt to transmit this packet
	ieee154e_vars.dataToSend->l2_numTxAttempts++;

	radiotimer_cancel();

   // calculate the frequency to transmit on
   ieee154e_vars.freq = calculateFrequency(schedule_getChannelOffset());

   // configure the radio for that frequency
   radio_setFrequency(ieee154e_vars.freq);

   ieee154e_vars.lastCapturedTime = capturedTime;

   // load the packet in the radio's Tx buffer
   //carrega pacote apartir da fila do RIT
	if (RITQueue_ElementPending < maxElements) {
		elequeue = RITQueue_Get_Element(RITQueue_ElementPending);
		radio_loadPacket(elequeue.msg , elequeue.msglength);
		//free the mensagem in the RIT message pool
		//vou limpara ela somente depois que foi confirmado o envio
		//ret = RITQueue_Free(RITQueue_ElementPending);
	}

   // enable the radio in Tx mode. This does not send the packet.
   radio_txEnable();

#if (TESTE_CSMA == 0)
  //duration = ieee154e_vars.lastCapturedTime + 1638; // delay entre Rx e tx de 52 ms
  //duration = ieee154e_vars.lastCapturedTime + 819; // delay entre Rx e tx de 27 ms
  duration = ieee154e_vars.lastCapturedTime + 400; // delay entre Rx e tx de 12,21 ms
  radiotimer_schedule(duration);
#else
  changeState(S_RIT_TXDATA);
  radio_txNow(1);
  radiotimer_schedule(TX_RIT_TXDATAECHO_MS);
#endif

#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TIMER == 1))
	{
		uint8_t *pucAux = (uint8_t *) &duration;
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_TX;
		rffbuf[pos++]= 0x05;
		rffbuf[pos++]= (uint8_t) ieee154e_vars.dataToSend->length;
		rffbuf[pos++]= (uint8_t) ieee154e_vars.dataReceived->length;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;
		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
#endif

}

port_INLINE void activityrx_preparetxack(PORT_RADIOTIMER_WIDTH capturedTime) {
   //PORT_SIGNED_INT_WIDTH timeCorrection;
   header_IE_ht header_desc;
   uint32_t duration;

   // change state
   changeState(S_RIT_TXACKPREPARE);

   incroute(0x6);

   radiotimer_cancel();

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

    // calculate the frequency to transmit on
   ieee154e_vars.freq = calculateFrequency(schedule_getChannelOffset());

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
#if (TESTE_CSMA == 0)
   duration = ieee154e_vars.lastCapturedTime+328;
   radiotimer_schedule(duration);
#else
   radiotimer_schedule(RX_RIT_DELAY_TX_TO_RX_MS);
#endif

#if 0 //((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
  {
	uint8_t   pos=0;
	uint8_t *pucAux = (uint8_t *) &capturedTime;
	uint8_t *pauxframe;

	pauxframe = (uint8_t *) &ieee154e_vars.ackToSend->packet[0];

	rffbuf[pos++]= RFF_IEEE802_RX;
	rffbuf[pos++]= 0x06;
	rffbuf[pos++]= ieee154e_vars.ackToSend->length;
	rffbuf[pos++]= pauxframe[1];
	rffbuf[pos++]= pauxframe[2];
	rffbuf[pos++]= pauxframe[5];
	rffbuf[pos++]= pauxframe[6];  //dest addr [6]
	rffbuf[pos++]= pauxframe[7];  //dest addr [7]
	rffbuf[pos++]= pauxframe[14];  //src addr [6]
	rffbuf[pos++]= pauxframe[15];  //src addr [7]

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
  }
#endif

#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_TIMER == 1))
   {
       uint8_t pos=0;
       uint8_t *pucAux = (uint8_t *) &duration;

	    rffbuf[pos++]= RFF_IEEE802_RX;
		rffbuf[pos++]= 0x07;
		rffbuf[pos++]= (uint8_t) ieee154e_vars.dataToSend->length;
		rffbuf[pos++]= (uint8_t) ieee154e_vars.dataReceived->length;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;

		pucAux = (uint8_t *) &ieee154e_vars.lastCapturedTime;
		rffbuf[pos++]= 0xcc;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos++);

   }
#endif

}

port_INLINE void activityrx_senddataack(void) {
  uint32_t duration;

  changeState(S_RIT_TXACK);
  incroute(0x07);

  //radiotimer_cancel();

  //ack does not use CSMA-CA - spec
  radio_txNow(TX_NOT_USE_CSMA_CA);

#if (TESTE_CSMA == 0)
  duration = DURATION_rt7;
  radiotimer_schedule(DURATION_rt7);
#else
  ritstat.rxola.countacktxrx++;
  radiotimer_schedule(RX_RIT_ACKECHO_TIMEOUT_MS);
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
   if (ieee154e_vars.radioOnThisSlot==TRUE){
	  ieee154e_vars.radioOnTics+=(radio_getTimerValue()-ieee154e_vars.radioOnInit);
   }

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

#if (TESTE_CSMA == 0)
   // inform upper layer of reception (after ACK sent)
   notif_receive(ieee154e_vars.dataReceived);
#else
   //nao envio para cima que vai dar erro
   if (ieee154e_vars.dataReceived!=NULL) {
	   openqueue_freePacketBuffer(ieee154e_vars.dataReceived);
   }
#endif

   // clear local variable
   ieee154e_vars.dataReceived = NULL;

#if (TESTE_CSMA == 1)
   ritstat.rxola.countacktxrxok++;
#endif
   // official end of Rx slot
   endSlot();
}

//======= frame validity check

/**
\brief Decides whether the packet I just received is valid received frame.

A valid Rx frame satisfies the following constraints:
- its IEEE802.15.4 header is well formatted
- it's a DATA of BEACON frame (i.e. not ACK and not COMMAND)
- it's sent on the same PANid as mine
- it's for me (unicast or broadcast)

\param[in] ieee802514_header IEEE802.15.4 header of the packet I just received

\returns TRUE if packet is valid received frame, FALSE otherwise
*/
port_INLINE bool isValidRxFrame(ieee802154_header_iht* ieee802514_header) {
   return ieee802514_header->valid==TRUE                                                           && \
          (
             ieee802514_header->frameType==IEEE154_TYPE_DATA                   ||
             ieee802514_header->frameType==IEEE154_TYPE_BEACON
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
      //leds_sync_on();
      resetStats();
   } else {
      //leds_sync_off();
      schedule_resetBackoff();
   }
}


//======= notifying upper layer

void notif_sendDone(OpenQueueEntry_t* packetSent, owerror_t error) {
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

#if (TESTE_CSMA == 0)
   // post RES's sendDone task
   scheduler_push_task(task_sixtopNotifSendDone,TASKPRIO_SIXTOP_NOTIF_TXDONE);
   // wake up the scheduler
   SCHEDULER_WAKEUP();
#endif
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
	uint8_t    lastpos;

	// change state
	changeState(S_RIT_SLEEP);

	radiotimer_cancel();

	//leds_debug_off();

	//leds_sync_off();

	// record the captured time
	//ieee154e_vars.lastCapturedTime = capturedTime;

	// turn off the radio
	radio_rfOff();

	lastpos = incroute(0xCC);

	//TODO!!! PARA QUE ELE USA ISSO
	// compute the duty cycle if radio has been turned on
	//if (ieee154e_vars.radioOnThisSlot==TRUE){
	//   ieee154e_vars.radioOnTics+=(radio_getTimerValue()-ieee154e_vars.radioOnInit);
	//}
#if (ENABLE_DEBUG_RFF == 1)
  {
	//uint8_t   *pucAux = (uint8_t *) &actualtime;
	uint8_t i=0;
	uint8_t pos=0;
	uint8_t imprimir=0;

		if (macRITstate == S_RIT_RX_window_state){
			rffbuf[pos++]= 0x95;

			imprimir = checkimprimir();
		}
		else {
			imprimir = 1;
		    rffbuf[pos++]= 0x99;
		}

		if (imprimir) {
			//rffbuf[pos++] = lastpos;
			for(i=0;i<lastpos;i++)
			{
				rffbuf[pos++]=ritroute[i];
			}

			if (macRITstate == S_RIT_TX_state)
			{
				//imprime as estatisticas
				rffbuf[pos++]= (uint8_t) ritstat.txdio.countdata;
				rffbuf[pos++]= (uint8_t) ritstat.txdio.countdatatxok;
				rffbuf[pos++]= (uint8_t) ritstat.txdao.countdata;
				rffbuf[pos++]= (uint8_t) ritstat.txdao.countdatatxok;
				rffbuf[pos++]= (uint8_t) ritstat.txdao.countacktxrxok;
			}

			openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);

	   }
  }
	#endif

#if (ENABLE_DEBUG_RFF == 1) && (DBG_RADIO_POWER_CONS == 1)
  {
	//uint8_t   *pucAux = (uint8_t *) &actualtime;
	uint8_t  *pucAux = (uint8_t *) &ieee154e_vars.radioOnTics;
	uint8_t i=0;
	uint8_t pos=0;

    //if (lastpos > 0)
    	//imprimir = true;

	//if (imprimir)
	{
		if (macRITstate == S_RIT_RX_window_state)
			rffbuf[pos++]= 0x95;
		else
		    rffbuf[pos++]= 0x99;

		//rffbuf[pos++] = lastpos;
		for(i=0;i<14;i++)
			{
				rffbuf[pos++]=ritroute[i];
	}

		/*
		if (RITQueue_IsEmpty() == false)
		{
			for (i=0;i<5;i++)
				rffbuf[pos++]= pvObjList[i].frameType;
		}*/

		//imprime as estatisticas
		rffbuf[pos++]= (uint8_t) ritstat.txdio.countdata;
		rffbuf[pos++]= (uint8_t) ritstat.txdio.countdatatxok;
	        rffbuf[pos++]= (uint8_t) ritstat.txdao.countdata;
	        rffbuf[pos++]= (uint8_t) ritstat.txdao.countdatatxok;
	        rffbuf[pos++]= (uint8_t) ritstat.txdao.countacktxrxok;

		//imprime tempo radio on
		rffbuf[pos++]= 0xdd;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
}
}
#endif

	//Limpo a fila de mensagens pendentes
	RITQueue_cleanupoldmsg();

	//macRITstate=S_RIT_sleep_state;

   //clear vars for duty cycle on this slot
	ieee154e_vars.radioOnTics=0;
	ieee154e_vars.radioOnThisSlot=FALSE;

   // clean up dataToSend
   if (ieee154e_vars.dataToSend!=NULL) {
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
      // reset local variable
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

	//radiotimer_schedule(macRITsleepPeriod);
}


bool ieee154e_isSynch(){
   return ieee154e_vars.isSync;
}
/**
\brief Indicates the FSM timer has fired.

This function executes in ISR mode, when the FSM timer fires.
*/
void isr_ieee154e_timer() {
   uint32_t capturedTime = radio_getTimerValue();

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
	  case S_RIT_RXDATA:                     //Rx85
		  activity_ritwindowend();
		  break;
      case S_RIT_TXACKPREPARE:               //Rx07
    	  activityrx_senddataack();
         break;
      case S_RIT_TXACK:                      //Rx88
    	 activityrx_noacktx();
         break;
  	   case S_RIT_TXDATAOFFSET:              //Tx02
  		 activitytx_rxolaprepare();
  		 break;
  	   case S_RIT_RXOLAPREPARE:              //Tx03
		 activitytx_rxwindowopen();
		 break;
 	  case S_RIT_RXOLA:                      //tx84
 		  activity_ritwindowend();
 		  break;
	   case S_RIT_TXDATAREADY:                   //tx06
		 activitytx_senddata();
		 break;
	   case S_RIT_TXDATA:
		 activitytx_datanoecho();                //Tx87
		 break;
	   case S_RIT_RXACK:                         //Tx89
		 activitytx_rxnoack();
		 break;
	   //case S_RIT_SLEEP:                         //Tx89
	  //	 endSlot();
	//	 break;
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

   switch (ieee154e_vars.state) {

     case S_RIT_TXOLA:                         //Rx03
    	 activityrx_dataprepare(capturedTime);
		 break;
	 case S_RIT_RXDATA:                            //Rx05
		 activity_rxnewframe(capturedTime);
		break;
	 case S_RIT_TXACK:                          //Rx08
		 activityrx_txackok(capturedTime);
		break;
	 case S_RIT_RXOLA:                         //Tx04
		 activity_rxnewframe(capturedTime);
		break;
	 case S_RIT_TXDATA:                            //Tx07
		 activitytx_senddone(capturedTime);
		break;
	 case S_RIT_RXACK:                             //Tx09
		 activitytx_rxackok(capturedTime);
		break;
	 default:
		// log the error
		openserial_printError(COMPONENT_IEEE802154E,ERR_WRONG_STATE_IN_ENDOFFRAME,
							  (errorparameter_t)ieee154e_vars.state,
							  (errorparameter_t)ieee154e_vars.slotOffset);
		// abort
		endSlot();
		break;
   }

   ieee154e_dbg.num_endOfFrame++;
}


#endif // (IEEE802154E_RIT == 1)

