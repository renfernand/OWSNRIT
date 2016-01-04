#include "opendefs.h"
#include "IEEE802154E.h"
#if (IEEE802154E_RIT == 1)
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
#include "IEEE802154RIT.h"


#if (IEEE802154E_RIT == 1)

#define MAX_RIT_LIST_ELEM 5
#define MAX_RIT_MSG_LEN   130

uint8_t     coappending;

typedef struct RIT_Queue {
	uint8_t     frameType;
	uint32_t    msglength;
	bool        isBroadcastMulticast;
	uint8_t     pending;
	uint8_t     countretry;
	uint64_t    timestamp;
	open_addr_t destaddr;
	uint8_t     msg[MAX_RIT_MSG_LEN];
} sRITqueue;

typedef struct RIT_Queue_element {
	uint64_t timestamp;
	open_addr_t destaddr;
	uint8_t frameType;
	uint8_t *msg;
	uint32_t msglength;
	bool isBroadcastMulticast;
} sRITelement;

typedef struct RIT_Statistics {
	uint8_t countprod;
	uint8_t countsendok;
	uint8_t countsenderror;
} sRITstat;


sRITelement element;
uint8_t RITQueue_ElementPending;   //salvo o elemento que acabou de enviar uma msg - para retornar apos o envio
sRITstat dio_stat;
sRITstat dao_stat;

//#endif


//=========================== variables =======================================
uint8_t flagSerialTx;

#define TRATA_ACK 1

extern uint8_t ucFlagForwarding;
uint8_t ucFlagTxReOpen;

extern scheduler_vars_t scheduler_vars;
extern scheduler_dbg_t  scheduler_dbg;

ieee154e_vars_t    ieee154e_vars;
ieee154e_stats_t   ieee154e_stats;
ieee154e_dbg_t     ieee154e_dbg;
open_addr_t        address_1;


sRITqueue pvObjList[MAX_RIT_LIST_ELEM];

uint32_t numOfQueueInsertionErrors;
uint8_t numAvailableElements;  // numero de lugares ocupados no pool de mensagem
uint8_t maxElements;
uint8_t u8NrMsgQueue=0;          // numero de posicoes utilizadas na queue de RIT Pending Msg to TX



//support variable for RIT procedure
uint8_t macRITstate;
uint8_t macRITActualPos;
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

extern uint8_t numAvailableElements;
extern uint8_t u8NrMsgQueue;          // numero de posicoes utilizadas na queue de RIT Pending Msg to TX


//teste rff
uint8_t rffslotOffset;
uint8_t rffframetype;
uint8_t rffframelen;
uint8_t rffstate;

#if ENABLE_DEBUG_RFF
static uint8_t rffbuf[30];
static uint8_t ritroute[20];
static uint8_t lastritroutepos = 0;

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



extern OpenQueueEntry_t advRIT;

#define DEBUG_ACK  0
uint8_t MsgNeedAck;

#define TESTE_TIMER 0
//=========================== prototypes ======================================
bool RITQueue_ExistFramePending(void);
bool  RITQueue_ClearAddress(open_addr_t* addr);
bool  RITQueue_copyaddress(open_addr_t* addr1, open_addr_t* addr2);
uint8_t RITQueue_cleanupoldmsg(void);
bool RITQueue_Free(uint8_t elementpos);
uint8_t RITQueue_Get_Pos(open_addr_t *paddr);
sRITqueue RITQueue_Get_Element(uint8_t elementpos);
uint8_t RITQueue_Put(sRITelement *psEle,uint8_t pending);
port_INLINE void prepare_tx_msg(uint8_t elementpos);

owerror_t openqueue_freePacketRITBuffer(OpenQueueEntry_t* pkt);
port_INLINE void SendTxRITProcedure(uint8_t direction);
sRITqueue RITQueue_Dequeue_byaddr(open_addr_t addr);
void RITQueue_Init(void);

bool RITQueue_Enqueue(sRITqueue *pmsg);
// SYNCHRONIZING
void     activity_synchronize_newSlot(void);
void     activity_synchronize_startOfFrame(PORT_RADIOTIMER_WIDTH capturedTime);
void     activity_synchronize_endOfFrame(PORT_RADIOTIMER_WIDTH capturedTime);
// TX
//void     StartRITProcedure(void);
void     activity_rxwindowend(void);
port_INLINE void StartTxRITProcedure(uint8_t elepending, uint8_t newtxframe);
port_INLINE void StartRxRITProcedure(void);
port_INLINE void activity_rxdataprepare(void);
void     activity_ti1ORri1(void);
void     activity_ti2(void);
//void     activity_tie1(void);
//void     activity_ti3(void);
//void     activity_tie2(void);
//void     activity_ti4(PORT_RADIOTIMER_WIDTH capturedTime);
void     activity_tie3(void);
void     activity_txsenddone(PORT_RADIOTIMER_WIDTH capturedTime);
void     activity_txwaitforack(void);
//void     activity_tie4(void);
void     activity_ti7(void);
void     activity_tie5(void);
//void     activity_ti8(PORT_RADIOTIMER_WIDTH capturedTime);
void     activity_txendackwindow(void);
void     activity_rxendackwindow(void);
void     activity_txandackendok(PORT_RADIOTIMER_WIDTH capturedTime);
// RX
void     activity_rxwindowopen(void);
//void     activity_rie1(void);
void     activity_ritrxlistening(void);
void     activity_ritwindowend(void);
//void     activity_ri4(PORT_RADIOTIMER_WIDTH capturedTime);
void     activity_rie3(void);
void     activity_rxnewframe(PORT_RADIOTIMER_WIDTH capturedTime);
void     activity_rxsendack(void);
//void     activity_rie4(void);
//void     activity_ri7(void);
void     activity_rie5(void);
//void     activity_ri8(PORT_RADIOTIMER_WIDTH capturedTime);
//void     activity_rie6(void);
void     activity_rxandackendok(PORT_RADIOTIMER_WIDTH capturedTime);

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
// interrupts
void     isr_ieee154e_newSlot(void);
void     isr_ieee154e_timer(void);

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

void ieee154e_init() {

   // initialize variables
   memset(&ieee154e_vars,0,sizeof(ieee154e_vars_t));
   memset(&ieee154e_dbg,0,sizeof(ieee154e_dbg_t));

   changeIsSync(TRUE);
   ucFlagForwarding = FALSE;
   ucFlagTxReOpen = FALSE;

   resetStats();
   ieee154e_stats.numDeSync                 = 0;

   flagSerialTx = 0;

   macRITstate =S_RIT_sleep_state;

   // switch radio on
   radio_rfOn();

	//set the period constant
	macAckWaitPeriod       = TICK_RIT_ACK_WAIT_PERIOD;
	macRITRXforTxPeriod    = TICK_MAC_RIT_RX_TO_TX_PERIOD;
	macRITDataWaitDuration = TICK_MAC_RIT_RX_WIND_PERIOD;
	macRITPeriod           = TICK_MAC_RIT_PERIOD;
	macRITTxPeriod         = TICK_MAC_RIT_TX_PERIOD;
	macRITsleepPeriod = (uint16_t) ((uint16_t)macRITPeriod-(uint16_t)macRITDataWaitDuration);

	//initiate RIT Queue
	RITQueue_Init();

   // set callback functions for the radio
   radio_setOverflowCb(isr_ieee154e_newSlot);  //timer indicando o inicio do slot
   radio_setCompareCb(isr_ieee154e_timer);     //timer diversos dentro do slot
   radio_setStartFrameCb(ieee154e_startOfFrame); //indica inicio do pacote
   radio_setEndFrameCb(ieee154e_endOfFrame);     //indica fim do um pacote
   // have the radio start its timer
   radio_startTimer(macRITPeriod);

}

//=========================== public ==========================================


void clearritroute(void)
{
#if ENABLE_DEBUG_RFF
	int i;

	lastritroutepos = 0;
	for (i=0;i<20;i++)
	ritroute[i] = 0;
#endif
}

uint8_t incroute(uint8_t element)
{
#if ENABLE_DEBUG_RFF
    //if (macRITstate == S_RIT_RX_for_TX_state)
    {
		if (lastritroutepos < 20)
		{
			ritroute[lastritroutepos] = element;
			lastritroutepos++;
		}
		else
		{
			lastritroutepos = 0;
			ritroute[lastritroutepos] = element;
		}
    }

    return lastritroutepos;
#else
    return 0;
#endif

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

void teste2(void){
#if 0 //ENABLE_DEBUG_RFF
	 uint32_t capturetime;
	 uint8_t *pucAux = (uint8_t *) &capturetime;
	 uint8_t pos=0;

	 capturetime = radio_getTimerValue();

	 rffbuf[pos++]= 0x00;
	 rffbuf[pos++]= *pucAux++;
	 rffbuf[pos++]= *pucAux++;
	 rffbuf[pos++]= *pucAux++;
	 rffbuf[pos++]= *pucAux;

     openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
#endif
}


void isr_ieee154e_newSlot() {

    activity_ti1ORri1();

   ieee154e_dbg.num_newSlot++;
}


port_INLINE void activity_txsenderror(PORT_RADIOTIMER_WIDTH capturedTime) {

	//open_addr_t address;
	//uint8_t elementpos=0;
	//uint8_t ret=0;
	sRITqueue elequeue;
	//uint8_t endslot=false;

   // cancel tt4
   radiotimer_cancel();
   incroute(0x72);

   // turn off the radio
   radio_rfOff();

    if (RITQueue_ElementPending < maxElements) {
    	elequeue = RITQueue_Get_Element(RITQueue_ElementPending);

    	if (elequeue.frameType == IANA_ICMPv6_RPL_DIO)
    	{
			dio_stat.countsenderror++;
    	}
    	else if (elequeue.frameType == IANA_ICMPv6_RPL_DAO)
    	{
			dao_stat.countsenderror++;
    	}

    	RITQueue_Free(RITQueue_ElementPending);
	}


#if ((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_TX == 1))
  {
	uint8_t   pos=0;

	rffbuf[pos++]= RFF_IEEE802_TX;
	rffbuf[pos++]= 0x54;
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
	   // indicate succesful Tx to schedule to keep statistics
	  schedule_indicateTx(&ieee154e_vars.asn,TRUE);
	  // indicate to upper later the packet was sent successfully
	  notif_sendDone(ieee154e_vars.dataToSend,E_FAIL);
	  // reset local variable
	  ieee154e_vars.dataToSend = NULL;
	  // abort
	  endSlot();
}


/**
\brief Indicates the FSM timer has fired.

This function executes in ISR mode, when the FSM timer fires.
*/
void isr_ieee154e_timer() {
   uint32_t capturedTime = radio_getTimerValue();

   switch (ieee154e_vars.state) {
	  case S_RIT_RXOLAREADY:
		  activity_rxdataprepare();
		 break;
  	   case S_RIT_RX_FOR_TX_BEGIN:
  		 activity_rxdataprepare();
  		 break;
  	   case S_RXDATAREADY:
		 activity_rxwindowopen();
		 break;
  	   case S_TXDATAOFFSET:
  		 activity_ti2();
		 break;
	   case S_TXDATAREADY:
		 SendTxRITProcedure(1);
		 break;
	   //case S_TXDATAPREPARE:
	   //	 SendTxRITProcedure(1);
	   //	 break;
	   case S_TXDATA:
		 activity_txsenderror(capturedTime);
		 break;
	  case S_RXDATA:
		  activity_ritwindowend();
		  break;
	  case S_RXACKOFFSET:
		  activity_txwaitforack();
		 break;
	  case S_RXACK:
		  activity_rxendackwindow();
		 break;
      case S_TXACK:
    	 activity_txendackwindow();
         break;
      case S_TXACKOFFSET: 
    	  activity_rxsendack();
         break;
	  case S_SLEEP:
		  //activity_ti1ORri1();
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

   switch (ieee154e_vars.state) {
	 case S_RIT_RXOLAREADY:
		 activity_rxwindowopen();
		 break;
	 case S_RXDATAREADY:   //aqui eh erro
         u8rffcounterror++;
		 activity_rxnewframe(capturedTime);
		break;
	 case S_RXDATA:
		 activity_rxnewframe(capturedTime);
		break;
	 case S_TXDATA:
		 activity_txsenddone(capturedTime);
		break;
	 case S_RXACK:
		 activity_txandackendok(capturedTime);
		break;
	 case S_TXACK:
		 activity_rxandackendok(capturedTime);
		break;
	 case S_SLEEP:
		endSlot();
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

//======= SYNCHRONIZING
#if 0
port_INLINE void activity_synchronize_newSlot() {
   // I'm in the middle of receiving a packet
   if (ieee154e_vars.state==S_SYNCRX) {
      return;
   }

   // if this is the first time I call this function while not synchronized,
   // switch on the radio in Rx mode
   if (ieee154e_vars.state!=S_SYNCLISTEN) {
      // change state
      changeState(S_SYNCLISTEN);

      // turn off the radio (in case it wasn't yet)
      radio_rfOff();

      // configure the radio to listen to the default synchronizing channel
      radio_setFrequency(SYNCHRONIZING_CHANNEL);

      // update record of current channel
      ieee154e_vars.freq = SYNCHRONIZING_CHANNEL;

      // switch on the radio in Rx mode.
      radio_rxEnable();
      ieee154e_vars.radioOnInit=radio_getTimerValue();
      ieee154e_vars.radioOnThisSlot=TRUE;
      radio_rxNow();
   }

   // increment ASN (used only to schedule serial activity)
   incrementAsnOffset();

}
#endif

#if (IEEE802154E_RIT == 0)

port_INLINE void activity_synchronize_startOfFrame(PORT_RADIOTIMER_WIDTH capturedTime) {

   // don't care about packet if I'm not listening
   if (ieee154e_vars.state!=S_SYNCLISTEN) {
      return;
   }

   // change state
   changeState(S_SYNCRX);

   // stop the serial
   openserial_stop();

   // record the captured time
   ieee154e_vars.lastCapturedTime = capturedTime;

   // record the captured time (for sync)
   ieee154e_vars.syncCapturedTime = capturedTime;
}

port_INLINE void activity_synchronize_endOfFrame(PORT_RADIOTIMER_WIDTH capturedTime) {
   ieee802154_header_iht ieee802514_header;
   uint16_t              lenIE;

   // check state
   if (ieee154e_vars.state!=S_SYNCRX) {
      // log the error
      openserial_printError(COMPONENT_IEEE802154E,ERR_WRONG_STATE_IN_ENDFRAME_SYNC,
                            (errorparameter_t)ieee154e_vars.state,
                            (errorparameter_t)0);
      // abort
      endSlot();
   }

   // change state
   changeState(S_SYNCPROC);

   // get a buffer to put the (received) frame in
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

      // break if packet too short
      if (ieee154e_vars.dataReceived->length<LENGTH_CRC || ieee154e_vars.dataReceived->length>LENGTH_IEEE154_MAX) {
         // break from the do-while loop and execute abort code below
          openserial_printError(COMPONENT_IEEE802154E,ERR_INVALIDPACKETFROMRADIO,
                            (errorparameter_t)0,
                            ieee154e_vars.dataReceived->length);
         break;
      }

      // toss CRC (2 last bytes)
      packetfunctions_tossFooter(   ieee154e_vars.dataReceived, LENGTH_CRC);

      // break if invalid CRC
      if (ieee154e_vars.dataReceived->l1_crc==FALSE) {
         // break from the do-while loop and execute abort code below
         break;
      }


      // parse the IEEE802.15.4 header (synchronize, end of frame)
      ieee802154_retrieveHeader(ieee154e_vars.dataReceived,&ieee802514_header);

      // break if invalid IEEE802.15.4 header
      if (ieee802514_header.valid==FALSE) {
         // break from the do-while loop and execute the clean-up code below
         break;
      }

      // store header details in packet buffer
      ieee154e_vars.dataReceived->l2_frameType = ieee802514_header.frameType;
      ieee154e_vars.dataReceived->l2_dsn       = ieee802514_header.dsn;
      memcpy(&(ieee154e_vars.dataReceived->l2_nextORpreviousHop),&(ieee802514_header.src),sizeof(open_addr_t));

      // toss the IEEE802.15.4 header -- this does not include IEs as they are processed
      // next.
      packetfunctions_tossHeader(ieee154e_vars.dataReceived,ieee802514_header.headerLength);

      // process IEs
      lenIE = 0;
#if 0
      if (
            (
               ieee802514_header.valid==TRUE                                                       &&
               ieee802514_header.ieListPresent==TRUE                                               &&
               ieee802514_header.frameType==IEEE154_TYPE_BEACON                                    &&
               packetfunctions_sameAddress(&ieee802514_header.panid,idmanager_getMyID(ADDR_PANID)) &&
               ieee154e_processIEs(ieee154e_vars.dataReceived,&lenIE)
            )==FALSE) {
         // break from the do-while loop and execute the clean-up code below
         break;
      }
#endif
      // turn off the radio
      radio_rfOff();
      
      // compute radio duty cycle
      ieee154e_vars.radioOnTics += (radio_getTimerValue()-ieee154e_vars.radioOnInit);

      // toss the IEs
      packetfunctions_tossHeader(ieee154e_vars.dataReceived,lenIE);

      // synchronize (for the first time) to the sender's ADV
      //synchronizePacket(ieee154e_vars.syncCapturedTime);

      // declare synchronized
      //changeIsSync(TRUE);

      // log the info
      openserial_printInfo(COMPONENT_IEEE802154E,ERR_SYNCHRONIZED,
                            (errorparameter_t)ieee154e_vars.slotOffset,
                            (errorparameter_t)0);


      // send received ADV up the stack so RES can update statistics (synchronizing)
      notif_receive(ieee154e_vars.dataReceived);

      // clear local variable
      ieee154e_vars.dataReceived = NULL;

      // official end of synchronization
      endSlot();

      // everything went well, return here not to execute the error code below
      return;

   } while(0);

   // free the (invalid) received data buffer so RAM memory can be recycled
   openqueue_freePacketBuffer(ieee154e_vars.dataReceived);

   // clear local variable
   ieee154e_vars.dataReceived = NULL;

   // return to listening state
   changeState(S_SYNCLISTEN);
}
#endif


#if 0
port_INLINE bool ieee154e_processIEs(OpenQueueEntry_t* pkt, uint16_t* lenIE) {
   uint8_t               ptr;
   uint8_t               byte0;
   uint8_t               byte1;
   uint8_t               temp_8b;
   uint8_t               gr_elem_id;
   uint8_t               subid;
   uint16_t              temp_16b;
   uint16_t              len;
   uint16_t              sublen;
   PORT_SIGNED_INT_WIDTH timeCorrection;

   ptr=0;

   //===== header or payload IE header

   //candidate IE header  if type ==0 header IE if type==1 payload IE
   temp_8b    = *((uint8_t*)(pkt->payload)+ptr);
   ptr++;

   temp_16b   = temp_8b + ((*((uint8_t*)(pkt->payload)+ptr))<< 8);
   ptr++;

   *lenIE     = ptr;

   if ((temp_16b & IEEE802154E_DESC_TYPE_PAYLOAD_IE) == IEEE802154E_DESC_TYPE_PAYLOAD_IE){
      // payload IE

      len          = (temp_16b & IEEE802154E_DESC_LEN_PAYLOAD_IE_MASK)>>IEEE802154E_DESC_LEN_PAYLOAD_IE_SHIFT;
      gr_elem_id   = (temp_16b & IEEE802154E_DESC_GROUPID_PAYLOAD_IE_MASK)>>IEEE802154E_DESC_GROUPID_PAYLOAD_IE_SHIFT;
   } else {
      // header IE

      len          = (temp_16b & IEEE802154E_DESC_LEN_HEADER_IE_MASK)>>IEEE802154E_DESC_LEN_HEADER_IE_SHIFT;
      gr_elem_id   = (temp_16b & IEEE802154E_DESC_ELEMENTID_HEADER_IE_MASK)>>IEEE802154E_DESC_ELEMENTID_HEADER_IE_SHIFT;
   }

   *lenIE         += len;

   //===== sub-elements

   switch(gr_elem_id){

      case IEEE802154E_MLME_IE_GROUPID:
         // MLME IE

         do {

            //read sub IE header
            temp_8b     = *((uint8_t*)(pkt->payload)+ptr);
            ptr         = ptr + 1;
            temp_16b    = temp_8b  +(*((uint8_t*)(pkt->payload)+ptr) << 8);
            ptr         = ptr + 1;

            len         = len - 2; //remove header fields len

            if ((temp_16b & IEEE802154E_DESC_TYPE_LONG) == IEEE802154E_DESC_TYPE_LONG){
               // long sub-IE

               sublen   = (temp_16b & IEEE802154E_DESC_LEN_LONG_MLME_IE_MASK)>>IEEE802154E_DESC_LEN_LONG_MLME_IE_SHIFT;
               subid    = (temp_16b & IEEE802154E_DESC_SUBID_LONG_MLME_IE_MASK)>>IEEE802154E_DESC_SUBID_LONG_MLME_IE_SHIFT;
            } else {
               // short sub-IE

               sublen   = (temp_16b & IEEE802154E_DESC_LEN_SHORT_MLME_IE_MASK)>>IEEE802154E_DESC_LEN_SHORT_MLME_IE_SHIFT;
               subid    = (temp_16b & IEEE802154E_DESC_SUBID_SHORT_MLME_IE_MASK)>>IEEE802154E_DESC_SUBID_SHORT_MLME_IE_SHIFT;
            }

            switch(subid){

               case IEEE802154E_MLME_SYNC_IE_SUBID:
                  // Sync IE: ASN and Join Priority

                  if (idmanager_getIsDAGroot()==FALSE) {
                     // ASN
                     asnStoreFromAdv((uint8_t*)(pkt->payload)+ptr);
                     ptr = ptr + 5;
                     // join priority
                     joinPriorityStoreFromAdv(*((uint8_t*)(pkt->payload)+ptr));
                     ptr = ptr + 1;
                  }
                  break;

               case IEEE802154E_MLME_SLOTFRAME_LINK_IE_SUBID:
                  processIE_retrieveSlotframeLinkIE(pkt,&ptr);
                  break;

               case IEEE802154E_MLME_TIMESLOT_IE_SUBID:
                  //TODO
                  break;

               default:
                  return FALSE;
                  break;
            }

            len = len - sublen;
         } while(len>0);

         break;

      case IEEE802154E_ACK_NACK_TIMECORRECTION_ELEMENTID:
         // timecorrection IE

         if (
               idmanager_getIsDAGroot()==FALSE &&
               neighbors_isPreferredParent(&(pkt->l2_nextORpreviousHop))
            ) {

            byte0 = *((uint8_t*)(pkt->payload)+ptr);
            ptr++;
            byte1 = *((uint8_t*)(pkt->payload)+ptr);
            ptr++;

            timeCorrection  = (int16_t)((uint16_t)byte1<<8 | (uint16_t)byte0);
            timeCorrection  = (timeCorrection / (PORT_SIGNED_INT_WIDTH)US_PER_TICK);
            timeCorrection  = -timeCorrection;

            synchronizeAck(timeCorrection);
         }
         break;

      default:
         *lenIE = 0; //no header or not recognized.
         return FALSE;
   }

   if(*lenIE>127) {
      // log the error
      openserial_printError(
         COMPONENT_IEEE802154E,
         ERR_HEADER_TOO_LONG,
         (errorparameter_t)*lenIE,
         (errorparameter_t)1
      );
   }
   return TRUE;
}
#endif

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

   ieee154e_vars.lastCapturedTime = 0; // radio_getTimerValue();
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

  //leds_sync_toggle();

  //teste rff - registra rota do TX
  clearritroute();

  //check if there is message pending from the last cycle
  //txpending = RITQueue_ExistFramePending();
  txpending = 0;

  if ((ieee154e_vars.dataToSend != NULL) || (txpending)) {   // I have a packet to send

	  //TODO!!! AQUI EU NAO ESTOU SALVANDO EM UMA FILA...NAO SERIA NECESSARIO CASO EU TENHA MULTIPLAS MSGS PARADAS...
    //se o estado anterior eh RIT_RX pode ser que estou esperando ainda um evento dele...
    //leds_debug_toggle();
	StartTxRITProcedure(txpending,newtxframe);

    //Programo um slot de Tx que eh geralmente maior que o de rx
    radio_setTimerPeriod(macRITTxPeriod);
  }
  else  {
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

		#else
		  openserial_stop();
		  openserial_startOutput();

		  StartRxRITProcedure();

		  radio_setTimerPeriod(macRITPeriod);
		  //leds_sync_toggle();
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

	   //adv->l2_frameType                     = IEEE154_TYPE_OLA;
	   adv->l2_frameType                     = IEEE154_TYPE_BEACON;
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

	changeState(S_RIT_RXOLAREADY);
	macRITstate=S_RIT_RX_window_state;

	//#################    escolhe o canal
    radiotimer_cancel();

	// calculate the frequency to transmit on
	ieee154e_vars.freq = calculateFrequency(schedule_getChannelOffset());

	// configure the radio for that frequency
	radio_setFrequency(ieee154e_vars.freq);

	//################ load the packet in the radio's Tx buffer
	//pega o frame do rit da camada sixtop
	getRitRequest();

	radio_loadPacket(ieee154e_vars.dataToSend->payload,
					ieee154e_vars.dataToSend->length);

	//################ enable the radio in Tx mode. Transmit the packet
	radio_txEnable();
	ieee154e_vars.radioOnInit=radio_getTimerValue();
	ieee154e_vars.radioOnThisSlot=TRUE;

    radio_txNow();

	//aqui devo aguardar um tempo para ligar o radio como RX...
    radiotimer_schedule(DURATION_rt3);


#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
	{
		uint8_t *pucAux = (uint8_t *) &ieee154e_vars.lastCapturedTime;
		uint8_t pos=0;

		rffbuf[pos++]= RFF_IEEE802_RX;
		rffbuf[pos++]= 0x01;
		rffbuf[pos++]= (uint8_t) ieee154e_vars.dataToSend->creator;
		rffbuf[pos++]= (uint8_t) ieee154e_vars.dataToSend->l2_frameType;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;
		rffbuf[pos++]= (uint8_t) ieee154e_vars.dataToSend->length;
		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
#endif

	leds_sync_on();


}

/*
 * Teste - SendDAO Diretamente...sem esperar
 *  */
port_INLINE void SendTxDAODirectly(void) {

    changeState(S_TXDATA);

	// change owner
	ieee154e_vars.dataToSend->owner = COMPONENT_IEEE802154E;
	// record that I attempt to transmit this packet
	ieee154e_vars.dataToSend->l2_numTxAttempts++;

	radiotimer_cancel();

	// calculate the frequency to transmit on
	ieee154e_vars.freq = calculateFrequency(schedule_getChannelOffset());

	// configure the radio for that frequency
	radio_setFrequency(ieee154e_vars.freq);

	// load the packet in the radio's Tx buffer
	radio_loadPacket(ieee154e_vars.dataToSend->payload,
					ieee154e_vars.dataToSend->length);

	// enable the radio in Tx mode. This does not send the packet.
	radio_txEnable();
	ieee154e_vars.radioOnInit=radio_getTimerValue();
	ieee154e_vars.radioOnThisSlot=TRUE;
	ieee154e_vars.lastCapturedTime = ieee154e_vars.radioOnInit;

#if 0 //ENABLE_DEBUG_RFF
   {
	uint32_t capturetime;
	uint8_t *pucAux = (uint8_t *) &capturetime;
	uint8_t pos=0;

	capturetime=radio_getTimerValue();

	rffbuf[pos++]= RFF_IEEE802_TX;
	rffbuf[pos++]= 0xF0;
	rffbuf[pos++]= ieee154e_vars.dataToSend->creator;
	rffbuf[pos++]= ieee154e_vars.dataToSend->l4_protocol;

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
   }
#endif

   radio_txNow();

   radiotimer_schedule(DURATION_tt3);

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
				dio_stat.countprod++;
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
			dao_stat.countprod++;

			if (ieee154e_vars.dataToSend->creator == COMPONENT_FORWARDING)
			{
				testerff = 0x03;

				 //salvo o endereco do destino que pode estar na posicao final do frame (RPL Transition)
				 address_1.type = ieee154e_vars.dataToSend->l3_destinationAdd.type;
				 if (ieee154e_vars.dataToSend->l3_destinationAdd.type == 3)
				 {
						testerff = 0x04;

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
	}
	else if (ieee154e_vars.dataToSend->l4_protocol == IANA_UNDEFINED) {
		testerff = 0x06;

	    //preparo elemento para ser colocado na fila do RIT
		pmsgout->msglength =  ieee154e_vars.dataToSend->length;
		pmsgout->timestamp = radio_getTimerValue();
		pmsgout->msg = (uint8_t *) ieee154e_vars.dataToSend->payload;

		// DIO DA BRIDGE ANTIGA
		if ((ieee154e_vars.dataToSend->l3_destinationAdd.type == ADDR_NONE) &&
			(*(ieee154e_vars.dataToSend->payload+19) == IANA_ICMPv6_RPL) &&
			(*(ieee154e_vars.dataToSend->payload+20) == IANA_ICMPv6_RPL_DIO))
		{
			testerff = 0x07;

			//AQUI EH O CASO DA BRIDGE ANTIGA!!!!
			macRIT_Pending_TX_frameType = IANA_ICMPv6_RPL_DIO;
			//macRIT_Pending_TX_frameType = 0;  //send directly
			pmsgout->frameType = macRIT_Pending_TX_frameType;
			pmsgout->destaddr  = ieee154e_vars.dataToSend->l2_nextORpreviousHop;
		}
		else if ((*(ieee154e_vars.dataToSend->payload+21) == 0x78) &&
				 (*(ieee154e_vars.dataToSend->payload+23) == IANA_UDP))
		{ //verifica se o IPHC header eh 6LowPAN e o Next eh UDP   (INFERENCIA DO COAP)
			testerff = 0x08;
			macRIT_Pending_TX_frameType = IANA_UDP;
			pmsgout->frameType = macRIT_Pending_TX_frameType;
			pmsgout->destaddr  = ieee154e_vars.dataToSend->l2_nextORpreviousHop;
		}
	}


	if (macRIT_Pending_TX_frameType > 0)
	{
		pmsgout->isBroadcastMulticast = packetfunctions_isBroadcastMulticast(&ieee154e_vars.dataToSend->l2_nextORpreviousHop);
		//coloco elemento na fila do RIT_Tx
		macRITActualPos = RITQueue_Put(pmsgout,flagpending);
	}

#if ((ENABLE_DEBUG_RFF) && (DBG_IEEE802_TX == 1))
{
	uint8_t pos=0;
	uint8_t *pucAux;

	incroute(0x01);

	rffbuf[pos++]= RFF_IEEE802_TX;
    rffbuf[pos++]= 0x01;
	rffbuf[pos++]= macRIT_Pending_TX_frameType;
	rffbuf[pos++]= testerff;
	rffbuf[pos++]= flagpending;
	rffbuf[pos++]= newtxframe;
	rffbuf[pos++]= ieee154e_vars.dataToSend->l4_protocol;
	rffbuf[pos++]= pmsgout->isBroadcastMulticast;
	rffbuf[pos++]= pmsgout->msglength;
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

	rffbuf[pos++]= 0xDD;
	rffbuf[pos++]= dio_stat.countprod;
	rffbuf[pos++]= dio_stat.countsendok;
	rffbuf[pos++]= dio_stat.countsenderror;
	rffbuf[pos++]= 0xEE;
	rffbuf[pos++]= dao_stat.countprod;
	rffbuf[pos++]= dao_stat.countsendok;
	rffbuf[pos++]= dao_stat.countsenderror;
	/*
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
	uint8_t i;
	//sRITelement element;

	changeState(S_RIT_RX_FOR_TX_BEGIN);
    macRITstate=S_RIT_RX_for_TX_state;

    leds_debug_on();
/*
 * TODO!!!! EXISTE OUTRO TIPO DE DADO SEM SER DO TIPO DATA OU BEACON ?
 */
	if (ieee154e_vars.dataToSend->l2_frameType == IEEE154_TYPE_DATA) {
		macRIT_Pending_TX_frameType = checkmsgtype(&element,txpending,newtxframe);
	}


	if ((macRIT_Pending_TX_frameType > 0) || (txpending))
	{
		radiotimer_schedule(DURATION_rt2);
	}
	else
	{
		SendTxDAODirectly();
	}

}

/* Tx Rit procedure - Aqui ele vai ter de esperar receber um frame de RIT para enviar
 *  Calcula Frequencia
 *  prepara o frame do RIT request
 *  carrega o frame
 */
port_INLINE void SendTxRITProcedure(uint8_t direction) {

    changeState(S_TXDATA);

    radio_txNow();

    incroute(0x06);

   radiotimer_schedule(RIT_DURATION_tt4);
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

port_INLINE void activity_txsenddone(PORT_RADIOTIMER_WIDTH capturedTime) {
//   bool listenForAck;
	open_addr_t address;
	uint8_t elementpos=0;
	uint8_t ret=0;
	sRITqueue elequeue;
	uint8_t endslot=false;

   // change state
   changeState(S_RXACKOFFSET);

   // cancel tt4
   radiotimer_cancel();

   incroute(0x07);

   // turn off the radio
   radio_rfOff();

   ieee154e_vars.radioOnTics+=(radio_getTimerValue()-ieee154e_vars.radioOnInit);

   //TESTE DA FILA...VOU REMOVER O ELEMENTO
	//verifica se existe msg pendente para este endereco de destino
    //   RITQueue_copyaddress(&address,&element.destaddr);
    if (RITQueue_ElementPending < maxElements) {
    	elequeue = RITQueue_Get_Element(RITQueue_ElementPending);

    	if (elequeue.frameType == IANA_ICMPv6_RPL_DIO)
    	{
			dio_stat.countsendok++;
    	}
    	else if (elequeue.frameType == IANA_ICMPv6_RPL_DAO)
    	{
			dao_stat.countsendok++;
    	}

	}

	if (elequeue.frameType  != IANA_ICMPv6_RPL_DIO)  {
       if (ieee154e_vars.dataToSend != NULL)
       {
			#if ((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_TX == 1))
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
			   activity_txwaitforack();
			   incroute(0x08);
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
			rffbuf[pos++]= 0x41;
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


	if (endslot)
	{
		  ret = RITQueue_Free(RITQueue_ElementPending);

#if 0 // ((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_TX == 1))
  {
	uint8_t   pos=0;

	rffbuf[pos++]= RFF_IEEE802_TX;
	rffbuf[pos++]= 0x05;
	rffbuf[pos++]= RITQueue_ElementPending;
	rffbuf[pos++]= elequeue.frameType;
	rffbuf[pos++]= elequeue.msglength;

/*
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
*/
	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
  }
#endif
		   // indicate succesful Tx to schedule to keep statistics
		  schedule_indicateTx(&ieee154e_vars.asn,TRUE);
		  // indicate to upper later the packet was sent successfully
		  notif_sendDone(ieee154e_vars.dataToSend,E_SUCCESS);
		  // reset local variable
		  ieee154e_vars.dataToSend = NULL;
		  // abort
		  endSlot();
	}
}




/*
 * Aqui quando eh Tx e eu preciso de um ack...abro o radio como RX esperando o ack.
 */

port_INLINE void activity_txwaitforack() {

   // change state
   changeState(S_RXACK);

	// calculate the frequency to transmit on
   ieee154e_vars.freq = calculateFrequency(schedule_getChannelOffset());

   // configure the radio for that frequency
   radio_setFrequency(ieee154e_vars.freq);

   // enable the radio in Rx mode. The radio is not actively listening yet.
   radio_rxEnable();

   //caputre init of radio for duty cycle calculation
   ieee154e_vars.radioOnInit=radio_getTimerValue();
   ieee154e_vars.radioOnThisSlot=TRUE;
   //ieee154e_vars.lastCapturedTime=ieee154e_vars.radioOnInit;

   radio_rxNow();
   radiotimer_schedule(macAckWaitPeriod);

#if 0// ENABLE_DEBUG_RFF
      {
   	     uint32_t  capturetime=radio_getTimerValue();
         uint8_t   *pucAux = (uint8_t *) &capturetime;
     	 uint8_t   pos=0;

		rffbuf[pos++]= RFF_IEEE802_TX;
		rffbuf[pos++]= 0x06;
		rffbuf[pos++]= ieee154e_vars.dataToSend->creator;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;

   		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
      }
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


port_INLINE void activity_txendackwindow() {

	incroute(0x59);

#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
   {
	   uint32_t rffcapture = ieee154e_vars.lastCapturedTime;
       uint8_t pos=0;
       uint8_t *pucAux = (uint8_t *) &rffcapture;

	    rffbuf[pos++]= RFF_IEEE802_RX;
		rffbuf[pos++]= 0x23;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos++);
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


port_INLINE void activity_rxendackwindow() {

	incroute(0x19);

#if ENABLE_DEBUG_RFF
   {
	   uint32_t rffcapture = ieee154e_vars.lastCapturedTime;
       uint8_t pos=0;
       uint8_t *pucAux = (uint8_t *) &rffcapture;

	    rffbuf[pos++]= RFF_IEEE802_TX;
		rffbuf[pos++]= 0x22;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos++);
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

#if 1

/*
 * Aqui indica que quando TX eu estava esperando um ack e ele chegou.
 * aviso as camadas de cima do sucesso.
 */
port_INLINE void activity_txandackendok(PORT_RADIOTIMER_WIDTH capturedTime) {
   ieee802154_header_iht     ieee802514_header;
//   uint16_t                  lenIE;
   uint8_t ret;
	sRITqueue elequeue;


   incroute(0x09);

   // change state
   changeState(S_TXPROC);

   // cancel tt8
   radiotimer_cancel();

   // turn off the radio
   radio_rfOff();

   //compute tics radio on.
   //ieee154e_vars.radioOnTics+=(radio_getTimerValue()-ieee154e_vars.radioOnInit);

   // record the captured time
   //ieee154e_vars.lastCapturedTime = capturedTime;

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

#if ((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_TX == 1))
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


       //RFF - se o frame eh COAP eu removo o flag de coap pending para liberar novos RPLs
	   elequeue = RITQueue_Get_Element(RITQueue_ElementPending);
       if (elequeue.frameType == IANA_UDP) {
    	   incroute(0x0A);
   		   coappending = 0;
       }

       ret = RITQueue_Free(RITQueue_ElementPending);

      // inform schedule of successful transmission
      schedule_indicateTx(&ieee154e_vars.asn,TRUE);

      // inform upper layer
      notif_sendDone(ieee154e_vars.dataToSend,E_SUCCESS);
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
#else
/*
 * ESTA ROTINA EH DO TRATAMENTO DA MENSAGEM QUANDO ELA NECESSITA DE UM ACK
 * AQUI ELA RECEBEU O ACK E ENTAO AVISA QUE OCORREU SUCESSO NO ENVIO...
 * COMO ESTA COM PROBLEMAS PARA ENVIAR O ACK AQUI EH FEITO UM RECONHECIMENTO AUTOMATICO
 * SEM MSG...POR ISSO A ROTINA FOI REDUZIDA
 */
port_INLINE void activity_ti9(PORT_RADIOTIMER_WIDTH capturedTime) {
   ieee802154_header_iht     ieee802514_header;
   uint16_t                  lenIE;

   // change state
   changeState(S_TXPROC);

   // cancel tt8
   radiotimer_cancel();

   // turn off the radio
   radio_rfOff();
   //compute tics radio on.
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

#if 1 //nao tenho mensagem para tratar

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

      // break if invalid IEEE802.15.4 header
      if (ieee802514_header.valid==FALSE) {
         // break from the do-while loop and execute the clean-up code below
         break;
      }

      // store header details in packet buffer
      ieee154e_vars.ackReceived->l2_frameType  = ieee802514_header.frameType;
      ieee154e_vars.ackReceived->l2_dsn        = ieee802514_header.dsn;
      memcpy(&(ieee154e_vars.ackReceived->l2_nextORpreviousHop),&(ieee802514_header.src),sizeof(open_addr_t));

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
#endif // if 0

      // inform schedule of successful transmission
      schedule_indicateTx(&ieee154e_vars.asn,TRUE);

      // inform upper layer
      notif_sendDone(ieee154e_vars.dataToSend,E_SUCCESS);
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

#endif

//======= RX
//Abre a janela para recepcao...
port_INLINE void activity_rxdataprepare() {

    changeState(S_RXDATAPREPARE);

	radiotimer_cancel();
	// turn off the radio
	radio_rfOff();

	incroute(0x02);

   // calculate the frequency to transmit on
   ieee154e_vars.freq = calculateFrequency(schedule_getChannelOffset());

   // configure the radio for that frequency
   radio_setFrequency(ieee154e_vars.freq);

   // enable the radio in Rx mode. The radio does not actively listen yet.
    radio_rxEnable();
    ieee154e_vars.radioOnInit=radio_getTimerValue();
    ieee154e_vars.radioOnThisSlot=TRUE;

    //teste rff
#if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))

   if (ucFlagTxReOpen == TRUE)
   {
 	uint32_t capturetime;
 	uint8_t *pucAux = (uint8_t *) &capturetime;
 	uint8_t pos=0;

 	ucFlagTxReOpen = FALSE;
 	//leds_sync_on();
     capturetime=radio_getTimerValue();

 	rffbuf[pos++]= RFF_IEEE802_RX;
 	rffbuf[pos++]= 0x04;
 	rffbuf[pos++]= macRITstate;
 	rffbuf[pos++]= *pucAux++;
 	rffbuf[pos++]= *pucAux++;
 	rffbuf[pos++]= *pucAux++;
 	rffbuf[pos++]= *pucAux;

 	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
   }
 #endif

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


   radiotimer_schedule(DURATION_rt3);

   changeState(S_RXDATAREADY);

}

//Abre a janela para recepcao...
port_INLINE void activity_rxwindowopen() {

    changeState(S_RXDATA);

	radiotimer_cancel();

    radio_rxNow();

	incroute(0x03);

#if ((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_RX == 1))
  if (macRITstate != S_RIT_RX_for_TX_state)
  {
	uint32_t capturetime;
	uint8_t *pucAux = (uint8_t *) &capturetime;
	uint8_t pos=0;

    capturetime=radio_getTimerValue();

	rffbuf[pos++]= RFF_IEEE802_RX;
	rffbuf[pos++]= 0x02;
	rffbuf[pos++]= *pucAux++;
	rffbuf[pos++]= *pucAux++;
	rffbuf[pos++]= *pucAux++;
	rffbuf[pos++]= *pucAux;

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
  }
  else
  {
/*
	incroute(0x03);
	uint32_t capturetime;
	uint8_t *pucAux = (uint8_t *) &capturetime;
	uint8_t pos=0;

	//leds_sync_on();
    capturetime=radio_getTimerValue();

	rffbuf[pos++]= RFF_IEEE802_TX;
	rffbuf[pos++]= 0x02;
	rffbuf[pos++]= *pucAux++;
	rffbuf[pos++]= *pucAux++;
	rffbuf[pos++]= *pucAux++;
	rffbuf[pos++]= *pucAux;
	//rffbuf[pos++]= ieee154e_vars.dataToSend->creator;
	//rffbuf[pos++]= ieee154e_vars.dataToSend->owner;
	//rffbuf[pos++]= ieee154e_vars.dataToSend->l2_frameType;

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
*/
  }
#endif

   //Programa os timers do RIT...
	switch (macRITstate){
		case S_RIT_RX_for_TX_state:
		   //radiotimer_schedule(macRITRXforTxPeriod);
			radiotimer_schedule(RIT_RX_TO_TX_PERIOD);
		   break;
		case S_RIT_RX_window_state:
		   //radiotimer_schedule(macRITDataWaitDuration);
			radiotimer_schedule(RIT_RX_WIND_PERIOD);
		   break;
	}

}

#if 0
port_INLINE void activity_rie1() {
   // log the error
   openserial_printError(COMPONENT_IEEE802154E,ERR_MAXRXDATAPREPARE_OVERFLOWS,
                         (errorparameter_t)ieee154e_vars.state,
                         (errorparameter_t)ieee154e_vars.slotOffset);

   // abort
   endSlot();
}

port_INLINE void activity_ritrxlistening() {

    //leds_debug_toggle();

	// change state
   changeState(S_RXDATA);

   // give the 'go' to receive
   radio_rxNow();

   radiotimer_cancel();

   ieee154e_vars.lastCapturedTime = radio_getTimerValue();

   //Programa os timers do RIT...
   switch (macRITstate){
	   case S_RIT_RX_for_TX_state:
		   radiotimer_schedule(macRITRXforTxPeriod);
		   break;
	   case S_RIT_RX_window_state:
		   radiotimer_schedule(macRITDataWaitDuration);
		   break;
   }

}
#endif

/* ACABOU O RIT AQUI E NAO RECEBI NENHUMA MENSAGEM
 * Quando TX significa que eu estou com uma mensagem pendente e fiquei esperando o ola do vizinho que nao veio...
 * Quando RX significa que abri a janela do RIT e nao teve nenhuma mensagem - caso normal.
 */

port_INLINE void activity_ritwindowend() {
	sRITqueue elequeue;

	ieee154e_vars.radioOnInit=radio_getTimerValue();
	ieee154e_vars.radioOnThisSlot=FALSE;

	leds_sync_off();
	// change state
    if (macRITstate == S_RIT_RX_for_TX_state)
    {
		//verifico se o frame atual é um RPL.DAO ou COAP
		elequeue = RITQueue_Get_Element(macRITActualPos);

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

	   // reset local variable
	   ieee154e_vars.dataToSend = NULL;
   }
   else // if (macRITstate == RIT_RX_window_state)
   {
	   //ESTADO NORMAL...SOMENTE ENTRO EM SLEEP E ESPERO PROXIMO RIT.
#if 0 //ENABLE_DEBUG_RFF
	   {
		    uint32_t capturetime;
		    uint8_t *pucAux = (uint8_t *) &capturetime;
			uint8_t pos=0;

			capturetime=radio_getTimerValue();
            pos=0;
		    rffbuf[pos++]= RFF_IEEE802_OLA;
			rffbuf[pos++]= 0x11;
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

/*
 * Aqui o frame foi recebido com sucesso...
 * se somente processo um frame por vez...fecho o radio e processo a resposta.
 *
 */
port_INLINE void activity_rxnewframe(PORT_RADIOTIMER_WIDTH capturedTime) {
   ieee802154_header_iht ieee802514_header;
   uint16_t lenIE=0;
   //uint8_t frameerror=FALSE;
   uint8_t discardframe = FALSE;
	open_addr_t address_2;
	uint8_t elementpos;
	uint8_t   *pauxframe;

   changeState(S_TXACKOFFSET);

   leds_sync_off();

   // cancel rt4
   radiotimer_cancel();

   // turn off the radio
   radio_rfOff();

   ieee154e_vars.radioOnTics+=radio_getTimerValue()-ieee154e_vars.radioOnInit;

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
         // jump to the error code below this do-while loop
         break;
      }

      // parse the IEEE802.15.4 header (RX DATA)
      // AQUI TAMBEM QUE ELE CHECA A TOPOLOGIA...SE O VIZINHO EH VALIDO...
      ieee802154_retrieveHeader(ieee154e_vars.dataReceived,&ieee802514_header);

      // break if invalid IEEE802.15.4 header
      if (ieee802514_header.valid==FALSE) {
         // break from the do-while loop and execute the clean-up code below
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
      //ieee154e_vars.lastCapturedTime = capturedTime;
      ieee154e_vars.lastCapturedTime = 0;

      // if I just received an invalid frame, stop
      if (isValidRxFrame(&ieee802514_header)==FALSE) {
         // jump to the error code below this do-while loop
         break;
      }


       pauxframe = (uint8_t *) &ieee154e_vars.dataReceived->packet[0];
       // pauxframe[1] == 0x61 e pauxframe[2] =  0xdc --> frame COAP.Request or COAP.Response
       // pauxframe[1] == 0x41 e pauxframe[2] =  0xd8 --> frame RPL.DIO
       // pauxframe[1] == 0x61 e pauxframe[2] =  0xdc --> frame RPL.DAO
       if ((pauxframe[1] == 0x61) && (pauxframe[2] =  0xdc)){
    	   incroute(0x41);
       }


       memcpy(&address_2, &(ieee154e_vars.dataReceived->l2_nextORpreviousHop),sizeof(open_addr_t));

       if (macRITstate == S_RIT_RX_for_TX_state) {
			//sendPending = FALSE;
			if (ieee802514_header.frameType == IEEE154_TYPE_OLA) {

				//verifica se existe msg pendente para este endereco de destino
				elementpos = RITQueue_Get_Pos(&address_2);

#if 0 //(ENABLE_DEBUG_RFF) && (DBG_IEEE802_TX == 1)
				{
					uint8_t   pos=0;

					rffbuf[pos++]= RFF_IEEE802_TX;
					rffbuf[pos++]= 0x02;
					rffbuf[pos++]= elementpos;
					rffbuf[pos++]= address_2.type;
					if (address_2.type == 2) {
						rffbuf[pos++]= address_2.addr_64b[6];
						rffbuf[pos++]= address_2.addr_64b[7];
					}
					else if (address_2.type == 3) {
						rffbuf[pos++]= address_2.addr_128b[14];
						rffbuf[pos++]= address_2.addr_128b[15];
					}

					openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);

				}
#endif


				if (elementpos < maxElements)
				{
					//descarto o frame recebido pois nao preciso mais dele
					openqueue_freePacketBuffer(ieee154e_vars.dataReceived);
					ieee154e_vars.dataReceived = NULL;

					//envio mensagem pendente
					//activity_ti2();
					prepare_tx_msg(elementpos);

				}
				else
				{
					//aqui eu estou com mensagem pendente...ou seja nao posso receber mais nada a nao ser um ola..
					discardframe = true;
				}

			}
			else
			{
				   incroute(0x42);
			}
	   }
	   else if (macRITstate == S_RIT_RX_window_state) {

          if (ieee802514_header.frameType != IEEE154_TYPE_OLA) {

           incroute(0x50);
			//TODO!!!! AQUI FALTA AINDA TRATAR SOMENTE FRAME QUE TEM A VER COMIGO...MEU ENDERECO...
			//         OU BROADCAST...MAS NO CASO DO DAO...ELE VAI ESTAR NO FINAL DO FRAME...
			// check if ack requested
             #if ((ENABLE_DEBUG_RFF ==1)  && (DBG_IEEE802_RX == 1))
			  {
				uint8_t   pos=0;
				uint8_t   mybestfriend=0;

				mybestfriend = neighbors_isPreferredParent(&(address_2));

				rffbuf[pos++]= RFF_IEEE802_RX;
				rffbuf[pos++]= 0x03;
				rffbuf[pos++]= ieee802514_header.ackRequested;
				rffbuf[pos++]= mybestfriend;
				rffbuf[pos++]= ieee154e_vars.dataReceived->l4_protocol;
				rffbuf[pos++]= ieee802514_header.frameType;
				rffbuf[pos++]= address_2.type;
				rffbuf[pos++]= address_2.addr_64b[6];
				rffbuf[pos++]= address_2.addr_64b[7];

				openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
			  }
			#endif

			if (ieee802514_header.ackRequested==1)
			{
				incroute(0x51);
				activity_rxsendack();
			}
			else
			{
				incroute(0x52);
			   // indicate reception to upper layer (no ACK asked)
			   notif_receive(ieee154e_vars.dataReceived);
			   ieee154e_vars.dataReceived = NULL;
			   endSlot();
			}

		}
		else {
		   // indicate reception to upper layer (no ACK asked)
		   //notif_receive(ieee154e_vars.dataReceived);
		   //TODO!!! AQUI TENHO DUVIDAS SE DEVO OU NAO DESPREZAR O FRAME DO RIT...
		   //Do jeito que esta a tabela de vizinhos nao esta sendo incrementada pelo nr de anuncios do RIT
		   //Porem se eu nunca comunicar com ele ele nao vai incrementar...mesmo se ele for um bom vizinho...
			incroute(0x53);
 		   openqueue_freePacketBuffer(ieee154e_vars.dataReceived);
		   ieee154e_vars.dataReceived = NULL;
		   endSlot();
		}

	   }

	   if (discardframe) {
			// indicate reception to upper layer (no ACK asked)
			//notif_receive(ieee154e_vars.dataReceived);
			incroute(0x55);
			//descarto o frame recebido pois nao preciso mais dele
			openqueue_freePacketBuffer(ieee154e_vars.dataReceived);
			ieee154e_vars.dataReceived = NULL;
			endSlot();
	  }

	  //openserial_startOutput();

	  // everything went well, return here not to execute the error code below
	  return;

   } while(0);

   // free the (invalid) received data so RAM memory can be recycled
   openqueue_freePacketBuffer(ieee154e_vars.dataReceived);

   // clear local variable
   ieee154e_vars.dataReceived = NULL;

   // abort
   endSlot();

   openserial_startOutput();
}


//mesma rotina do activity_ti2
//somente alterado o buffer para ler da fila ao inves da variavel global

port_INLINE void prepare_tx_msg(uint8_t elementpos) {

	uint8_t ret = false;
	uint8_t *msg;
	uint8_t msglen=0;
	sRITqueue elequeue;

   // change state
   //changeState(S_TXDATAPREPARE);
   changeState(S_TXDATAREADY);

	// change owner
	ieee154e_vars.dataToSend->owner = COMPONENT_IEEE802154E;
	// record that I attempt to transmit this packet
	ieee154e_vars.dataToSend->l2_numTxAttempts++;

	radiotimer_cancel();

   // calculate the frequency to transmit on
   ieee154e_vars.freq = calculateFrequency(schedule_getChannelOffset());

   // configure the radio for that frequency
   radio_setFrequency(ieee154e_vars.freq);

   incroute(0x05);
   // load the packet in the radio's Tx buffer
   //carrega pacote apartir da fila do RIT
	if (elementpos < maxElements) {
		elequeue = RITQueue_Get_Element(elementpos);
		RITQueue_ElementPending = elementpos;
		radio_loadPacket(elequeue.msg , elequeue.msglength);
		//free the mensagem in the RIT message pool
		//vou limpara ela somente depois que foi confirmado o envio
		//ret = RITQueue_Free(elementpos);

#if  0  // ((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_TX == 1))
  {
	uint8_t   pos=0;

	rffbuf[pos++]= RFF_IEEE802_TX;
	rffbuf[pos++]= 0x04;
	rffbuf[pos++]= numAvailableElements;  // nr de elementos ocupados no pool de msg
	rffbuf[pos++]= elementpos;
	rffbuf[pos++]= ret;
	rffbuf[pos++]= elequeue.frameType;
	rffbuf[pos++]= elequeue.msglength;
	rffbuf[pos++]= elequeue.destaddr.type;
    if (elequeue.destaddr.type == 0x01)	{
		rffbuf[pos++]= elequeue.destaddr.addr_16b[0];
		rffbuf[pos++]= elequeue.destaddr.addr_16b[1];
    }
    if (elequeue.destaddr.type == 0x02)	{
		rffbuf[pos++]= elequeue.destaddr.addr_64b[6];
		rffbuf[pos++]= elequeue.destaddr.addr_64b[7];
    }
	else if (elequeue.destaddr.type == 0x03) {
		rffbuf[pos++]= elequeue.destaddr.addr_128b[14];
		rffbuf[pos++]= elequeue.destaddr.addr_128b[15];
	}

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
  }
#endif


	}

   // enable the radio in Tx mode. This does not send the packet.
   radio_txEnable();
   ieee154e_vars.radioOnInit=radio_getTimerValue();
   ieee154e_vars.radioOnThisSlot=TRUE;

  //radio_txNow();

  // TODO!!!!! NAO SEI POR QUE NAO FUNCIONA EU LIGAR ESTE TIMER ANTES DE TRANSMITIR ??????
  //radiotimer_schedule(22);

  SendTxRITProcedure(1);

}

port_INLINE void activity_ti2() {
   // change state
   //changeState(S_TXDATAPREPARE);
   changeState(S_TXDATAREADY);

	// change owner
	ieee154e_vars.dataToSend->owner = COMPONENT_IEEE802154E;
	// record that I attempt to transmit this packet
	ieee154e_vars.dataToSend->l2_numTxAttempts++;

	radiotimer_cancel();

   // calculate the frequency to transmit on
   ieee154e_vars.freq = calculateFrequency(schedule_getChannelOffset());

   // configure the radio for that frequency
   radio_setFrequency(ieee154e_vars.freq);

   // load the packet in the radio's Tx buffer
      radio_loadPacket(ieee154e_vars.dataToSend->payload,
                    ieee154e_vars.dataToSend->length);




   // enable the radio in Tx mode. This does not send the packet.
   radio_txEnable();
   ieee154e_vars.radioOnInit=radio_getTimerValue();
   ieee154e_vars.radioOnThisSlot=TRUE;

#if  ((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_TX == 1))
  {
	uint8_t   pos=0;

	rffbuf[pos++]= RFF_IEEE802_TX;
	rffbuf[pos++]= 0x33;
	rffbuf[pos++]= ieee154e_vars.dataToSend->creator;
	rffbuf[pos++]= ieee154e_vars.dataToSend->owner;
	rffbuf[pos++]= ieee154e_vars.dataToSend->l2_frameType;



	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
  }
#endif

  //radio_txNow();

  // TODO!!!!! NAO SEI POR QUE NAO FUNCIONA EU LIGAR ESTE TIMER ANTES DE TRANSMITIR ??????
  //radiotimer_schedule(22);

  SendTxRITProcedure(1);

}

port_INLINE void activity_rxsendack() {
   //PORT_SIGNED_INT_WIDTH timeCorrection;
   header_IE_ht header_desc;

   // change state
   changeState(S_TXACK);

   incroute(0x56);

   // get a buffer to put the ack to send in
   ieee154e_vars.ackToSend = openqueue_getFreePacketBuffer(COMPONENT_IEEE802154E);
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
   ieee154e_vars.radioOnInit=radio_getTimerValue();
   radiotimer_cancel();
   //ieee154e_vars.lastCapturedTime = ieee154e_vars.radioOnInit;
   ieee154e_vars.radioOnThisSlot=TRUE;

   radiotimer_schedule(DURATION_rt7);

   radio_txNow();

#if ENABLE_DEBUG_RFF
   {
	   uint32_t rffcapture = ieee154e_vars.lastCapturedTime;
       uint8_t pos=0;
       uint8_t *pucAux = (uint8_t *) &rffcapture;

	    rffbuf[pos++]= RFF_IEEE802_RX;
		rffbuf[pos++]= 0x03;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos++);
   }
#endif
   incroute(0x57);

}



port_INLINE void activity_rie5() {

#if ENABLE_DEBUG_RFF
	rffbuf[0]= RFF_IEEE802_RX;
	rffbuf[1]= 0x14;
	rffbuf[2]= 255;
	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,3);
#else
   // log the error
   openserial_printError(COMPONENT_IEEE802154E,ERR_WDRADIOTX_OVERFLOWS,
                         (errorparameter_t)ieee154e_vars.state,
                         (errorparameter_t)ieee154e_vars.slotOffset);
#endif
   // abort
   endSlot();
}

port_INLINE void activity_rxandackendok(PORT_RADIOTIMER_WIDTH capturedTime) {
   // change state
   changeState(S_RXPROC);

   // cancel rt8
   radiotimer_cancel();

   // record the captured time
   ieee154e_vars.lastCapturedTime = capturedTime;

   incroute(0x58);

#if ENABLE_DEBUG_RFF
   {
	   uint32_t rffcapture = ieee154e_vars.lastCapturedTime;
       uint8_t pos=0;
       uint8_t *pucAux = (uint8_t *) &rffcapture;

	    rffbuf[pos++]= RFF_IEEE802_RX;
		rffbuf[pos++]= 0x33;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux++;
		rffbuf[pos++]= *pucAux;

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos++);
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

	// change state
	changeState(S_SLEEP);

	macRITstate=S_RIT_sleep_state;

	radiotimer_cancel();

	leds_debug_off();
	leds_sync_off();

	// record the captured time
	//ieee154e_vars.lastCapturedTime = capturedTime;

	// turn off the radio
	radio_rfOff();

	//TODO!!! PARA QUE ELE USA ISSO
	// compute the duty cycle if radio has been turned on
	if (ieee154e_vars.radioOnThisSlot==TRUE){
	   ieee154e_vars.radioOnTics+=(radio_getTimerValue()-ieee154e_vars.radioOnInit);
	}

	//Limpo a fila de mensagens pendentes
	RITQueue_cleanupoldmsg();

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

	radiotimer_schedule(macRITsleepPeriod);
}

bool ieee154e_isSynch(){
   return ieee154e_vars.isSync;
}


port_INLINE void activity_rxwindowend(void) {
//   bool listenForAck;

   //leds_debug_off();

   radiotimer_cancel();

   //leds_sync_toggle();

   // record the captured time
   //ieee154e_vars.lastCapturedTime = capturedTime;

   // turn off the radio
   //radio_rfOff();

   //TODO!!! PARA QUE ELE USA ISSO
   // compute the duty cycle if radio has been turned on
   if (ieee154e_vars.radioOnThisSlot==TRUE){
      ieee154e_vars.radioOnTics+=(radio_getTimerValue()-ieee154e_vars.radioOnInit);
   }


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

   // change state
   changeState(S_SLEEP);

   radiotimer_schedule(macRITsleepPeriod);

}

inline bool RITQueue_IsEmpty(void) {	return (numAvailableElements == 0) ? true : false; }

inline bool RITQueue_IsFull(void) { 	return (numAvailableElements == maxElements) ? true : false; }

/**
\brief Write the 64-bit address of some neighbor to some location.
   address_1 - Endereco da mensagem que chegou
   adrress_2 - endereco da mensagem que esta pendente
    se endereco da mensagem pendente eh do SINK entao envio mesmo assim.
    TODO!!! MELHORAR
*/

bool  RITQueue_AddrIsTheSame(open_addr_t* addr1, open_addr_t* addr2){

	uint8_t i, nroctets = 0;

    if ((addr1->type == 0x02) && (addr1->addr_64b[6] == 0xFF) && (addr1->addr_64b[7] == 0xFF))
    { //if address is broadcast then send for anyone.
        return true;
    }
    else if (addr1->type == addr2->type)
	{
		switch (addr1->type)
		{
		case 0x01:
			nroctets = 2;
			break;
		case 0x02:
			nroctets = 8;
			break;
		case 0x03:
			nroctets = 16;
			break;
		default:
			nroctets = 0;
			break;
		}

		if (nroctets > 0) {
			for (i = 0; i < nroctets; i++) {
				if (addr1->addr_16b[i] != addr2->addr_16b[i])
				{
					return false;
				}
			}
			return true;
		}
		else
			return false;

	}
    else {
    	// AQUI EU TENHO QUE CHECAR SE O ADDRESS 1 eh do SINK... (FF 02 ...00 02)
        if ((addr1->type == 0x03) &&  (addr2->type == 0x02))  {
        	if ((addr1->addr_64b[0] == 0xFF) &&
                (addr1->addr_64b[1] == 0x02) &&
        	    (addr1->addr_64b[14] == 0x00) &&
        	    (addr1->addr_64b[15] == 0x02)) {
    	        return true;
        	}
    	}

    }

	return false;
}

bool  RITQueue_copyaddress(open_addr_t* addr1, open_addr_t* addr2){

	uint8_t i, nroctets = 0;

	addr1->type = addr2->type;

	switch (addr2->type)
	{
		case 0x01:
			nroctets = 2;
			break;
		case 0x02:
			nroctets = 8;
			break;
		case 0x03:
			nroctets = 16;
			break;
		default:
			nroctets = 0;
			break;
	}

	if (nroctets > 0) {
		for (i = 0; i < nroctets; i++) {
			addr1->addr_128b[i] = addr2->addr_128b[i];
		}
		return true;
	}

	return false;
}

uint8_t RITQueue_Put(sRITelement *psEle,uint8_t pending)
{
	uint8_t  i;

	if (psEle == NULL)
		return maxElements; //msg invalida

	//EnterCriticalSection
	INTERRUPT_DECLARATION();
	DISABLE_INTERRUPTS();

	if (RITQueue_IsFull() == false)
	{
		//procuro o primeiro elemento da fila disponivel
		for (i = 0; i < maxElements; i++)
		{
			if ((pvObjList[i].frameType == 0) && (pvObjList[i].msglength == 0))
			{
				/* Insert msg into the queue. */
				pvObjList[i].destaddr = psEle->destaddr;
				pvObjList[i].timestamp = psEle->timestamp;
				pvObjList[i].msglength = psEle->msglength;
				pvObjList[i].frameType = psEle->frameType;
				pvObjList[i].isBroadcastMulticast = psEle->isBroadcastMulticast;
				memcpy(pvObjList[i].msg, psEle->msg, psEle->msglength);
				pvObjList[i].pending = pending;

				if (psEle->frameType == IANA_UDP)
					coappending = true;

				if (psEle->frameType != IANA_ICMPv6_RPL_DIO) {
					pvObjList[i].countretry++;
				}

				numAvailableElements++;
				break;
			}
		}

	}


	//LeaveCriticalSection
	ENABLE_INTERRUPTS();

	return i;
}



sRITqueue RITQueue_Get_Element(uint8_t pos)
{
	sRITqueue  elem;

	//EnterCriticalSection
	INTERRUPT_DECLARATION();
	DISABLE_INTERRUPTS();

	if (RITQueue_IsEmpty() == false)
	{
		if ( pos < maxElements)
		{
			elem = pvObjList[pos];
		}
		else
		{
			elem.frameType = 0;
			RITQueue_ClearAddress(&elem.destaddr);
 		}
	}
	else
	{
		elem.frameType = 0;
		RITQueue_ClearAddress(&elem.destaddr);
	}

	//LeaveCriticalSection
	ENABLE_INTERRUPTS();

	return elem;
}

uint8_t RITQueue_Get_Pos(open_addr_t *paddr)
{
	uint8_t i=maxElements;

	//EnterCriticalSection
	INTERRUPT_DECLARATION();
	DISABLE_INTERRUPTS();

	if (RITQueue_IsEmpty() == false)
	{
		for (i = 0; i < maxElements; i++)
		{
			if (RITQueue_AddrIsTheSame(&pvObjList[i].destaddr, paddr) == true)
			{
				break;
			}
		}
	}

	//LeaveCriticalSection
	ENABLE_INTERRUPTS();

	return i;
}


bool  RITQueue_ClearAddress(open_addr_t* addr){

	uint8_t i;

	addr->type = 0;
	for (i=0;i<16;i++)
	{
	  addr->addr_128b[i] = 0;
	}

	return (true);
}

bool RITQueue_PutPending(uint8_t elementpos)
{
	bool ret = false;
	uint8_t  i;

	//EnterCriticalSection
	INTERRUPT_DECLARATION();
	DISABLE_INTERRUPTS();

	if (RITQueue_IsEmpty() == false)
	{
		if (elementpos < maxElements)
		{
			pvObjList[elementpos].pending = 1;
			pvObjList[elementpos].countretry++;
			ret = true;
		}
	}
	else
	{
		//printf("LISTA VAZIA!!!!! \n");
	}


	//LeaveCriticalSection
	ENABLE_INTERRUPTS();

	return ret;
}

/*
 * Verifica se existe algum frame pendente
 */
bool RITQueue_ExistFramePending(void)
{
	bool ret = false;
	uint8_t  i;

	//EnterCriticalSection
	INTERRUPT_DECLARATION();
	DISABLE_INTERRUPTS();

	for (i = 0; i < maxElements; i++)
	{
		if (pvObjList[i].pending)
		{
			if (pvObjList[i].countretry < 3)
			{
				ret = true;
				break;
			}
			else  //REMOVO O ELEMENTO QUE JA ESTA A TEMPOS AQUI
			{
				RITQueue_Free(i);
			}
		}
	}

   if (coappending)
	   ret = true;


	//LeaveCriticalSection
	ENABLE_INTERRUPTS();

	return ret;
}
/*
 * Retorna a posicao do 1o frame pendente
 * se ja existir um frame pendente e ja ultrapassou o nro de retry entao
 * eh removido ele da fila...
 */
uint8_t RITQueue_GetPending(void)
{
	bool pos = maxElements;
	uint8_t  i;

	//EnterCriticalSection
	INTERRUPT_DECLARATION();
	DISABLE_INTERRUPTS();

	if (RITQueue_IsEmpty() == false)
	{
		for (i = 0; i < maxElements; i++)
		{
			if (pvObjList[i].pending)
			{
				if (pvObjList[i].countretry < 3)
				{
					pos = i;
					break;
				}
				else  //REMOVO O ELEMENTO QUE JA ESTA A TEMPOS AQUI
				{
					RITQueue_Free(i);
				}
			}
		}
	}
	else
	{
		//printf("LISTA VAZIA!!!!! \n");
	}


	//LeaveCriticalSection
	ENABLE_INTERRUPTS();

	return pos;
}

bool RITQueue_Free(uint8_t elementpos)
{
	bool ret = false;
	uint8_t  i;

	//EnterCriticalSection
	INTERRUPT_DECLARATION();
	DISABLE_INTERRUPTS();

	if (elementpos < maxElements)
	{
		RITQueue_ClearAddress(&pvObjList[elementpos].destaddr);
		pvObjList[elementpos].timestamp   = 0;
		pvObjList[elementpos].msglength   = 0;
		pvObjList[elementpos].frameType   = 0;
		pvObjList[elementpos].pending     = 0;
		pvObjList[elementpos].countretry  = 0;
		pvObjList[elementpos].isBroadcastMulticast = 0;

		if (numAvailableElements > 0)
			numAvailableElements--;

		ret = true;
	}


	//LeaveCriticalSection
	ENABLE_INTERRUPTS();

	return ret;
}


/*
  Deleto todas as mensagens que estao na fila que nao tem prioridades
  criterios:
      - msg com a mais de 2000 segundos
      - msg DIO que nao foram enviadas
  retorno o numero de elementos deletados
*/
#define TIMEOUT_2SEC 65530
uint8_t RITQueue_cleanupoldmsg(void){
	uint32_t actualtime;
	uint32_t timeout;
	uint8_t i;
	uint8_t count = 0;
	uint8_t pos=0;
	bool imprimir = false;

	//get actualtime
	actualtime = radio_getTimerValue();

#if ((ENABLE_DEBUG_RFF) && (DBG_IEEE802_TX == 1))
  {
	uint8_t   *pucAux = (uint8_t *) &actualtime;
	uint8_t    lastpos;

	lastpos = incroute(0xCC);

    if (lastpos > 1)
    	imprimir = true;

	if (imprimir)
	{
		rffbuf[pos++]= RFF_IEEE802_TX;
		rffbuf[pos++]= 0x99;
		rffbuf[pos++] = lastpos;
		if (lastpos < 15)
		{
			for(i=0;i<lastpos;i++)
			{
				rffbuf[pos++]=ritroute[i];
			}
		}

		if (RITQueue_IsEmpty() == false)
		{
			for (i=0;i<5;i++)
				rffbuf[pos++]= pvObjList[i].frameType;
		}

	}
  }
#endif

	if (RITQueue_IsEmpty() == false)
	{
		for (i = 0; i < maxElements; i++)
		{
			if ((pvObjList[i].msglength > 0))
			{
				if ((pvObjList[i].frameType == 1) || (pvObjList[i].frameType == 2) ||
				    ((pvObjList[i].timestamp + TIMEOUT_2SEC) < actualtime))
				{
					if (RITQueue_Free(i)) {
						count++;
					}
				}
			}
		}

	}

#if ((ENABLE_DEBUG_RFF) && (DBG_IEEE802_TX == 1))

	if (imprimir)
	{
		rffbuf[pos++]= count;  // nr de elementos desocupados no pool de msg
		rffbuf[pos++]= numAvailableElements;  // nr de elementos ocupados no pool de msg

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}

#endif


	return count;
}

void RITQueue_Init(void){

	uint32_t i;

	numOfQueueInsertionErrors = 0;
	numAvailableElements = 0;
	maxElements = MAX_RIT_LIST_ELEM;

	dio_stat.countprod = 0;
	dio_stat.countsenderror = 0;
	dio_stat.countsendok = 0;

	dao_stat.countprod = 0;
	dao_stat.countsenderror = 0;
	dao_stat.countsendok = 0;

	coappending = 0;

	for (i = 0; i < maxElements; i++)
	{
		RITQueue_Free(i);
	}
}


#endif // (IEEE802154E_RIT == 1)

