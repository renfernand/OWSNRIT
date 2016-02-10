#include "opendefs.h"
#include "icmpv6rpl.h"
#include "icmpv6.h"
#include "openserial.h"
#include "openqueue.h"
#include "neighbors.h"
#include "packetfunctions.h"
#include "openrandom.h"
#include "scheduler.h"
#include "idmanager.h"
#include "opentimers.h"
#include "IEEE802154E.h"
#include "board.h"
#include "leds.h"

//=========================== variables =======================================
extern neighbors_vars_t neighbors_vars;
icmpv6rpl_vars_t             icmpv6rpl_vars;
extern scheduler_vars_t scheduler_vars;
extern scheduler_dbg_t  scheduler_dbg;

#if (DEBUG_LOG_RIT == 1)
extern ieee154e_vars_t    ieee154e_vars;
extern ieee154e_stats_t   ieee154e_stats;
extern ieee154e_dbg_t     ieee154e_dbg;
static uint8_t rffbuf[10];
open_addr_t         dao_address;

#endif
//=========================== prototypes ======================================

// DIO-related
void icmpv6rpl_timer_DIO_cb(void);
void icmpv6rpl_timer_DIO_task(void);
void sendDIO(void);
// DAO-related
void icmpv6rpl_timer_DAO_cb(void);
void icmpv6rpl_timer_DAO_task(void);
void sendDAO(void);

//=========================== public ==========================================

/**
\brief Initialize this module.
*/ 
#if (NEW_DAG_BRIDGE == 1)
void icmpv6rpl_init() {
   uint8_t         dodagid[16];
   
   // retrieve my prefix and EUI64
   memcpy(&dodagid[0],idmanager_getMyID(ADDR_PREFIX)->prefix,8); // prefix
   memcpy(&dodagid[8],idmanager_getMyID(ADDR_64B)->addr_64b,8);  // eui64
   
   //===== reset local variables
   memset(&icmpv6rpl_vars,0,sizeof(icmpv6rpl_vars_t));
   
   //=== admin
   
   icmpv6rpl_vars.busySending               = FALSE;
   icmpv6rpl_vars.fDodagidWritten           = 0;
   
   //=== DIO
   
   icmpv6rpl_vars.dio.rplinstanceId         = 0x00;        ///< TODO: put correct value
   icmpv6rpl_vars.dio.verNumb               = 0x00;        ///< TODO: put correct value
   // rank: to be populated upon TX
   icmpv6rpl_vars.dio.rplOptions            = MOP_DIO_A | \
                                              MOP_DIO_B | \
                                              MOP_DIO_C | \
                                              PRF_DIO_A | \
                                              PRF_DIO_B | \
                                              PRF_DIO_C | \
                                              G_DIO ;
   icmpv6rpl_vars.dio.DTSN                  = 0x33;        ///< TODO: put correct value
   icmpv6rpl_vars.dio.flags                 = 0x00;
   icmpv6rpl_vars.dio.reserved              = 0x00;
   memcpy(
      &(icmpv6rpl_vars.dio.DODAGID[0]),
      dodagid,
      sizeof(icmpv6rpl_vars.dio.DODAGID)
   ); // can be replaced later
   
   icmpv6rpl_vars.dioDestination.type = ADDR_128B;
   memcpy(&icmpv6rpl_vars.dioDestination.addr_128b[0],all_routers_multicast,sizeof(all_routers_multicast));
   
   icmpv6rpl_vars.periodDIO                 = TIMER_DIO_TIMEOUT+(openrandom_get16b()&0xff);
   icmpv6rpl_vars.timerIdDIO                = opentimers_start(
                                                icmpv6rpl_vars.periodDIO,
                                                TIMER_PERIODIC,
                                                TIME_MS,
                                                icmpv6rpl_timer_DIO_cb
                                             );
   
   //=== DAO
   
   icmpv6rpl_vars.dao.rplinstanceId         = 0x00;        ///< TODO: put correct value
   icmpv6rpl_vars.dao.K_D_flags             = FLAG_DAO_A   | \
                                              FLAG_DAO_B   | \
                                              FLAG_DAO_C   | \
                                              FLAG_DAO_D   | \
                                              FLAG_DAO_E   | \
                                              PRF_DIO_C    | \
                                              FLAG_DAO_F   | \
                                              D_DAO        |
                                              K_DAO;
   icmpv6rpl_vars.dao.reserved              = 0x00;
   icmpv6rpl_vars.dao.DAOSequence           = 0x00;
   memcpy(
      &(icmpv6rpl_vars.dao.DODAGID[0]),
      dodagid,
      sizeof(icmpv6rpl_vars.dao.DODAGID)
   );  // can be replaced later
   
   icmpv6rpl_vars.dao_transit.type          = OPTION_TRANSIT_INFORMATION_TYPE;
   // optionLength: to be populated upon TX
   icmpv6rpl_vars.dao_transit.E_flags       = E_DAO_Transit_Info;
   icmpv6rpl_vars.dao_transit.PathControl   = PC1_A_DAO_Transit_Info | \
                                              PC1_B_DAO_Transit_Info | \
                                              PC2_A_DAO_Transit_Info | \
                                              PC2_B_DAO_Transit_Info | \
                                              PC3_A_DAO_Transit_Info | \
                                              PC3_B_DAO_Transit_Info | \
                                              PC4_A_DAO_Transit_Info | \
                                              PC4_B_DAO_Transit_Info;  
   icmpv6rpl_vars.dao_transit.PathSequence  = 0x00; // to be incremented at each TX
   icmpv6rpl_vars.dao_transit.PathLifetime  = 0xAA;
   //target information
   icmpv6rpl_vars.dao_target.type  = OPTION_TARGET_INFORMATION_TYPE;
   icmpv6rpl_vars.dao_target.optionLength  = 0;
   icmpv6rpl_vars.dao_target.flags  = 0;
   icmpv6rpl_vars.dao_target.prefixLength = 0;
   
   icmpv6rpl_vars.periodDAO                 = TIMER_DAO_TIMEOUT+(openrandom_get16b()&0xff);
   icmpv6rpl_vars.timerIdDAO                = opentimers_start(
                                                icmpv6rpl_vars.periodDAO,
                                                TIMER_PERIODIC,
                                                TIME_MS,
                                                icmpv6rpl_timer_DAO_cb
                                             );
   
}

void  icmpv6rpl_writeDODAGid(uint8_t* dodagid) {
   
   // write DODAGID to DIO/DAO
   memcpy(
      &(icmpv6rpl_vars.dio.DODAGID[0]),
      dodagid,
      sizeof(icmpv6rpl_vars.dio.DODAGID)
   );
   memcpy(
      &(icmpv6rpl_vars.dao.DODAGID[0]),
      dodagid,
      sizeof(icmpv6rpl_vars.dao.DODAGID)
   );
   
   // remember I got a DODAGID
   icmpv6rpl_vars.fDodagidWritten = 1;
}

#else
void icmpv6rpl_init() {

   //===== reset local variables
   memset(&icmpv6rpl_vars,0,sizeof(icmpv6rpl_vars_t));
   
   //=== admin
   
   icmpv6rpl_vars.busySending               = FALSE;
   icmpv6rpl_vars.DODAGIDFlagSet            = 0;

#if (IEEE802154E_RIT == 1)
   //RFF RIT
   icmpv6rpl_vars.busySendingDAO = FALSE;
   icmpv6rpl_vars.CountSendingDAO = 0;
#endif

   //=== DIO-related
   
   icmpv6rpl_vars.dio.rplinstanceId         = 0x00;        ///< TODO: put correct value
   icmpv6rpl_vars.dio.verNumb               = 0x00;        ///< TODO: put correct value
   // rank: to be populated upon TX
   icmpv6rpl_vars.dio.rplOptions            = MOP_DIO_A | \
                                              MOP_DIO_B | \
                                              MOP_DIO_C | \
                                              PRF_DIO_A | \
                                              PRF_DIO_B | \
                                              PRF_DIO_C | \
                                              G_DIO ;
   icmpv6rpl_vars.dio.DTSN                  = 0x33;        ///< TODO: put correct value
   icmpv6rpl_vars.dio.flags                 = 0x00;
   icmpv6rpl_vars.dio.reserved              = 0x00;
   // DODAGID: to be populated upon receiving DIO
   
   icmpv6rpl_vars.dioDestination.type = ADDR_128B;
   memcpy(&icmpv6rpl_vars.dioDestination.addr_128b[0],all_routers_multicast,sizeof(all_routers_multicast));
   
   icmpv6rpl_vars.periodDIO                 = TIMER_DIO_TIMEOUT+(openrandom_get16b()&0xff);
   icmpv6rpl_vars.timerIdDIO                = opentimers_start(
                                                icmpv6rpl_vars.periodDIO,
                                                TIMER_PERIODIC,
                                                TIME_MS,
                                                icmpv6rpl_timer_DIO_cb
                                             );
   
   //=== DAO-related
   
   icmpv6rpl_vars.dao.rplinstanceId         = 0x00;        ///< TODO: put correct value
   icmpv6rpl_vars.dao.K_D_flags             = FLAG_DAO_A   | \
                                              FLAG_DAO_B   | \
                                              FLAG_DAO_C   | \
                                              FLAG_DAO_D   | \
                                              FLAG_DAO_E   | \
                                              PRF_DIO_C    | \
                                              FLAG_DAO_F   | \
                                              D_DAO        |
                                              K_DAO;
   icmpv6rpl_vars.dao.reserved              = 0x00;
   icmpv6rpl_vars.dao.DAOSequence           = 0x00;
   // DODAGID: to be populated upon receiving DIO
   
   icmpv6rpl_vars.dao_transit.type          = OPTION_TRANSIT_INFORMATION_TYPE;
   // optionLength: to be populated upon TX
   icmpv6rpl_vars.dao_transit.E_flags       = E_DAO_Transit_Info;
   icmpv6rpl_vars.dao_transit.PathControl   = PC1_A_DAO_Transit_Info | \
                                              PC1_B_DAO_Transit_Info | \
                                              PC2_A_DAO_Transit_Info | \
                                              PC2_B_DAO_Transit_Info | \
                                              PC3_A_DAO_Transit_Info | \
                                              PC3_B_DAO_Transit_Info | \
                                              PC4_A_DAO_Transit_Info | \
                                              PC4_B_DAO_Transit_Info;  
   icmpv6rpl_vars.dao_transit.PathSequence  = 0x00; // to be incremented at each TX
   icmpv6rpl_vars.dao_transit.PathLifetime  = 0xAA;
   //target information
   icmpv6rpl_vars.dao_target.type  = OPTION_TARGET_INFORMATION_TYPE;
   icmpv6rpl_vars.dao_target.optionLength  = 0;
   icmpv6rpl_vars.dao_target.flags  = 0;
   icmpv6rpl_vars.dao_target.prefixLength = 0;
   
   icmpv6rpl_vars.periodDAO                 = TIMER_DAO_TIMEOUT+(openrandom_get16b()&0xff);
   icmpv6rpl_vars.timerIdDAO                = opentimers_start(
                                                icmpv6rpl_vars.periodDAO,
                                                TIMER_PERIODIC,
                                                TIME_MS,
                                                icmpv6rpl_timer_DAO_cb
                                             );
   
}
#endif

uint8_t icmpv6rpl_getRPLIntanceID(){
	return icmpv6rpl_vars.dao.rplinstanceId;
}

/**
\brief Called when DIO/DAO was sent.

\param[in] msg   Pointer to the message just sent.
\param[in] error Outcome of the sending.
*/
void icmpv6rpl_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
   
   // take ownership over that packet
   msg->owner = COMPONENT_ICMPv6RPL;
   
#if (IEEE802154E_RIT == 1)
   icmpv6rpl_vars.busySendingDAO = FALSE;
   icmpv6rpl_vars.CountSendingDAO = 0;
#endif

#if ((DEBUG_LOG_RIT  == 1) && (DBG_RPL == 1))
   {
	   uint8_t ucPos=0;

	   rffbuf[ucPos++]= RFF_ICMPv6RPL_TX;
	   rffbuf[ucPos++]= 0xFE;
	   rffbuf[ucPos++]= (uint8_t) msg->creator;
	   rffbuf[ucPos++]= (uint8_t) msg->creator;
	   //rffbuf[ucPos++]= (uint8_t) msg->owner;
	   //rffbuf[ucPos++]= (uint8_t) msg->length;
	   //rffbuf[ucPos++]= (uint8_t) msg->l2_frameType;
	   openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,ucPos);
   }
#endif

   // make sure I created it
   if (msg->creator!=COMPONENT_ICMPv6RPL) {
      openserial_printError(COMPONENT_ICMPv6RPL,ERR_UNEXPECTED_SENDDONE,
                            (errorparameter_t)0,
                            (errorparameter_t)0);
   }
   
   // free packet
   openqueue_freePacketBuffer(msg);
   
   // I'm not busy sending anymore
   icmpv6rpl_vars.busySending = FALSE;
}

/**
\brief Called when RPL message received.

\param[in] msg   Pointer to the received message.
*/
#if (NEW_DAG_BRIDGE == 1)
void icmpv6rpl_receive(OpenQueueEntry_t* msg) {
   uint8_t      icmpv6code;
   open_addr_t  myPrefix;
   
   // take ownership
   msg->owner      = COMPONENT_ICMPv6RPL;
   
   // retrieve ICMPv6 code
   icmpv6code      = (((ICMPv6_ht*)(msg->payload))->code);
   
   // toss ICMPv6 header
   packetfunctions_tossHeader(msg,sizeof(ICMPv6_ht));
   
   //teste rff
   //leds_error_toggle();
   //teste rff

#if (DEBUG_LOG_RIT  == 1) && (DBG_RPL == 1)
   {
	   uint8_t ucPos=0;

	   rffbuf[ucPos++]= RFF_ICMPv6RPL_RX;
	   rffbuf[ucPos++]= (uint8_t) icmpv6code;
	   rffbuf[ucPos++]= (uint8_t) msg->creator;
	   //rffbuf[ucPos++]= (uint8_t) msg->owner;
	   //rffbuf[ucPos++]= (uint8_t) msg->length;
	   //rffbuf[ucPos++]= (uint8_t) msg->l2_frameType;
	   openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,ucPos);
   }
#endif

   // handle message
   switch (icmpv6code) {
      
      case IANA_ICMPv6_RPL_DIO:
         if (idmanager_getIsDAGroot()==TRUE) {
            // stop here if I'm in the DAG root
            break; // break, don't return
         }
         
         // update neighbor table
         neighbors_indicateRxDIO(msg);
         
         // write DODAGID in DIO and DAO
         icmpv6rpl_writeDODAGid(&(((icmpv6rpl_dio_ht*)(msg->payload))->DODAGID[0]));
         
         // update my prefix
         myPrefix.type = ADDR_PREFIX;
         memcpy(
            myPrefix.prefix,
            &((icmpv6rpl_dio_ht*)(msg->payload))->DODAGID[0],
            sizeof(myPrefix.prefix)
         );
         idmanager_setMyID(&myPrefix);
         
         break;
      
      case IANA_ICMPv6_RPL_DAO:
         // this should never happen
         openserial_printCritical(COMPONENT_ICMPv6RPL,ERR_UNEXPECTED_DAO,
                               (errorparameter_t)0,
                               (errorparameter_t)0);
         break;
      
      default:
         // this should never happen
         openserial_printCritical(COMPONENT_ICMPv6RPL,ERR_MSG_UNKNOWN_TYPE,
                               (errorparameter_t)icmpv6code,
                               (errorparameter_t)0);
         break;
      
   }
   
   // free message
   openqueue_freePacketBuffer(msg);
}
#else


void icmpv6rpl_receive(OpenQueueEntry_t* msg) {
   uint8_t      icmpv6code;
   open_addr_t  myPrefix;
   
   // take ownership
   msg->owner      = COMPONENT_ICMPv6RPL;
   
   // retrieve ICMPv6 code
   icmpv6code      = (((ICMPv6_ht*)(msg->payload))->code);
   
   // toss ICMPv6 header
   packetfunctions_tossHeader(msg,sizeof(ICMPv6_ht));
   
#if ((DEBUG_LOG_RIT  == 1) && (DBG_RPL == 1))
   {
		open_addr_t* address;
		uint8_t pos=0;

		address =  &(msg->l2_nextORpreviousHop);

		rffbuf[pos++]= RFF_ICMPv6RPL_RX;
		rffbuf[pos++]= 0x02;
	    rffbuf[pos++]= (uint8_t) icmpv6code;
  	    rffbuf[pos++]= address->type;

  	    if (address->type == 3)
		{
			rffbuf[pos++]= address->addr_128b[14];
			rffbuf[pos++]= address->addr_128b[15];
		}
		else //considero igual a 2
		{
			rffbuf[pos++]= address->addr_64b[6];
			rffbuf[pos++]= address->addr_64b[7];
		}

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
   }
#endif

   // handle message
   switch (icmpv6code) {

      case IANA_ICMPv6_RPL_DIO:
         if (idmanager_getIsBridge()==TRUE) {
            // stop here if I'm in bridge mode
            break; // break, don't return
         }

         // update neighbor table
         neighbors_indicateRxDIO(msg);
         

         // update DODAGID in DIO/DAO
         memcpy(
            &(icmpv6rpl_vars.dio.DODAGID[0]),
            &(((icmpv6rpl_dio_ht*)(msg->payload))->DODAGID[0]),
            sizeof(icmpv6rpl_vars.dio.DODAGID)
         );
         memcpy(
            &(icmpv6rpl_vars.dao.DODAGID[0]),
            &(((icmpv6rpl_dio_ht*)(msg->payload))->DODAGID[0]),
            sizeof(icmpv6rpl_vars.dao.DODAGID)
         );
         
         // remember I got a DODAGID
         icmpv6rpl_vars.DODAGIDFlagSet=1;
         
         // update my prefix
         myPrefix.type = ADDR_PREFIX;
         memcpy(
            myPrefix.prefix,
            &((icmpv6rpl_dio_ht*)(msg->payload))->DODAGID[0],
            sizeof(myPrefix.prefix)
         );
         idmanager_setMyID(&myPrefix);
         
         break;
      
      case IANA_ICMPv6_RPL_DAO:
         // this should never happen
         openserial_printCritical(COMPONENT_ICMPv6RPL,ERR_UNEXPECTED_DAO,
                               (errorparameter_t)0,
                               (errorparameter_t)0);
         break;
      
      default:
         // this should never happen
         openserial_printCritical(COMPONENT_ICMPv6RPL,ERR_MSG_UNKNOWN_TYPE,
                               (errorparameter_t)icmpv6code,
                               (errorparameter_t)0);
         break;
      
   }
   
   // free message
   openqueue_freePacketBuffer(msg);
}
#endif

//=========================== private =========================================

//===== DIO-related

/**
\brief DIO timer callback function.

\note This function is executed in interrupt context, and should only push a 
   task.
*/
void icmpv6rpl_timer_DIO_cb() {
   scheduler_push_task(icmpv6rpl_timer_DIO_task,TASKPRIO_RPL);
}

/**
\brief Handler for DIO timer event.

\note This function is executed in task context, called by the scheduler.
*/
void icmpv6rpl_timer_DIO_task() {
   uint8_t           isFramePending=0;

   // update the delayDIO
   icmpv6rpl_vars.delayDIO = (icmpv6rpl_vars.delayDIO+1)%5;

   // check whether we need to send DIO
   if (icmpv6rpl_vars.delayDIO==0) {
#if (IEEE802154E_TSCH == 0)
	  isFramePending = RITQueue_ExistFramePending();

	  if (isFramePending == 0)
	  {
		  //leds_debug_toggle();
	      // send DIO
	      sendDIO();
	  }
#else
      sendDIO();
#endif
      
      // pick a new pseudo-random periodDIO
      icmpv6rpl_vars.periodDIO = TIMER_DIO_TIMEOUT+(openrandom_get16b()&0xff);
      
      // arm the DIO timer with this new value
      opentimers_setPeriod(
         icmpv6rpl_vars.timerIdDIO,
         TIME_MS,
         icmpv6rpl_vars.periodDIO
      );
   }
}

/**
\brief Prepare and a send a RPL DIO.
*/
void sendDIO() {
   OpenQueueEntry_t*    msg;
   
   // stop if I'm not sync'ed
   if (ieee154e_isSynch()==FALSE) {
      
      // remove packets genereted by this module (DIO and DAO) from openqueue
      openqueue_removeAllCreatedBy(COMPONENT_ICMPv6RPL);
      
      // I'm not busy sending a DIO/DAO
      icmpv6rpl_vars.busySending  = FALSE;
      
      // stop here
      return;
   }
#if (NEW_DAG_BRIDGE == 0)      
   // do not send DIO if I'm in in bridge mode
   if (idmanager_getIsBridge()==TRUE) {
      return;
   }
#endif
   
   // do not send DIO if I have the default DAG rank
   if (neighbors_getMyDAGrank()==DEFAULTDAGRANK) {
      return;
   }
   
   // do not send DIO if I'm already busy sending
   if (icmpv6rpl_vars.busySending==TRUE) {
      return;
   }
   
#if (IEEE802154E_RIT == 1)
   // Isto foi criado pois no RIT o DIO esta sobrepondo a mensagem do DAO que ainda nao foi enviada
   // Entao ele fica pendente até ser enviado o DAO ou senao a 3 tentativas de DIO sem sucesso.
   if ((icmpv6rpl_vars.busySendingDAO==TRUE) && (icmpv6rpl_vars.CountSendingDAO < 3)) {
	   icmpv6rpl_vars.CountSendingDAO++;
      return;
   }

   icmpv6rpl_vars.busySendingDAO = FALSE;
   icmpv6rpl_vars.CountSendingDAO = 0;
#endif

   // if you get here, all good to send a DIO
   
   // I'm now busy sending
   icmpv6rpl_vars.busySending = TRUE;
   
   // reserve a free packet buffer for DIO
   msg = openqueue_getFreePacketBuffer(COMPONENT_ICMPv6RPL);
   if (msg==NULL) {
      openserial_printError(COMPONENT_ICMPv6RPL,ERR_NO_FREE_PACKET_BUFFER,
                            (errorparameter_t)0,
                            (errorparameter_t)0);
      icmpv6rpl_vars.busySending = FALSE;
      
      return;
   }
   
   // take ownership
   msg->creator                             = COMPONENT_ICMPv6RPL;
   msg->owner                               = COMPONENT_ICMPv6RPL;
   
   // set transport information
   msg->l4_protocol                         = IANA_ICMPv6;
   msg->l4_sourcePortORicmpv6Type           = IANA_ICMPv6_RPL;
   
   // set DIO destination
   memcpy(&(msg->l3_destinationAdd),&icmpv6rpl_vars.dioDestination,sizeof(open_addr_t));
   
   //===== DIO payload
   // note: DIO is already mostly populated
   icmpv6rpl_vars.dio.rank                  = neighbors_getMyDAGrank();
   packetfunctions_reserveHeaderSize(msg,sizeof(icmpv6rpl_dio_ht));
   memcpy(
      ((icmpv6rpl_dio_ht*)(msg->payload)),
      &(icmpv6rpl_vars.dio),
      sizeof(icmpv6rpl_dio_ht)
   );
   
   //===== ICMPv6 header
   packetfunctions_reserveHeaderSize(msg,sizeof(ICMPv6_ht));
   ((ICMPv6_ht*)(msg->payload))->type       = msg->l4_sourcePortORicmpv6Type;
   ((ICMPv6_ht*)(msg->payload))->code       = IANA_ICMPv6_RPL_DIO;
   packetfunctions_calculateChecksum(msg,(uint8_t*)&(((ICMPv6_ht*)(msg->payload))->checksum));//call last
   
   //send
   if (icmpv6_send(msg)!=E_SUCCESS) {
      icmpv6rpl_vars.busySending = FALSE;
      openqueue_freePacketBuffer(msg);
   } else {
      icmpv6rpl_vars.busySending = FALSE; 
   }

#if (DEBUG_LOG_RIT  == 1) && (DBG_RPL == 1)
   {
	   uint8_t pos = 0;

	    rffbuf[pos++]= RFF_ICMPv6RPL_TX;
	    rffbuf[pos++]= 0x01;
	    rffbuf[pos++]= msg->l4_protocol;
	    rffbuf[pos++]= icmpv6rpl_vars.busySendingDAO;
	    rffbuf[pos++]= icmpv6rpl_vars.CountSendingDAO;
		rffbuf[pos++]= scheduler_dbg.numTasksCur;
		rffbuf[pos++]= scheduler_dbg.numTasksMax;
		rffbuf[pos++]= 0xcc;
		rffbuf[pos++]= neighbors_vars.neighbors[0].addr_64b.addr_64b[6];
		rffbuf[pos++]= neighbors_vars.neighbors[0].addr_64b.addr_64b[7];
		rffbuf[pos++]= neighbors_vars.neighbors[0].numTx;
		rffbuf[pos++]= neighbors_vars.neighbors[0].numRx;
		rffbuf[pos++]= neighbors_vars.neighbors[0].numTxACK;
		rffbuf[pos++]= neighbors_vars.neighbors[0].numWraps;

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
   }
#endif

}

//===== DAO-related

/**
\brief DAO timer callback function.

\note This function is executed in interrupt context, and should only push a
   task.
*/
void icmpv6rpl_timer_DAO_cb() {
   scheduler_push_task(icmpv6rpl_timer_DAO_task,TASKPRIO_RPL);
}

/**
\brief Handler for DAO timer event.

\note This function is executed in task context, called by the scheduler.
*/
void icmpv6rpl_timer_DAO_task() {
   
   // update the delayDAO
   icmpv6rpl_vars.delayDAO = (icmpv6rpl_vars.delayDAO+1)%5;
   
   // check whether we need to send DAO
   if (icmpv6rpl_vars.delayDAO==0) {
      
      // send DAO
      sendDAO();
      
      // pick a new pseudo-random periodDAO
      icmpv6rpl_vars.periodDAO = TIMER_DAO_TIMEOUT+(openrandom_get16b()&0xff);
      
      // arm the DAO timer with this new value
      opentimers_setPeriod(
         icmpv6rpl_vars.timerIdDAO,
         TIME_MS,
         icmpv6rpl_vars.periodDAO
      );
   }
}

/**
\brief Prepare and a send a RPL DAO.
*/
void sendDAO() {
   OpenQueueEntry_t*    msg;                // pointer to DAO messages
   uint8_t              nbrIdx;             // running neighbor index
   uint8_t              numTransitParents,numTargetParents;  // the number of parents indicated in transit option
   open_addr_t         address;
   open_addr_t*        prefix;
   
   if (ieee154e_isSynch()==FALSE) {
      // I'm not sync'ed 
      
      // delete packets genereted by this module (DIO and DAO) from openqueue
      openqueue_removeAllCreatedBy(COMPONENT_ICMPv6RPL);
      
      // I'm not busy sending a DIO/DAO
      icmpv6rpl_vars.busySending = FALSE;
      
      // stop here
      return;
   }

   // dont' send a DAO if you're the DAG root
   #if (NEW_DAG_BRIDGE == 1)
   if (idmanager_getIsDAGroot()==TRUE) {
      return;
   }
   #else    
   if (idmanager_getIsBridge()==TRUE) {
      return;
   }
   #endif

#if 0 //(DEBUG_LOG_RIT  == 1) && (DBG_RPL == 1)
   {
		uint8_t pos=0;

		rffbuf[pos++]= RFF_ICMPv6RPL_TX;
		rffbuf[pos++]= 0x02;
	    rffbuf[pos++]= icmpv6rpl_vars.busySendingDAO;
	    rffbuf[pos++]= icmpv6rpl_vars.CountSendingDAO;
		rffbuf[pos++]= msg->creator;
		rffbuf[pos++]= msg->l2_frameType;

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
#endif
//teste rff

   // dont' send a DAO if you did not acquire a DAGrank
   if (neighbors_getMyDAGrank()==DEFAULTDAGRANK) {
       return;
   }
   
   // dont' send a DAO if you're still busy sending the previous one
   if (icmpv6rpl_vars.busySending==TRUE) {
      return;
   }
   
#if (IEEE802154E_RIT == 1)
   // if you get here, you start construct DAO
   icmpv6rpl_vars.busySendingDAO =TRUE;
#endif
   
   // reserve a free packet buffer for DAO
   msg = openqueue_getFreePacketBuffer(COMPONENT_ICMPv6RPL);
   if (msg==NULL) {
      openserial_printError(COMPONENT_ICMPv6RPL,ERR_NO_FREE_PACKET_BUFFER,
                            (errorparameter_t)0,
                            (errorparameter_t)0);
      return;
   }
   
   // take ownership
   msg->creator                             = COMPONENT_ICMPv6RPL;
   msg->owner                               = COMPONENT_ICMPv6RPL;
   
   // set transport information
   msg->l4_protocol                         = IANA_ICMPv6;
   msg->l4_sourcePortORicmpv6Type           = IANA_ICMPv6_RPL;
   
   // set DAO destination
   msg->l3_destinationAdd.type=ADDR_128B;
   memcpy(msg->l3_destinationAdd.addr_128b,icmpv6rpl_vars.dio.DODAGID,sizeof(icmpv6rpl_vars.dio.DODAGID));
   
   //===== fill in packet
   
   //NOTE: limit to preferrred parent only the number of DAO transit addresses to send
   
   //=== transit option -- from RFC 6550, page 55 - 1 transit information header per parent is required. 
   //getting only preferred parent as transit
   numTransitParents=0;
   neighbors_getPreferredParentEui64(&address);
   packetfunctions_writeAddress(msg,&address,OW_BIG_ENDIAN);
   prefix=idmanager_getMyID(ADDR_PREFIX);
   packetfunctions_writeAddress(msg,prefix,OW_BIG_ENDIAN);
   // update transit info fields
   // from rfc6550 p.55 -- Variable, depending on whether or not the DODAG ParentAddress subfield is present.
   // poipoi xv: it is not very clear if this includes all fields in the header. or as target info 2 bytes are removed.
   // using the same pattern as in target information.
   icmpv6rpl_vars.dao_transit.optionLength  = LENGTH_ADDR128b + sizeof(icmpv6rpl_dao_transit_ht)-2;
   icmpv6rpl_vars.dao_transit.PathControl=0; //todo. this is to set the preference of this parent.      
   icmpv6rpl_vars.dao_transit.type=OPTION_TRANSIT_INFORMATION_TYPE;
           
   // write transit info in packet
   packetfunctions_reserveHeaderSize(msg,sizeof(icmpv6rpl_dao_transit_ht));
   memcpy(
          ((icmpv6rpl_dao_transit_ht*)(msg->payload)),
          &(icmpv6rpl_vars.dao_transit),
          sizeof(icmpv6rpl_dao_transit_ht)
   );
   numTransitParents++;
   
   //target information is required. RFC 6550 page 55.
   /*
   One or more Transit Information options MUST be preceded by one or
   more RPL Target options.   
   */
    numTargetParents                        = 0;
    for (nbrIdx=0;nbrIdx<MAXNUMNEIGHBORS;nbrIdx++) {
      if ((neighbors_isNeighborWithHigherDAGrank(nbrIdx))==TRUE) {
         // this neighbor is of higher DAGrank as I am. so it is my child
         
         // write it's address in DAO RFC6550 page 80 check point 1.
         neighbors_getNeighbor(&address,ADDR_64B,nbrIdx); 

         packetfunctions_writeAddress(msg,&address,OW_BIG_ENDIAN);
         prefix=idmanager_getMyID(ADDR_PREFIX);
         packetfunctions_writeAddress(msg,prefix,OW_BIG_ENDIAN);
        
         // update target info fields 
         // from rfc6550 p.55 -- Variable, length of the option in octets excluding the Type and Length fields.
         // poipoi xv: assuming that type and length fields refer to the 2 first bytes of the header
         icmpv6rpl_vars.dao_target.optionLength  = LENGTH_ADDR128b +sizeof(icmpv6rpl_dao_target_ht) - 2; //no header type and length
         icmpv6rpl_vars.dao_target.type  = OPTION_TARGET_INFORMATION_TYPE;
         icmpv6rpl_vars.dao_target.flags  = 0;       //must be 0
         icmpv6rpl_vars.dao_target.prefixLength = 128; //128 leading bits  -- full address.
         
         // write transit info in packet
         packetfunctions_reserveHeaderSize(msg,sizeof(icmpv6rpl_dao_target_ht));
         memcpy(
               ((icmpv6rpl_dao_target_ht*)(msg->payload)),
               &(icmpv6rpl_vars.dao_target),
               sizeof(icmpv6rpl_dao_target_ht)
         );
         
         // remember I found it
         numTargetParents++;
      }  
      //limit to MAX_TARGET_PARENTS the number of DAO target addresses to send
      //section 8.2.1 pag 67 RFC6550 -- using a subset
      // poipoi TODO base selection on ETX rather than first X.
      if (numTargetParents>=MAX_TARGET_PARENTS) break;
   }
   
   
   // stop here if no parents found
   if (numTransitParents==0) {
      openqueue_freePacketBuffer(msg);
      return;
   }
   
   icmpv6rpl_vars.dao_transit.PathSequence++; //increment path sequence.
   // if you get here, you will send a DAO
   
   
   //=== DAO header
   packetfunctions_reserveHeaderSize(msg,sizeof(icmpv6rpl_dao_ht));
   memcpy(
      ((icmpv6rpl_dao_ht*)(msg->payload)),
      &(icmpv6rpl_vars.dao),
      sizeof(icmpv6rpl_dao_ht)
   );
   
   //=== ICMPv6 header
   packetfunctions_reserveHeaderSize(msg,sizeof(ICMPv6_ht));
   ((ICMPv6_ht*)(msg->payload))->type       = msg->l4_sourcePortORicmpv6Type;
   ((ICMPv6_ht*)(msg->payload))->code       = IANA_ICMPv6_RPL_DAO;
   packetfunctions_calculateChecksum(msg,(uint8_t*)&(((ICMPv6_ht*)(msg->payload))->checksum)); //call last

   //teste rff  DBGRPL - Aqui ele envia um DAO
#if (DEBUG_LOG_RIT  == 1) && (DBG_RPL == 1)
   {
		uint8_t pos=0;

		rffbuf[pos++]= RFF_ICMPv6RPL_TX;
		rffbuf[pos++]= 0x03;
		rffbuf[pos++]= msg->l2_frameType;
	    rffbuf[pos++]= icmpv6rpl_vars.busySendingDAO;
	    rffbuf[pos++]= icmpv6rpl_vars.CountSendingDAO;
		rffbuf[pos++]= 0xcc;
		rffbuf[pos++]= neighbors_vars.neighbors[0].addr_64b.addr_64b[6];
		rffbuf[pos++]= neighbors_vars.neighbors[0].addr_64b.addr_64b[7];
		rffbuf[pos++]= neighbors_vars.neighbors[0].numTx;
		rffbuf[pos++]= neighbors_vars.neighbors[0].numRx;
		rffbuf[pos++]= neighbors_vars.neighbors[0].numTxACK;
		rffbuf[pos++]= neighbors_vars.neighbors[0].numWraps;
		/*
		rffbuf[pos++]= msg->l3_destinationAdd.type;
		if (msg->l3_destinationAdd.type == 3)
		{
			rffbuf[pos++]= msg->l3_destinationAdd.addr_128b[14];
			rffbuf[pos++]= msg->l3_destinationAdd.addr_128b[15];
		}

		if (numTargetParents > 0)
        {
       	 uint8_t address_length=0;

            switch (address.type)
            {
               case ADDR_16B:
               case ADDR_PANID:
                  address_length = 2;
                  break;
               case ADDR_64B:
               case ADDR_PREFIX:
                  address_length = 8;
                  break;
               case ADDR_128B:
               case ADDR_ANYCAST:
                  address_length = 16;
                  break;
            }

            memcpy(&dao_address,&address,address_length);
        } */
        //teste rff



		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	}
   #endif
   //teste rff

   //===== send
   if (icmpv6_send(msg)==E_SUCCESS) {
      icmpv6rpl_vars.busySending = TRUE;
   } else {
      openqueue_freePacketBuffer(msg);
   }



}
