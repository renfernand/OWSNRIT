#include "opendefs.h"
#include "icmpv6.h"
#include "icmpv6echo.h"
#include "icmpv6rpl.h"
#include "forwarding.h"
#include "openqueue.h"
#include "openserial.h"
#include "IEEE802154E.h"
#include "IEEE802154RIT.h"

//=========================== variables =======================================
//=========================== prototypes ======================================

//=========================== public ==========================================

void icmpv6_init() {
}

owerror_t icmpv6_send(OpenQueueEntry_t* msg) {
   msg->owner       = COMPONENT_ICMPv6;
   msg->l4_protocol = IANA_ICMPv6;
   return forwarding_send(msg);
}

void icmpv6_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
#if 0 //(DEBUG_LOG_RIT  == 1)
   {
   	 uint8_t   pos=0;

	rffbuf[pos++]= RFF_ICMPv6RPL_TX;
	rffbuf[pos++]= 0x12;
	rffbuf[pos++]= msg->creator;
	rffbuf[pos++]= msg->owner;
	rffbuf[pos++]= msg->l4_sourcePortORicmpv6Type;
	rffbuf[pos++]= error;
	rffbuf[pos++]= msg->l2_frameType;
	rffbuf[pos++]= msg->l4_protocol;

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
   }
#endif

   msg->owner = COMPONENT_ICMPv6;
   switch (msg->l4_sourcePortORicmpv6Type) {
      case IANA_ICMPv6_ECHO_REQUEST:
      case IANA_ICMPv6_ECHO_REPLY:
         icmpv6echo_sendDone(msg, error);
         break;
      case IANA_ICMPv6_RPL:
         icmpv6rpl_sendDone(msg, error);
         break;
      default:
         openserial_printCritical(COMPONENT_ICMPv6,ERR_UNSUPPORTED_ICMPV6_TYPE,
                               (errorparameter_t)msg->l4_sourcePortORicmpv6Type,
                               (errorparameter_t)0);
         // free the corresponding packet buffer
         openqueue_freePacketBuffer(msg);
         break;
   }
}

void icmpv6_receive(OpenQueueEntry_t* msg) {
   msg->owner = COMPONENT_ICMPv6;
   msg->l4_sourcePortORicmpv6Type = ((ICMPv6_ht*)(msg->payload))->type;

#if (DEBUG_LOG_RIT  == 1) && (DBG_RPL == 1)
   {
   	 uint8_t   pos=0;

	rffbuf[pos++]= RFF_ICMPv6RPL_RX;
	rffbuf[pos++]= 0x01;
	rffbuf[pos++]= msg->l4_protocol;
	rffbuf[pos++]= msg->creator;
	rffbuf[pos++]= msg->owner;
	rffbuf[pos++]= msg->l2_sendDoneError;
	rffbuf[pos++]= msg->l2_frameType;

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
   }
#endif

   switch (msg->l4_sourcePortORicmpv6Type) {
      case IANA_ICMPv6_ECHO_REQUEST:
      case IANA_ICMPv6_ECHO_REPLY:
         icmpv6echo_receive(msg);
         break;
      case IANA_ICMPv6_RPL:
         icmpv6rpl_receive(msg);
         break;
      default:
         openserial_printError(COMPONENT_ICMPv6,ERR_UNSUPPORTED_ICMPV6_TYPE,
                               (errorparameter_t)msg->l4_sourcePortORicmpv6Type,
                               (errorparameter_t)1);
         // free the corresponding packet buffer
         openqueue_freePacketBuffer(msg);
         break;
   }
}
