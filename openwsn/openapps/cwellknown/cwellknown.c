#include "board.h"
#include "opendefs.h"
#include "cwellknown.h"
#include "opencoap.h"
#include "openqueue.h"
#include "packetfunctions.h"
#include "openserial.h"
#include "idmanager.h"
#include "debug.h"
#include "leds.h"

#if 0
//=========================== variables =======================================

cwellknown_vars_t cwellknown_vars;

const uint8_t cwellknown_path0[]       = ".well-known";
const uint8_t cwellknown_path1[]       = "core";

//=========================== prototypes ======================================

owerror_t cwellknown_receive(
   OpenQueueEntry_t* msg,
   coap_header_iht*  coap_header,
   coap_option_iht*  coap_options
);

void    cwellknown_sendDone(
   OpenQueueEntry_t* msg,
   owerror_t         error
);

//=========================== public ==========================================

void cwellknown_init() {
   if(idmanager_getIsDAGroot()==TRUE) return; 
   
   // prepare the resource descriptor for the /.well-known/core path
   cwellknown_vars.desc.path0len            = sizeof(cwellknown_path0)-1;
   cwellknown_vars.desc.path0val            = (uint8_t*)(&cwellknown_path0);
   cwellknown_vars.desc.path1len            = sizeof(cwellknown_path1)-1;
   cwellknown_vars.desc.path1val            = (uint8_t*)(&cwellknown_path1);
   cwellknown_vars.desc.componentID         = COMPONENT_CWELLKNOWN;
   cwellknown_vars.desc.callbackRx          = &cwellknown_receive;
   cwellknown_vars.desc.callbackSendDone    = &cwellknown_sendDone;
   
   opencoap_register(&cwellknown_vars.desc);
}

//=========================== private =========================================

owerror_t cwellknown_receive(
      OpenQueueEntry_t* msg,
      coap_header_iht*  coap_header,
      coap_option_iht*  coap_options
   ) {
   owerror_t outcome;
   
   switch(coap_header->Code) {
      case COAP_CODE_REQ_GET:
         // reset packet payload
         msg->payload        = &(msg->packet[127]);
         msg->length         = 0;
         
         // have CoAP module write links to all resources
         opencoap_writeLinks(msg);
         
         packetfunctions_reserveHeaderSize(msg,1);
         msg->payload[0]     = COAP_PAYLOAD_MARKER;
            
         // add return option
         packetfunctions_reserveHeaderSize(msg,2);
         msg->payload[0]     = COAP_OPTION_NUM_CONTENTFORMAT << 4 | 1;
         msg->payload[1]     = COAP_MEDTYPE_APPLINKFORMAT;
         
         // set the CoAP header
         coap_header->Code   = COAP_CODE_RESP_CONTENT;
         
         outcome             = E_SUCCESS;
         
         break;
      default:
         outcome             = E_FAIL;
         break;
   }
   
   return outcome;
}

void cwellknown_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
   openqueue_freePacketBuffer(msg);
}
#else
//=========================== variables =======================================

cwellknown_vars_t cwellknown_vars;
RIT_stats_t ritstat;

const uint8_t cwellknown_path0[]       = "d";
const uint8_t cwellknown_path1[]       = " ";

//=========================== prototypes ======================================

owerror_t cwellknown_receive(
   OpenQueueEntry_t* msg,
   coap_header_iht*  coap_header,
   coap_option_iht*  coap_options
);

void    cwellknown_sendDone(
   OpenQueueEntry_t* msg,
   owerror_t         error
);

//=========================== public ==========================================

void cwellknown_init() {
   if(idmanager_getIsDAGroot()==TRUE) return;

   // prepare the resource descriptor for the /.well-known/core path
   cwellknown_vars.desc.path0len            = sizeof(cwellknown_path0)-1;
   cwellknown_vars.desc.path0val            = (uint8_t*)(&cwellknown_path0);
   cwellknown_vars.desc.path1len            = 0;
   cwellknown_vars.desc.path1val            = NULL;
   cwellknown_vars.desc.componentID         = COMPONENT_CWELLKNOWN;
   cwellknown_vars.desc.callbackRx          = &cwellknown_receive;
   cwellknown_vars.desc.callbackSendDone    = &cwellknown_sendDone;

   opencoap_register(&cwellknown_vars.desc);
}

//=========================== private =========================================
uint8_t printvar1(uint8_t *var,uint8_t size, uint8_t *buf,uint8_t pos){

	//buf[pos++]= 0xBB;

	switch(size)
	{
		case 1:
			buf[pos++]= *var;
			break;
		case 2: //inverti...agora eh formato big endian
			buf[pos++]= *(var+1);
			buf[pos++]= *(var+0);
			break;
		case 4: //inverti...agora eh formato big endian
			buf[pos++]= *(var+3);
			buf[pos++]= *(var+2);
			buf[pos++]= *(var+1);
			buf[pos++]= *(var+0);
			break;
		default:
			break;
	}

    return pos;
}

owerror_t cwellknown_receive(
      OpenQueueEntry_t* msg,
      coap_header_iht*  coap_header,
      coap_option_iht*  coap_options
   ) {
   owerror_t outcome;
   uint8_t pos=0;

   switch(coap_header->Code) {
      case COAP_CODE_REQ_GET:
         // reset packet payload
         msg->payload        = &(msg->packet[127]);
         msg->length         = 0;

         // have CoAP module write links to all resources
         opencoap_writeLinks(msg);

         packetfunctions_reserveHeaderSize(msg,1);
         msg->payload[0]     = COAP_PAYLOAD_MARKER;

         // add return option
         packetfunctions_reserveHeaderSize(msg,21);
#if 1 //debug RFF
		 msg->payload[pos++] = 0xCA;
		 msg->payload[pos++] = 0xCA;
		 pos = printvar1((uint8_t *)&ritstat.txola.countdatatx,sizeof(uint16_t),msg->payload,pos);
		 pos = printvar1((uint8_t *)&ritstat.txola.countdatatxok,sizeof(uint16_t),msg->payload,pos);
		 msg->payload[pos++] = 0xD1;
		 pos = printvar1((uint8_t *)&ritstat.txdio.countdatatx,sizeof(uint16_t),msg->payload,pos);
		 pos = printvar1((uint8_t *)&ritstat.txdio.countdatatxok,sizeof(uint16_t),msg->payload,pos);
		 msg->payload[pos++]=0xDA;
		 pos = printvar1((uint8_t *)&ritstat.txdao.countdatatx,sizeof(uint16_t),msg->payload,pos);
		 pos = printvar1((uint8_t *)&ritstat.txdao.countdatatxok,sizeof(uint16_t),msg->payload,pos);
		 msg->payload[pos++]=0xCA;
		 pos = printvar((uint8_t *)&ritstat.txcoap.countdatatx,sizeof(uint16_t),msg->payload,pos);
		 pos = printvar((uint8_t *)&ritstat.txcoap.countdatatxok,sizeof(uint16_t),msg->payload,pos);

#else
         msg->payload[0]     = COAP_OPTION_NUM_CONTENTFORMAT << 4 | 1;
         msg->payload[1]     = COAP_MEDTYPE_APPLINKFORMAT;
#endif
         // set the CoAP header
         coap_header->Code   = COAP_CODE_RESP_CONTENT;

         outcome             = E_SUCCESS;

         break;
      default:
         outcome             = E_FAIL;
         break;
   }

   return outcome;
}

void cwellknown_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
   openqueue_freePacketBuffer(msg);
}

#endif
