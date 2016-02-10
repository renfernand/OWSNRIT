#include "opendefs.h"
#include "openbridge.h"
#include "openserial.h"
#include "packetfunctions.h"
#include "iphc.h"
#include "idmanager.h"
#include "openqueue.h"
#include "leds.h"
#include "IEEE802154E.h"

//=========================== variables =======================================
#if (DEBUG_LOG_RIT  == 1)
extern openserial_vars_t openserial_vars;
extern ieee154e_vars_t    ieee154e_vars;
extern ieee154e_stats_t   ieee154e_stats;
extern ieee154e_dbg_t     ieee154e_dbg;
static uint8_t rffbuf[10];
#define TESTE_RIT_GENERATE_DATA_MSG  0

#endif
//=========================== prototypes ======================================
bool RITQueue_ExistFramePending(void);
//=========================== public ==========================================
extern uint8_t  coappending;

void openbridge_init() {
}

#if (SINK_SIMULA_COAP == 1)
void openbridge_simucoap() {
   uint8_t           input_buffer[136]={0x10, 0x10, 0x00, 0x2B, 0xC0, 0x56, 0x00, 0x20, 0x3E, 0xFA, 0x00, 0x2B, 0x9B, 0x00, 0x3B, 0x6F, 0xDB, 0x68, 0x00, 0x2B, 0x28, 0x00, 0xFF, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0xBB, 0xBB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x12, 0x4B, 0x00, 0x04, 0x0E, 0xFC, 0x87, 0x00, 0x02, 0x00, 0x12, 0x4B, 0x00, 0x04, 0x0E, 0xFC, 0x87, 0x51, 0x00, 0x7E, 0x68, 0x7F, 0x00, 0x20, 0x00, 0x05};
   OpenQueueEntry_t* pkt;
   uint8_t           numDataBytes;

   numDataBytes = 70;


#if (NEW_DAG_BRIDGE == 1)
   if (idmanager_getIsDAGroot()==TRUE && numDataBytes>0) {
#else
   if (idmanager_getIsBridge()==TRUE && numDataBytes>0) {
#endif
#if (SINK == 1)
	  //leds_debug_toggle();
#endif

      pkt = openqueue_getFreePacketBuffer(COMPONENT_OPENBRIDGE);
      if (pkt==NULL) {
         openserial_printError(COMPONENT_OPENBRIDGE,ERR_NO_FREE_PACKET_BUFFER,
                               (errorparameter_t)0,
                               (errorparameter_t)0);
         return;
      }
      //admin
      pkt->creator  = COMPONENT_OPENBRIDGE;
      pkt->owner    = COMPONENT_OPENBRIDGE;
      //l2
      pkt->l2_nextORpreviousHop.type = ADDR_64B;
      memcpy(&(pkt->l2_nextORpreviousHop.addr_64b[0]),&(input_buffer[0]),8);
      //payload
      packetfunctions_reserveHeaderSize(pkt,numDataBytes-8);
      memcpy(pkt->payload,&(input_buffer[8]),numDataBytes-8);

      //this is to catch the too short packet. remove it after fw-103 is solved.
      if (numDataBytes<16){
              openserial_printError(COMPONENT_OPENBRIDGE,ERR_INVALIDSERIALFRAME,
                            (errorparameter_t)0,
                            (errorparameter_t)0);
      }

#if ENABLE_DEBUG_RFF
	   {
 			uint8_t pos=0;

			 rffbuf[pos++]= 0x75;
			 rffbuf[pos++]= 0x01;
			 rffbuf[pos++]= numDataBytes;
			 rffbuf[pos++]= input_buffer[8];
			 rffbuf[pos++]= input_buffer[9];
			 rffbuf[pos++]= input_buffer[10];
			 rffbuf[pos++]= input_buffer[11];
			 rffbuf[pos++]= input_buffer[12];
			 rffbuf[pos++]= input_buffer[13];
			 rffbuf[pos++]= input_buffer[14];
			 rffbuf[pos++]= input_buffer[15];

			openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	   }
#endif
	   //send
      if ((iphc_sendFromBridge(pkt))==E_FAIL) {
         openqueue_freePacketBuffer(pkt);
      }
   }
}
#endif

void openbridge_triggerData() {
   uint8_t           input_buffer[136];//worst case: 8B of next hop + 128B of data
   OpenQueueEntry_t* pkt;
   uint8_t           numDataBytes;
   uint8_t           isFramePending=0;

   numDataBytes = openserial_getNumDataBytes();
  
   //poipoi xv
   //this is a temporal workaround as we are never supposed to get chunks of data
   //longer than input buffer size.. I assume that HDLC will solve that.
   // MAC header is 13B + 8 next hop so we cannot accept packets that are longer than 118B
   if (numDataBytes>(136 - 21) || numDataBytes<8){
   //to prevent too short or too long serial frames to kill the stack  
       openserial_printError(COMPONENT_OPENBRIDGE,ERR_INPUTBUFFER_LENGTH,
                   (errorparameter_t)numDataBytes,
                   (errorparameter_t)0);
       return;
   }
  
   //copying the buffer once we know it is not too big
   openserial_getInputBuffer(&(input_buffer[0]),numDataBytes);
#if (NEW_DAG_BRIDGE == 1)
   if (idmanager_getIsDAGroot()==TRUE && numDataBytes>0) {
#else
   if (idmanager_getIsBridge()==TRUE && numDataBytes>0) {
#endif
#if (SINK == 1)
	  //leds_debug_toggle();
#endif

      pkt = openqueue_getFreePacketBuffer(COMPONENT_OPENBRIDGE);
      if (pkt==NULL) {
         openserial_printError(COMPONENT_OPENBRIDGE,ERR_NO_FREE_PACKET_BUFFER,
                               (errorparameter_t)0,
                               (errorparameter_t)0);
         return;
      }
      //admin
      pkt->creator  = COMPONENT_OPENBRIDGE;
      pkt->owner    = COMPONENT_OPENBRIDGE;
      //l2
      pkt->l2_nextORpreviousHop.type = ADDR_64B;
      memcpy(&(pkt->l2_nextORpreviousHop.addr_64b[0]),&(input_buffer[0]),8);
      //payload
      packetfunctions_reserveHeaderSize(pkt,numDataBytes-8);
      memcpy(pkt->payload,&(input_buffer[8]),numDataBytes-8);
      
      //this is to catch the too short packet. remove it after fw-103 is solved.
      if (numDataBytes<16){
		  openserial_printError(COMPONENT_OPENBRIDGE,ERR_INVALIDSERIALFRAME,
						(errorparameter_t)0,
						(errorparameter_t)0);
      }

#if (IEEE802154E_RIT == 1)
		isFramePending = 0;
		/* indica que eh um frame RPL.DIO
		* se existir um frame de dados pendente..eu ignoro o RPL.DIO
		* TODO!!!! usar um metodo melhor (ao inves do tamanho do frame) para descobrir se o frame eh RPL...
		*/
		if (numDataBytes == 40) {
 		   isFramePending = RITQueue_ExistFramePending();
		}
#endif
	   //send
	  if (isFramePending == 0) {
			#if ENABLE_DEBUG_RFF
				   {
						uint8_t pos=0;

						 rffbuf[pos++]= RFF_OPENBRIDGE_TX;
						 rffbuf[pos++]= 0x01;
						 rffbuf[pos++]= numDataBytes;
						 rffbuf[pos++]= input_buffer[8];
						 rffbuf[pos++]= input_buffer[9];
						 rffbuf[pos++]= input_buffer[10];
						 rffbuf[pos++]= input_buffer[11];
						 rffbuf[pos++]= input_buffer[12];
						 rffbuf[pos++]= input_buffer[13];
						 rffbuf[pos++]= input_buffer[14];
						 rffbuf[pos++]= input_buffer[15];

						openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
				   }
			#endif

		  if ((iphc_sendFromBridge(pkt))==E_FAIL) {
	         openqueue_freePacketBuffer(pkt);
	      }
	  }
	  else
	  {
		#if ENABLE_DEBUG_RFF
		   {
				uint8_t pos=0;

				 rffbuf[pos++]= 0xFF;
				 rffbuf[pos++]= 0xFF;
				 rffbuf[pos++]= numDataBytes;
				 rffbuf[pos++]= input_buffer[8];
				 rffbuf[pos++]= input_buffer[9];
				 rffbuf[pos++]= input_buffer[10];
				 rffbuf[pos++]= input_buffer[11];
				 rffbuf[pos++]= input_buffer[12];
				 rffbuf[pos++]= input_buffer[13];
				 rffbuf[pos++]= input_buffer[14];
				 rffbuf[pos++]= input_buffer[15];

				openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
		   }
		#endif

	  }
   }
}

void openbridge_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
   msg->owner = COMPONENT_OPENBRIDGE;
   if (msg->creator!=COMPONENT_OPENBRIDGE) {
      openserial_printError(COMPONENT_OPENBRIDGE,ERR_UNEXPECTED_SENDDONE,
                            (errorparameter_t)0,
                            (errorparameter_t)0);
   }
   openqueue_freePacketBuffer(msg);
}

/**
\brief Receive a frame at the openbridge, which sends it out over serial.
*/
void openbridge_receive(OpenQueueEntry_t* msg) {
   
   // prepend previous hop
   packetfunctions_reserveHeaderSize(msg,LENGTH_ADDR64b);
   memcpy(msg->payload,msg->l2_nextORpreviousHop.addr_64b,LENGTH_ADDR64b);
   
   // prepend next hop (me)
   packetfunctions_reserveHeaderSize(msg,LENGTH_ADDR64b);
   memcpy(msg->payload,idmanager_getMyID(ADDR_64B)->addr_64b,LENGTH_ADDR64b);
   

#if ENABLE_DEBUG_RFF
{
	uint8_t pos=0;

	 rffbuf[pos++]= RFF_OPENBRIDGE_RX;
	 rffbuf[pos++]= 0x01;
	 rffbuf[pos++]= msg->length;

#if (IEEE802154E_TSCH == 0)
	 if (msg->length > 69)
		 coappending = 0;
#endif

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
}
#endif
   // send packet over serial (will be memcopied into serial buffer)
   openserial_printData((uint8_t*)(msg->payload),msg->length);
   
   // free packet
   openqueue_freePacketBuffer(msg);
}

//=========================== private =========================================
