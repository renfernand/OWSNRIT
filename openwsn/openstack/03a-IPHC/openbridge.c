#include "opendefs.h"
#include "openbridge.h"
#include "openserial.h"
#include "packetfunctions.h"
#include "iphc.h"
#include "idmanager.h"
#include "openqueue.h"
#include "leds.h"
#include "neighbors.h"
#include "topology.h"
#include "IEEE802154E.h"
#if IEEE802154E_RITMC == 1
#include "IEEE802154RITMC.h"
#elif IEEE802154E_AMAC == 1
#include "IEEE802154AMAC.h"
#elif IEEE802154E_AMCA == 1
#include "IEEE802154AMCA.h"
#elif IEEE802154E_RIT == 1
#include "IEEE802154RIT.h"
#endif
#include "debug.h"
//=========================== variables =======================================
#if (DEBUG_LOG_RIT  == 1)
extern openserial_vars_t openserial_vars;
extern ieee154e_vars_t    ieee154e_vars;
extern ieee154e_stats_t   ieee154e_stats;
extern ieee154e_dbg_t     ieee154e_dbg;
uint8_t coapcountrx=0;
RIT_stats_t ritstat;

#endif
//=========================== prototypes ======================================
bool RITQueue_ExistFramePending(void);
//=========================== public ==========================================
extern uint8_t  coappending;

void openbridge_init() {
}

#if (SINK_SIMULA_COAP == 1)

#if (MOTE_QTDE_SALTOS == 1)
	/* FRAME COAP 1 SALTO - PARA Nó M1 */
	#define SIMU_COAP_MOTE    MOTE1

	#if SIMU_WELL_KNOWN
		//msg /.well-known
		#define SIMU_COAP_LEN     69
		const uint8_t frmsimucoap[SIMU_COAP_LEN] = {
			0x00,0x12,0x4B,0x00,0x02,0xf4,0xAC,0x09,0x78,0x00,0x11,0x80,
			0xBB,0xBB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,
			0xBB,0xBB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0x4B,0x00,0x02,0xF4,0xAC,0x09,
			0xDC,0x0E,
			0x16,0x33,0x00,0x19,
			0x3C,0xC9,
			0x41,0x01,
			0x1b,0x40,0xbb,
			0xBB,0x2e,0x77,0x65,0x6c,0x6c,0x2d,0x6b,0x6e,0x6f,0x77,0x6e
			 /*    .    w    e    l    l    -    k    n    o    w    n   */
		};
	#else
		//msg /d
		#define SIMU_COAP_LEN     59
		const uint8_t frmsimucoap[SIMU_COAP_LEN] = {
			0x00,0x12,0x4B,0x00,0x02,0xf4,0xAC,0x09,0x78,0x00,0x11,0x80,
			0xBB,0xBB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,
			0xBB,0xBB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0x4B,0x00,0x02,0xF4,0xAC,0x09,
			0xDC,0x0E,
			0x16,0x33,0x00,0x0F,
			0x3C,0xC9,
			0x41,0x01,
			0x5F,0x8A,0x5A,
			0xB1,0x64
		};
	#endif

#elif (MOTE_QTDE_SALTOS == 2)

	#define SIMU_COAP_MOTE    MOTE2

	#if SIMU_WELL_KNOWN
		#define SIMU_COAP_LEN     85
		const uint8_t frmsimucoap[SIMU_COAP_LEN] = {
			0x00,0x12,0x4B,0x00,0x02,0xf4,0xAC,0x09,0x78,0x00,0x2B,0x80,
			0xBB,0xBB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,
			0xBB,0xBB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0x4B,0x00,0x02,0xF4,0xAC,0x09,
			0x11,0x01,0x03,0x01,
			0x88,0x00,0x00,0x00,
			0x00,0x12,0x4B,0x00,0x02,0xF4,0xAF,0xC0,
			0xF2,0x38,
			0x16,0x33,0x00,0x19,
			0xe4,0xab,
			0x41,0x01,
			0xd4,0x6e,0x5d,
			0xBB,0x2e,0x77,0x65,0x6c,0x6c,0x2d,0x6b,0x6e,0x6f,0x77,0x6e
		};
	#else
		#define SIMU_COAP_LEN     75
		const uint8_t frmsimucoap[SIMU_COAP_LEN] = {
			0x00,0x12,0x4B,0x00,0x02,0xf4,0xAC,0x09,0x78,0x00,0x2B,0x80,
			0xBB,0xBB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,
			0xBB,0xBB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0x4B,0x00,0x02,0xF4,0xAC,0x09,
			0x11,0x01,0x03,0x01,
			0x88,0x00,0x00,0x00,
			0x00,0x12,0x4B,0x00,0x02,0xF4,0xAF,0xC0,
			0xF2,0x38,
			0x16,0x33,0x00,0x0F,
			0xFD,0x78,
			0x41,0x01,
			0x8C,0xF9,0x52,
			0xB1,0x64
		};

	#endif

#elif (MOTE_QTDE_SALTOS == 3)

	#define SIMU_COAP_MOTE    MOTE3

	#if SIMU_WELL_KNOWN

		#define SIMU_COAP_LEN     93
		const uint8_t frmsimucoap[SIMU_COAP_LEN] = {
			0x00,0x12,0x4B,0x00,0x02,0xf4,0xAC,0x09,0x78,0x00,0x2B,0x80,
			0xBB,0xBB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,
			0xBB,0xBB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0x4B,0x00,0x02,0xF4,0xAC,0x09,
			0x11,0x02,0x03,0x02,
			0x88,0x00,0x00,0x00,
			0x00,0x12,0x4b,0x00,0x04,0x0e,0xfc,0x87,
			0x00,0x12,0x4b,0x00,0x02,0xf4,0xaf,0xc0,
			0xf8,0xb6,
			0x16,0x33,0x00,0x19,
			0xbc,0x27,
			0x41,0x01,
			0xd4,0x6e,0x5d,
			0xBB,0x2e,0x77,0x65,0x6c,0x6c,0x2d,0x6b,0x6e,0x6f,0x77,0x6e
		};
	#else
		#define SIMU_COAP_LEN     83
		const uint8_t frmsimucoap[SIMU_COAP_LEN] = {
			0x00,0x12,0x4B,0x00,0x02,0xf4,0xAC,0x09,0x78,0x00,0x2B,0x80,
			0xBB,0xBB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,
			0xBB,0xBB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0x4B,0x00,0x02,0xF4,0xAC,0x09,
			0x11,0x02,0x03,0x02,
			0x88,0x00,0x00,0x00,
			0x00,0x12,0x4b,0x00,0x04,0x0e,0xfc,0x87,
			0x00,0x12,0x4b,0x00,0x02,0xf4,0xaf,0xc0,
			0xf8,0xb6,
			0x16,0x33,0x00,0x0f,
			0xbc,0x27,
			0x41,0x01,
			0xa4,0xeb,0x27,
			0xb1,0x64
		};
	#endif


#elif (MOTE_QTDE_SALTOS == 4)
#define SIMU_COAP_MOTE    MOTE4
#define SIMU_COAP_LEN     91
const uint8_t frmsimucoap[SIMU_COAP_LEN] = {
	0x00,0x12,0x4B,0x00,0x02,0xf4,0xAC,0x09,0x78,0x00,0x2B,0x80,
	0xBB,0xBB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,
	0xBB,0xBB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0x4B,0x00,0x02,0xF4,0xAC,0x09,
	0x11,0x03,0x03,0x03,
	0x88,0x00,0x00,0x00,
	0x00,0x12,0x4b,0x00,0x03,0xa6,0x51,0x52,
	0x00,0x12,0x4b,0x00,0x04,0x0e,0xfc,0x87,
	0x00,0x12,0x4b,0x00,0x02,0xf4,0xaf,0xc0,
	0xfa,0x8a,
	0x16,0x33,0x00,0x0f,
	0x41,0x52,
	0x41,0x01,
	0xf4,0x89,0xfc,
	0xb1,0x64
};
#endif

uint8_t openbridge_simucoap (void) {
	OpenQueueEntry_t* pkt;
	uint8_t           numDataBytes=SIMU_COAP_LEN;
#if 0 //(MOTE_QTDE_SALTOS == 1)
	uint8_t frmsimucoap[51] = {	0x78,0x00,0x11,0x80,
								0xBB,0xBB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,
								0xBB,0xBB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0x4B,0x00,0x02,0xF4,0xAC,0x09,
								0xDC,0x0E,0x16,0x33,0x00,0x0F,0x3C,0xC9,0x41,0x01,0x5F,0x8A,0x5A,0xB1,0x64};
#endif
	open_addr_t      dstaddr;

	pkt = openqueue_getFreePacketBuffer(COMPONENT_OPENBRIDGE);
	if (pkt==NULL) {
	 openserial_printError(COMPONENT_OPENBRIDGE,ERR_NO_FREE_PACKET_BUFFER,
						   (errorparameter_t)0,
						   (errorparameter_t)0);
	 return 0;
	}

	//admin
	pkt->creator  = COMPONENT_OPENBRIDGE;
	pkt->owner    = COMPONENT_OPENBRIDGE;

	//l2
	pkt->l2_nextORpreviousHop.type = ADDR_64B;

	if (getaddressMote(&dstaddr,ADDR_64B,SIMU_COAP_MOTE) == FALSE){
		 openserial_printError(COMPONENT_OPENBRIDGE,ERR_NO_FREE_PACKET_BUFFER,
							   (errorparameter_t)0,
							   (errorparameter_t)0);
	     return 0;
	}

#if (MOTE_QTDE_SALTOS == 1)
	//memcpy(&(pkt->l2_nextORpreviousHop.addr_64b[0]),&dstaddr.addr_64b[0],8);
	//packetfunctions_reserveHeaderSize(pkt,numDataBytes-8);
	//memcpy(pkt->payload,&(frmsimucoap[0]),numDataBytes-8);
	//memcpy((pkt->payload+28),&(dstaddr.addr_64b[0]),8);

	memcpy(&(pkt->l2_nextORpreviousHop.addr_64b[0]),&(frmsimucoap[0]),8);
	packetfunctions_reserveHeaderSize(pkt,numDataBytes-8);
	memcpy(pkt->payload,&(frmsimucoap[8]),numDataBytes-8);

#endif

#if (MOTE_QTDE_SALTOS == 2)
    memcpy(&(pkt->l2_nextORpreviousHop.addr_64b[0]),&(frmsimucoap[0]),8);
	packetfunctions_reserveHeaderSize(pkt,numDataBytes-8);
	memcpy(pkt->payload,&(frmsimucoap[8]),numDataBytes-8);
#endif

	if ((iphc_sendFromBridge(pkt))==E_FAIL) {
       openqueue_freePacketBuffer(pkt);
    }

#if 1 //((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_TX == 1))
  {
	uint8_t   pos=0;

	rffbuf[pos++]= 0xCA;
	rffbuf[pos++]= 0xCA;
	rffbuf[pos++]= 0x10;
	rffbuf[pos++]= numDataBytes;

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
  }
#endif
  coappending = TRUE;
  ritstat.txcoap.countdata++;
}

void printdatasimu(uint8_t* buffer, uint8_t length) {

   coapcountrx++;

#if 1 //((ENABLE_DEBUG_RFF == 1) && (DBG_IEEE802_TX == 1))
  {
	uint8_t   pos=0;

	rffbuf[pos++]= 0xCA;
	rffbuf[pos++]= 0xCA;
	rffbuf[pos++]= 0xCA;
	rffbuf[pos++]= 0x15;
	rffbuf[pos++]= length;
	pos = printvar((uint8_t *)&ritstat.txcoap.countdata,sizeof(uint16_t),rffbuf,pos);
	pos = printvar((uint8_t *)&coapcountrx,sizeof(uint16_t),rffbuf,pos);

	openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
  }
#endif

}
#endif

void openbridge_triggerData() {
   uint8_t           input_buffer[136];//worst case: 8B of next hop + 128B of data
   OpenQueueEntry_t* pkt;
   uint8_t           numDataBytes;
   uint8_t           isFramePending=0;

   numDataBytes = openserial_getInputBufferFilllevel();
  
   //poipoi xv
   //this is a temporal workaround as we are never supposed to get chunks of data
   //longer than input buffer size.. I assume that HDLC will solve that.
   // MAC header is 13B + 8 next hop so we cannot accept packets that are longer than 118B
   if (numDataBytes>(136 - 10/*21*/) || numDataBytes<8){
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

				if (numDataBytes == 0x28) {  //TX FRAME DIO
					 rffbuf[pos++]= RFF_OPENBRIDGE_TX;
					 rffbuf[pos++]= 0x01;
				}
				else{  //TX_COAP
					 rffbuf[pos++]= RFF_OPENBRIDGE_TX;
					 rffbuf[pos++]= 0xCA;
					 rffbuf[pos++]= 0xCA;
					 rffbuf[pos++]= 0xCA;
					 rffbuf[pos++]= 0xCA;
				}
				 rffbuf[pos++]= numDataBytes;
				/*
				 rffbuf[pos++]= input_buffer[8];
				 rffbuf[pos++]= input_buffer[9];
				 rffbuf[pos++]= input_buffer[10];
				 rffbuf[pos++]= input_buffer[11];
				 rffbuf[pos++]= input_buffer[12];
				 rffbuf[pos++]= input_buffer[13];
				 rffbuf[pos++]= input_buffer[14];
				 rffbuf[pos++]= input_buffer[15];
				*/
				openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);

				openserial_startOutput();
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

#if ENABLE_DEBUG_RFF
   {
		uint8_t pos=0;
		if (coappending == TRUE) {

			ritstat.txcoap.countacktxrxok++;

			rffbuf[pos++]= 0xCA;
			rffbuf[pos++]= 0x19;
			rffbuf[pos++]= msg->length;
		}
		else {
			rffbuf[pos++]= RFF_OPENBRIDGE_TX;
			rffbuf[pos++]= 0x05;
			rffbuf[pos++]= msg->length;
		}

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
   }
#endif
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
	uint8_t *pauxframe;

	pauxframe = (uint8_t *) &msg->packet[0];

	#if (IEEE802154E_TSCH == 0)
		 if (msg->length > 69)
			 coappending = 0;
	#endif
	 if (msg->length < 0x50) {
		rffbuf[pos++]= RFF_OPENBRIDGE_RX;
 		rffbuf[pos++]= 0xCA;  //COAP RESPONSE
 		rffbuf[pos++]= 0xCA;  //COAP RESPONSE
 		rffbuf[pos++]= 0xCA;  //COAP RESPONSE
 		rffbuf[pos++]= 0xCA;  //COAP RESPONSE
	 }
	 else{
		rffbuf[pos++]= RFF_OPENBRIDGE_RX;
		rffbuf[pos++]= 0xDA;  //DAO  RESPONSE
		rffbuf[pos++]= 0xDA;  //DAO  RESPONSE
		rffbuf[pos++]= 0xDA;  //DAO  RESPONSE
		rffbuf[pos++]= 0xDA;  //DAO  RESPONSE
	 }
	 rffbuf[pos++]= msg->length;
	 rffbuf[pos++]= pauxframe[1];
	 rffbuf[pos++]= pauxframe[2];


	 openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
}
#endif

#if (SINK_SIMULA_COAP == 1)
	 if (msg->length > 69) { //COAP RESPONSE
		 printdatasimu((uint8_t*)(msg->payload),msg->length);
	 }
	 else {  //DIO RESPONSE
	   // send packet over serial (will be memcopied into serial buffer)
	   openserial_printData((uint8_t*)(msg->payload),msg->length);
	 }

#else
   // send packet over serial (will be memcopied into serial buffer)
   openserial_printData((uint8_t*)(msg->payload),msg->length);
#endif
   // free packet
   openqueue_freePacketBuffer(msg);
}

//=========================== private =========================================
