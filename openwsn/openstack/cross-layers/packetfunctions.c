#include "packetfunctions.h"
#include "leds.h"
#include "openserial.h"
#include "idmanager.h"
#include "iphc.h"
//#include "opendefs.h"
#include "IEEE802154.h"
//#include "IEEE802154RIT.h"
#include "neighbors.h"
#include "radio.h"
#include "debug.h"
#include <string.h>
#if IEEE802154E_RITMC == 1
#include "IEEE802154RITMC.h"
#elif IEEE802154E_AMAC == 1
#include "IEEE802154AMAC.h"
#elif IEEE802154E_AMCA == 1
#include "IEEE802154AMCA.h"
#elif IEEE802154E_RIT == 1
#include "IEEE802154RIT.h"
#endif
//=========================== variables =======================================
extern OpenQueueEntry_t advRIT;
uint8_t livelistasn;
uint8_t olaasn;
extern RIT_stats_t ritstat;
extern ieee154e_vars_t    ieee154e_vars;
extern uint16_t absdutycyclems;          //variavel global pois sera utilizada no frame de diagnostico...no proximo ciclo...ele vai ter o ultimo valor
extern uint16_t relativedutycyclems;    //variavel global pois sera utilizada no frame de diagnostico...no proximo ciclo...ele vai ter o ultimo valor
extern uint8_t dutycyclecount;
uint32_t txwaitola;

#if (IEEE802154E_RITMC == 1)
extern uint16_t ritmc_txolaack;
extern uint16_t ritmc_rxolaack;
extern uint16_t ritmc_rx04err;
extern ecw_vars_t ecwvars;
#endif

#if (IEEE802154E_ARM == 1) || (IEEE802154E_AMAC == 1)
extern uint16_t counttxrts;
extern uint16_t countrxrts;
extern uint16_t counttxcts;
extern uint16_t countrxcts;
extern uint16_t countrxHA;
extern uint16_t counttxcw;
extern uint16_t countrxcw;
#endif
//=========================== prototypes ======================================

void onesComplementSum(uint8_t* global_sum, uint8_t* ptr, int length);
void iphc_retrieveIPv6Header(OpenQueueEntry_t* msg, ipv6_header_iht* ipv6_header);
//=========================== public ==========================================
//Aqui eu retiro o frame type (DIO,DAO, COAP) para propositos de debug apenas...PROVISORIO!!!!!
uint8_t parserframerx(OpenQueueEntry_t *msg) {
     ipv6_header_iht      ipv6_header;
	 //ipv6_hopbyhop_iht    ipv6_hop_header;
	 //rpl_option_ht        rpl_option;
	 uint8_t frametype=0;

	 //  memcpy(&rxaddr_dst, &(ieee802514_header.dest),sizeof(open_addr_t));
	 //  memcpy(&newsrcaddr, &(ieee802514_header.src),sizeof(open_addr_t));

	 if (msg->l2_frameType == IEEE154_TYPE_DATA) {
		  if (packetfunctions_isBroadcastMulticast(&msg->l2_nextORpreviousHop)){
            frametype = IANA_ICMPv6_RPL_DIO;

			ipv6_header.header_length = 0;
			ipv6_header.next_header = 0;
			ipv6_header.next_header_compressed = 0;
			ipv6_header.traffic_class = 0;
		  }
		  else { //if (idmanager_isMyAddress(rxaddr_dst) && Ackrequest){
			  //DAO OR COAP
			iphc_retrieveIPv6Header(msg,&ipv6_header);

			if ((ipv6_header.next_header == IANA_UDP) || (ipv6_header.next_header == IANA_IPv6ROUTE))
	            frametype = IANA_UDP;
			else
				frametype = IANA_ICMPv6_RPL_DAO;
		  }

	 }
	 else if ((msg->l2_frameType == IEEE154_TYPE_CMD) && (msg->packet[12] == 0xA1)) {
		 frametype = RFF_LIVELIST;
	 }


	return frametype;

}




//======= address translation

//assuming an ip128b is a concatenation of prefix64b followed by a mac64b
void packetfunctions_ip128bToMac64b(
      open_addr_t* ip128b,
      open_addr_t* prefix64btoWrite,
      open_addr_t* mac64btoWrite) {
   if (ip128b->type!=ADDR_128B) {
      openserial_printCritical(COMPONENT_PACKETFUNCTIONS,ERR_WRONG_ADDR_TYPE,
                            (errorparameter_t)ip128b->type,
                            (errorparameter_t)0);
      mac64btoWrite->type=ADDR_NONE;
      return;
   }
   prefix64btoWrite->type=ADDR_PREFIX;
   memcpy(prefix64btoWrite->prefix, &(ip128b->addr_128b[0]), 8);
   mac64btoWrite->type=ADDR_64B;
   memcpy(mac64btoWrite->addr_64b , &(ip128b->addr_128b[8]), 8);
}
void packetfunctions_mac64bToIp128b(
      open_addr_t* prefix64b,
      open_addr_t* mac64b,
      open_addr_t* ip128bToWrite) {
   if (prefix64b->type!=ADDR_PREFIX || mac64b->type!=ADDR_64B) {
      openserial_printCritical(COMPONENT_PACKETFUNCTIONS,ERR_WRONG_ADDR_TYPE,
                            (errorparameter_t)prefix64b->type,
                            (errorparameter_t)1);
      ip128bToWrite->type=ADDR_NONE;
      return;
   }
   ip128bToWrite->type=ADDR_128B;
   memcpy(&(ip128bToWrite->addr_128b[0]), &(prefix64b->prefix[0]), 8);
   memcpy(&(ip128bToWrite->addr_128b[8]), &(mac64b->addr_64b[0]),  8);
}

//assuming an mac16b is lower 2B of mac64b
void packetfunctions_mac64bToMac16b(open_addr_t* mac64b, open_addr_t* mac16btoWrite) {
   if (mac64b->type!=ADDR_64B) {
      openserial_printCritical(COMPONENT_PACKETFUNCTIONS,ERR_WRONG_ADDR_TYPE,
                            (errorparameter_t)mac64b->type,
                            (errorparameter_t)2);
      mac16btoWrite->type=ADDR_NONE;
      return;
   }
   mac16btoWrite->type = ADDR_16B;
   mac16btoWrite->addr_16b[0] = mac64b->addr_64b[6];
   mac16btoWrite->addr_16b[1] = mac64b->addr_64b[7];
}
void packetfunctions_mac16bToMac64b(open_addr_t* mac16b, open_addr_t* mac64btoWrite) {
   if (mac16b->type!=ADDR_16B) {
      openserial_printCritical(COMPONENT_PACKETFUNCTIONS,ERR_WRONG_ADDR_TYPE,
                            (errorparameter_t)mac16b->type,
                            (errorparameter_t)3);
      mac64btoWrite->type=ADDR_NONE;
      return;
   }
   mac64btoWrite->type = ADDR_64B;
   mac64btoWrite->addr_64b[0] = 0;
   mac64btoWrite->addr_64b[1] = 0;
   mac64btoWrite->addr_64b[2] = 0;
   mac64btoWrite->addr_64b[3] = 0;
   mac64btoWrite->addr_64b[4] = 0;
   mac64btoWrite->addr_64b[5] = 0;
   mac64btoWrite->addr_64b[6] = mac16b->addr_16b[0];
   mac64btoWrite->addr_64b[7] = mac16b->addr_16b[1];
}

//======= address recognition

bool packetfunctions_isBroadcastMulticast(open_addr_t* address) {
   uint8_t i;
   uint8_t address_length;
   //IPv6 multicast
   if (address->type==ADDR_128B) {
      if (address->addr_128b[0]==0xff) {
         return TRUE;
      } else {
         return FALSE;
      }
   }
   //15.4 broadcast
   switch (address->type) {
      case ADDR_16B:
         address_length = 2;
         break;
      case ADDR_64B:
         address_length = 8;
         break;
      default:
         openserial_printCritical(COMPONENT_PACKETFUNCTIONS,ERR_WRONG_ADDR_TYPE,
                               (errorparameter_t)address->type,
                               (errorparameter_t)4);
         return FALSE;
   }
   for (i=0;i<address_length;i++) {
      if (address->addr_128b[i]!=0xFF) {
         return FALSE;
      }
   }
   return TRUE;
}

bool packetfunctions_isAllRoutersMulticast(open_addr_t* address) {
   if (
      address->type          == ADDR_128B &&
      address->addr_128b[0]  == 0xff &&
      address->addr_128b[1]  == 0x02 &&
      address->addr_128b[2]  == 0x00 &&
      address->addr_128b[3]  == 0x00 &&
      address->addr_128b[4]  == 0x00 &&
      address->addr_128b[5]  == 0x00 &&
      address->addr_128b[6]  == 0x00 &&
      address->addr_128b[7]  == 0x00 &&
      address->addr_128b[8]  == 0x00 &&
      address->addr_128b[9]  == 0x00 &&
      address->addr_128b[10] == 0x00 &&
      address->addr_128b[11] == 0x00 &&
      address->addr_128b[12] == 0x00 &&
      address->addr_128b[13] == 0x00 &&
      address->addr_128b[14] == 0x00 &&
      address->addr_128b[15] == 0x02
   ) {
      return TRUE;
   }
   return FALSE;
}

bool packetfunctions_isAllHostsMulticast(open_addr_t* address) {
   if (
      address->type          == ADDR_128B &&
      address->addr_128b[0]  == 0xff &&
      address->addr_128b[1]  == 0x02 &&
      address->addr_128b[2]  == 0x00 &&
      address->addr_128b[3]  == 0x00 &&
      address->addr_128b[4]  == 0x00 &&
      address->addr_128b[5]  == 0x00 &&
      address->addr_128b[6]  == 0x00 &&
      address->addr_128b[7]  == 0x00 &&
      address->addr_128b[8]  == 0x00 &&
      address->addr_128b[9]  == 0x00 &&
      address->addr_128b[10] == 0x00 &&
      address->addr_128b[11] == 0x00 &&
      address->addr_128b[12] == 0x00 &&
      address->addr_128b[13] == 0x00 &&
      address->addr_128b[14] == 0x00 &&
      address->addr_128b[15] == 0x01
   ) {
      return TRUE;
   }
   return FALSE;
}

bool packetfunctions_sameAddress(open_addr_t* address_1, open_addr_t* address_2) {
   uint8_t address_length;
   
   if (address_1->type!=address_2->type) {
      return FALSE;
   }
   switch (address_1->type) {
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
    
      default:
         openserial_printCritical(COMPONENT_PACKETFUNCTIONS,ERR_WRONG_ADDR_TYPE,
                               (errorparameter_t)address_1->type,
                               (errorparameter_t)5);
         return FALSE;
   }
   if (memcmp((void*)address_1->addr_128b,(void*)address_2->addr_128b,address_length)==0) {
      return TRUE;
   }
   return FALSE;
}


open_addr_t packetfunctions_convert2AddressType(open_addr_t* address_1, uint8_t addrtype) {

   open_addr_t address_ret;

   if (address_1->type == addrtype){
	   memcpy((void *)&address_ret,(void *)address_1,sizeof(open_addr_t));
   }
   else if (addrtype == ADDR_16B){
	   //copiar somente os 2 bytes
	   address_ret.type = addrtype;
	   if (address_1->type == ADDR_64B){
		   address_ret.addr_16b[0] = address_1->addr_64b[6];
		   address_ret.addr_16b[1] = address_1->addr_64b[7];
	   }
	   else if (address_1->type == ADDR_128B){
			   address_ret.addr_16b[0] = address_1->addr_64b[14];
			   address_ret.addr_16b[1] = address_1->addr_64b[15];
		   }
   }
   else if (addrtype == ADDR_64B){
	   address_ret.type = 0;
   }
   else {
	   //nao da para copiar
	   address_ret.type = 0;
   }

   return address_ret;
}

// compare address 128 bits with 64 bits
bool packetfunctions_sameAddress128_64(open_addr_t* address_1, open_addr_t* address_2) {

	uint8_t ret=FALSE;

   if ((address_1->type == 3) && (address_2->type == 2))
   {
	   if ( (address_1->addr_128b[15] == address_2->addr_64b[7]) &&
		    (address_1->addr_128b[14] == address_2->addr_64b[6]) &&
		    (address_1->addr_128b[13] == address_2->addr_64b[5]) &&
		    (address_1->addr_128b[12] == address_2->addr_64b[4]) &&
		    (address_1->addr_128b[11] == address_2->addr_64b[3]) &&
  	        (address_1->addr_128b[10] == address_2->addr_64b[2]) )
	   {
		  return TRUE;
	   }
   }

   return ret;
}

//======= address read/write

void packetfunctions_readAddress(uint8_t* payload, uint8_t type, open_addr_t* writeToAddress, bool littleEndian) {
   uint8_t i;
   uint8_t address_length;
   
   writeToAddress->type = type;
   switch (type) {
      case ADDR_16B:
      case ADDR_PANID:
         address_length = 2;
         break;
      case ADDR_64B:
      case ADDR_PREFIX:
         address_length = 8;
         break;
      case ADDR_128B:
         address_length = 16;
         break;
      default:
         openserial_printCritical(COMPONENT_PACKETFUNCTIONS,ERR_WRONG_ADDR_TYPE,
                               (errorparameter_t)type,
                               (errorparameter_t)6);
         return;
   }
   
   for (i=0;i<address_length;i++) {
      if (littleEndian) {
         writeToAddress->addr_128b[address_length-1-i] = *(payload+i);
      } else {
         writeToAddress->addr_128b[i]   = *(payload+i);
      }
   }
}

void packetfunctions_writeAddress(OpenQueueEntry_t* msg, open_addr_t* address, bool littleEndian) {
   uint8_t i;
   uint8_t address_length;
   
   switch (address->type) {
      case ADDR_16B:
      case ADDR_PANID:
         address_length = 2;
         break;
      case ADDR_64B:
      case ADDR_PREFIX:
         address_length = 8;
         break;
      case ADDR_128B:
         address_length = 16;
         break;
      default:
         openserial_printCritical(COMPONENT_PACKETFUNCTIONS,ERR_WRONG_ADDR_TYPE,
                               (errorparameter_t)address->type,
                               (errorparameter_t)7);
         return;
   }
   
   for (i=0;i<address_length;i++) {
      msg->payload      -= sizeof(uint8_t);
      msg->length       += sizeof(uint8_t);
      if (littleEndian) {
         *((uint8_t*)(msg->payload)) = address->addr_128b[i];
      } else {
         *((uint8_t*)(msg->payload)) = address->addr_128b[address_length-1-i];
      }
   }
}

//======= reserving/tossing headers

void packetfunctions_reserveHeaderSize(OpenQueueEntry_t* pkt, uint8_t header_length) {
   pkt->payload -= header_length;
   pkt->length  += header_length;
   if ( (uint8_t*)(pkt->payload) < (uint8_t*)(pkt->packet) ) {
      openserial_printCritical(COMPONENT_PACKETFUNCTIONS,ERR_HEADER_TOO_LONG,
                            (errorparameter_t)0,
                            (errorparameter_t)pkt->length);
   }
}

void packetfunctions_tossHeader(OpenQueueEntry_t* pkt, uint8_t header_length) {
   pkt->payload += header_length;
   pkt->length  -= header_length;
   if ( (uint8_t*)(pkt->payload) > (uint8_t*)(pkt->packet+126) ) {
      openserial_printError(COMPONENT_PACKETFUNCTIONS,ERR_HEADER_TOO_LONG,
                            (errorparameter_t)1,
                            (errorparameter_t)pkt->length);
   }
}

void packetfunctions_reserveFooterSize(OpenQueueEntry_t* pkt, uint8_t header_length) {
   pkt->length  += header_length;
   if (pkt->length>127) {
      openserial_printError(COMPONENT_PACKETFUNCTIONS,ERR_HEADER_TOO_LONG,
                            (errorparameter_t)2,
                            (errorparameter_t)pkt->length);
   }
}

void packetfunctions_tossFooter(OpenQueueEntry_t* pkt, uint8_t header_length) {
   pkt->length  -= header_length;
   if (pkt->length>128) {//wraps around, so a negative value will be >128
      openserial_printError(COMPONENT_PACKETFUNCTIONS,ERR_HEADER_TOO_LONG,
                            (errorparameter_t)3,
                            (errorparameter_t)pkt->length);
   }
}

//======= CRC calculation

void packetfunctions_calculateCRC(OpenQueueEntry_t* msg) {
   uint16_t crc;
   uint8_t  i;
   uint8_t  count;
   crc = 0;
   for (count=1;count<msg->length-2;count++) {
      crc = crc ^ (uint8_t)*(msg->payload+count);
      //crc = crc ^ (uint16_t)*ptr++ << 8;
      for (i=0;i<8;i++) {
         if (crc & 0x1) {
            crc = crc >> 1 ^ 0x8408;
         } else {
            crc = crc >> 1;
         }
      }
   }
   *(msg->payload+(msg->length-2)) = crc%256;
   *(msg->payload+(msg->length-1)) = crc/256;
}

bool packetfunctions_checkCRC(OpenQueueEntry_t* msg) {
   uint16_t crc;
   uint8_t  i;
   uint8_t  count;
   crc = 0;
   for (count=0;count<msg->length-2;count++) {
      crc = crc ^ (uint8_t)*(msg->payload+count);
      //crc = crc ^ (uint16_t)*ptr++ << 8;
      for (i=0;i<8;i++) {
         if (crc & 0x1) {
            crc = crc >> 1 ^ 0x8408;
         } else {
            crc = crc >> 1;
         }
      }
   }
   if (*(msg->payload+(msg->length-2))==crc%256 &&
       *(msg->payload+(msg->length-1))==crc/256) {
          return TRUE;
       } else {
          return FALSE;
       }
}

//======= checksum calculation

//see http://www-net.cs.umass.edu/kurose/transport/UDP.html, or http://tools.ietf.org/html/rfc1071
//see http://en.wikipedia.org/wiki/User_Datagram_Protocol#IPv6_PSEUDO-HEADER
void packetfunctions_calculateChecksum(OpenQueueEntry_t* msg, uint8_t* checksum_ptr) {
   uint8_t temp_checksum[2];
   uint8_t little_helper[2];
   
   // initialize running checksum
   temp_checksum[0]  = 0;
   temp_checksum[1]  = 0;
   
   //===== IPv6 pseudo header
   
   // source address (prefix and EUI64)
   onesComplementSum(temp_checksum,(idmanager_getMyID(ADDR_PREFIX))->prefix,8);
   onesComplementSum(temp_checksum,(idmanager_getMyID(ADDR_64B))->addr_64b,8);
   
   // destination address
   onesComplementSum(temp_checksum,msg->l3_destinationAdd.addr_128b,16);
   
   // length
   little_helper[0] = 0;
   little_helper[1] = msg->length;
   onesComplementSum(temp_checksum,little_helper,2);
   
   // next header
   little_helper[0] = 0;
   little_helper[1] = msg->l4_protocol;
   onesComplementSum(temp_checksum,little_helper,2);
   
   //===== payload
   
   // reset the checksum currently in the payload
   *checksum_ptr     = 0;
   *(checksum_ptr+1) = 0;
   
   onesComplementSum(temp_checksum,msg->payload,msg->length);
   temp_checksum[0] ^= 0xFF;
   temp_checksum[1] ^= 0xFF;
   
   //write in packet
   *checksum_ptr     = temp_checksum[0];
   *(checksum_ptr+1) = temp_checksum[1];
}


void onesComplementSum(uint8_t* global_sum, uint8_t* ptr, int length) {
   uint32_t sum = 0xFFFF & (global_sum[0]<<8 | global_sum[1]);
   while (length>1) {
      sum     += 0xFFFF & (*ptr<<8 | *(ptr+1));
      ptr     += 2;
      length  -= 2;
   }
   if (length) {
      sum     += (0xFF & *ptr)<<8;
   }
   while (sum>>16) {
      sum      = (sum & 0xFFFF)+(sum >> 16);
   }
   global_sum[0] = (sum>>8) & 0xFF;
   global_sum[1] = sum & 0xFF;
}

//======= endianness

void packetfunctions_htons( uint16_t val, uint8_t* dest ) {
   dest[0] = (val & 0xff00) >> 8;
   dest[1] = (val & 0x00ff);
}

uint16_t packetfunctions_ntohs( uint8_t* src ) {
   return (((uint16_t) src[0]) << 8) |
      (((uint16_t) src[1])
      );
}

void packetfunctions_htonl( uint32_t val, uint8_t* dest ) {
   dest[0] = (val & 0xff000000) >> 24;
   dest[1] = (val & 0x00ff0000) >> 16;
   dest[2] = (val & 0x0000ff00) >> 8;
   dest[3] = (val & 0x000000ff);
}

uint32_t packetfunctions_ntohl( uint8_t* src ) {
   return (((uint32_t) src[0]) << 24) |
      (((uint32_t) src[1]) << 16)     |
      (((uint32_t) src[2]) << 8)      |
      (((uint32_t) src[3])
      );
}


/* ####################################
 * ROTINAS ESPECIFICAS DO RIT
 * ####################################
 */


/*
 * Montagem do frame de OLA...para o protocolo AMAC o OLA tem o BestChannel
 */
port_INLINE uint8_t activityrx_prepareritdatareq(uint8_t hellotype,uint16_t *dstaddr) {

//	header_IE_ht header_desc;
//	OpenQueueEntry_t adv;
	uint8_t i;
	uint8_t pos=0;
	uint8_t frame[128];
//	uint32_t duration;
	open_addr_t myaddr;
	open_addr_t *pmyaddr=(open_addr_t *)&myaddr;
	uint8_t *pucAux= (uint8_t *) dstaddr;

	// MONTA O FRAME DE RIT Data Request
	// Se addr.source 16bits e addr.dest 64bits = 0xE840
    //        0110.0011
	// bits 0-2   - FrameType      - 011 - MAC Command
	// bit  3     - SecurityEnable - 0 = false
	// bit  4     - Frame Pending  - 0 = false
	// bit  5     - Ack Request    - 0 = false
	// bit  6     - Intra Pan      - 1 = true
	// bit  7-9   - Reserved       - Reserved
	// bit  10-11 - Dest Addr Mode - 2 = short 16 bits
	// bit  12-13 - Frame Ver      - 2 = Version 2
	// bit  14-15 - Src Addr Mode  - 2 = short 16 bits

	pmyaddr = idmanager_getMyID(ADDR_16B);


	if ((hellotype == HELLOTYPE_1) || (hellotype == HELLOTYPE_3))
		frame[pos++] = 0x40 + IEEE154_TYPE_CMD;  //FCS[0] = IntraPan
	else
		frame[pos++] = 0x60 + IEEE154_TYPE_CMD;  //FCS[0] = IntraPan and AckRequest

	frame[pos++] = 0xA8;  //FCS[1] MSB
	frame[pos++] = olaasn;  //Seq Number
	frame[pos++] = 0xfe;
	frame[pos++] = 0xca;
	//address dest
	if  (hellotype == HELLOTYPE_1){
		frame[pos++] = pucAux[0];
		frame[pos++] = pucAux[1];
	}
	else if (hellotype == HELLOTYPE_2){
		frame[pos++] = pucAux[0];
		frame[pos++] = pucAux[1];
	}
	else { // HELLOTYPE_3
		frame[pos++] = pucAux[1];
		frame[pos++] = 0x80;
	}

	//address source
	frame[pos++] = pmyaddr->addr_16b[1];
	frame[pos++] = pmyaddr->addr_16b[0];

	//Command Frame Identifier
	frame[pos++] = CMDFRMID;
	//frame[pos++] = hellospec;   //hellospec 0=Dont need ACK ; 1=Need ACK
	frame[pos++] = macneighbors_getMyBestChan();
/*
    for (i=0;i<114;i++) {
	 frame[pos++] = 0x00;
	}
*/
	//crc
	frame[pos++] = 0x00;
	frame[pos++] = 0x00;

	// space for 2-byte CRC
	//packetfunctions_reserveFooterSize(&adv,2);

	radio_loadPacket((uint8_t *)&frame[0],pos);

	return pos;
}

//hellospec 0=Dont need ACK ; 1=Need ACK
port_INLINE sRITelement activityrx_preparemultichannelhello(uint8_t hellospec, uint8_t *frame,uint8_t len) {

	uint8_t pos=0,i=0,auxlen;
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


	frame[pos++] = 0x40 | IEEE154_TYPE_LIVELIST ;  //00...FCS[0] LSB
	frame[pos++] = 0xA8;                           //01...FCS[1] MSB
	frame[pos++] = livelistasn;                  //02...Seq Number
	frame[pos++] = 0xfe;                           //03...PAN[0]
	frame[pos++] = 0xca;                           //04...PAN[1]
	//address dest
	frame[pos++] = psEle.destaddr.addr_16b[0];     //05...DST[0]
	frame[pos++] = psEle.destaddr.addr_16b[1];     //06...DST[1]
	//address source

	pmyaddr = idmanager_getMyID(ADDR_16B);
	frame[pos++] = pmyaddr->addr_16b[1];           //07...SRC[0]
	frame[pos++] = pmyaddr->addr_16b[0];           //08...SRC[1]

	//Command Frame Identifier
	frame[pos++] = CMDLIVELIST;                    //09...CMDFRMID; TESTE RFF - SOMENTE PARA MELHORAR DEBUG...PROVISORIO
 	frame[pos++] = hellospec;                      //10...hellospec 0=Dont need ACK ; 1=Need ACK
    //

    //Quando informado o len indica o tamanho de frame desejado...
 	if (len > 18) {

 		//imprime estatisticas no payload...
		if (len > 40) {
 		    frame[pos++] = 0xAA;
#if (IEEE802154E_RITMC == 1)
			pos = printvar((uint8_t *)&ritstat.txola.countdatatx,sizeof(uint16_t),frame,pos);
			pos = printvar((uint8_t *)&ritstat.txola.countdatatxok,sizeof(uint16_t),frame,pos);
			//pos = printvar((uint8_t *)&ritmc_txolaack,sizeof(uint16_t),frame,pos);
			//pos = printvar((uint8_t *)&ecwvars.rxcwoccurs,sizeof(uint16_t),frame,pos);
			pos = printvar((uint8_t *)&ritstat.txola.countacktxrxok,sizeof(uint16_t),frame,pos);
 		    frame[pos++] = 0xAA;
			pos = printvar((uint8_t *)&ritmc_rxolaack,sizeof(uint16_t),frame,pos);  //rx04
			pos = printvar((uint8_t *)&ritstat.rxola.countdatatxok,sizeof(uint16_t),frame,pos); //rx07
			pos = printvar((uint8_t *)&ritstat.rxola.countacktxrxok,sizeof(uint16_t),frame,pos); //rx0b
			//pos = printvar((uint8_t *)&ritmc_rx04err,sizeof(uint16_t),frame,pos);  //rx04
 		    frame[pos++] = 0xAA;
			pos = printvar((uint8_t *)&ecwvars.ecw1occurs,sizeof(uint16_t),frame,pos);
			pos = printvar((uint8_t *)&ecwvars.ecw2occurs,sizeof(uint16_t),frame,pos);
			pos = printvar((uint8_t *)&ecwvars.rxcwoccurs,sizeof(uint16_t),frame,pos);
			pos = printvar((uint8_t *)&ecwvars.ecw2DataTimeout,sizeof(uint16_t),frame,pos);
//			pos = printvar((uint8_t *)&ecwvars.tx0aFrameNotOk,sizeof(uint16_t),frame,pos);
//			pos = printvar((uint8_t *)&ritmc_rx04err,sizeof(uint16_t),frame,pos);


#elif (IEEE802154E_ARM == 1)
			pos = printvar((uint8_t *)&ritstat.txola.countdatatx,sizeof(uint16_t),frame,pos);
			pos = printvar((uint8_t *)&ritstat.txola.countdatatxok,sizeof(uint16_t),frame,pos);
			pos = printvar((uint8_t *)&ritstat.txola.countacktxrxok,sizeof(uint16_t),frame,pos);
 			pos = printvar((uint8_t *)&counttxrts,sizeof(uint16_t),frame,pos);
			pos = printvar((uint8_t *)&countrxcts,sizeof(uint16_t),frame,pos);
		    frame[pos++] = 0xAA;
			pos = printvar((uint8_t *)&ritstat.rxola.countdatatxok,sizeof(uint16_t),frame,pos);
			pos = printvar((uint8_t *)&ritstat.rxola.countacktxrxok,sizeof(uint16_t),frame,pos);
			pos = printvar((uint8_t *)&countrxrts,sizeof(uint16_t),frame,pos);
			pos = printvar((uint8_t *)&counttxcts,sizeof(uint16_t),frame,pos);
#elif (IEEE802154E_AMAC == 1)
			pos = printvar((uint8_t *)&ritstat.txola.countdatatx,sizeof(uint16_t),frame,pos);
			pos = printvar((uint8_t *)&ritstat.txola.countdatatxok,sizeof(uint16_t),frame,pos);
			pos = printvar((uint8_t *)&ritstat.txola.countacktxrxok,sizeof(uint16_t),frame,pos);
 			pos = printvar((uint8_t *)&counttxrts,sizeof(uint16_t),frame,pos);
			pos = printvar((uint8_t *)&countrxcts,sizeof(uint16_t),frame,pos);
 		    frame[pos++] = 0xAA;
			pos = printvar((uint8_t *)&ritstat.rxola.countdatatxok,sizeof(uint16_t),frame,pos);
			pos = printvar((uint8_t *)&ritstat.rxola.countacktxrxok,sizeof(uint16_t),frame,pos);
			pos = printvar((uint8_t *)&countrxrts,sizeof(uint16_t),frame,pos);
			pos = printvar((uint8_t *)&counttxcts,sizeof(uint16_t),frame,pos);
			pos = printvar((uint8_t *)&countrxHA,sizeof(uint16_t),frame,pos);
			pos = printvar((uint8_t *)&counttxcw,sizeof(uint16_t),frame,pos);
			pos = printvar((uint8_t *)&countrxcw,sizeof(uint16_t),frame,pos);
#else
			pos = printvar((uint8_t *)&ritstat.txola.countdatatx,sizeof(uint16_t),frame,pos);
			pos = printvar((uint8_t *)&ritstat.txola.countdatatxok,sizeof(uint16_t),frame,pos);
			pos = printvar((uint8_t *)&ritstat.txola.countacktxrxok,sizeof(uint16_t),frame,pos);
 		    frame[pos++] = 0xAA;
			pos = printvar((uint8_t *)&ritstat.rxola.countdatatxok,sizeof(uint16_t),frame,pos);
			pos = printvar((uint8_t *)&ritstat.rxola.countacktxrxok,sizeof(uint16_t),frame,pos);
#endif
			frame[pos++] = 0xee;
 			frame[pos++] = ieee154e_vars.lastRSSI;
 		 	frame[pos++] = ieee154e_vars.lastlqi;
 		    //[pos++] = 0xee;
 		    //frame[pos++] = dutycyclecount;
			//pos = printvar((uint8_t *)&absdutycyclems,sizeof(uint16_t),frame,pos);
			//pos = printvar((uint8_t *)&relativedutycyclems,sizeof(uint16_t),frame,pos);

		}
		/*
		frame[pos++] = 0xA1;
		frame[pos++] = 0xA2;
		frame[pos++] = 0xA3;
		frame[pos++] = 0xA4;
		*/
		auxlen = len - pos - 2;

        for (i=0;i<auxlen;i++) {
		 frame[pos++] = 0xaa;
		}
 	}
 	else{
 		//livelist ack + delta ja foi preenchido anteriormente
		if (pos < (len-2))
			pos = len-2;
  	}

	//crc
	frame[pos++] = 0x00;
	frame[pos++] = 0x00;

	// space for 2-byte CRC
	//packetfunctions_reserveFooterSize(&adv,2);

    psEle.msglength = pos;

	return psEle;
}

// MONTA O FRAME DE CW
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

port_INLINE uint8_t activityrx_preparecw(uint8_t *frame, uint8_t *pecwstatus, open_addr_t *pdstaddr, uint8_t *seqnr) {
	uint8_t pos=0;
	open_addr_t myaddr;
	open_addr_t *pmyaddr=(open_addr_t *)&myaddr;

	frame[pos++] = 0x40 | IEEE154_TYPE_ACK ;       //00...FCS[0] LSB
	frame[pos++] = 0xaa;                           //01...FCS[1] MSB
	frame[pos++] = *seqnr;                         //02...Seq Number
	frame[pos++] = CW_MASK_PANID;                  //03...PAN[0]
	frame[pos++] = CW_MASK_PANID;                  //04...PAN[1]
	//address dest
#if 1
	frame[pos++] = pdstaddr->addr_16b[1];          //05...DST[0]
	frame[pos++] = pdstaddr->addr_16b[0];          //06...DST[1]
#else
	frame[pos++] = 0x80;                           //05...DST[0]
	frame[pos++] = 0x80;                           //06...DST[1]
#endif
	//address source
	pmyaddr = idmanager_getMyID(ADDR_16B);
	frame[pos++] = pmyaddr->addr_16b[1];           //07...SRC[0]
	frame[pos++] = pmyaddr->addr_16b[0];           //08...SRC[1]

	frame[pos++] = *pecwstatus;                    //09...STATUS CW
	//crc
	frame[pos++] = 0x00;
	frame[pos++] = 0x00;


   return (pos);
}

/*
 * Atualizo o destination address da msg para o valor do address
 * Utilizado pela livelist para ao inves de enviar um broadcast enviar o endereco do destino...
 * Esta mudanca eh importante para reconhecer e enviar um ack se necessario...
 */
port_INLINE void updatedstaddr(uint8_t* packet, open_addr_t  address) {

	//address dest
	packet[6] = address.addr_16b[0];
	packet[5] = address.addr_16b[1];

}

//=========================== private =========================================
