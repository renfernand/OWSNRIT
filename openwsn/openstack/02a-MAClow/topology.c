#include "board.h"
#include "opendefs.h"
#include "topology.h"
#include "idmanager.h"
#include "neighbors.h"

//=========================== defines =========================================

//=========================== variables =======================================
extern const forceNeighbor_t tableNeighbor[];
extern const uint8_t MoteAddrTable_1byte[];
extern const uint8_t MoteAddrTable_2bytes[];
extern const uint8_t MoteBestChanTable[];
//=========================== prototypes ======================================
uint8_t getMotePos(uint8_t addr,uint8_t len);
//=========================== public ==========================================

/**
\brief Force a topology.

This function is used to force a certain topology, by hard-coding the list of
acceptable neighbors for a given mote. This function is invoked each time a
packet is received. If it returns FALSE, the packet is silently dropped, as if
it were never received. If it returns TRUE, the packet is accepted.

Typically, filtering packets is done by analyzing the IEEE802.15.4 header. An
example body for this function which forces a topology is:

   switch (idmanager_getMyID(ADDR_64B)->addr_64b[7]) {
      case TOPOLOGY_MOTE1:
         if (ieee802514_header->src.addr_64b[7]==TOPOLOGY_MOTE2) {
            returnVal=TRUE;
         } else {
            returnVal=FALSE;
         }
         break;
      case TOPOLOGY_MOTE2:
         if (ieee802514_header->src.addr_64b[7]==TOPOLOGY_MOTE1 ||
             ieee802514_header->src.addr_64b[7]==TOPOLOGY_MOTE3) {
            returnVal=TRUE;
         } else {
            returnVal=FALSE;
         }
         break;
      default:
         returnVal=TRUE;
   }
   return returnVal;

By default, however, the function should return TRUE to *not* force any
topology.

\param[in] ieee802514_header The parsed IEEE802.15.4 MAC header.

\return TRUE if the packet can be received.
\return FALSE if the packet should be silently dropped.
motes atuais CC2538EM
[1] bbbb::0012:4b00:02f4:ac09 [0x09] --> dagroot
[2] bbbb::0012:4b00:02f4:afc0 [0xc0]
[3] bbbb::0012:4b00:040e:fc87 [0x87]
[4] bbbb::0012:4b00:03a6:5152 [0x52]
[5] bbbb::0012:4b00:03a6:4cbe [0xbe]
[6] bbbb::0012:4b00:02f4:AF58 [0x58]

#define MOTE0   0xFD
#define MOTE1   0x09
#define MOTE2   0xC0
#define MOTE3   0x87
#define MOTE4   0x52
#define MOTE5   0xBE
#define MOTE6   0x58
*/

#if 0
bool topology_isAcceptablePacket(ieee802154_header_iht* ieee802514_header) {
#if (FORCETOPOLOGY == 1)
   bool returnVal;
   
   returnVal=FALSE;
   switch (idmanager_getMyID(ADDR_64B)->addr_64b[7]) {
      case MOTE1:  //MUDAR
         if (
    //           ieee802514_header->src.addr_64b[7]== MOTE2 ||
               ieee802514_header->src.addr_64b[7]== MOTE3
            ) {
            returnVal=TRUE;
         }
         break;
      case MOTE2: // MUDAR
         if (
               ieee802514_header->src.addr_64b[7]==MOTE4 ||
               ieee802514_header->src.addr_64b[7]==MOTE6
            ) {
            returnVal=TRUE;
         }
         break;
      case MOTE3: //OK
         if (
               ieee802514_header->src.addr_64b[7]==MOTE1 ||
               ieee802514_header->src.addr_64b[7]==MOTE5
            ) {
            returnVal=TRUE;
         }
         break;
      case MOTE4: //MUDAR
         if (
               ieee802514_header->src.addr_64b[7]==MOTE2 ||
               ieee802514_header->src.addr_64b[7]==MOTE5
            ) {
            returnVal=TRUE;
         }
         break;
      case MOTE5: //MUDAR
         if (
               ieee802514_header->src.addr_64b[7]==MOTE3 ||
               ieee802514_header->src.addr_64b[7]==MOTE4
            ) {
            returnVal=TRUE;
         }
      case MOTE6: //MUDAR
         if (
   //            ieee802514_header->src.addr_64b[7]==MOTE4 ||
               ieee802514_header->src.addr_64b[7]==MOTE2
            ) {
            returnVal=TRUE;
         }
         break;
   }
   return returnVal;
#else
   return TRUE;
#endif
}
#else
#if (USE_RITREQ_SHORT == 0)
bool topology_isAcceptablePacket(ieee802154_header_iht* ieee802514_header) {
#if (FORCETOPOLOGY == 1)
   bool returnVal;
   returnVal=FALSE;

   switch (idmanager_getMyID(ADDR_64B)->addr_64b[7]) {
      case MOTE0:
		  if (
				ieee802514_header->src.addr_64b[7]== MOTE1 ||
				ieee802514_header->src.addr_64b[7]== MOTE2
				// ieee802514_header->src.addr_64b[7]== MOTE3
			 ) {
			 returnVal=TRUE;
		  }
		  break;
      case MOTE1:
         if (
               ieee802514_header->src.addr_64b[7]== MOTE0 ||
               ieee802514_header->src.addr_64b[7]== MOTE2
               // ieee802514_header->src.addr_64b[7]== MOTE3
            ) {
            returnVal=TRUE;
         }
         break;
      case MOTE2: //
         if (
               ieee802514_header->src.addr_64b[7]==MOTE0 ||
               ieee802514_header->src.addr_64b[7]==MOTE1 ||
               ieee802514_header->src.addr_64b[7]==MOTE3
            ) {
            returnVal=TRUE;
         }
         break;
      case MOTE3:
         if (
               //ieee802514_header->src.addr_64b[7]==MOTE1 ||
               ieee802514_header->src.addr_64b[7]==MOTE2 ||
               ieee802514_header->src.addr_64b[7]==MOTE4 ||
               ieee802514_header->src.addr_64b[7]==MOTE5
            ) {
            returnVal=TRUE;
         }
         break;
      case MOTE4:
         if (
               ieee802514_header->src.addr_64b[7]==MOTE2 ||
               ieee802514_header->src.addr_64b[7]==MOTE3 ||
               ieee802514_header->src.addr_64b[7]==MOTE6 ||
               ieee802514_header->src.addr_64b[7]==MOTE5
            ) {
            returnVal=TRUE;
         }
         break;
      case MOTE5:
         if (
               ieee802514_header->src.addr_64b[7]==MOTE3 ||
               ieee802514_header->src.addr_64b[7]==MOTE6 ||
               ieee802514_header->src.addr_64b[7]==MOTE4
            ) {
            returnVal=TRUE;
         }
      case MOTE6:
         if (
               ieee802514_header->src.addr_64b[7]==MOTE5 ||
               ieee802514_header->src.addr_64b[7]==MOTE4
            ) {
            returnVal=TRUE;
         }
         break;
   }
   return returnVal;
#else
   return TRUE;
#endif
}

#else // USE_RITREQ_SHORT
bool topology_isAcceptablePacket(ieee802154_header_iht* ieee802514_header)
{
   bool returnVal;
   uint8_t sraddr[2];
   uint8_t i,pos=0;
   returnVal=FALSE;

#if (FORCETOPOLOGY == 1)

   if (ieee802514_header->src.type == 1){
	   sraddr[0] = ieee802514_header->src.addr_16b[0];
	   sraddr[1] = ieee802514_header->src.addr_16b[1];
   }
   else if (ieee802514_header->src.type == 2){
	   sraddr[0] = ieee802514_header->src.addr_64b[6];
	   sraddr[1] = ieee802514_header->src.addr_64b[7];
   }
   else if (ieee802514_header->src.type == 3){
	   sraddr[0] = ieee802514_header->src.addr_128b[14];
	   sraddr[1] = ieee802514_header->src.addr_128b[15];
   }

   pos = getMotePos(idmanager_getMyID(ADDR_64B)->addr_64b[7],1);

   for (i=0;i < tableNeighbor[pos].numNeighbor;i++){
	  if ((sraddr[0] == *(((uint8_t *)&tableNeighbor[pos].ele[i].addr16b)+1)) &&
		  (sraddr[1] == *(((uint8_t *)&tableNeighbor[pos].ele[i].addr16b)+0))) {
		  returnVal = TRUE;
		  break;
	  }
  }

   return returnVal;
#else
   return TRUE;
#endif
}
#endif // USE_RITREQ_SHORT

#endif

uint8_t getMotePos(uint8_t addr,uint8_t len){

	uint8_t i;
	uint8_t *pucAux;

	if (len == 1){
		for (i=0;i<6;i++){
			  if (addr == MoteAddrTable_1byte[i])
				  break;
		}
	}
/*
	else //len == 2
	{
		for (i=0;i<6;i++){
			pucAux = (uint8_t *)&MoteAddrTable_2bytes[i];
			  if ((addr == *(pucAux+1)) &&
				  (addr == *(pucAux+0)))
				  break;
		}

	}
*/

	return i;
}


//=========================== TOPOLOGY AND NEIGHBOORHOOD TABLE ==========================================

/* A TABELA ABAIXO SEGUE ESTA ORDEM
motes atuais CC2538EM
[0] bbbb::0012:4b00:02f4:acfd [0xFD]
[1] bbbb::0012:4b00:02f4:ac09 [0x09]
[2] bbbb::0012:4b00:02f4:afc0 [0xc0]
[3] bbbb::0012:4b00:040e:fc87 [0x87]
[4] bbbb::0012:4b00:03a6:5152 [0x52]
[5] bbbb::0012:4b00:03a6:4cbe [0xbe]
[6] bbbb::0012:4b00:02f4:AF58 [0x58]
*/


const uint8_t MoteAddrTable_1byte[] = {
 MOTE0,MOTE1,MOTE2,MOTE3,MOTE4,MOTE5,MOTE6
};

const uint8_t MoteAddrTable_2bytes[] = {
 ADDR16b_MOTE0, ADDR16b_MOTE1, ADDR16b_MOTE2, ADDR16b_MOTE3, ADDR16b_MOTE4, ADDR16b_MOTE5, ADDR16b_MOTE6,
};

const uint8_t MoteBestChanTable[] = {
 MOTE0_BC,MOTE1_BC,MOTE2_BC,MOTE3_BC,MOTE4_BC,MOTE5_BC,MOTE6_BC
};

const forceNeighbor_t tableNeighbor[] = {
      //{numNeighbor,{{ele[0].addr16b,ele[0].bc},{ele[1].addr16b,ele[1].bc},{ele[2].addr16b,ele[2].bc}}},
/* MOTE0 */ {2,{{ADDR16b_MOTE1,MOTE1_BC},{ADDR16b_MOTE2,MOTE2_BC},{0,0}}},
/* MOTE1 */ {1,{{ADDR16b_MOTE0,MOTE0_BC},{0,0},{0,0}}},
/* MOTE2 */ {1,{{ADDR16b_MOTE0,MOTE0_BC},{0,0},{0,0}}},
/* MOTE3 */ {1,{{ADDR16b_MOTE0,MOTE0_BC},{0,0},{0,0}}},
/* MOTE4 */ {1,{{ADDR16b_MOTE0,MOTE0_BC},{0,0},{0,0}}},
/* MOTE5 */ {1,{{ADDR16b_MOTE0,MOTE0_BC},{0,0},{0,0}}},
/* MOTE6 */ {1,{{ADDR16b_MOTE0,MOTE0_BC},{0,0},{0,0}}}
};
