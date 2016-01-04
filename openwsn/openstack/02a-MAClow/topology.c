#include "opendefs.h"
#include "topology.h"
#include "idmanager.h"

//=========================== defines =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

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
*/
#define MOTE1   0x09
#define MOTE2   0xC0
#define MOTE3   0x87
#define MOTE4   0x52
#define MOTE5   0xBE
#define MOTE6   0x58

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
bool topology_isAcceptablePacket(ieee802154_header_iht* ieee802514_header) {
#if (FORCETOPOLOGY == 1)
   bool returnVal;

   returnVal=FALSE;
   switch (idmanager_getMyID(ADDR_64B)->addr_64b[7]) {
      case MOTE1:
         if (
               ieee802514_header->src.addr_64b[7]== MOTE2 ||
               ieee802514_header->src.addr_64b[7]== MOTE3
            ) {
            returnVal=TRUE;
         }
         break;
      case MOTE2: //
         if (
               ieee802514_header->src.addr_64b[7]==MOTE1 ||
               ieee802514_header->src.addr_64b[7]==MOTE3 ||
               ieee802514_header->src.addr_64b[7]==MOTE4
            ) {
            returnVal=TRUE;
         }
         break;
      case MOTE3:
         if (
               ieee802514_header->src.addr_64b[7]==MOTE1 ||
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

#endif
//=========================== private =========================================
