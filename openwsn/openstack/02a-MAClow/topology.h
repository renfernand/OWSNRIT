#ifndef __TOPOLOGY_H
#define __TOPOLOGY_H

/**
\addtogroup MAClow
\{
\addtogroup topology
\{
*/

#include "opendefs.h"
#include "IEEE802154.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== prototypes ======================================

bool topology_isAcceptablePacket(ieee802154_header_iht* ieee802514_header);
uint8_t convertaddress16to64 (open_addr_t *dstaddr, open_addr_t *srcaddr);
uint8_t getaddressMote (open_addr_t *dstaddr,uint8_t addrtype, uint8_t mote);

/**
\}
\}
*/

#endif
