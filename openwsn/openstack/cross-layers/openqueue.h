#ifndef __OPENQUEUE_H
#define __OPENQUEUE_H

/**
\addtogroup cross-layers
\{
\addtogroup OpenQueue
\{
*/

#include "opendefs.h"
#include "IEEE802154.h"

//=========================== define ==========================================

#define QUEUELENGTH  10

#define MAX_RIT_LIST_ELEM 5
#define MAX_RIT_MSG_LEN   130


typedef struct RIT_Queue {
	uint8_t     frameType;
	uint32_t    msglength;
	bool        isBroadcastMulticast;
	uint8_t     pending;
	uint8_t     numTargetParents;
	uint8_t     countretry;
	uint64_t    timestamp;
	uint32_t    lasttxduration;
	open_addr_t destaddr;
	uint8_t     msg[MAX_RIT_MSG_LEN];
} sRITqueue;

typedef struct RIT_Queue_element {
	uint64_t timestamp;
	open_addr_t destaddr;
	uint8_t frameType;
	uint8_t *msg;
	uint32_t msglength;
	uint32_t lasttxduration;
	bool isBroadcastMulticast;
} sRITelement;
//=========================== typedef =========================================

typedef struct {
   uint8_t  creator;
   uint8_t  owner;
} debugOpenQueueEntry_t;

//=========================== module variables ================================

typedef struct {
   OpenQueueEntry_t queue[QUEUELENGTH];
} openqueue_vars_t;

//=========================== prototypes ======================================

// admin
void               openqueue_init(void);
bool               debugPrint_queue(void);
// called by any component
OpenQueueEntry_t*  openqueue_getFreePacketBuffer(uint8_t creator);
owerror_t         openqueue_freePacketBuffer(OpenQueueEntry_t* pkt);
void               openqueue_removeAllCreatedBy(uint8_t creator);
void               openqueue_removeAllOwnedBy(uint8_t owner);
// called by res
OpenQueueEntry_t*  openqueue_sixtopGetSentPacket(void);
OpenQueueEntry_t*  openqueue_sixtopGetReceivedPacket(void);
// called by IEEE80215E
OpenQueueEntry_t*  openqueue_macGetDataPacket(open_addr_t* toNeighbor);
OpenQueueEntry_t*  openqueue_macGetAdvPacket(void);


bool  RITQueue_ClearAddress(open_addr_t* addr);
bool  RITQueue_copyaddress(open_addr_t* addr1, open_addr_t* addr2);
uint8_t RITQueue_cleanupoldmsg(void);
bool RITQueue_Free(uint8_t elementpos);
uint8_t RITQueue_Get_Pos(open_addr_t *paddr);
sRITqueue RITQueue_Get_Element(uint8_t elementpos);
uint8_t RITQueue_Put(sRITelement *psEle,uint8_t pending, uint8_t numTargetParents);
sRITqueue RITQueue_Dequeue_byaddr(open_addr_t addr);
void RITQueue_Init(void);
bool RITQueue_Enqueue(sRITqueue *pmsg);

/**
\}
\}
*/

#endif
