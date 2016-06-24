#ifndef __NEIGHBORS_H
#define __NEIGHBORS_H

/**
\addtogroup MAChigh
\{
\addtogroup Neighbors
\{
*/
#include "board.h"
#include "opendefs.h"
#include "icmpv6rpl.h"

//=========================== define ==========================================

#define MAXNUMNEIGHBORS           10
#define MAXNUMMACNEIGHBORS         5   //used IEEE802154_AMCA

#define MAXPREFERENCE             2
#define BADNEIGHBORMAXRSSI        -80 //dBm
#define GOODNEIGHBORMINRSSI       -90 //dBm
#define SWITCHSTABILITYTHRESHOLD  3
#define DEFAULTLINKCOST           15

#define MAXDAGRANK                0xffff
#define DEFAULTDAGRANK            MAXDAGRANK
#define MINHOPRANKINCREASE        256  //default value in RPL and Minimal 6TiSCH draft

/*
motes atuais CC2538EM
[1] bbbb::0012:4b00:02f4:ac09 [0x09] --> dagroot
[2] bbbb::0012:4b00:02f4:afc0 [0xc0]
[3] bbbb::0012:4b00:040e:fc87 [0x87]
[4] bbbb::0012:4b00:03a6:5152 [0x52]
[5] bbbb::0012:4b00:03a6:4cbe [0xbe]
[6] bbbb::0012:4b00:02f4:AF58 [0x58]
*/
#define MOTE0   0xFD
#define MOTE1   0x09
#define MOTE2   0xC0
#define MOTE3   0x87
#define MOTE4   0x52
#define MOTE5   0xBE
#define MOTE6   0x58

#define ADDR16b_MOTE0   0xACFD
#define ADDR16b_MOTE1   0xAC09
#define ADDR16b_MOTE2   0xAFC0
#define ADDR16b_MOTE3   0xFC87
#define ADDR16b_MOTE4   0x5152
#define ADDR16b_MOTE5   0x4CBE
#define ADDR16b_MOTE6   0xAF58

/* best channel */
#define MOTE0_BC   20
#define MOTE1_BC   20
#define MOTE2_BC   20
#define MOTE3_BC   20
#define MOTE4_BC   20
#define MOTE5_BC   20
#define MOTE6_BC   20

#define MAX_NUM_MOTES 7   //este define eh usado na montagem da topologia
//=========================== typedef =========================================

#if (IEEE802154E_AMCA == 1)
BEGIN_PACK
typedef struct {
   bool             used;
   uint8_t          parentPreference;
   bool             stableNeighbor;
   uint8_t          switchStabilityCounter;
   open_addr_t      addr_16b;
   uint8_t          numRx;
   uint8_t          numTx;
   uint8_t          numTxACK;
   uint8_t          bestchan;           //bestchannel
   uint8_t          broadcastPending;   //quando msg broadcast flag indica se enviou msg
} macneighborRow_t;
END_PACK
typedef struct {
   macneighborRow_t     neighbors[MAXNUMMACNEIGHBORS];
   uint8_t              mybestchan;   //bestchannel
} macneighbors_vars_t;

typedef struct {
   uint16_t addr16b;
   uint8_t  bc;
}forceNeighbor_ele_t;
typedef struct {
   uint8_t numNeighbor;
   forceNeighbor_ele_t ele[3];
} forceNeighbor_t;

#endif



BEGIN_PACK
typedef struct {
   bool             used;
   uint8_t          parentPreference;
   bool             stableNeighbor;
   uint8_t          switchStabilityCounter;
   open_addr_t      addr_64b;
   dagrank_t        DAGrank;
   int8_t           rssi;
   uint8_t          numRx;
   uint8_t          numTx;
   uint8_t          numTxACK;
   uint8_t          numWraps;   //number of times the tx counter wraps. can be removed if memory is a restriction. also check openvisualizer then.
   asn_t            asn;
   uint8_t          joinPrio;
} neighborRow_t;
END_PACK

BEGIN_PACK
typedef struct {
   uint8_t         row;
   neighborRow_t   neighborEntry;
} debugNeighborEntry_t;
END_PACK

BEGIN_PACK
typedef struct {
   uint8_t         last_addr_byte;   // last byte of the neighbor's address
   int8_t          rssi;
   uint8_t         parentPreference;
   dagrank_t       DAGrank;
   uint16_t        asn; 
} netDebugNeigborEntry_t;
END_PACK

//=========================== module variables ================================
   
typedef struct {
   neighborRow_t        neighbors[MAXNUMNEIGHBORS];
   dagrank_t            myDAGrank;
   uint8_t              debugRow;
   icmpv6rpl_dio_ht*    dio; //keep it global to be able to debug correctly.
} neighbors_vars_t;

//=========================== prototypes ======================================

void          neighbors_init(void);
bool neighbors_isValidNeighbor(uint8_t index);
// getters
uint8_t macneigh_getBChan(open_addr_t* address);
void macneigh_getBChanMultiCast(uint8_t *bestchannel,open_addr_t* targetaddr);
uint8_t macneighbors_getMyBestChan(void);
uint8_t macneighbors_getFixedBestChan(uint8_t lastbyte);
bool macisThisAddressPendingBroadcast(open_addr_t* address) ;

dagrank_t     neighbors_getMyDAGrank(void);
uint8_t       neighbors_getNumNeighbors(void);
bool          neighbors_getPreferredParentEui64(open_addr_t* addressToWrite);
open_addr_t*  neighbors_getKANeighbor(uint16_t kaPeriod);
uint8_t neighbors_howmanyIhave(void);

uint8_t macneighbors_getNumNeighbors(void);
uint8_t macneighbors_setBroadCastPending(void);
uint8_t macneighbors_clearBroadcastPending(open_addr_t actualsrcaddr);

// interrogators
bool          neighbors_isStableNeighbor(open_addr_t* address);
bool          neighbors_isPreferredParent(open_addr_t* address);
bool          neighbors_isNeighborWithLowerDAGrank(uint8_t index);
bool          neighbors_isNeighborWithHigherDAGrank(uint8_t index);

// updating neighbor information
void          neighbors_indicateRx(
   open_addr_t*         l2_src,
   int8_t               rssi,
   asn_t*               asnTimestamp,
   bool                 joinPrioPresent,
   uint8_t              joinPrio
);
void          neighbors_indicateTx(
   open_addr_t*         dest,
   uint8_t              numTxAttempts,
   bool                 was_finally_acked,
   asn_t*               asnTimestamp
);
void          neighbors_indicateRxDIO(OpenQueueEntry_t* msg);

// get addresses
void          neighbors_getNeighbor(open_addr_t* address,uint8_t addr_type,uint8_t index);
// managing routing info
void          neighbors_updateMyDAGrankAndNeighborPreference(void);
// maintenance
void          neighbors_removeOld(void);
// debug
bool          debugPrint_neighbors(void);

/**
\}
\}
*/

#endif
