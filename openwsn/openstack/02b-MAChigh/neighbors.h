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
#define MAXNUMMACNEIGHBORS         5

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
[0] bbbb::0012:4b00:02f4:acfd [0xfd] --> DAGROOT
[1] bbbb::0012:4b00:02f4:ac09 [0x09]
[2] bbbb::0012:4b00:02f4:afc0 [0xc0]
[3] bbbb::0012:4b00:040e:fc87 [0x87]
[4] bbbb::0012:4b00:03a6:5152 [0x52]
[5] bbbb::0012:4b00:03a6:FC89 [0x89]
[6] bbbb::0012:4b00:02f4:AF58 [0x58]
[7] bbbb::0012:4b00:03a6:4cbe [0xbe]
 MOTEMBA
[5]       0012:4B00:05AA:EF2B
[6]       0012:4B00:05AA:EF1A
[7]       0012:4B00:05AA:EF2C
[8]       0012:4B00:05AA:EF44
[9]       0012:4B00:05AA:EF17
[10]      0012:4B00:05AA:EF4B
[11]      0012:4B00:05AA:EF15
[12]      0012:4B00:05AA:EF39
[13]      0012:4B00:05AA:EF3C
[14]      0012:4B00:05AA:EF20
[15]      0012:4B00:05AA:EF45

  0012:4B00:05AA:EF4A   - ERRO NO TX RX...
*/


#define MOTE0    0xFD
#define MOTE1    0x09
#define MOTE2    0xC0
#define MOTE3    0x87
#define MOTE4    0x52
#define MOTE5    0x2B
#define MOTE6    0x1A
#define MOTE7    0x2C
#define MOTE8    0x44
#define MOTE9    0x17
#define MOTE10   0x39
#define MOTE11   0x15
#define MOTE12   0x4B
#define MOTE13   0x3C
#define MOTE14   0x20
#define MOTE15   0x45

#define ADDR16b_MOTE0   0xACFD
#define ADDR16b_MOTE1   0xAC09
#define ADDR16b_MOTE2   0xAFC0
#define ADDR16b_MOTE3   0xFC87
#define ADDR16b_MOTE4   0x5152
#define ADDR16b_MOTE5   0xEF2B
#define ADDR16b_MOTE6   0xEF1A
#define ADDR16b_MOTE7   0xEF2C
#define ADDR16b_MOTE8   0xEF44
#define ADDR16b_MOTE9   0xEF17
#define ADDR16b_MOTE10  0xEF39
#define ADDR16b_MOTE11  0xEF15
#define ADDR16b_MOTE12  0xEF4B
#define ADDR16b_MOTE13  0xEF3C
#define ADDR16b_MOTE14  0xEF20
#define ADDR16b_MOTE15  0xEF45


#define FIXEDRANK_MOTE0   0x0000
#define FIXEDRANK_MOTE1   0x0100
#define FIXEDRANK_MOTE2   0x0200
#define FIXEDRANK_MOTE3   0x0300
#define FIXEDRANK_MOTE4   0x0400
#define FIXEDRANK_MOTE5   0x0500
#define FIXEDRANK_MOTE6   0x0600
#define FIXEDRANK_MOTE7   0x0700
#define FIXEDRANK_MOTE8   0x0800
#define FIXEDRANK_MOTE9   0x0900
#define FIXEDRANK_MOTE10  0x0A00
#define FIXEDRANK_MOTE11  0x0B00
#define FIXEDRANK_MOTE12  0x0C00
#define FIXEDRANK_MOTE13  0x0D00
#define FIXEDRANK_MOTE14  0x0E00
#define FIXEDRANK_MOTE15  0x0F00

/* best channel */
#if (ENABLE_MULTICHANNEL == 1)
#define CONTROL_CHAN  19 //used in ARM and AMCA protocols

#define MOTE0_BC   20
#define MOTE1_BC   21
#define MOTE2_BC   22
#define MOTE3_BC   23
#define MOTE4_BC   24
#define MOTE5_BC   25
#define MOTE6_BC   26
#define MOTE7_BC   19
#define MOTE8_BC   18
#define MOTE9_BC   17
#define MOTE10_BC  16
#define MOTE11_BC  16
#define MOTE12_BC  15
#define MOTE13_BC  14
#define MOTE14_BC  13
#define MOTE15_BC  12

#else
#define CONTROL_CHAN  20 //used in ARM and AMAC protocols

#define MOTE0_BC   20

#define MOTE1_BC   20
#define MOTE2_BC   20
#define MOTE3_BC   20
#define MOTE4_BC   20
#define MOTE5_BC   20

#define MOTE6_BC   20
#define MOTE7_BC   20
#define MOTE8_BC   20
#define MOTE9_BC   20
#define MOTE10_BC  20

#define MOTE11_BC  20
#define MOTE12_BC  20
#define MOTE13_BC  20
#define MOTE14_BC  20
#define MOTE15_BC  20
#endif

#define MAX_NUM_MOTES 16   //este define eh usado na montagem da topologia
//=========================== typedef =========================================

#if (IEEE802154E_TSCH == 0)
BEGIN_PACK
typedef struct {
   bool             used;
   uint8_t          parentPreference;
   bool             stableNeighbor;
   uint8_t          switchStabilityCounter;
   open_addr_t      addr_16b;
   uint16_t         numTxErr;
   uint16_t         numTxDIO;
   uint16_t         numTxDIOErr;
   uint16_t         numTxDAO;
   uint16_t         numTxDAOErr;
   uint16_t         numTxCOAP;
   uint16_t         numTxCOAPErr;
   uint16_t         numTxHello;
   uint8_t          bestchan;           // bestchannel
   uint16_t         lasttxidletime[2];  // contem os tempos dos ultimos txidle para aquele vizinho baseado na livelist
   uint16_t         lastrxHA[2];        // contem os tempos dos ultimos rxHA para aquele vizinho baseado na livelist
   uint8_t          broadcastPending;   // quando msg broadcast flag indica se enviou msg
   uint16_t         weight;             // O peso define a prioridade - menores pesos sao enviados primeiro
} macneighborRow_t;
END_PACK
typedef struct {
   macneighborRow_t     neighbors[MAXNUMMACNEIGHBORS];
   uint8_t              mybestchan;   //bestchannel
   uint8_t              controlchan;   //controlchannel used in the protocols ARM and AMAC
} macneighbors_vars_t;

typedef struct {
   uint16_t addr16b;
   uint8_t  bc;
}forceNeighbor_ele_t;
typedef struct {
   uint8_t numNeighbor;
   forceNeighbor_ele_t ele[MAXNUMMACNEIGHBORS];
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


#if (IEEE802154E_TSCH == 0)
//esta tabela de vizinhos eh diferentes do RPL pois o do rpl somente tem os vizinhos que entraram no RPL...
//esta outra tem a ver com todos os vizinhos que mandaram ola para o mote
extern macneighbors_vars_t macneighbors_vars;
#endif
//=========================== prototypes ======================================

void          neighbors_init(void);
bool neighbors_isValidNeighbor(uint8_t index);
// getters
uint8_t macneigh_getBChan(open_addr_t* address);
uint8_t macneigh_getBChanMultiCast(uint8_t *bestchannel,open_addr_t* targetaddr, uint8_t);
uint8_t macneighbors_getMyBestChan(void);
uint8_t macneighbors_getFixedBestChan(uint8_t* paddr);
bool macisThisAddressPendingBroadcast(open_addr_t* address) ;
bool macisThisAddressmyneighboor(open_addr_t* address) ;

dagrank_t     neighbors_getMyDAGrank(void);
uint8_t       neighbors_getNumNeighbors(void);
bool          neighbors_getPreferredParentEui64(open_addr_t* addressToWrite);
open_addr_t*  neighbors_getKANeighbor(uint16_t kaPeriod);
uint8_t neighbors_howmanyIhave(void);
void testerffimprimelivelist(void);
uint8_t macneighbors_getNumNeighbors(void);
uint8_t macneighbors_setBroadCastPending(uint8_t );
uint8_t macneighbors_setPendingOnlyFirst(open_addr_t *paddr);
uint8_t macneighbors_clearBroadcastPending(open_addr_t actualsrcaddr);
uint16_t macneigh_getTxIdleTime(open_addr_t* address);
void macneighbors_updtlivelist(open_addr_t actualsrcaddr, uint16_t txidletime, uint8_t status);
void macneighbors_clearlivelist(open_addr_t actualsrcaddr);
uint8_t macneighbors_calcbroadcastsent(void);
void macneighbors_updtstatistics(open_addr_t address, uint8_t frameType,uint8_t status);
void macneighbors_clearlivelistweigth(void);
uint8_t macneighbors_getControlChan(void);

// interrogators
bool          neighbors_isStableNeighbor(open_addr_t* address);
bool          neighbors_isPreferredParent(open_addr_t* address);
bool          neighbors_isNeighborWithLowerDAGrank(uint8_t index);
bool          neighbors_isNeighborWithHigherDAGrank(uint8_t index);
uint8_t macneighbors_getaddrbroadcastsent(open_addr_t *address);
open_addr_t getAddressPendingBroadcast(void);

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
