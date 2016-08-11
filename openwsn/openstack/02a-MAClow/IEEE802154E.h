#ifndef __IEEE802154E_H
#define __IEEE802154E_H

/**
\addtogroup MAClow
\{
\addtogroup IEEE802154E
\{
*/

#include "opendefs.h"
#include "board.h"
#include "schedule.h"
#include "processIE.h"

//=========================== debug define ====================================


//=========================== define ==========================================
#define RIT_CLOCK 32768/1000

//time in milisecond
#define RIT100ms   0
#define RIT200ms   1
#define RIT400ms   0
#define RIT500ms   0
#define RIT1000ms  0

//valores dos ticks
//    1 ms -    33 ticks
//   10 ms -   328 ticks
//   12 ms -   393 ticks
//   25 ms -   819 ticks
//   50 ms -  1638 ticks
//  100 ms -  3277 ticks
//  200 ms -  6554 ticks
//  300 ms -  9830 ticks
//  400 ms - 13107 ticks
//  500 ms - 16384 ticks   --> AQUI JA ESTOURA O SCHED_TIMER que vai ate 16500
// 1000 ms - 32768 ticks   --> AQUI JA ESTOURA O SCHED_TIMER que vai ate 16500

#define TICK_MAC_RITMC_PERIOD   33


#if (RIT100ms == 1)
	#define TICK_MAC_RIT_PERIOD           3277
	#define TICK_MAC_RIT_RX_TO_TX_PERIOD  TICK_MAC_RIT_PERIOD + 819 //RIT + 25 ms - tenho que garantir ao menos um ola por periodo
	#define TICK_MAC_RIT_TX_PERIOD        TICK_MAC_RIT_RX_TO_TX_PERIOD
	#define TICK_MAC_RIT_RX_WIND_PERIOD   1638 // 50 ms -  O Rx_wind pode ser pequeno...tamanho da msg
	#define TICK_RIT_ACK_WAIT_PERIOD      (TICK_MAC_RIT_RX_WIND_PERIOD / 2)
#elif (RIT200ms == 1)
	#define TICK_MAC_RIT_PERIOD           6554
	#define TICK_MAC_RIT_RX_TO_TX_PERIOD  TICK_MAC_RIT_PERIOD*2  //400ms - tenho que garantir ao menos um ola por periodo
	#define TICK_MAC_RIT_TX_PERIOD        TICK_MAC_RIT_PERIOD + 3277
	#define TICK_MAC_RIT_RX_WIND_PERIOD   3277 // 100 ms
	#define TICK_RIT_ACK_WAIT_PERIOD      (TICK_MAC_RIT_RX_WIND_PERIOD / 2)
#elif (RIT400ms == 1)
	#define TICK_MAC_RIT_PERIOD           13107
	#define TICK_MAC_RIT_RX_TO_TX_PERIOD  TICK_MAC_RIT_PERIOD + 1638 //1638 =  50 ms   RIT + 50 ms - tenho que garantir ao menos um ola por periodo
	#define TICK_MAC_RIT_TX_PERIOD        TICK_MAC_RIT_RX_TO_TX_PERIOD
	#define TICK_MAC_RIT_RX_WIND_PERIOD   1638 //1638 =  50 ms       O Rx_wind pode ser pequeno...tamanho da msg
	#define TICK_RIT_ACK_WAIT_PERIOD      (TICK_MAC_RIT_RX_WIND_PERIOD / 2)
#elif (RIT500ms == 1)
	#define TICK_MAC_RIT_PERIOD           16384
	#define TICK_MAC_RIT_RX_TO_TX_PERIOD  TICK_MAC_RIT_PERIOD +1638 //1638 =  50 ms   RIT + 50 ms - tenho que garantir ao menos um ola por periodo
	#define TICK_MAC_RIT_TX_PERIOD        TICK_MAC_RIT_RX_TO_TX_PERIOD
	#define TICK_MAC_RIT_RX_WIND_PERIOD   1638 //1638 =  50 ms       O Rx_wind pode ser pequeno...tamanho da msg
	#define TICK_RIT_ACK_WAIT_PERIOD      (TICK_MAC_RIT_RX_WIND_PERIOD / 2)
#elif (RIT1000ms == 1)
	#define TICK_MAC_RIT_PERIOD           32768
	#define TICK_MAC_RIT_RX_TO_TX_PERIOD  TICK_MAC_RIT_PERIOD +1638 //1638 =  50 ms   RIT + 50 ms - tenho que garantir ao menos um ola por periodo
	#define TICK_MAC_RIT_TX_PERIOD        TICK_MAC_RIT_RX_TO_TX_PERIOD
	#define TICK_MAC_RIT_RX_WIND_PERIOD   1638 //1638 =  50 ms       O Rx_wind pode ser pequeno...tamanho da msg
	#define TICK_RIT_ACK_WAIT_PERIOD      (TICK_MAC_RIT_RX_WIND_PERIOD / 2)
#endif


#if 0
#define TICK_MAC_RIT_PERIOD           6554 //16384 =  500 ms       START_MAC_RIT_PERIOD*RIT_CLOCK
//#define TICK_MAC_RIT_PERIOD           32768 //32768 = 1000 ms       START_MAC_RIT_PERIOD*RIT_CLOCK
#define TICK_MAC_RIT_RX_TO_TX_PERIOD  TICK_MAC_RIT_PERIOD +1638 //1638 =  50 ms   RIT + 50 ms - tenho que garantir ao menos um ola por periodo
#define TICK_MAC_RIT_TX_PERIOD        TICK_MAC_RIT_RX_TO_TX_PERIOD
#define TICK_MAC_RIT_RX_WIND_PERIOD   1638 //1638 =  50 ms       O Rx_wind pode ser pequeno...tamanho da msg
//#define TICK_MAC_RIT_RX_WIND_PERIOD   16384 //16384 =  500 ms       O Rx_wind pode ser pequeno...tamanho da msg
#define TICK_RIT_ACK_WAIT_PERIOD      (TICK_MAC_RIT_RX_WIND_PERIOD / 2)

/*
#define TICK_MAC_RIT_RX_TO_TX_PERIOD  36045 //16384 =  500 ms        START_MAC_RIT_RX_TO_TX_PERIOD*RIT_CLOCK
#define TICK_MAC_RIT_RX_WIND_PERIOD   16384 //16384 =  500 ms        START_MAC_RIT_RX_WIND_PERIOD*RIT_CLOCK
#define TICK_MAC_RIT_PERIOD           22938 //32768 = 1000 ms        START_MAC_RIT_PERIOD*RIT_CLOCK
#define TICK_MAC_RIT_TX_PERIOD        49152 //32768 = 1000 ms        START_MAC_RIT_PERIOD*RIT_CLOCK
#define TICK_RIT_ACK_WAIT_PERIOD      8192  //8192  =  250 ms        RIT_ACK_WAIT_PERIOD*RIT_CLOCK
*/

#endif

#define SYNCHRONIZING_CHANNEL       20 // channel the mote listens on to synchronize
#define TXRETRIES                    3 // number of MAC retries before declaring failed
#define TX_POWER                    31 // 1=-25dBm, 31=0dBm (max value)
#define RESYNCHRONIZATIONGUARD       5 // in 32kHz ticks. min distance to the end of the slot to successfully synchronize
#define US_PER_TICK                 30 // number of us per 32kHz clock tick
#define ADVTIMEOUT                  30 // in seconds: sending ADV every 30 seconds
#define MAXKAPERIOD               2000 // in slots: @15ms per slot -> ~30 seconds. Max value used by adaptive synchronization.
#define DESYNCTIMEOUT             2333 // in slots: @15ms per slot -> ~35 seconds. A larger DESYNCTIMEOUT is needed if using a larger KATIMEOUT.
#define LIMITLARGETIMECORRECTION     5 // threshold number of ticks to declare a timeCorrection "large"
#define LENGTH_IEEE154_MAX         128 // max length of a valid radio packet  
#define DUTY_CYCLE_WINDOW_LIMIT    (0xFFFFFFFF>>1) // limit of the dutycycle window

//15.4e information elements related
#define IEEE802154E_PAYLOAD_DESC_LEN_SHIFT                 0x04
#define IEEE802154E_PAYLOAD_DESC_GROUP_ID_MLME             (1<<1)
#define IEEE802154E_DESC_TYPE_LONG                         0x01
#define IEEE802154E_DESC_TYPE_SHORT                        0x00

#define IEEE802154E_DESC_TYPE_HEADER_IE                    0x00
#define IEEE802154E_DESC_TYPE_PAYLOAD_IE                   0x01
//len field on PAYLOAD/HEADER DESC
#define IEEE802154E_DESC_LEN_HEADER_IE_MASK                0xFE00
#define IEEE802154E_DESC_LEN_PAYLOAD_IE_MASK               0xFFE0

#define IEEE802154E_DESC_LEN_HEADER_IE_SHIFT               9
#define IEEE802154E_DESC_LEN_PAYLOAD_IE_SHIFT              5

//groupID/elementID field on PAYLOAD/HEADER DESC
#define IEEE802154E_DESC_ELEMENTID_HEADER_IE_MASK          0x01FE
#define IEEE802154E_DESC_GROUPID_PAYLOAD_IE_MASK           0x001E

#define IEEE802154E_DESC_ELEMENTID_HEADER_IE_SHIFT         1
#define IEEE802154E_DESC_GROUPID_PAYLOAD_IE_SHIFT          1

//MLME Sub IE LONG page 83
#define IEEE802154E_DESC_LEN_LONG_MLME_IE_MASK             0xFFE0
#define IEEE802154E_DESC_SUBID_LONG_MLME_IE_MASK           0x001E

#define IEEE802154E_DESC_LEN_LONG_MLME_IE_SHIFT            5
#define IEEE802154E_DESC_SUBID_LONG_MLME_IE_SHIFT          1

//MLME Sub IE SHORT page 82
#define IEEE802154E_DESC_LEN_SHORT_MLME_IE_MASK            0xFF00
#define IEEE802154E_DESC_SUBID_SHORT_MLME_IE_MASK          0x00FE

#define IEEE802154E_DESC_LEN_SHORT_MLME_IE_SHIFT           8
#define IEEE802154E_DESC_SUBID_SHORT_MLME_IE_SHIFT         1

#define IEEE802154E_MLME_SYNC_IE_SUBID                     0x1A
#define IEEE802154E_MLME_SYNC_IE_SUBID_SHIFT               1
#define IEEE802154E_MLME_SLOTFRAME_LINK_IE_SUBID           0x1B
#define IEEE802154E_MLME_SLOTFRAME_LINK_IE_SUBID_SHIFT     1
#define IEEE802154E_MLME_TIMESLOT_IE_SUBID                 0x1c
#define IEEE802154E_MLME_TIMESLOT_IE_SUBID_SHIFT           1

#define IEEE802154E_MLME_IE_GROUPID                        0x01
#define IEEE802154E_ACK_NACK_TIMECORRECTION_ELEMENTID      0x1E

/**
When a packet is received, it is written inside the OpenQueueEntry_t->packet
buffer, starting at the byte defined below. When a packet is relayed, it
traverses the stack in which the MAC and IPHC headers are parsed and stripped
off, then put on again. During that process, the IPv6 hop limit field is
decremented. Depending on the new value of the hop limit, the IPHC header
compression algorithm might not be able to compress it, and hence has to carry
it inline, adding a byte to the header. To avoid having to move bytes around
inside OpenQueueEntry_t->packet buffer, we start writing the received packet a
bit after the start of the packet.
*/
#define FIRST_FRAME_BYTE             1


#if (IEEE802154E_RIT == 1) || (IEEE802154E_AMAC == 1) || (IEEE802154E_AMCA == 1) ||(IEEE802154E_RITMC == 1)
// the different states of the IEEE802.15.4e state machine
typedef enum {
   S_RIT_SLEEP                   = 0x00,   // ready for next slot
   S_RIT_RXSLEEP                 = 0x01,   // Finish the rx Message
   // TX
   S_RIT_TXDATAOFFSET            = 0x02,   // waiting to prepare for Tx data
   //S_RIT_TXDATAPREPARE           = 0x05,   // preparing for Tx data
   S_RIT_TXDATAREADY             = 0x03,   // ready to Tx data, waiting for 'go'
   S_RIT_TXSLEEP                 = 0x07,   // Finish the tx Message
   S_RIT_TXDATA                  = 0x04,   // Tx data SFD received, sending bytes

   S_RIT_RXACKOFFSET             = 0x05,   // Tx data done, waiting to prepare for Rx ACK
   S_RIT_RXNEWFRM                = 0x0a,   // RX new frame - usando tanto pelo TX quanto pelo RX

   //S_RIT_RXACKREADY              = 0x0b,   // ready to Rx ACK, waiting for 'go'
   //S_RIT_RXACKLISTEN             = 0x0c,   // idle listening for ACK
   S_RIT_RXACK                   = 0x06,   // Rx ACK SFD received, receiving bytes
   S_RIT_TXPROC                  = 0x07,   // processing sent data
   // RX
   S_RIT_RXDATAOFFSET            = 0x08,   // waiting to prepare for Rx data
   S_RIT_RXDATAPREPARE           = 0x09,   // preparing for Rx data
   S_RIT_RXDATAREADY             = 0x10,   // ready to Rx data, waiting for 'go'
   S_RIT_RXDATALISTEN            = 0x11,   // idle listening for data
   S_RIT_RXDATA                  = 0x12,   // data SFD received, receiving more bytes
   S_RIT_TXACKOFFSET             = 0x13,   // waiting to prepare for Tx ACK
   S_RIT_TXACKPREPARE            = 0x15,   // preparing for Tx ACK
   S_RIT_TXACKREADY              = 0x15,   // Tx ACK ready, waiting for 'go'
   S_RIT_TXACKDELAY              = 0x16,   // 'go' signal given, waiting for SFD Tx ACK
   S_RIT_TXACK                   = 0x17,   // Tx ACK SFD received, sending bytes
   S_RIT_RXPROC                  = 0x18,   // processing received data
   S_RIT_TX_CONTINUEWAIT         = 0x19,   // received a invalid packet then open window again

   S_RIT_RXOLAPREPARE        = 0x20,   // Open the window to receive data !!!!!
   S_RIT_RXOLA               = 0x21,   // Open the window to receive data !!!!!
   S_RIT_RXOLANOECHO         = 0x22,   // Open the window to receive data !!!!!
   S_RIT_TXDATANOECHO        = 0x23,    // nao houve echo para um tx
   S_RIT_RXNOACK             = 0x24,    // Atividade do Tx (transmissor) - esperou um ack mas nao ocorreu
   S_RIT_OLAACKPREPARE       = 0x25,
   S_RIT_OLAACK              = 0x26,
   S_RIT_OLAACKECHO          = 0x27,
   S_RIT_NOOLAACKECHO        = 0x28,
   S_RIT_TXOLA               = 0x29,
   S_RIT_TXOLAPREPARE        = 0x30,
   S_RIT_RXOLAACK            = 0x31,
   S_RIT_RXOLAACKPREPARE     = 0x32,
   S_RIT_TXDATAECHO          = 0x33,
   S_RIT_NOECHOTXACK         = 0x34,
   S_RIT_TXDATAPREPARE       = 0x35,
   S_RIT_RXOLA1              = 0x36,
   S_RIT_TXMULTICAST         = 0x37,
   S_RIT_TXMULTICASTTIMEOUT  = 0x38,
} ieee154e_state_t;



#else
// the different states of the IEEE802.15.4e state machine
typedef enum {
   S_SLEEP                   = 0x00,   // ready for next slot
   // synchronizing
   S_SYNCLISTEN              = 0x01,   // listened for packet to synchronize to network
   S_SYNCRX                  = 0x02,   // receiving packet to synchronize to network
   S_SYNCPROC                = 0x03,   // processing packet just received
   // TX
   S_TXDATAOFFSET            = 0x04,   // waiting to prepare for Tx data
   S_TXDATAPREPARE           = 0x05,   // preparing for Tx data
   S_TXDATAREADY             = 0x06,   // ready to Tx data, waiting for 'go'
   S_TXDATADELAY             = 0x07,   // 'go' signal given, waiting for SFD Tx data
   S_TXDATA                  = 0x08,   // Tx data SFD received, sending bytes
   S_RXACKOFFSET             = 0x09,   // Tx data done, waiting to prepare for Rx ACK
   S_RXACKPREPARE            = 0x0a,   // preparing for Rx ACK
   S_RXACKREADY              = 0x0b,   // ready to Rx ACK, waiting for 'go'
   S_RXACKLISTEN             = 0x0c,   // idle listening for ACK
   S_RXACK                   = 0x0d,   // Rx ACK SFD received, receiving bytes
   S_TXPROC                  = 0x0e,   // processing sent data
   // RX
   S_RXDATAOFFSET            = 0x0f,   // waiting to prepare for Rx data
   S_RXDATAPREPARE           = 0x10,   // preparing for Rx data
   S_RXDATAREADY             = 0x11,   // ready to Rx data, waiting for 'go'
   S_RXDATALISTEN            = 0x12,   // idle listening for data
   S_RXDATA                  = 0x13,   // data SFD received, receiving more bytes
   S_TXACKOFFSET             = 0x14,   // waiting to prepare for Tx ACK
   S_TXACKPREPARE            = 0x15,   // preparing for Tx ACK
   S_TXACKREADY              = 0x16,   // Tx ACK ready, waiting for 'go'
   S_TXACKDELAY              = 0x17,   // 'go' signal given, waiting for SFD Tx ACK
   S_TXACK                   = 0x18,   // Tx ACK SFD received, sending bytes
   S_RXPROC                  = 0x19,   // processing received data
} ieee154e_state_t;

#endif

// Atomic durations
// expressed in 32kHz ticks:
//    - ticks = duration_in_seconds * 32768
//    - duration_in_seconds = ticks / 32768
enum ieee154e_atomicdurations_enum {
   // time-slot related
   TsTxOffset                =  131,                  //  4000us
   TsLongGT                  =   43,                  //  1300us
   TsTxAckDelay              =  151,                  //  4606us
   TsShortGT                 =   16,                  //   500us
   TsSlotDuration            =  PORT_TsSlotDuration,  // 15000us
   // execution speed related
   maxTxDataPrepare          =  PORT_maxTxDataPrepare,
   maxRxAckPrepare           =  PORT_maxRxAckPrepare,
   maxRxDataPrepare          =  PORT_maxRxDataPrepare,
   maxTxAckPrepare           =  PORT_maxTxAckPrepare,
   // radio speed related
   delayTx                   =  PORT_delayTx,         // between GO signal and SFD
   delayRx                   =  PORT_delayRx,         // between GO signal and start listening
   // radio watchdog
   wdRadioTx                 =   33,                  //  1000us (needs to be >delayTx)
   wdDataDuration            =  164,                  //  5000us (measured 4280us with max payload)
   wdAckDuration             =   98,                  //  3000us (measured 1000us)
};

//shift of bytes in the linkOption bitmap
enum ieee154e_linkOption_enum {
   FLAG_TX_S                 = 7,
   FLAG_RX_S                 = 6,
   FLAG_SHARED_S             = 5,
   FLAG_TIMEKEEPING_S        = 4,   
};

// FSM timer durations (combinations of atomic durations)
// TX
#define DURATION_tt1 ieee154e_vars.lastCapturedTime+TsTxOffset-delayTx-maxTxDataPrepare
#define DURATION_tt2 ieee154e_vars.lastCapturedTime+TsTxOffset-delayTx
#define DURATION_tt3 ieee154e_vars.lastCapturedTime+TsTxOffset-delayTx+wdRadioTx
#define DURATION_tt4 ieee154e_vars.lastCapturedTime+wdDataDuration
#define DURATION_tt5 ieee154e_vars.lastCapturedTime+TsTxAckDelay-TsShortGT-delayRx-maxRxAckPrepare
#define DURATION_tt6 ieee154e_vars.lastCapturedTime+TsTxAckDelay-TsShortGT-delayRx
#define DURATION_tt7 ieee154e_vars.lastCapturedTime+TsTxAckDelay+TsShortGT
#define DURATION_tt8 ieee154e_vars.lastCapturedTime+wdAckDuration
// RX
#define DURATION_rt1 ieee154e_vars.lastCapturedTime+TsTxOffset-TsLongGT-delayRx-maxRxDataPrepare
#define DURATION_rt2 ieee154e_vars.lastCapturedTime+TsTxOffset-TsLongGT-delayRx
#define DURATION_rt3 ieee154e_vars.lastCapturedTime+TsTxOffset+TsLongGT
#define DURATION_rt4 ieee154e_vars.lastCapturedTime+wdDataDuration
#define DURATION_rt5 ieee154e_vars.lastCapturedTime+TsTxAckDelay-delayTx-maxTxAckPrepare
#define DURATION_rt6 ieee154e_vars.lastCapturedTime+TsTxAckDelay-delayTx
#define DURATION_rt7 ieee154e_vars.lastCapturedTime+TsTxAckDelay-delayTx+wdRadioTx
#define DURATION_rt8 ieee154e_vars.lastCapturedTime+wdAckDuration


// TX
#define RIT_DURATION_tt1 TsTxOffset-delayTx-maxTxDataPrepare
#define RIT_DURATION_tt2 TsTxOffset-delayTx
#define RIT_DURATION_tt3 TsTxOffset-delayTx+wdRadioTx
#define RIT_DURATION_tt4 500 //wdDataDuration
#define RIT_DURATION_tt5 TsTxAckDelay-TsShortGT-delayRx-maxRxAckPrepare
#define RIT_DURATION_tt6 TsTxAckDelay-TsShortGT-delayRx
#define RIT_DURATION_tt7 TsTxAckDelay+TsShortGT
#define RIT_DURATION_tt8 wdAckDuration
// RX
#define RIT_DURATION_rt1 TsTxOffset-TsLongGT-delayRx-maxRxDataPrepare
#define RIT_DURATION_rt2 TsTxOffset-TsLongGT-delayRx
#define RIT_DURATION_rt3 TsTxOffset+TsLongGT
#define RIT_DURATION_rt4 wdDataDuration
#define RIT_DURATION_rt5 TsTxAckDelay-delayTx-maxTxAckPrepare
#define RIT_DURATION_rt6 TsTxAckDelay-delayTx
#define RIT_DURATION_rt7 TsTxAckDelay-delayTx+wdRadioTx
#define RIT_DURATION_rt8 wdAckDuration

#define RIT_RX_TO_TX_PERIOD ieee154e_vars.lastCapturedTime + TICK_MAC_RIT_RX_TO_TX_PERIOD
#define RIT_RX_WIND_PERIOD  ieee154e_vars.lastCapturedTime + TICK_MAC_RIT_RX_WIND_PERIOD


//=========================== typedef =========================================

// IEEE802.15.4E acknowledgement (ACK)
typedef struct {
   PORT_SIGNED_INT_WIDTH timeCorrection;
} IEEE802154E_ACK_ht;

// includes payload header IE short + MLME short Header + Sync IE
#define ADV_PAYLOAD_LENGTH sizeof(payload_IE_ht) + \
                           sizeof(mlme_IE_ht)     + \
                           sizeof(sync_IE_ht)

//=========================== module variables ================================

typedef struct {
   // misc
   asn_t                     asn;                     // current absolute slot number
   slotOffset_t              slotOffset;              // current slot offset
   slotOffset_t              nextActiveSlotOffset;    // next active slot offset
   PORT_RADIOTIMER_WIDTH     deSyncTimeout;           // how many slots left before looses sync
   bool                      isSync;                  // TRUE iff mote is synchronized to network
   // as shown on the chronogram
   ieee154e_state_t          state;                   // state of the FSM
   OpenQueueEntry_t*         dataToSend;              // pointer to the data to send
   OpenQueueEntry_t*         dataReceived;            // pointer to the data received
   OpenQueueEntry_t*         ackToSend;               // pointer to the ack to send
   OpenQueueEntry_t*         ackReceived;             // pointer to the ack received
   PORT_RADIOTIMER_WIDTH     lastCapturedTime;        // last captured time
   PORT_RADIOTIMER_WIDTH     syncCapturedTime;        // captured time used to sync
   // channel hopping
   uint8_t                   freq;                    // frequency of the current slot
   uint8_t                   asnOffset;               // offset inside the frame
   
   PORT_RADIOTIMER_WIDTH     radioOnInit;             // when within the slot the radio turns on
   PORT_RADIOTIMER_WIDTH     radioOnTics;             // how many tics within the slot the radio is on
   bool                      radioOnThisSlot;         // to control if the radio has been turned on in a slot.

   uint8_t                   RITQueue_ElementPending;      //salvo o elemento atual na lista do RIT para ser enviado
   uint8_t                   macRIT_Pending_TX_frameType;  //RIT - flag TX message pending
   open_addr_t               targetaddr;                   //quando broadcast o atual endereco que estou buscando
} ieee154e_vars_t;

BEGIN_PACK
typedef struct {
   uint8_t                   numSyncPkt;              // how many times synchronized on a non-ACK packet
   uint8_t                   numSyncAck;              // how many times synchronized on an ACK
   int16_t                   minCorrection;           // minimum time correction
   int16_t                   maxCorrection;           // maximum time correction
   uint8_t                   numDeSync;               // number of times a desync happened
   uint32_t                  numTicsOn;               // mac dutyCycle
   uint32_t                  numTicsTotal;            // total tics for which the dutycycle is computed
} ieee154e_stats_t;
END_PACK

typedef struct {
   PORT_RADIOTIMER_WIDTH     num_newSlot;
   PORT_RADIOTIMER_WIDTH     num_timer;
   PORT_RADIOTIMER_WIDTH     num_startOfFrame;
   PORT_RADIOTIMER_WIDTH     num_endOfFrame;
   PORT_RADIOTIMER_WIDTH     num_txslot;
   PORT_RADIOTIMER_WIDTH     num_rxslot;
   PORT_RADIOTIMER_WIDTH     num_txend;
   PORT_RADIOTIMER_WIDTH     num_rxend;
} ieee154e_dbg_t;

//=========================== prototypes ======================================

// admin
void               ieee154e_init(void);
// public
PORT_RADIOTIMER_WIDTH   ieee154e_asnDiff(asn_t* someASN);
bool               ieee154e_isSynch(void);
void               ieee154e_getAsn(uint8_t* array);
// events
void               ieee154e_startOfFrame(PORT_RADIOTIMER_WIDTH capturedTime);
void               ieee154e_endOfFrame(PORT_RADIOTIMER_WIDTH capturedTime);
// misc
bool               debugPrint_asn(void);
bool               debugPrint_isSync(void);
bool               debugPrint_macStats(void);

/**
\}
\}
*/

#endif
