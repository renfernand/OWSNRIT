#ifndef __IEEE802154ARM_H
#define __IEEE802154ARM_H


#include "IEEE802154E.h"
#if 1

#define RX_ARM_PERIOD_MS            100
#define RX_ARM_TXOLAPREPARE_MS        1
#define RX_ARM_TXOLAECHO_MS          15
#define RX_ARM_DELAYTX2RX_MS          1
#define RX_ARM_RTSTIMEOUT_MS         30
#define RX_ARM_DELAYRTS2CTS_MS        1
#define RX_ARM_DELAYCTS2DATA_MS       1
#define RX_ARM_DELAYDATA2ACK_MS       1
#define RX_ARM_ACKECHO_MS            10
#define RX_ARM_OLATIMEOUT_MS         30
#define RX_ARM_DATATIMEOUT_MS        30
#define RX_ARM_CTSECHO_MS            10

#define TX_ARM_OLATIMEOUT_MS       RX_ARM_PERIOD_MS+250
#define TX_ARM_PERIOD_MS           TX_ARM_OLATIMEOUT_MS+50
#define TX_ARM_RXOLAPREPARE_MS       1
#define TX_ARM_DELAYCONTWAIT_MS      1
#define TX_ARM_DELAYRX2TX_MS         1
#define TX_ARM_RTSECHO_MS           10
#define TX_ARM_DELAYRTS2CTS_MS       1
#define TX_ARM_CTSTIMEOUT_MS        20
#define TX_ARM_DATAECHO_MS          10
#define TX_ARM_ACKTIMEOUT_MS        10

#define TX_ARM_SLEEP_MS            100
#define TX_ARM_TXOLAACKECHO_MS      10      //Para enviar receber o echo de 11 bytes foi na ordem de 1 a 3 ms em media
#define TX_ARM_CW_TIMEOUT_MS         5      //5ms funcionou...

//Service AMCA MultiChannelHello - RIT Neighboorhood Discover (time in ms)
#define ARM_MULTICHANNELHELLO_MS   MULTICHANNELHELLO_MS

#define ARM_RX_ONLY 0

#define ARM_CTS_CMD  0x15
#define ARM_RTS_CMD  0x10

// the different states of the IEEE802.15.4e state machine
typedef enum {
   S_ARM_SLEEP               = 0x00,   // ready for next slot
//RX
   S_ARM_TXOLAPREPARE        = 0x01,   //ARM Rx00
   S_ARM_TXOLA               = 0x02,   //ARM Rx01
   S_ARM_OLANOECHO           = 0x03,   //ARM Rxe01
   S_ARM_RXRTSPREPARE        = 0x04,   //ARM Rx02
   S_ARM_RXRTS               = 0x05,   //ARM Rx03
   S_ARM_RTSTIMEOUT          = 0x06,   //ARM Rxe03
   S_ARM_TXCTSOFFSET         = 0x07,   //ARM Rx04
   S_ARM_TXCTSPREPARE        = 0x08,   //ARM Rx ???
   S_ARM_TXCTS               = 0x09,   //ARM Rx05
   S_ARM_CTSNOECHO           = 0x0a,   //ARM Rxe05
   S_ARM_RXDATAPREPARE       = 0x0b,   //ARM Rx06
   S_ARM_RXDATA              = 0x0c,   //ARM Rx07
   S_ARM_DATATIMEOUT         = 0x0d,   //ARM Rxe07
   S_ARM_RXNEWDATA           = 0x0e,   //ARM Rx08
   S_ARM_TXACKOFFSET         = 0x0f,   //ARM Rx85
   S_ARM_TXACKPREPARE        = 0x10,   //ARM Rx09
   S_ARM_TXACK               = 0x11,   //ARM Rx0a
   S_ARM_ACKNOECHO           = 0x12,   //ARM Rxe0a
   S_ARM_RXPROC              = 0x13,   //ARM Rx0b
//TX
   S_ARM_RXOLAOFFSET         = 0x21,   //ARM tx00
   S_ARM_TXFRAME             = 0x22,   //ARM tx01
   S_ARM_TXFRMERROR          = 0x23,   //ARM txe01
   S_ARM_RXOLAPREPARE        = 0x24,   //ARM tx02
   S_ARM_RXOLA               = 0x25,   //ARM tx03
   S_ARM_OLATIMEOUT          = 0x26,   //ARM txe03
   S_ARM_RXNEWOLA            = 0x27,   //ARM tx04
   S_ARM_TX_CONTINUEWAIT     = 0x28,   //ARM tx41
   S_ARM_TXRTSOFFSET         = 0x29,   //ARM tx42
   S_ARM_TXRTSPREPARE        = 0x2a,   //ARM tx05
   S_ARM_TXRTS               = 0x2b,   //ARM tx05
   S_ARM_RTSNOECHO           = 0x2c,   //ARM txe05
   S_ARM_RXCTSPREPARE        = 0x2e,   //ARM tx06
   S_ARM_RXCTS               = 0x2f,   //ARM tx07
   S_ARM_CTSTIMEOUT          = 0x30,   //ARM txe07
   S_ARM_TXDATAREADY         = 0x31,   //ARM tx08
   S_ARM_TXDATA              = 0x32,   //ARM tx09
   S_ARM_DATANOECHO          = 0x33,   //ARM txe09
   S_ARM_RXACKOFFSET         = 0x34,   //ARM tx0a
   S_ARM_RXACK               = 0x35,   //ARM tx0b
   S_ARM_ACKTIMEOUT          = 0x36,   //ARM txe0b
   S_ARM_TXPROC              = 0x37,    //ARM tx0c
   S_ARM_LLTXACKOFFSET
} ieee154e_state_t;

typedef struct {
   // misc
   asn_t                     asn;                     // current absolute slot number
   slotOffset_t              slotOffset;              // current slot offset
   slotOffset_t              nextActiveSlotOffset;    // next active slot offset
   PORT_RADIOTIMER_WIDTH     deSyncTimeout;           // how many slots left before looses sync
   bool                      isSync;                  // TRUE iff mote is synchronized to network
   // as shown on the chronogram
   ieee154e_state_t          state;                   // state of the FSM
   ieee154e_state_t          laststate;                   // state of the FSM
   OpenQueueEntry_t*         dataToSend;              // pointer to the data to send
   OpenQueueEntry_t*         dataReceived;            // pointer to the data received
   OpenQueueEntry_t*         ackToSend;               // pointer to the ack to send
   OpenQueueEntry_t*         ackReceived;             // pointer to the ack received
   PORT_RADIOTIMER_WIDTH     lastCapturedTime;        // last captured time
   PORT_RADIOTIMER_WIDTH     syncCapturedTime;        // captured time used to sync
   uint8_t                   lastRSSI;                // captured last RX RSSI
   uint8_t                   lastlqi;                 // captured last RX LQI calculated
 // channel hopping
   uint8_t                   freq;                    // frequency of the current slot
   uint8_t                   asnOffset;               // offset inside the frame

   PORT_RADIOTIMER_WIDTH     radioRxOnInit;             // when within the slot the radio turns on
   PORT_RADIOTIMER_WIDTH     radioRxOnTics;             // how many tics within the slot the radio is on
   PORT_RADIOTIMER_WIDTH     radioTxOnInit;             // when within the slot the radio turns on
   PORT_RADIOTIMER_WIDTH     radioTxOnTics;             // when within the slot the radio turns on
   bool                      radioOnThisSlot;         // to control if the radio has been turned on in a slot.

   uint8_t                   RITQueue_ElementPending;      //salvo o elemento atual na lista do RIT para ser enviado
   uint8_t                   macRIT_Pending_TX_frameType;  //RIT - flag TX message pending
   open_addr_t               targetaddr;                   //quando broadcast o atual endereco que estou buscando
} ieee154e_vars_t;

enum discardfrm {
  DISCARD_NO_ENDSLOT_NO=0,
  DISCARD_NO_ENDSLOT_YES,
  DISCARD_YES_ENDSLOT_NO,
  DISCARD_YES_ENDSLOT_YES,
};
#endif
#endif
