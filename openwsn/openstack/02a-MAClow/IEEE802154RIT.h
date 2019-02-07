#ifndef __IEEE802154RIT_H
#define __IEEE802154RIT_H

#include "IEEE802154E.h"
#if 1
#define RITRX_50    0     //program RIT 50 ms
#define RITRX_100   1     //program RIT 100 ms
#define RITRX_200   0     //program RIT 200 ms
#define RITRX_500   0     //program RIT 500 ms   -> ESTE EH O MAXIMO SUPORTADO ATUALMENTE!!!!!

#define RX_RIT_PERIOD_MS            100
#define RX_RIT_TXOLAPREPARE_MS        1
#define RX_RIT_TXOLAECHO_MS          15     //posso colocar este tempo min=5ms para ola de 12 bytes
#define RX_RIT_DELAY_TX_TO_RX_MS      1
#define RX_RIT_TIMEOUT_MS            95      //Teste == estava 50ms...e estava bom..
#define RX_RIT_TXACKEND_MS           10
#define RX_RIT_DELAY_TX2RX_US       200
#define RX_RIT_DELAY_TX2RX_MS         1

#define TX_RIT_TIMEOUT_MS          RX_RIT_PERIOD_MS+200     // este valor ficou bom...300
#define TX_RIT_PERIOD_MS           TX_RIT_TIMEOUT_MS+50    //ficou bom..350
#define TX_RIT_RXOLAPREPARE_MS       1
#define TX_RIT_TXDATAECHO_MS        50
#define TX_RIT_DELAYRXTX_MS          1
#define TX_RIT_DELAYCONTWAIT_MS      1
#define TX_RIT_ACK_TIMEOUT_MS       50                         //valor bom 100
#define TX_RIT_TXOLAACKECHO_MS      10      //Para enviar receber o echo de 11 bytes foi na ordem de 1 a 3 ms em media
#define TX_RIT_CW_TIMEOUT_MS         5      //5ms funcionou...

//Service RIT MultiChannelHello - RIT Neighboorhood Discover (time in ms)
#define RIT_MULTICHANNELHELLO_MS   1000


#define RIT_RX_ONLY 0
#define RIT_RX_SNIFFER 1

//ticks
#if (ENABLE_CSMA_CA == 0)
//este tempo dependo do setting do timer...TSCH o clock é de 30 us
#define MACTIMERCLOCK 32768
#else
//este tempo dependo do setting do timer...para CSMACA clock eh de 320us=3125Hz
#define MACTIMERCLOCK 3125
#endif

#define CONVERT_MS_IN_TICKS(s)     (MACTIMERCLOCK * s)/1000

#define TX_RIT_PERIOD_TICKS           CONVERT_MS_IN_TICKS(TX_RIT_PERIOD_MS)
#define TX_RIT_TIMEOUT_TICKS          CONVERT_MS_IN_TICKS(TX_RIT_TIMEOUT_MS)
#define TX_RIT_TXDATAECHO_TICKS       CONVERT_MS_IN_TICKS(TX_RIT_TXDATAECHO_MS)
#define TX_RIT_DELAYRXTX_TICKS        CONVERT_MS_IN_TICKS(TX_RIT_DELAYRXTX_MS)
#define TX_RIT_ACK_TIMEOUT_TICKS      CONVERT_MS_IN_TICKS(TX_RIT_ACK_TIMEOUT_MS)

#define RX_RIT_PERIOD_TICKS            CONVERT_MS_IN_TICKS(RX_RIT_PERIOD_MS)
#define RX_RIT_TXOLAPREPARE_TICKS      50 //CONVERT_MS_IN_TICKS(RX_RIT_TXOLAPREPARE_MS)
#define RX_RIT_TXOLAECHO_TICKS         CONVERT_MS_IN_TICKS(RX_RIT_TXOLAECHO_MS)
#define RX_RIT_DELAY_TX_TO_RX_TICKS    CONVERT_MS_IN_TICKS(RX_RIT_DELAY_TX_TO_RX_MS)
#define RX_RIT_TIMEOUT_TICKS           CONVERT_MS_IN_TICKS(RX_RIT_TIMEOUT_MS)
#define RX_RIT_ACKECHO_TIMEOUT_TICKS   CONVERT_MS_IN_TICKS(RX_RIT_ACKECHO_TIMEOUT_MS)


// the different states of the IEEE802.15.4e state machine
typedef enum {
   S_RIT_SLEEP                   = 0x00,   // ready for next slot
// RX
   S_RIT_TXOLAPREPARE          ,   //rx00
   S_RIT_TXOLA                 ,   //rx01
   S_RIT_TXOLAERROR            ,   //
   S_RIT_RXNEWDATA               ,   // rx04
   S_RIT_LLTXACKOFFSET           ,
   S_RIT_RXDATATIMEOUT           ,   // ready to Rx ACK, waiting for 'go'
   S_RIT_RXDATAPREPARE           ,   // preparing for Rx data
   S_RIT_RXDATA                  ,   // data SFD received, receiving more bytes
   S_RIT_TXACKPREPARE            ,   // preparing for Tx ACK
   S_RIT_TXACK                   ,   // Rx07
   S_RIT_TXACKERROR              ,   //Rxe07
   S_RIT_RXPROC                  ,   // Rx08
   S_RIT_TX_CONTINUEWAIT         ,   // received a invalid packet then open window again
// TX
   S_RIT_RXOLAOFFSET       = 0x50,  //tx00
   S_RIT_RXOLAPREPARE      ,   // tx02
   S_RIT_RXOLA             ,   // tx03
   S_RIT_OLATIMEOUT        ,   // txe03
   S_RIT_RXNEWOLA          ,
   S_RIT_TXDATAOFFSET      ,   // tx40
   S_RIT_TXDATAREADY       ,   // tx05
   S_RIT_TXDATA            ,   // tx06
   S_RIT_TXDATAERROR       ,   // txe06
   S_RIT_RXACKOFFSET       ,   // tx07
   S_RIT_RXACKPREPARE      ,   // tx08
   S_RIT_RXACK             ,   // tx81
   S_RIT_RXNOACK           ,   // txe08
   S_RIT_TXPROC            ,   // tx09

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
