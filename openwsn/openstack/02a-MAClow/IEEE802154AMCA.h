#ifndef __IEEE802154AMCA_H
#define __IEEE802154AMCA_H



#include "IEEE802154E.h"

#define RX_AMCA_PERIOD_MS            100
#define RX_AMCA_TXOLAPREPARE_MS        1
#define RX_AMCA_TXOLAECHO_MS          15     //posso colocar este tempo min=5ms para ola de 12 bytes
#define RX_AMCA_DELAY_TX_TO_RX_MS      1
#define RX_AMCA_TIMEOUT_MS            28      //Tempo maximo foi de 9,28ms x 3 = 28
#define RX_AMCA_TXACKEND_MS           10
#define RX_AMCA_DELAY_TX2RX_US       200
#define RX_AMCA_DELAY_TX2RX_MS         1

/* Atencao!!! Problemas no ajuste do TX_AMCA_TIMEOUT_MS
 * Os testes P2P coloquei o valor de RX_PERIOD + 50 ms...ou seja...150ms pois com throughput alto este tempo influencia
 * o envio de comandos quando a janela Tx eh grande e o periodo de resposta eh pequeno...
 * Porem para o OpenWSN ele enfluencia o RPL.DIO quando tenho muitos vizinhos...neste caso para ter tres vizinhos e conseguir
 * enviar broadcast para os 3 um tempo de 150 ms pega um ou dois vizinhos no maximo...o melhor valor neste caso eh 300ms
 */
#define TX_AMCA_TIMEOUT_MS          RX_AMCA_PERIOD_MS+200  
#define TX_AMCA_PERIOD_MS           TX_AMCA_TIMEOUT_MS+50    //ficou bom..350
#define TX_AMCA_RXOLAPREPARE_MS       1
#define TX_AMCA_TXDATAECHO_MS        50
#define TX_AMCA_DELAYRXTX_MS          1
#define TX_AMCA_DELAYCONTWAIT_MS      1
#define TX_AMCA_ACK_TIMEOUT_MS       50                         //valor bom 100
#define TX_AMCA_TXOLAACKECHO_MS      10      //Para enviar receber o echo de 11 bytes foi na ordem de 1 a 3 ms em media
#define TX_AMCA_CW_TIMEOUT_MS         5      //5ms funcionou...

//Service AMCA MultiChannelHello - RIT Neighboorhood Discover (time in ms)
#define AMCA_MULTICHANNELHELLO_MS   MULTICHANNELHELLO_MS


#define AMCA_RX_ONLY 0


// the different states of the IEEE802.15.4e state machine
typedef enum {
   S_AMCA_SLEEP                   = 0x00,   // ready for next slot
// RX
   S_AMCA_TXOLAPREPARE          ,   //rx00
   S_AMCA_TXOLA                 ,   //rx01
   S_AMCA_TXOLAERROR            ,   //
   S_AMCA_RXNEWDATA               ,   // rx04
   S_AMCA_LLTXACKOFFSET           ,
   S_AMCA_RXDATATIMEOUT           ,   // ready to Rx ACK, waiting for 'go'
   S_AMCA_RXDATAPREPARE           ,   // preparing for Rx data
   S_AMCA_RXDATA                  ,   // data SFD received, receiving more bytes
   S_AMCA_TXACKPREPARE            ,   // preparing for Tx ACK
   S_AMCA_TXACK                   ,   // Rx07
   S_AMCA_TXACKERROR              ,   //Rxe07
   S_AMCA_RXPROC                  ,   // Rx08
   S_AMCA_TX_CONTINUEWAIT         ,   // received a invalid packet then open window again
// TX
   S_AMCA_RXOLAOFFSET       = 0x50,  //tx00
   S_AMCA_RXOLAPREPARE      ,   // tx02
   S_AMCA_RXOLA             ,   // tx03
   S_AMCA_OLATIMEOUT        ,   // txe03
   S_AMCA_RXNEWOLA          ,
   S_AMCA_TXDATAOFFSET      ,   // tx40
   S_AMCA_TXDATAREADY       ,   // tx05
   S_AMCA_TXDATA            ,   // tx06
   S_AMCA_TXDATAERROR       ,   // txe06
   S_AMCA_RXACKOFFSET       ,   // tx07
   S_AMCA_RXACKPREPARE      ,   // tx08
   S_AMCA_RXACK             ,   // tx81
   S_AMCA_RXNOACK           ,   // txe08
   S_AMCA_TXPROC            ,   // tx09

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
