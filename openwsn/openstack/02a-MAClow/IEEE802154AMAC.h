#ifndef IEEE802154AMAC_H_
#define IEEE802154AMAC_H_


#include "IEEE802154E.h"
#if 1
#define RX_AMAC_PERIOD_MS            100
#define RX_AMAC_TXOLAPREPARE_MS        1
#define RX_AMAC_RX_OLA2TXPREPARE_MS    1
#define RX_AMAC_TXOLAEND_MS           10     //posso colocar este tempo min=5ms para ola de 12 bytes
#define RX_AMAC_DELAY_TX2RX_MS         1
#define RX_AMAC_DELAY_RX2TX_US       192
#define RX_AMAC_CW_BASE_US           800     //tempo base do Contention Window = 20 Jiffies - E Random apartir dai..
#define RX_AMAC_CW_RANDOM_OFFSET_US   10     //tempo base do Contention Window = 20 Jiffies - E Random apartir dai..
#define RX_AMAC_RXCW_BASE_US         500     //tempo base do Contention Window = 20 Jiffies - E Random apartir dai..
#define RX_AMAC_TIMEOUT_MS            50     //timeout (2) RxOla1 = media tempo 8.30 x 3
#define RX_AMAC_DATATIMEOUT_MS        50
#define RX_AMAC_CWTIMEOUT_MS          60
#define RX_AMAC_HA_TIMEOUT_MS         10
#define RX_AMAC_TXACKEND_MS           20

#define TX_AMAC_TIMEOUT_MS          RX_AMAC_PERIOD_MS+250
#define TX_AMAC_PERIOD_MS           TX_AMAC_TIMEOUT_MS+50
#define TX_AMAC_RXOLAPREPARE_MS       1
#define TX_AMAC_RXOLAPREPARE_US     100
#define TX_AMAC_TXDATAECHO_MS        50
#define TX_AMAC_DELAYRXTX_MS          1
#define TX_AMAC_DELAYCONTWAIT_MS      1
#define TX_AMAC_ACK_TIMEOUT_MS       50
#define TX_AMAC_TXOLAACKECHO_MS      10    //Para enviar receber o echo de 11 bytes foi na ordem de 1 a 3 ms em media
#define TX_AMAC_CW_TIMEOUT_MS  50      //5ms funcionou...
#define TX_AMAC_SLEEP_MS            100
#define TX_AMAC_RXDATABASE_US       600

//Service AMCA MultiChannelHello - RIT Neighboorhood Discover (time in ms)
#define AMAC_MULTICHANNELHELLO_MS   MULTICHANNELHELLO_MS

#define AMAC_RX_ONLY 0


// the different states of the IEEE802.15.4e state machine
typedef enum {
   S_AMAC_SLEEP                   = 0x00,   // ready for next slot
 // TX
   S_AMAC_RXOLA1OFFSET,   // tx00
   S_AMAC_TXCHKFRAME,     // tx01
   S_AMAC_FRAMEERROR,     // txe01
   S_AMAC_RXOLA1PREPARE,  // tx02
   S_AMAC_RXOLA1,         // tx03
   S_AMAC_OLA1TIMEOUT,    // txe03
   S_AMAC_RXNEWOLA1,      // tx04
   S_AMAC_RXOLA2OFFSET,   // tx02
   S_AMAC_RXOLA2PREPARE,  // tx02
   S_AMAC_RXOLA2,         // tx03
   S_AMAC_TXOLA3OFFSET,   // tx02
   S_AMAC_TXOLA3,         // tx03
   S_AMAC_TXOLA3ERROR,    // tx03
   S_AMAC_OLA2TIMEOUT,    // txe03
   S_AMAC_RXNEWOLA2,      // tx04
   S_AMAC_TXDATA,         // tx09
   S_AMAC_TXDATAERROR,    // txe09
   S_AMAC_RXCWORACKPREPARE,// tx06
   S_AMAC_RXCWORACK,       // tx07
   S_AMAC_NEWCWORACK,      // txe07
   S_AMAC_RXCWORACKTIMEOUT,// tx71
   S_AMAC_RXACKTIMEOUT,    // txe07
   S_AMAC_CONTINUEWAIT1,   // tx45
   S_AMAC_CONTINUEWAIT2,   // tx45
   S_AMAC_TXDATAREADY,     // tx08
   S_AMAC_RXACKOFFSET,     // tx0a
   S_AMAC_RXACK,           // tx0b
   S_AMAC_TXPROC,          // tx0c
// RX
   S_AMAC_TXOLA1PREPARE = 0x50,   // rx00
   S_AMAC_TXOLA1,       // rx01
   S_AMAC_TXOLA1ERROR,  // rxe01
   S_AMAC_RXOLA3PREPARE,// rx00
   S_AMAC_RXOLA3,       // rx00
   S_AMAC_OLA3TIMEOUT,  // rx00
   S_AMAC_TXOLA2PREPARE,// rx00
   S_AMAC_TXOLA2,       // rx01
   S_AMAC_TXOLA2ERROR,  // rxe01
   S_AMAC_RXOLAACK,     // rx02
   S_AMAC_OLAACKTIMEOUT,// rxe02
   S_AMAC_RXDATAOFFSET, // rx03
   S_AMAC_RXDATAPREPARE,// Rx56
   S_AMAC_RXDATA,       // rx04
   S_AMAC_DATATIMEOUT,  // rxe04
   S_AMAC_RXNEWDATA,    // rx05
   S_AMAC_TXCWOFFSET,   // rx54
   S_AMAC_TXCW,         // rx55
   S_AMAC_TXCWERROR,    // rxe55
   S_AMAC_TXACKOFFSET,  // rx06
   S_AMAC_TXACKPREPARE, // rx07
   S_AMAC_TXACK,        // rx08
   S_AMAC_TXACKERROR,   // rxe08
   S_AMAC_RXPROC,       // rx09
   S_AMAC_LLTXACKOFFSET,// rx71
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


#define HELLOTYPE_1 1   //Ola tipo 1 e quando nao tem acknowledge
#define HELLOTYPE_2 2   //Ola tipo 2 tem ack
#define HELLOTYPE_3 3   //Ola tipo 3 é o ola do Transmissor com a mascara no Source...sem ack

enum {
   ECW1 = 1,
   ECW2 = 2,
}enumecw;

#define ECW_STATUS_ERROR 1
#define ECW_STATUS_OK    0

typedef struct {
   uint8_t       ecwflag;   //event colision window detected : 0 = No event ; 1= Event 1; 2= Event 2
   uint8_t       ecwstatus; //Status=0 = OK; 1=Erro. Necessita reenvio do Data
   open_addr_t   dstaddr;
   uint8_t       ecwcnt;   //conta a quantidade de ocorrencias dentro do mesmo slot time
   uint16_t      ecw1occurs;   //quando rx - conta a quantidade de ecw1 ocorridos
   uint16_t      ecw2occurs;   //quando rx - conta a quantidade de ecw2 ocorridos
   uint16_t      rxcwoccurs;   //quando tx - conta a quantidade de cw recebidos
   uint8_t       cwerr;        //sequence numbers
}ecw_vars_t;

enum discardfrm {
  DISCARD_NO_ENDSLOT_NO=0,
  DISCARD_NO_ENDSLOT_YES,
  DISCARD_YES_ENDSLOT_NO,
  DISCARD_YES_ENDSLOT_YES,
};

#endif

#endif /* IEEE802154AMAC_H_ */
