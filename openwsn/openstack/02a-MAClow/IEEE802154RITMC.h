#ifndef __IEEE802154RITMC_H
#define __IEEE802154RITMC_H


#include "IEEE802154E.h"
#if 1
#define RX_RITMC_PERIOD_MS            100
#define RX_RITMC_TXOLAPREPARE_MS        1
#define RX_RITMC_TXOLAECHO_MS          15     //posso colocar este tempo min=5ms para ola de 12 bytes
#define RX_RITMC_DELAY_TX2RX_MS         1    
#define RX_RITMC_DELAY_TXRX_CW_MS       1     //AQUI EU POSSO DEMORAR POIS O TX vai demorar
#define RX_RITMC_TIMEOUT_MS            50
#define RX_RITMC_DATATIMEOUT_MS        50     //anteriormente estava 30ms
#define RX_RITMC_CWTIMEOUT_MS          60
#define RX_RITMC_HA_TIMEOUT_MS         30     //Tempo medio de um Rx.HA eh de 2,1ms (fazendo 3x)
#define RX_RITMC_REOPENRX_MS           30
#define RX_RITMC_ACKECHO_TIMEOUT_MS    10
#define RX_RITMC_TXACKEND_MS           20
#define RX_RITMC_DELAY_TX2RX_US       200
#define RX_RITMC_CW_BASE_US           800     //tempo base do Contention Window = 20 Jiffies - E Random apartir dai..
#define RX_RITMC_DELAY_RX2RX_US       200
#define RX_RITMC_DELAY_RX2TX_US       300

#define TX_RITMC_MAX_SYNC_TXIDLE      RX_RITMC_PERIOD_MS
#define TX_RITMC_TIMEOUT_MS           RX_RITMC_PERIOD_MS+50   //teste 160118 - estava +200
#define TX_RITMC_PERIOD_MS            TX_RITMC_TIMEOUT_MS+5
#define TX_RITMC_RXOLAPREPARE_MS       1
#define TX_RITMC_RXOLAPREPARE_US     250
#define TX_RITMC_TXDATAECHO_MS        50
#define TX_RITMC_DELAYRXTX_MS          1
#define TX_RITMC_DELAYCONTWAIT_MS      1
#define TX_RITMC_ACK_TIMEOUT_MS       50
#define TX_RITMC_TXOLAACKECHO_MS      10      //Para enviar receber o echo de 11 bytes foi na ordem de 1 a 3 ms em media
#define TX_RITMC_CW_TIMEOUT_MS        50      //10ms anterior...mas para ficar igual ao AMCA
#define TX_RITMC_RXCWORACKTIMEOUT_MS  50
#define TX_RITMC_RXDATABASE_US       600

//Service AMCA MultiChannelHello - RIT Neighboorhood Discover (time in ms)
#define RITMC_MULTICHANNELHELLO_MS    MULTICHANNELHELLO_MS

#define RITMC_RX_ONLY 0

//habilita o envio do diagnostico (contention windows CW) no final do frame (tx e Rx)
#define RITMC_DIAG_ENABLE 1

#define ENABLE_SYNC_PWMAC 0

//habilita o filtro de endereco para o ack e o data..
#define RITMC_ENABLE_FILTER 0

#define TEST_MEASURE_NEIGHBOR_RATE 0


// the different states of the IEEE802.15.4e state machine
typedef enum {
   S_RITMC_SLEEP                   = 0x00,   // ready for next slot
 // TX
   S_RITMC_RXOLAOFFSET,   // tx00
   S_RITMC_TXCHKFRAME,   // tx01
   S_RITMC_FRAMEERROR,   // txe01
   S_RITMC_RXOLAPREPARE,   // tx02
   S_RITMC_RXOLA       ,   // tx03
   S_RITMC_OLATIMEOUT              ,   // txe03
   S_RITMC_RXNEWOLA                ,   // tx04
   S_RITMC_OLAACKOFFSET            ,   // tx40
   S_RITMC_CONTINUEWAIT            ,   // tx45
   S_RITMC_TXOLAACK                ,   // tx05
   S_RITMC_OLAACKNOECHO            ,   // txe05
   S_RITMC_TXDATAPREPARE           ,   // tx06
   S_RITMC_TXDATA                  ,   // tx07
   S_RITMC_TXDATANOECHO            ,   // txe07
   S_RITMC_RXCWACKPREPARE          ,   // tx08
   S_RITMC_RXCWACK                 ,   // tx09
   S_RITMC_RXCWACKTIMEOUT          ,   // tx91
   S_RITMC_RXACKTIMEOUT            ,   // txe09
   S_RITMC_NEWCWACK                ,   // tx0A
   S_RITMC_RXACK                   ,   // tx0b
   S_RITMC_TXPROC                  ,   // tx0c
// RX
   S_RITMC_TXOLAPREPARE            = 0x50,   // rx00
   S_RITMC_TXOLA                   ,   // rx01
   S_RITMC_OLANOECHO               ,   // rxe01
   S_RITMC_RXOLAACKPREPARE         ,   // rx02
   S_RITMC_RXOLAACK                ,   // rx03
   S_RITMC_OLAACKTIMEOUT           ,   // rxe03
   S_RITMC_RXDATAOFFSET            ,   // rx04
   S_RITMC_TXCWOFFSET              ,   // rx74
   S_RITMC_TXCW                    ,   // rx75
   S_RITMC_TXCWERROR               ,   // rxe75
   S_RITMC_CWNOECHO                ,   // rxe043
   S_RITMC_RXDATAPREPARE           ,   // rx05
   S_RITMC_RXDATA                  ,   // rx06
   S_RITMC_DATATIMEOUT             ,   // rxe06
   S_RITMC_RXNEWDATA               ,   // rx07
   S_RITMC_TXACKOFFSET             ,   // rx08
   S_RITMC_TXACKPREPARE            ,   // rx09
   S_RITMC_TXACK                   ,   // rx0a
   S_RITMC_ACKNOECHO               ,   // rxe0a
   S_RITMC_RXPROC                  ,   // rx0b
   S_RITMC_LLTXACKOFFSET           ,   // rx81
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
   PORT_RADIOTIMER_WIDTH     syncCapturedTime0;        // captured time used to sync
   PORT_RADIOTIMER_WIDTH     syncCapturedTime1;        // captured time used to sync
   PORT_RADIOTIMER_WIDTH     syncCapturedTime2;        // captured time used to sync
   PORT_RADIOTIMER_WIDTH     syncCapturedTime3;        // captured time used to sync
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

enum {
   RITMC_ECW1 = 1,
   RITMC_ECW2 = 2,
}enumRITMCECW;

#define ECW_STATUS_ERROR 1
#define ECW_STATUS_OK    0

typedef struct {
   uint8_t       ecwflag;   //event colision window detected : 0 = No event ; 1= Event 1; 2= Event 2
   uint8_t       ecwstatus; //Status=0 = OK; 1=Erro. Necessita reenvio do Data
   uint8_t       ecwstatus2; //Status=0 = OK; 1=Erro. Necessita reenvio do Data
   open_addr_t   dstaddr;
   uint8_t       ecwcnt;   //conta a quantidade de ocorrencias dentro do mesmo slot time
   uint16_t      ecw1occurs;   //quando rx - conta a quantidade de ecw1 ocorridos
   uint16_t      ecw2occurs;   //quando rx - conta a quantidade de ecw2 ocorridos
   uint16_t      rxcwoccurs;   //quando tx - conta a quantidade de cw recebidos
   uint16_t      tx0aTimeout;   //quando tx - conta a quantidade de timeouts do CW ou ACK
   uint16_t      ecw2DataTimeout;   //Rxe06
   uint16_t      tx0aFrameNotOk;   //quando tx - conta a quantidade de frames de ack recebidos not ok

   uint8_t       cwerr;        //sequence numbers
}ecw_vars_t;

enum discardfrm {
  DISCARD_NO_ENDSLOT_NO=0,
  DISCARD_NO_ENDSLOT_YES,
  DISCARD_YES_ENDSLOT_NO,
  DISCARD_YES_ENDSLOT_YES,
};
#endif
#endif
