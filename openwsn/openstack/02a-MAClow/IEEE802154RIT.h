#ifndef __IEEE802154RIT_H
#define __IEEE802154RIT_H

//Command Frame Identifier
#define CMDFRMID_RIT        0x20
#define CMDFRMID_AMCA_HELLO 0x1E

#define CMDFRMID    CMDFRMID_AMCA_HELLO

#define RITRX_50    0     //program RIT 50 ms
#define RITRX_100   1     //program RIT 100 ms
#define RITRX_200   0     //program RIT 200 ms
#define RITRX_500   0     //program RIT 500 ms   -> ESTE EH O MAXIMO SUPORTADO ATUALMENTE!!!!!

#define TX_RIT_PERIOD_MS           450
#define TX_RIT_SLEEP_MS            100
#define TX_RIT_RXOLAPREPARE_MS       1
#define TX_RIT_TIMEOUT_MS          400
#define TX_RIT_TXDATAECHO_MS        50
#define TX_RIT_DELAYRXTX_MS          5
#define TX_RIT_DELAYCONTWAIT_MS      2
#define TX_RIT_ACK_TIMEOUT_MS      100

#if (RITRX_50 == 1)
#define RX_RIT_PERIOD_MS             50
#define RX_RIT_TXOLAPREPARE_MS        1
#define RX_RIT_TXOLAECHO_MS          15     //posso colocar este tempo min=5ms para ola de 12 bytes
#define RX_RIT_DELAY_TX_TO_RX_MS      2
#define RX_RIT_TIMEOUT_MS            20
#define RX_RIT_ACKECHO_TIMEOUT_MS    10
#elif (RITRX_100 == 1)
#define RX_RIT_PERIOD_MS            100
#define RX_RIT_TXOLAPREPARE_MS        1
#define RX_RIT_TXOLAECHO_MS          15     //posso colocar este tempo min=5ms para ola de 12 bytes
#define RX_RIT_DELAY_TX_TO_RX_MS      1
#define RX_RIT_TIMEOUT_MS            70
#define RX_RIT_ACKECHO_TIMEOUT_MS    10
#elif (RITRX_200 == 1)
#define RX_RIT_PERIOD_MS            200
#define RX_RIT_TXOLAPREPARE_MS        1
#define RX_RIT_TXOLAECHO_MS          15     //posso colocar este tempo min=5ms para ola de 12 bytes
#define RX_RIT_DELAY_TX_TO_RX_MS      2
#define RX_RIT_TIMEOUT_MS           150
#define RX_RIT_ACKECHO_TIMEOUT_MS    10
#elif (RITRX_500 == 1)
#define RX_RIT_PERIOD_MS            500
#define RX_RIT_TXOLAPREPARE_MS        1
#define RX_RIT_TXOLAECHO_MS          15     //posso colocar este tempo min=5ms para ola de 12 bytes
#define RX_RIT_DELAY_TX_TO_RX_MS      2
#define RX_RIT_TIMEOUT_MS           400
#define RX_RIT_ACKECHO_TIMEOUT_MS    10
#endif



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

//Service AMCA MultiChannelHello - RIT Neighboorhood Discover (time in ms)
#define RIT_MULTICHANNELHELLO_MS 30000

#define MAX_ELEM_ROUTE 50
extern uint8_t rffbuf[MAX_ELEM_ROUTE];
/*
 *
 */
//----------------RIT stage
enum RITstate{
	S_RIT_sleep_state         = 0x00,
	S_RIT_RX_state            = 0x01,
	S_RIT_TX_state            = 0x02,
	S_RIT_multichnhello_state = 0x03
};

enum CSMAstate{
	TX_NOT_USE_CSMA_CA     = 0x00,
	TX_USE_CSMA_CA         = 0x01
};

typedef struct {
	uint16_t countdata;
	uint16_t countdatatx;
	uint16_t countdatatxok;
	uint16_t countdatatxerr;
	uint16_t countack;
	uint16_t countacktxrx;
	uint16_t countacktxrxok;
	uint16_t countacktxrxerr;
} sRITstat;

typedef struct {
	sRITstat txola;
	sRITstat rxola;
	sRITstat txdio;
	sRITstat txdao;
	uint16_t rxdio;
	uint16_t rxdao;
    sRITstat txcoap;
} RIT_stats_t;


uint8_t printaddress(open_addr_t addr,uint8_t *buf,uint8_t pos);
uint8_t printvar(uint8_t *var,uint8_t size, uint8_t *buf,uint8_t pos);

#endif
