#ifndef __IEEE802154AMCA_H
#define __IEEE802154AMCA_H



#if 0
//ticks
//este tempo dependo do setting do timer...para CSMACA clock eh de 320us=3125Hz
#define MACTIMERCLOCK 3125
#define CONVERT_MS_IN_TICKS(s)     (MACTIMERCLOCK * s)/1000


#define TX_RIT_PERIOD_MS           450
#define TX_RIT_RXOLAPREPARE_MS       1
#define TX_RIT_TIMEOUT_MS          400
#define TX_RIT_TXDATAECHO_MS        50
#define TX_RIT_DELAYRXTX_MS         10
#define TX_RIT_ACK_TIMEOUT_MS      100

#define TX_RIT_PERIOD_TICKS           CONVERT_MS_IN_TICKS(TX_RIT_PERIOD_MS)
#define TX_RIT_TIMEOUT_TICKS          CONVERT_MS_IN_TICKS(TX_RIT_TIMEOUT_MS)
#define TX_RIT_TXDATAECHO_TICKS       CONVERT_MS_IN_TICKS(TX_RIT_TXDATAECHO_MS)
#define TX_RIT_DELAYRXTX_TICKS        CONVERT_MS_IN_TICKS(TX_RIT_DELAYRXTX_MS)
#define TX_RIT_ACK_TIMEOUT_TICKS      CONVERT_MS_IN_TICKS(TX_RIT_ACK_TIMEOUT_MS)

#define RX_RIT_PERIOD_MS            100
#define RX_RIT_TXOLAPREPARE_MS        1
#define RX_RIT_TXOLAECHO_MS          10     //posso colocar este tempo min=5ms para ola de 12 bytes
#define RX_RIT_DELAY_TX_TO_RX_MS      2
#define RX_RIT_TIMEOUT_MS            70
#define RX_RIT_ACKECHO_TIMEOUT_MS    10

#define RX_RIT_PERIOD_TICKS            CONVERT_MS_IN_TICKS(RX_RIT_PERIOD_MS)
#define RX_RIT_TXOLAECHO_TICKS         CONVERT_MS_IN_TICKS(RX_RIT_TXOLAECHO_MS)
#define RX_RIT_DELAY_TX_TO_RX_TICKS    CONVERT_MS_IN_TICKS(RX_RIT_DELAY_TX_TO_RX_MS)
#define RX_RIT_TIMEOUT_TICKS           CONVERT_MS_IN_TICKS(RX_RIT_TIMEOUT_MS)
#define RX_RIT_ACKECHO_TIMEOUT_TICKS   CONVERT_MS_IN_TICKS(RX_RIT_ACKECHO_TIMEOUT_MS)


/*
 *
 */
//----------------RIT stage
enum RITstate{
	S_RIT_sleep_state     = 0x00,
	S_RIT_RX_state        = 0x01,
	S_RIT_TX_state        = 0x02
};

enum CSMAstate{
	TX_NOT_USE_CSMA_CA     = 0x00,
	TX_USE_CSMA_CA         = 0x01
};

typedef struct {
	uint32_t countdata;
	uint32_t countdatatx;
	uint32_t countdatatxok;
	uint32_t countdatatxerr;
	uint32_t countack;
	uint32_t countacktxrx;
	uint32_t countacktxrxok;
	uint32_t countacktxrxerr;
} sRITstat;

typedef struct {
	sRITstat txola;
	sRITstat rxola;
	sRITstat txdio;
	sRITstat txdao;
    sRITstat txcoap;
} RIT_stats_t;

#endif

