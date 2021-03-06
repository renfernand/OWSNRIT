#ifndef __RADIO_H
#define __RADIO_H

/**
\addtogroup BSP
\{
\addtogroup radio
\{

\brief Cross-platform declaration "radio" bsp module.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, February 2012.
*/

#include "radiotimer.h"
#include "board.h"

//=========================== define ==========================================

#define LENGTH_CRC 2

/**
\brief Current state of the radio.

\note This radio driver is very minimal in that it does not follow a state machine.
      It is up to the MAC layer to ensure that the different radio operations 
      are called in the righr order. The radio keeps a state for debugging purposes only.
*/
typedef enum {
   RADIOSTATE_STOPPED             = 0x00,   ///< Completely stopped.
   RADIOSTATE_RFOFF               = 0x01,   ///< Listening for commands, but RF chain is off.
   RADIOSTATE_SETTING_FREQUENCY   = 0x02,   ///< Configuring the frequency.
   RADIOSTATE_FREQUENCY_SET       = 0x03,   ///< Done configuring the frequency.
   RADIOSTATE_LOADING_PACKET      = 0x04,   ///< Loading packet into the radio's TX buffer.
   RADIOSTATE_PACKET_LOADED       = 0x05,   ///< Packet is fully loaded in the radio's TX buffer.
   RADIOSTATE_ENABLING_TX         = 0x06,   ///< The RF TX chaing is being enabled (includes locking the PLL).
   RADIOSTATE_TX_ENABLED          = 0x07,   ///< Radio ready to transmit.
   RADIOSTATE_TRANSMITTING        = 0x08,   ///< Busy transmitting bytes.
   RADIOSTATE_ENABLING_RX         = 0x09,   ///< The RF RX chain is being enabled (includes locking the PLL).
   RADIOSTATE_LISTENING           = 0x0a,   ///< RF chain is on, listening, but no packet received yet.
   RADIOSTATE_RECEIVING           = 0x0b,   ///< Busy receiving bytes.
   RADIOSTATE_TXRX_DONE           = 0x0c,   ///< Frame has been sent/received completely.
   RADIOSTATE_TURNING_OFF         = 0x0d,   ///< Turning the RF chain off.
   RADIOSTATE_TX_DONE             = 0x0e,   ///< Turning the RF chain off.
   RADIOSTATE_RX_DONE             = 0x0f,   ///< Turning the RF chain off.
} radio_state_t;

//=========================== typedef =========================================

typedef struct {
   radiotimer_capture_cbt    startFrame_cb;
   radiotimer_capture_cbt    endFrame_cb;
   radio_state_t             state;
   uint8_t                   filter_en;
   uint8_t                   filter_matched;
} radio_vars_t;


//=========================== variables =======================================

//=========================== prototypes ======================================

// admin
void     radio_init(void);
void     radio_setOverflowCb(radiotimer_compare_cbt cb);
void     radio_setCompareCb(radiotimer_compare_cbt cb);
void     radio_setStartFrameCb(radiotimer_capture_cbt cb);
void     radio_setEndFrameCb(radiotimer_capture_cbt cb);
// reset
void     radio_reset(void);
// timer
void     radio_startTimer(PORT_TIMER_WIDTH period);
PORT_TIMER_WIDTH radio_getTimerValue(void);
void     radio_setTimerPeriod(PORT_TIMER_WIDTH period);
PORT_TIMER_WIDTH radio_getTimerPeriod(void);
// RF admin
void     radio_setFrequency(uint8_t frequency);
void     radio_rfOn(void);
void     radio_rfOff(void);
void     radio_rfOffAckDisable(void);

// TX
void     radio_loadPacket(uint8_t* packet, uint8_t len);
void     radio_txEnable(void);
#if (ENABLE_CSMA_CA == 1)
void     radio_txNow(uint8_t flagUseCsma);
void      macCspTxBusyIsr(PORT_TIMER_WIDTH capturedTime);
#else
void     radio_txNow(void);
#endif
// RX
void     radio_flushfifos(void);
void     radio_rxEnable(void);
void     radio_rxEnableANDFilter(uint16_t srcaddr,uint16_t destaddr,uint16_t panID);
void     radio_rxEnable3(void);
void     radio_rxNow(void);
void     radio_getReceivedFrame(uint8_t* bufRead,
                                uint8_t* lenRead,
                                uint8_t  maxBufLen,
                                 int8_t* rssi,
                                uint8_t* lqi,
                                   bool* crc);

void radio_autoackenable (void);
void radio_autoackdisable (void);
void radio_rxNowAutoAck (void);
void radio_rxNowANDTxAutoAck (void);
void radio_rxNowAckOnly(void);
void radio_rxNowCmdOnly(void);
void radio_txNowAndWaitAck(void);
uint8_t radio_getrxfilterstatus(void);
void radio_rxEnableAndNowCmdOnly ( uint16_t srcaddr,uint16_t dstaddr, uint16_t panID );
// interrupt handlers
kick_scheduler_t   radio_isr(void);

/**
\}
\}
*/

#endif
