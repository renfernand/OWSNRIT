/*
 * cc2538rf.h
 *
 *  Created on: 18/09/2013
 *      Author: xvilajosana
 */

#ifndef CC2538RF_H_
#define CC2538RF_H_


#include "hw_rfcore_xreg.h"
#include "hw_rfcore_sfr.h"


/*---------------------------------------------------------------------------
 * RF Config
 * 0xD5 is equal to +3dBm. You can get the register values according to your desired TX power
 * with the SmartRF Studio 7: http://www.ti.com/tool/smartrftm-studio&DCMP=hpa_rf_general&HQS=Other+OT+smartrfstudio
 *
 *---------------------------------------------------------------------------*/
/* Constants */
#define POWER_3db   0xD5 //+3dBm - recommended
#define POWER_1db   0xC5 //+1dBm
#define POWER_0db   0xB6 //+0dBm
#define POWER_n1db  0xB0 //-1dBm
#define POWER_n3db  0xA1 //-3dBm
#define POWER_n5db  0x91 //-5dBm
#define POWER_n11db  0x62 //-11dBm
#define POWER_n15db  0x42 //-15dBm
#define POWER_n24db  0x00 //-24dBm

#define CC2538_RF_CONF_TX_POWER  POWER_n11db

#define CC2538_RF_CCA_THRES_USER_GUIDE 0xF8
#define CC2538_RF_TX_POWER_RECOMMENDED 0xD5
#define CC2538_RF_CHANNEL_MIN            11 //poipoi -- in fact is sending on 0x17 check that.
#define CC2538_RF_CHANNEL_MAX            26
#define CC2538_RF_CHANNEL_SPACING         5
#define CC2538_RF_MAX_PACKET_LEN        127
#define CC2538_RF_MIN_PACKET_LEN          4
#define CC2538_RF_CCA_CLEAR               1
#define CC2538_RF_CCA_BUSY                0
/*---------------------------------------------------------------------------*/
#ifdef CC2538_RF_CONF_TX_POWER
#define CC2538_RF_TX_POWER CC2538_RF_CONF_TX_POWER
#else
#define CC2538_RF_TX_POWER CC2538_RF_TX_POWER_RECOMMENDED
#endif /* CC2538_RF_CONF_TX_POWER */


#ifdef CC2538_RF_CONF_CHANNEL
#define CC2538_RF_CHANNEL CC2538_RF_CONF_CHANNEL
#else
#define CC2538_RF_CHANNEL CC2538_RF_CHANNEL_MIN
#endif /* CC2538_RF_CONF_CHANNEL */

#ifdef CC2538_RF_CONF_AUTOACK
#define CC2538_RF_AUTOACK CC2538_RF_CONF_AUTOACK
#else
#define CC2538_RF_AUTOACK 1
#endif /* CC2538_RF_CONF_AUTOACK */
/*---------------------------------------------------------------------------
 * Command Strobe Processor
 *---------------------------------------------------------------------------*/
/* OPCODES */
#define CC2538_RF_CSP_OP_ISRXON                0xE3
#define CC2538_RF_CSP_OP_ISTXON                0xE9
#define CC2538_RF_CSP_OP_ISTXONCCA             0xEA
#define CC2538_RF_CSP_OP_ISRFOFF               0xEF
#define CC2538_RF_CSP_OP_ISFLUSHRX             0xED
#define CC2538_RF_CSP_OP_ISFLUSHTX             0xEE

/**
 * \brief Send an RX ON command strobe to the CSP
 */
#define CC2538_RF_CSP_ISRXON()    \
  do { HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISRXON; } while(0)

/**
 * \brief Send a TX ON command strobe to the CSP
 */
#define CC2538_RF_CSP_ISTXON()    \
  do { HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISTXON; } while(0)

/**
 * \brief Send a RF OFF command strobe to the CSP
 */
#define CC2538_RF_CSP_ISRFOFF()   \
  do { HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISRFOFF; } while(0)

/**
 * \brief Flush the RX FIFO
 */
#define CC2538_RF_CSP_ISFLUSHRX()  do { \
  HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISFLUSHRX; \
  HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISFLUSHRX; \
} while(0)

/**
 * \brief Flush the TX FIFO
 */
#define CC2538_RF_CSP_ISFLUSHTX()  do { \
  HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISFLUSHTX; \
  HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISFLUSHTX; \
} while(0)
/*---------------------------------------------------------------------------*/


#endif /* CC2538RF_H_ */
