/**
\brief Definition of the "openserial" driver.

\author Fabien Chraim <chraim@eecs.berkeley.edu>, March 2012.
*/

#include "opendefs.h"
#include "openserial.h"
#include "IEEE802154E.h"
#include "neighbors.h"
#include "sixtop.h"
#include "icmpv6echo.h"
#include "idmanager.h"
#include "openqueue.h"
#include "openbridge.h"
#include "leds.h"
#include "schedule.h"
#include "uart.h"
#include "opentimers.h"
#include "openhdlc.h"
#include "debugpins.h"

#if (DEBUG_VIA_SERIAL == 1)
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include "uarthal.h"
#endif
//=========================== variables =======================================

openserial_vars_t openserial_vars;
uint8_t rffinputcount;
uint8_t debugstatusRFF;

#define DEBUG_UART1_DIRECT 0
uint8_t bufRffTx[100];
uint8_t bufRffTxCount;
uint8_t bufRffTxLen;

#define DEBUG_CMD_RPL 0
volatile uint8_t bufrffrx[100];
volatile uint8_t bufrffrxlen;

#if (DEBUG_LOG_RIT  == 1)
static uint8_t rffbuf[10];
#endif
//=========================== prototypes ======================================

owerror_t openserial_printInfoErrorCritical(
   char             severity,
   uint8_t          calling_component,
   uint8_t          error_code,
   errorparameter_t arg1,
   errorparameter_t arg2
);
// HDLC output
void outputHdlcOpen(void);
void outputHdlcWrite(uint8_t b);
void outputHdlcClose(void);
// HDLC input
void inputHdlcOpen(void);
void inputHdlcWrite(uint8_t b);
void inputHdlcClose(void);

#if (DEBUG_VIA_SERIAL == 1)

#define DBG_MAX_REPLY_SIZE       20
//=========================== variables =======================================
// static char pucUdpLog[DBG_MAX_REPLY_SIZE + 1];
// static uint32_t g_ui32Base = UART0_BASE;
static const char * const g_pcHex = "0123456789abcdef";

extern void UARTprintf(const char *pcString, ...);
extern void UARTvprintf(const char *pcString, __VALIST vaArgP);
#endif
//=========================== public ==========================================

void openserial_init() {
   uint16_t crc;
   
   // reset variable
   memset(&openserial_vars,0,sizeof(openserial_vars_t));
   
   // admin
   openserial_vars.mode                = MODE_OFF;
   openserial_vars.debugPrintCounter   = 0;
   
   // input
   openserial_vars.reqFrame[0]         = HDLC_FLAG;
   openserial_vars.reqFrame[1]         = SERFRAME_MOTE2PC_REQUEST;
   crc = HDLC_CRCINIT;
   crc = crcIteration(crc,openserial_vars.reqFrame[1]);
   crc = ~crc;
   openserial_vars.reqFrame[2]         = (crc>>0)&0xff;
   openserial_vars.reqFrame[3]         = (crc>>8)&0xff;
   openserial_vars.reqFrame[4]         = HDLC_FLAG;
   openserial_vars.reqFrameIdx         = 0;
   openserial_vars.lastRxByte          = HDLC_FLAG;
   openserial_vars.busyReceiving       = FALSE;
   openserial_vars.inputEscaping       = FALSE;
   openserial_vars.inputBufFill        = 0;
   
   // ouput
   openserial_vars.outputBufFilled     = FALSE;
   openserial_vars.outputBufIdxR       = 0;
   openserial_vars.outputBufIdxW       = 0;
   
   debugstatusRFF = 0;

   // set callbacks
   uart_setCallbacks(isr_openserial_tx,
                     isr_openserial_rx);

#if (DBG_USING_UART1 == 1)
   uart1_setCallbacks(isr1_openserial_tx,
                      isr1_openserial_rx);
#endif
}

#if (DBG_USING_UART1 == 1)

#if (DEBUG_UART1_DIRECT == 1)
owerror_t openserial_printStatusrff(uint8_t statusElement,uint8_t* buffer, uint8_t length) {
   uint8_t i;

	// flush buffer UART1
	uart1_clearTxInterrupts();
	uart1_clearRxInterrupts();          // clear possible pending interrupts
	uart1_enableInterrupts();           // Enable USCI_A1 TX & RX interrupt
	//DISABLE_INTERRUPTS();

    bufRffTx[0]= SERFRAME_MOTE2PC_STATUS;
    bufRffTx[1]= idmanager_getMyID(ADDR_16B)->addr_16b[0];
    bufRffTx[2]= idmanager_getMyID(ADDR_16B)->addr_16b[1];
    bufRffTx[3]= statusElement;

    for (i=0;i<length;i++){
    	bufRffTx[4+i]=(buffer[i]);
    }

    bufRffTxLen = length + 4;
    bufRffTxCount = 0;

    //ENABLE_INTERRUPTS();

    uart1_writeByte(bufRffTx[bufRffTxCount]);

    return E_SUCCESS;
}
#endif

owerror_t openserial_printStatus(uint8_t statusElement,uint8_t* buffer, uint8_t length) {
   uint8_t i;


   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();

   if (statusElement == STATUS_RFF )
   {
	   openserial_vars.outputBufFilled  = TRUE;
	    debugstatusRFF  = TRUE;

	   outputHdlcOpen();
	   outputHdlcWrite(SERFRAME_MOTE2PC_STATUS);
	   outputHdlcWrite(idmanager_getMyID(ADDR_16B)->addr_16b[0]);
	   outputHdlcWrite(idmanager_getMyID(ADDR_16B)->addr_16b[1]);
	   outputHdlcWrite(statusElement);
	   for (i=0;i<length;i++){
	      outputHdlcWrite(buffer[i]);
	   }
	   outputHdlcClose();

   }
   else
   {
	   openserial_vars.outputBufFilled  = TRUE;

	   outputHdlcOpen();
	   outputHdlcWrite(SERFRAME_MOTE2PC_STATUS);
	   outputHdlcWrite(idmanager_getMyID(ADDR_16B)->addr_16b[0]);
	   outputHdlcWrite(idmanager_getMyID(ADDR_16B)->addr_16b[1]);
	   outputHdlcWrite(statusElement);
	   for (i=0;i<length;i++){
	      outputHdlcWrite(buffer[i]);
	   }
	   outputHdlcClose();
   }


   ENABLE_INTERRUPTS();


   return E_SUCCESS;
}

#else


owerror_t openserial_printStatus(uint8_t statusElement,uint8_t* buffer, uint8_t length) {
   uint8_t i;
   INTERRUPT_DECLARATION();
   
   DISABLE_INTERRUPTS();
   openserial_vars.outputBufFilled  = TRUE;
   outputHdlcOpen();
   outputHdlcWrite(SERFRAME_MOTE2PC_STATUS);
   outputHdlcWrite(idmanager_getMyID(ADDR_16B)->addr_16b[0]);
   outputHdlcWrite(idmanager_getMyID(ADDR_16B)->addr_16b[1]);
   outputHdlcWrite(statusElement);
   for (i=0;i<length;i++){
      outputHdlcWrite(buffer[i]);
   }
   outputHdlcClose();
   ENABLE_INTERRUPTS();

   return E_SUCCESS;
}

#endif


owerror_t openserial_printInfoErrorCritical(
      char             severity,
      uint8_t          calling_component,
      uint8_t          error_code,
      errorparameter_t arg1,
      errorparameter_t arg2
   ) {
   INTERRUPT_DECLARATION();
   
   DISABLE_INTERRUPTS();
   openserial_vars.outputBufFilled  = TRUE;
   outputHdlcOpen();
   outputHdlcWrite(severity);
   outputHdlcWrite(idmanager_getMyID(ADDR_16B)->addr_16b[0]);
   outputHdlcWrite(idmanager_getMyID(ADDR_16B)->addr_16b[1]);
   outputHdlcWrite(calling_component);
   outputHdlcWrite(error_code);
   outputHdlcWrite((uint8_t)((arg1 & 0xff00)>>8));
   outputHdlcWrite((uint8_t) (arg1 & 0x00ff));
   outputHdlcWrite((uint8_t)((arg2 & 0xff00)>>8));
   outputHdlcWrite((uint8_t) (arg2 & 0x00ff));
   outputHdlcClose();
   ENABLE_INTERRUPTS();
   
   return E_SUCCESS;
}

owerror_t openserial_printData(uint8_t* buffer, uint8_t length) {
   uint8_t  i;
   uint8_t  asn[5];
   INTERRUPT_DECLARATION();
   
   // retrieve ASN
   ieee154e_getAsn(asn);// byte01,byte23,byte4
   
   DISABLE_INTERRUPTS();
   openserial_vars.outputBufFilled  = TRUE;
   outputHdlcOpen();
   outputHdlcWrite(SERFRAME_MOTE2PC_DATA);
   outputHdlcWrite(idmanager_getMyID(ADDR_16B)->addr_16b[1]);
   outputHdlcWrite(idmanager_getMyID(ADDR_16B)->addr_16b[0]);
   outputHdlcWrite(asn[0]);
   outputHdlcWrite(asn[1]);
   outputHdlcWrite(asn[2]);
   outputHdlcWrite(asn[3]);
   outputHdlcWrite(asn[4]);
   for (i=0;i<length;i++){
      outputHdlcWrite(buffer[i]);
   }
   outputHdlcClose();
   ENABLE_INTERRUPTS();
   
   return E_SUCCESS;
}

owerror_t openserial_printInfo(uint8_t calling_component, uint8_t error_code,
                              errorparameter_t arg1,
                              errorparameter_t arg2) {
   return openserial_printInfoErrorCritical(
      SERFRAME_MOTE2PC_INFO,
      calling_component,
      error_code,
      arg1,
      arg2
   );
}

owerror_t openserial_printError(uint8_t calling_component, uint8_t error_code,
                              errorparameter_t arg1,
                              errorparameter_t arg2) {
   // blink error LED, this is serious
   leds_error_toggle();
   
   return openserial_printInfoErrorCritical(
      SERFRAME_MOTE2PC_ERROR,
      calling_component,
      error_code,
      arg1,
      arg2
   );
}

owerror_t openserial_printCritical(uint8_t calling_component, uint8_t error_code,
                              errorparameter_t arg1,
                              errorparameter_t arg2) {
   // blink error LED, this is serious
   leds_error_blink();
   
   // schedule for the mote to reboot in 10s
   opentimers_start(10000,
                    TIMER_ONESHOT,TIME_MS,
                    board_reset);
   
   return openserial_printInfoErrorCritical(
      SERFRAME_MOTE2PC_CRITICAL,
      calling_component,
      error_code,
      arg1,
      arg2
   );
}

uint8_t openserial_getNumDataBytes() {
   uint8_t inputBufFill;
   INTERRUPT_DECLARATION();
   
   DISABLE_INTERRUPTS();
   inputBufFill = openserial_vars.inputBufFill;
   ENABLE_INTERRUPTS();

   return inputBufFill-1; // removing the command byte
}

uint8_t openserial_getInputBuffer(uint8_t* bufferToWrite, uint8_t maxNumBytes) {
   uint8_t numBytesWritten;
   uint8_t inputBufFill;
   INTERRUPT_DECLARATION();
   
   DISABLE_INTERRUPTS();
   inputBufFill = openserial_vars.inputBufFill;
   ENABLE_INTERRUPTS();
   
   if (maxNumBytes<inputBufFill-1) {
      openserial_printError(COMPONENT_OPENSERIAL,ERR_GETDATA_ASKS_TOO_FEW_BYTES,
                            (errorparameter_t)maxNumBytes,
                            (errorparameter_t)inputBufFill-1);
      numBytesWritten = 0;
   } else {
      numBytesWritten = inputBufFill-1;
      memcpy(bufferToWrite,&(openserial_vars.inputBuf[1]),numBytesWritten);
   }
   
   return numBytesWritten;
}

void openserial_startInput() {

	//leds_error_toggle();

#if (ENABLE_BRIDGE_UART0 == 1)
   INTERRUPT_DECLARATION();
   
   if (openserial_vars.inputBufFill>0) {
      openserial_printError(COMPONENT_OPENSERIAL,ERR_INPUTBUFFER_LENGTH,
                            (errorparameter_t)openserial_vars.inputBufFill,
                            (errorparameter_t)0);
      DISABLE_INTERRUPTS();
      openserial_vars.inputBufFill=0;
      ENABLE_INTERRUPTS();
   }
   
   uart_clearTxInterrupts();
   uart_clearRxInterrupts();      // clear possible pending interrupts
   uart_enableInterrupts();       // Enable USCI_A1 TX & RX interrupt
   
   DISABLE_INTERRUPTS();
   openserial_vars.busyReceiving  = FALSE;
   openserial_vars.mode           = MODE_INPUT;
   openserial_vars.reqFrameIdx    = 0;
#ifdef FASTSIM
   uart_writeBufferByLen_FASTSIM(
      openserial_vars.reqFrame,
      sizeof(openserial_vars.reqFrame)
   );
   openserial_vars.reqFrameIdx = sizeof(openserial_vars.reqFrame);
#else
#if ( DEBUG_VIA_SERIAL == 0)
   rffinputcount = 0;
   uart_writeByte(openserial_vars.reqFrame[openserial_vars.reqFrameIdx]);
#endif
#endif
   ENABLE_INTERRUPTS();

#endif
}

void openserial_startOutput() {
   //schedule a task to get new status in the output buffer
   uint8_t debugPrintCounter;
   
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   openserial_vars.debugPrintCounter = (openserial_vars.debugPrintCounter+1)%STATUS_MAX;
   debugPrintCounter = openserial_vars.debugPrintCounter;
   ENABLE_INTERRUPTS();

#if (ENABLE_BRIDGE_UART0 == 1)
   // print debug information
   switch (debugPrintCounter) {
      case STATUS_ISSYNC:
         if (debugPrint_isSync()==TRUE) {
            break;
         }
      case STATUS_ID:
         if (debugPrint_id()==TRUE) {
            break;
         }
      case STATUS_DAGRANK:
         if (debugPrint_myDAGrank()==TRUE) {
            break;
         }
      case STATUS_OUTBUFFERINDEXES:
         if (debugPrint_outBufferIndexes()==TRUE) {
            break;
         }
      case STATUS_ASN:
         if (debugPrint_asn()==TRUE) {
            break;
         }
      case STATUS_MACSTATS:
         if (debugPrint_macStats()==TRUE) {
            break;
         }
      case STATUS_SCHEDULE:
         if(debugPrint_schedule()==TRUE) {
            break;
         }
      case STATUS_BACKOFF:
         if(debugPrint_backoff()==TRUE) {
            break;
         }
      case STATUS_QUEUE:
         if(debugPrint_queue()==TRUE) {
            break;
         }
      case STATUS_NEIGHBORS:
         if (debugPrint_neighbors()==TRUE) {
            break;
         }
      case STATUS_KAPERIOD:
         if (debugPrint_kaPeriod()==TRUE) {
            break;
         }
      case STATUS_RFF:
            break;
      default:
         DISABLE_INTERRUPTS();
         openserial_vars.debugPrintCounter=0;
         ENABLE_INTERRUPTS();
   }
#endif // if SINK

#if (DBG_USING_UART1 == 1)
	// flush buffer UART1
	uart1_clearTxInterrupts();
	uart1_clearRxInterrupts();          // clear possible pending interrupts
	uart1_enableInterrupts();           // Enable USCI_A1 TX & RX interrupt
#endif

	// flush buffer
   uart_clearTxInterrupts();
   uart_clearRxInterrupts();          // clear possible pending interrupts
   uart_enableInterrupts();           // Enable USCI_A1 TX & RX interrupt
   DISABLE_INTERRUPTS();
   openserial_vars.mode=MODE_OUTPUT;
   if (openserial_vars.outputBufFilled) {
#ifdef FASTSIM
      uart_writeCircularBuffer_FASTSIM(
         openserial_vars.outputBuf,
         &openserial_vars.outputBufIdxR,
         &openserial_vars.outputBufIdxW
      );
#else

#if ( DEBUG_VIA_SERIAL == 0)

#if (DBG_USING_UART1 == 0)
      uart_writeByte(openserial_vars.outputBuf[openserial_vars.outputBufIdxR++]);
#else
      if (debugstatusRFF)
      {
         uart1_writeByte(openserial_vars.outputBuf[openserial_vars.outputBufIdxR++]);
      }
      else
      {
    	  uart_writeByte(openserial_vars.outputBuf[openserial_vars.outputBufIdxR++]);
      }
#endif

#endif //( DEBUG_VIA_SERIAL == 0)

#endif //FASTSIM

   } else {
      openserial_stop();
   }
   ENABLE_INTERRUPTS();
}

void openserial_stop() {
   uint8_t inputBufFill;
   uint8_t cmdByte;
   bool busyReceiving;
   INTERRUPT_DECLARATION();
   
   DISABLE_INTERRUPTS();
   busyReceiving = openserial_vars.busyReceiving;
   inputBufFill = openserial_vars.inputBufFill;
   ENABLE_INTERRUPTS();
   
   // disable USCI_A1 TX & RX interrupt
   uart_disableInterrupts();
   
   DISABLE_INTERRUPTS();
   openserial_vars.mode=MODE_OFF;
   ENABLE_INTERRUPTS();
   //the inputBuffer has to be reset if it is not reset where the data is read.
   //or the function openserial_getInputBuffer is called (which resets the buffer)
   if (busyReceiving==TRUE){
      openserial_printError(COMPONENT_OPENSERIAL,ERR_BUSY_RECEIVING,
                                  (errorparameter_t)0,
                                  (errorparameter_t)inputBufFill);
   }
   
   if (busyReceiving == FALSE && inputBufFill>0) {

      DISABLE_INTERRUPTS();
      cmdByte = openserial_vars.inputBuf[0];
      ENABLE_INTERRUPTS();

      switch (cmdByte) {
#if (NEW_DAG_BRIDGE == 0)
         case SERFRAME_PC2MOTE_SETROOTBRIDGE:
            idmanager_triggerAboutRoot();
            idmanager_triggerAboutBridge();
            break;
         case SERFRAME_PC2MOTE_SETBRIDGE:
            idmanager_triggerAboutBridge();
            break;
#endif
         case SERFRAME_PC2MOTE_SETROOT:
            idmanager_triggerAboutRoot();
            break;
         case SERFRAME_PC2MOTE_DATA:
            openbridge_triggerData();
            break;
         case SERFRAME_PC2MOTE_TRIGGERSERIALECHO:
            //echo function must reset input buffer after reading the data.
            openserial_echo(&openserial_vars.inputBuf[1],inputBufFill-1);
            break;   
         default:
            openserial_printError(COMPONENT_OPENSERIAL,ERR_UNSUPPORTED_COMMAND,
                                  (errorparameter_t)cmdByte,
                                  (errorparameter_t)0);
            //reset here as it is not being reset in any other callback
            DISABLE_INTERRUPTS();
            openserial_vars.inputBufFill = 0;
            ENABLE_INTERRUPTS();
            break;
      }
   }
   
   DISABLE_INTERRUPTS();
   openserial_vars.inputBufFill  = 0;
   openserial_vars.busyReceiving = FALSE;
   ENABLE_INTERRUPTS();
}

/**
\brief Trigger this module to print status information, over serial.

debugPrint_* functions are used by the openserial module to continuously print
status information about several modules in the OpenWSN stack.

\returns TRUE if this function printed something, FALSE otherwise.
*/
bool debugPrint_outBufferIndexes() {
   uint16_t temp_buffer[2];
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   temp_buffer[0] = openserial_vars.outputBufIdxW;
   temp_buffer[1] = openserial_vars.outputBufIdxR;
   ENABLE_INTERRUPTS();
   openserial_printStatus(STATUS_OUTBUFFERINDEXES,(uint8_t*)temp_buffer,sizeof(temp_buffer));
   return TRUE;
}

//=========================== private =========================================

//===== hdlc (output)

/**
\brief Start an HDLC frame in the output buffer.
*/
port_INLINE void outputHdlcOpen() {
   // initialize the value of the CRC
   openserial_vars.outputCrc                          = HDLC_CRCINIT;
   
   // write the opening HDLC flag
   openserial_vars.outputBuf[openserial_vars.outputBufIdxW++]     = HDLC_FLAG;
}
/**
\brief Add a byte to the outgoing HDLC frame being built.
*/
port_INLINE void outputHdlcWrite(uint8_t b) {
   
   // iterate through CRC calculator
   openserial_vars.outputCrc = crcIteration(openserial_vars.outputCrc,b);
   
   // add byte to buffer
   if (b==HDLC_FLAG || b==HDLC_ESCAPE) {
      openserial_vars.outputBuf[openserial_vars.outputBufIdxW++]  = HDLC_ESCAPE;
      b                                               = b^HDLC_ESCAPE_MASK;
   }
   openserial_vars.outputBuf[openserial_vars.outputBufIdxW++]     = b;
   
}
/**
\brief Finalize the outgoing HDLC frame.
*/
port_INLINE void outputHdlcClose() {
   uint16_t   finalCrc;
    
   // finalize the calculation of the CRC
   finalCrc   = ~openserial_vars.outputCrc;
   
   // write the CRC value
   outputHdlcWrite((finalCrc>>0)&0xff);
   outputHdlcWrite((finalCrc>>8)&0xff);
   
   // write the closing HDLC flag
   openserial_vars.outputBuf[openserial_vars.outputBufIdxW++]   = HDLC_FLAG;
}

//===== hdlc (input)

/**
\brief Start an HDLC frame in the input buffer.
*/
port_INLINE void inputHdlcOpen() {
   // reset the input buffer index
   openserial_vars.inputBufFill                       = 0;
   
   // initialize the value of the CRC
   openserial_vars.inputCrc                           = HDLC_CRCINIT;
}
/**
\brief Add a byte to the incoming HDLC frame.
*/
port_INLINE void inputHdlcWrite(uint8_t b) {
   if (b==HDLC_ESCAPE) {
      openserial_vars.inputEscaping = TRUE;
   } else {
      if (openserial_vars.inputEscaping==TRUE) {
         b                             = b^HDLC_ESCAPE_MASK;
         openserial_vars.inputEscaping = FALSE;
      }
      
      // add byte to input buffer
      openserial_vars.inputBuf[openserial_vars.inputBufFill] = b;
      openserial_vars.inputBufFill++;
      
      // iterate through CRC calculator
      openserial_vars.inputCrc = crcIteration(openserial_vars.inputCrc,b);
   }
}
/**
\brief Finalize the incoming HDLC frame.
*/
port_INLINE void inputHdlcClose() {
   //teste rff
#if DEBUG_CMD_RPL
	//copio o buffer recebido da serial para um buffer
    memcpy (bufrffrx,openserial_vars.inputBuf,openserial_vars.inputBufFill);
    bufrffrxlen = openserial_vars.inputBufFill;

#endif
   //teste rff
   // verify the validity of the frame
   if (openserial_vars.inputCrc==HDLC_CRCGOOD) {
      // the CRC is correct
      
      // remove the CRC from the input buffer
      openserial_vars.inputBufFill    -= 2;
   } else {
      // the CRC is incorrect
      
      // drop the incoming fram
      openserial_vars.inputBufFill     = 0;
   }
}

//=========================== interrupt handlers ==============================

//executed in ISR, called from scheduler.c
void isr_openserial_tx() {
   switch (openserial_vars.mode) {
      case MODE_INPUT:
         openserial_vars.reqFrameIdx++;
#if (DEBUG_VIA_SERIAL == 0)
         if (openserial_vars.reqFrameIdx<sizeof(openserial_vars.reqFrame)) {
            uart_writeByte(openserial_vars.reqFrame[openserial_vars.reqFrameIdx]);
         }
     	 rffinputcount = openserial_vars.reqFrameIdx;
#endif
         break;

      case MODE_OUTPUT:
         if (openserial_vars.outputBufIdxW==openserial_vars.outputBufIdxR) {
            openserial_vars.outputBufFilled = FALSE;
         }
#if (DEBUG_VIA_SERIAL == 0)
         if (openserial_vars.outputBufFilled) {
            uart_writeByte(openserial_vars.outputBuf[openserial_vars.outputBufIdxR++]);
         }
#endif
         break;

      case MODE_OFF:
    	  break;
      default:
         break;
   }
}

#if ( DEBUG_UART1_DIRECT == 1)
void isr1_openserial_tx() {

	if (bufRffTxLen > 0) {
		if (bufRffTxCount < bufRffTxLen) {
			uart1_writeByte(bufRffTx[bufRffTxCount++]);
		}
		else {
			bufRffTxCount = 0;
			bufRffTxLen = 0;
			openserial_vars.outputBufFilled = FALSE;
			debugstatusRFF = 0;
		}
	}

}
#else
void isr1_openserial_tx() {

	 if (openserial_vars.outputBufIdxW==openserial_vars.outputBufIdxR) {
		openserial_vars.outputBufFilled = FALSE;
		debugstatusRFF = 0;
	 }
	 if (openserial_vars.outputBufFilled) {
		uart1_writeByte(openserial_vars.outputBuf[openserial_vars.outputBufIdxR++]);
	 }

}
#endif

// executed in ISR, called from scheduler.c
void isr_openserial_rx() {
   uint8_t rxbyte;
   uint8_t inputBufFill;
   
   // stop if I'm not in input mode
   if (openserial_vars.mode!=MODE_INPUT) {
      return;
   }
   
   // read byte just received
   rxbyte = uart_readByte();
   //keep lenght
   inputBufFill=openserial_vars.inputBufFill;
   
   if        (
                openserial_vars.busyReceiving==FALSE  &&
                openserial_vars.lastRxByte==HDLC_FLAG &&
                rxbyte!=HDLC_FLAG
              ) {
      // start of frame
	  //leds_error_toggle();
      
      // I'm now receiving
      openserial_vars.busyReceiving         = TRUE;
      
      // create the HDLC frame
      inputHdlcOpen();
      
      // add the byte just received
      inputHdlcWrite(rxbyte);
   } else if (
                openserial_vars.busyReceiving==TRUE   &&
                rxbyte!=HDLC_FLAG
             ) {
      // middle of frame
      
      // add the byte just received
      inputHdlcWrite(rxbyte);
      if (openserial_vars.inputBufFill+1>SERIAL_INPUT_BUFFER_SIZE){
         // input buffer overflow
         openserial_printError(COMPONENT_OPENSERIAL,ERR_INPUT_BUFFER_OVERFLOW,
                               (errorparameter_t)0,
                               (errorparameter_t)0);
         openserial_vars.inputBufFill       = 0;
         openserial_vars.busyReceiving      = FALSE;
         openserial_stop();
      }
   } else if (
                openserial_vars.busyReceiving==TRUE   &&
                rxbyte==HDLC_FLAG
              ) {
         // end of frame
         
         // finalize the HDLC frame
         inputHdlcClose();
         
         if (openserial_vars.inputBufFill==0){
            // invalid HDLC frame
            openserial_printError(COMPONENT_OPENSERIAL,ERR_WRONG_CRC_INPUT,
                                  (errorparameter_t)inputBufFill,
                                  (errorparameter_t)0);
         
         }
         
         openserial_vars.busyReceiving      = FALSE;
         openserial_stop();
   }
   
   openserial_vars.lastRxByte = rxbyte;
}

void isr1_openserial_rx() {
   uint8_t rxbyte;
   uint8_t inputBufFill;

   // stop if I'm not in input mode
   if (openserial_vars.mode!=MODE_INPUT) {
      return;
   }

   // read byte just received
   rxbyte = uart_readByte();
   //keep lenght
   inputBufFill=openserial_vars.inputBufFill;

   if        (
                openserial_vars.busyReceiving==FALSE  &&
                openserial_vars.lastRxByte==HDLC_FLAG &&
                rxbyte!=HDLC_FLAG
              ) {
      // start of frame

      // I'm now receiving
      openserial_vars.busyReceiving         = TRUE;

      // create the HDLC frame
      inputHdlcOpen();

      // add the byte just received
      inputHdlcWrite(rxbyte);
   } else if (
                openserial_vars.busyReceiving==TRUE   &&
                rxbyte!=HDLC_FLAG
             ) {
      // middle of frame

      // add the byte just received
      inputHdlcWrite(rxbyte);
      if (openserial_vars.inputBufFill+1>SERIAL_INPUT_BUFFER_SIZE){
         // input buffer overflow
         openserial_printError(COMPONENT_OPENSERIAL,ERR_INPUT_BUFFER_OVERFLOW,
                               (errorparameter_t)0,
                               (errorparameter_t)0);
         openserial_vars.inputBufFill       = 0;
         openserial_vars.busyReceiving      = FALSE;
         openserial_stop();
      }
   } else if (
                openserial_vars.busyReceiving==TRUE   &&
                rxbyte==HDLC_FLAG
              ) {
         // end of frame

         // finalize the HDLC frame
         inputHdlcClose();

         if (openserial_vars.inputBufFill==0){
            // invalid HDLC frame
            openserial_printError(COMPONENT_OPENSERIAL,ERR_WRONG_CRC_INPUT,
                                  (errorparameter_t)inputBufFill,
                                  (errorparameter_t)0);

         }

         openserial_vars.busyReceiving      = FALSE;
         openserial_stop();
   }

   openserial_vars.lastRxByte = rxbyte;
}

//======== SERIAL ECHO =============

void openserial_echo(uint8_t* buf, uint8_t bufLen){
   INTERRUPT_DECLARATION();
   // echo back what you received
   openserial_printData(
      buf,
      bufLen
   );
   
    DISABLE_INTERRUPTS();
    openserial_vars.inputBufFill = 0;
    ENABLE_INTERRUPTS();
}



#if (DEBUG_VIA_SERIAL == 1)

/*
uint8_t debug_send_frame(char *frame, uint8_t size)
{
 	uint16_t ui16Length=size;
    register uint16_t ui16Idx = 0;

    DISABLE_INTERRUPTS();
    while((ui16Idx < ui16Length))
    {
    	uart_writeByte(frame[ui16Idx++]);
    }
    ENABLE_INTERRUPTS();

    return ((uint8_t) ui16Length);
}

void UdpLogMsg(char *pucMsg, ...)
{
  int iUdpLogSize;
  char * pch;

  pch=strchr(pucMsg,0x00);   //find the end of the string
  iUdpLogSize = pch - pucMsg;

  if (iUdpLogSize >= DBG_MAX_REPLY_SIZE)
    iUdpLogSize = DBG_MAX_REPLY_SIZE;

  memcpy(pucUdpLog, pucMsg, iUdpLogSize);

  strcat(pucUdpLog, "\r\n");
  debug_send_frame(pucUdpLog, iUdpLogSize+2);

}
*/


//*****************************************************************************
//
//! Writes a string of characters to the UART output.
//!
//! \param pcBuf points to a buffer containing the string to transmit.
//! \param ui32Len is the length of the string to transmit.
//!
//! This function will transmit the string to the UART output.  The number of
//! characters transmitted is determined by the \e ui32Len parameter.  This
//! function does no interpretation or translation of any characters.  Since
//! the output is sent to a UART, any LF (/n) characters encountered will be
//! replaced with a CRLF pair.
//!
//! Besides using the \e ui32Len parameter to stop transmitting the string, if a
//! null character (0) is encountered, then no more characters will be
//! transmitted and the function will return.
//!
//! In non-buffered mode, this function is blocking and will not return until
//! all the characters have been written to the output FIFO.  In buffered mode,
//! the characters are written to the UART transmit buffer and the call returns
//! immediately.  If insufficient space remains in the transmit buffer,
//! additional characters are discarded.
//!
//! \return Returns the count of characters written.
//
//*****************************************************************************
int
UARTwrite(const char *pcBuf, uint32_t ui32Len)
{
#ifdef UART_BUFFERED
    unsigned int uIdx;

    //
    // Check for valid arguments.
    //
    ASSERT(pcBuf != 0);
    ASSERT(g_ui32Base != 0);

    //
    // Send the characters
    //
    for(uIdx = 0; uIdx < ui32Len; uIdx++)
    {
        //
        // If the character to the UART is \n, then add a \r before it so that
        // \n is translated to \n\r in the output.
        //
        if(pcBuf[uIdx] == '\n')
        {
            if(!TX_BUFFER_FULL)
            {
                g_pcUARTTxBuffer[g_ui32UARTTxWriteIndex] = '\r';
                ADVANCE_TX_BUFFER_INDEX(g_ui32UARTTxWriteIndex);
            }
            else
            {
                //
                // Buffer is full - discard remaining characters and return.
                //
                break;
            }
        }

        //
        // Send the character to the UART output.
        //
        if(!TX_BUFFER_FULL)
        {
            g_pcUARTTxBuffer[g_ui32UARTTxWriteIndex] = pcBuf[uIdx];
            ADVANCE_TX_BUFFER_INDEX(g_ui32UARTTxWriteIndex);
        }
        else
        {
            //
            // Buffer is full - discard remaining characters and return.
            //
            break;
        }
    }

    //
    // If we have anything in the buffer, make sure that the UART is set
    // up to transmit it.
    //
    if(!TX_BUFFER_EMPTY)
    {
        UARTPrimeTransmit(g_ui32Base);
        UARTIntEnable(g_ui32Base, UART_INT_TX);
    }

    //
    // Return the number of characters written.
    //
    return(uIdx);
#else
    unsigned int uIdx;

    //
    // Check for valid UART base address, and valid arguments.
    //
    //ASSERT(g_ui32Base != 0);
    //ASSERT(pcBuf != 0);

    //
    // Send the characters
    //
    DISABLE_INTERRUPTS();
    for(uIdx = 0; uIdx < ui32Len; uIdx++)
    {
        //
        // If the character to the UART is \n, then add a \r before it so that
        // \n is translated to \n\r in the output.
        //
        if(pcBuf[uIdx] == '\n')
        {
        	uart_writeByte('\r');
        }

        //
        // Send the character to the UART output.
        //
        //UARTCharPut(g_ui32Base, );
		uart_writeByte(pcBuf[uIdx]);
    }
    ENABLE_INTERRUPTS();

    //
    // Return the number of characters written.
    //
    return(uIdx);
#endif
}

//*****************************************************************************
//
//! A simple UART based vprintf function supporting \%c, \%d, \%p, \%s, \%u,
//! \%x, and \%X.
//!
//! \param pcString is the format string.
//! \param vaArgP is a variable argument list pointer whose content will depend
//! upon the format string passed in \e pcString.
//!
//! This function is very similar to the C library <tt>vprintf()</tt> function.
//! All of its output will be sent to the UART.  Only the following formatting
//! characters are supported:
//!
//! - \%c to print a character
//! - \%d or \%i to print a decimal value
//! - \%s to print a string
//! - \%u to print an unsigned decimal value
//! - \%x to print a hexadecimal value using lower case letters
//! - \%X to print a hexadecimal value using lower case letters (not upper case
//! letters as would typically be used)
//! - \%p to print a pointer as a hexadecimal value
//! - \%\% to print out a \% character
//!
//! For \%s, \%d, \%i, \%u, \%p, \%x, and \%X, an optional number may reside
//! between the \% and the format character, which specifies the minimum number
//! of characters to use for that value; if preceded by a 0 then the extra
//! characters will be filled with zeros instead of spaces.  For example,
//! ``\%8d'' will use eight characters to print the decimal value with spaces
//! added to reach eight; ``\%08d'' will use eight characters as well but will
//! add zeroes instead of spaces.
//!
//! The type of the arguments in the variable arguments list must match the
//! requirements of the format string.  For example, if an integer was passed
//! where a string was expected, an error of some kind will most likely occur.
//!
//! \return None.
//
//*****************************************************************************
void
UARTvprintf(const char *pcString, __VALIST vaArgP)
{
    uint32_t ui32Idx, ui32Value, ui32Pos, ui32Count, ui32Base, ui32Neg;
    char *pcStr, pcBuf[16], cFill;

    //
    // Check the arguments.
    //
    //ASSERT(pcString != 0);

    //
    // Loop while there are more characters in the string.
    //
    while(*pcString)
    {
        //
        // Find the first non-% character, or the end of the string.
        //
        for(ui32Idx = 0; (pcString[ui32Idx] != '%') && (pcString[ui32Idx] != '\0');
            ui32Idx++)
        {
        }

        //
        // Write this portion of the string.
        //
        UARTwrite(pcString, ui32Idx);

        //
        // Skip the portion of the string that was written.
        //
        pcString += ui32Idx;

        //
        // See if the next character is a %.
        //
        if(*pcString == '%')
        {
            //
            // Skip the %.
            //
            pcString++;

            //
            // Set the digit count to zero, and the fill character to space
            // (i.e. to the defaults).
            //
            ui32Count = 0;
            cFill = ' ';

            //
            // It may be necessary to get back here to process more characters.
            // Goto's aren't pretty, but effective.  I feel extremely dirty for
            // using not one but two of the beasts.
            //
again:

            //
            // Determine how to handle the next character.
            //
            switch(*pcString++)
            {
                //
                // Handle the digit characters.
                //
                case '0':
                case '1':
                case '2':
                case '3':
                case '4':
                case '5':
                case '6':
                case '7':
                case '8':
                case '9':
                {
                    //
                    // If this is a zero, and it is the first digit, then the
                    // fill character is a zero instead of a space.
                    //
                    if((pcString[-1] == '0') && (ui32Count == 0))
                    {
                        cFill = '0';
                    }

                    //
                    // Update the digit count.
                    //
                    ui32Count *= 10;
                    ui32Count += pcString[-1] - '0';

                    //
                    // Get the next character.
                    //
                    goto again;
                }

                //
                // Handle the %c command.
                //
                case 'c':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ui32Value = va_arg(vaArgP, uint32_t);

                    //
                    // Print out the character.
                    //
                    UARTwrite((char *)&ui32Value, 1);

                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle the %d and %i commands.
                //
                case 'd':
                case 'i':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ui32Value = va_arg(vaArgP, uint32_t);

                    //
                    // Reset the buffer position.
                    //
                    ui32Pos = 0;

                    //
                    // If the value is negative, make it positive and indicate
                    // that a minus sign is needed.
                    //
                    if((int32_t)ui32Value < 0)
                    {
                        //
                        // Make the value positive.
                        //
                        ui32Value = -(int32_t)ui32Value;

                        //
                        // Indicate that the value is negative.
                        //
                        ui32Neg = 1;
                    }
                    else
                    {
                        //
                        // Indicate that the value is positive so that a minus
                        // sign isn't inserted.
                        //
                        ui32Neg = 0;
                    }

                    //
                    // Set the base to 10.
                    //
                    ui32Base = 10;

                    //
                    // Convert the value to ASCII.
                    //
                    goto convert;
                }

                //
                // Handle the %s command.
                //
                case 's':
                {
                    //
                    // Get the string pointer from the varargs.
                    //
                    pcStr = va_arg(vaArgP, char *);

                    //
                    // Determine the length of the string.
                    //
                    for(ui32Idx = 0; pcStr[ui32Idx] != '\0'; ui32Idx++)
                    {
                    }

                    //
                    // Write the string.
                    //
                    UARTwrite(pcStr, ui32Idx);

                    //
                    // Write any required padding spaces
                    //
                    if(ui32Count > ui32Idx)
                    {
                        ui32Count -= ui32Idx;
                        while(ui32Count--)
                        {
                            UARTwrite(" ", 1);
                        }
                    }

                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle the %u command.
                //
                case 'u':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ui32Value = va_arg(vaArgP, uint32_t);

                    //
                    // Reset the buffer position.
                    //
                    ui32Pos = 0;

                    //
                    // Set the base to 10.
                    //
                    ui32Base = 10;

                    //
                    // Indicate that the value is positive so that a minus sign
                    // isn't inserted.
                    //
                    ui32Neg = 0;

                    //
                    // Convert the value to ASCII.
                    //
                    goto convert;
                }

                //
                // Handle the %x and %X commands.  Note that they are treated
                // identically; i.e. %X will use lower case letters for a-f
                // instead of the upper case letters is should use.  We also
                // alias %p to %x.
                //
                case 'x':
                case 'X':
                case 'p':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ui32Value = va_arg(vaArgP, uint32_t);

                    //
                    // Reset the buffer position.
                    //
                    ui32Pos = 0;

                    //
                    // Set the base to 16.
                    //
                    ui32Base = 16;

                    //
                    // Indicate that the value is positive so that a minus sign
                    // isn't inserted.
                    //
                    ui32Neg = 0;

                    //
                    // Determine the number of digits in the string version of
                    // the value.
                    //
convert:
                    for(ui32Idx = 1;
                        (((ui32Idx * ui32Base) <= ui32Value) &&
                         (((ui32Idx * ui32Base) / ui32Base) == ui32Idx));
                        ui32Idx *= ui32Base, ui32Count--)
                    {
                    }

                    //
                    // If the value is negative, reduce the count of padding
                    // characters needed.
                    //
                    if(ui32Neg)
                    {
                        ui32Count--;
                    }

                    //
                    // If the value is negative and the value is padded with
                    // zeros, then place the minus sign before the padding.
                    //
                    if(ui32Neg && (cFill == '0'))
                    {
                        //
                        // Place the minus sign in the output buffer.
                        //
                        pcBuf[ui32Pos++] = '-';

                        //
                        // The minus sign has been placed, so turn off the
                        // negative flag.
                        //
                        ui32Neg = 0;
                    }

                    //
                    // Provide additional padding at the beginning of the
                    // string conversion if needed.
                    //
                    if((ui32Count > 1) && (ui32Count < 16))
                    {
                        for(ui32Count--; ui32Count; ui32Count--)
                        {
                            pcBuf[ui32Pos++] = cFill;
                        }
                    }

                    //
                    // If the value is negative, then place the minus sign
                    // before the number.
                    //
                    if(ui32Neg)
                    {
                        //
                        // Place the minus sign in the output buffer.
                        //
                        pcBuf[ui32Pos++] = '-';
                    }

                    //
                    // Convert the value into a string.
                    //
                    for(; ui32Idx; ui32Idx /= ui32Base)
                    {
                        pcBuf[ui32Pos++] = g_pcHex[(ui32Value / ui32Idx) % ui32Base];
                    }

                    //
                    // Write the string.
                    //
                    UARTwrite(pcBuf, ui32Pos);

                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle the %% command.
                //
                case '%':
                {
                    //
                    // Simply write a single %.
                    //
                    UARTwrite(pcString - 1, 1);

                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle all other commands.
                //
                default:
                {
                    //
                    // Indicate an error.
                    //
                    UARTwrite("ERROR", 5);

                    //
                    // This command has been handled.
                    //
                    break;
                }
            }
        }
    }
}

//*****************************************************************************
//
//! A simple UART based printf function supporting \%c, \%d, \%p, \%s, \%u,
//! \%x, and \%X.
//!
//! \param pcString is the format string.
//! \param ... are the optional arguments, which depend on the contents of the
//! format string.
//!
//! This function is very similar to the C library <tt>fprintf()</tt> function.
//! All of its output will be sent to the UART.  Only the following formatting
//! characters are supported:
//!
//! - \%c to print a character
//! - \%d or \%i to print a decimal value
//! - \%s to print a string
//! - \%u to print an unsigned decimal value
//! - \%x to print a hexadecimal value using lower case letters
//! - \%X to print a hexadecimal value using lower case letters (not upper case
//! letters as would typically be used)
//! - \%p to print a pointer as a hexadecimal value
//! - \%\% to print out a \% character
//!
//! For \%s, \%d, \%i, \%u, \%p, \%x, and \%X, an optional number may reside
//! between the \% and the format character, which specifies the minimum number
//! of characters to use for that value; if preceded by a 0 then the extra
//! characters will be filled with zeros instead of spaces.  For example,
//! ``\%8d'' will use eight characters to print the decimal value with spaces
//! added to reach eight; ``\%08d'' will use eight characters as well but will
//! add zeroes instead of spaces.
//!
//! The type of the arguments after \e pcString must match the requirements of
//! the format string.  For example, if an integer was passed where a string
//! was expected, an error of some kind will most likely occur.
//!
//! \return None.
//
//*****************************************************************************
void UARTprintf(const char *pcString, ...)
{
	__VALIST vaArgP;

    //
    // Start the varargs processing.
    //
    va_start(vaArgP, pcString);

    UARTvprintf(pcString, vaArgP);

    //
    // We're finished with the varargs now.
    //
    va_end(vaArgP);
}

#endif /* (DEBUG_VIA_SERIAL == 1)*/


