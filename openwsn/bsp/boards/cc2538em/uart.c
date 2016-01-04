/**
\brief CC2538-specific definition of the "uart" bsp module.

\author Xavier Vilajosana <xvilajosana@eecs.berkeley.edu>, September 2013.
*/


#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "uart.h"
#include "uarthal.h"
#include "hw_ints.h"
#include "interrupt.h"
#include "sys_ctrl.h"
#include "gpio.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "board.h"
#include "ioc.h"
#include "hw_ioc.h"
#include "debugpins.h"
#include "osens.h"
#include "osens_itf.h"
#include "openserial.h"

//=========================== defines =========================================

#define PIN_UART_RXD            GPIO_PIN_0 // PA0 is UART RX
#define PIN_UART_TXD            GPIO_PIN_1 // PA1 is UART TX

//=========================== variables =======================================

typedef struct {
   uart_tx_cbt txCb;
   uart_rx_cbt rxCb;
} uart_vars_t;

uart_vars_t uart_vars;
extern uint8_t frame[OSENS_MAX_FRAME_SIZE];
extern uint8_t num_rx_bytes;
kick_scheduler_t uart1_tx_isr(void);
kick_scheduler_t uart1_rx_isr(void);

#if (DBG_USING_UART1 == 1)
uart_vars_t uart1_vars;
extern uint8_t frame1[OSENS_MAX_FRAME_SIZE];
extern uint8_t num_rx_bytes1;
kick_scheduler_t uart1_tx_isr(void);
kick_scheduler_t uart1_rx_isr(void);
#endif
//=========================== prototypes ======================================

static void uart_isr_private(void);
#if (DBG_USING_UART1 == 1)
static  void uart1_isr_private(void);
uart_vars_t uart1_vars;
#else
void uart1_isr_private(void);
#endif
//=========================== public ==========================================

void uart_init() {
   register uint32_t i;
   
   // reset local variables
   memset(&uart_vars,0,sizeof(uart_vars_t));
   
   // wait some time before initializing UART, since don't want the
   // OpenMoteCC2538 to start generating data before the FTDI chip on the
   // OpenBase or XBee Explorer has fully initialized
   for(i=0;i<320000;i++);
   
   // ENABLE UART 0 - STANDARD

   // Disable UART function
   UARTDisable(UART0_BASE);

   // Disable all UART module interrupts
   UARTIntDisable(UART0_BASE, 0x1FFF);

   // Set IO clock as UART clock source
   UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

   // Map UART signals to the correct GPIO pins and configure them as
   // hardware controlled. GPIO-A pin 0 and 1
   IOCPinConfigPeriphOutput(GPIO_A_BASE, PIN_UART_TXD, IOC_MUX_OUT_SEL_UART0_TXD);
   GPIOPinTypeUARTOutput(GPIO_A_BASE, PIN_UART_TXD);
   IOCPinConfigPeriphInput(GPIO_A_BASE, PIN_UART_RXD, IOC_UARTRXD_UART0);
   GPIOPinTypeUARTInput(GPIO_A_BASE, PIN_UART_RXD);

   // Configure the UART for 115,200, 8-N-1 operation.
   // This function uses SysCtrlClockGet() to get the system clock
   // frequency.  This could be also be a variable or hard coded value
   // instead of a function call.
#if (DEBUG_VIA_SERIAL == 1)
   UARTConfigSetExpClk(UART0_BASE, SysCtrlIOClockGet(), 9600,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
#else
   UARTConfigSetExpClk(UART0_BASE, SysCtrlIOClockGet(), 115200,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
#endif
   // Enable UART hardware
   UARTEnable(UART0_BASE);

   // Disable FIFO as we only one 1byte buffer
   UARTFIFODisable(UART0_BASE);

   // Raise interrupt at end of tx (not by fifo)
   UARTTxIntModeSet(UART0_BASE, UART_TXINT_MODE_EOT);

   // Register isr in the nvic and enable isr at the nvic
   UARTIntRegister(UART0_BASE, uart_isr_private);

   // Enable the UART0 interrupt
   IntEnable(INT_UART0);
}

void uart_setCallbacks(uart_tx_cbt txCb, uart_rx_cbt rxCb) {
    uart_vars.txCb = txCb;
    uart_vars.rxCb = rxCb;
}

void uart_enableInterrupts(){
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_TX);
}

void uart_disableInterrupts(){
    UARTIntDisable(UART0_BASE, UART_INT_RX | UART_INT_TX);
}

void uart_clearRxInterrupts(){
    UARTIntClear(UART0_BASE, UART_INT_RX);
}

void uart_clearTxInterrupts(){
    UARTIntClear(UART0_BASE, UART_INT_TX);
}

void  uart_writeByte(uint8_t byteToWrite){
	UARTCharPut(UART0_BASE, byteToWrite);
}

uint8_t uart_readByte(){
	 int32_t i32Char;
     i32Char = UARTCharGet(UART0_BASE);
	 return (uint8_t)(i32Char & 0xFF);
}

//=========================== interrupt handlers ==============================

static void uart_isr_private(void){
	uint32_t reg;
	debugpins_isr_set();

	// Read interrupt source
	reg = UARTIntStatus(UART0_BASE, true);

	// Clear UART interrupt in the NVIC
	IntPendClear(INT_UART0);

	// Process TX interrupt
	if(reg & UART_INT_TX){
	     uart_tx_isr();
	}

	// Process RX interrupt
	if(reg & (UART_INT_RX )) {
		uart_rx_isr();
	}

	debugpins_isr_clr();
}

kick_scheduler_t uart_tx_isr() {
   uart_clearTxInterrupts(); // TODO: do not clear, but disable when done
   if (uart_vars.txCb != NULL) {
       uart_vars.txCb();
   }
   return DO_NOT_KICK_SCHEDULER;
}

kick_scheduler_t uart_rx_isr() {
   uart_clearRxInterrupts(); // TODO: do not clear, but disable when done
   if (uart_vars.txCb != NULL) {
       uart_vars.rxCb();
   }
   return DO_NOT_KICK_SCHEDULER;
}

/*  =================================================
 *  This routines is used when wish to use the UART1 as the debug
 *  the idea is have the same default debug in the UART0 and only
 *  the especial debugs in the UART1
 */

#if (DBG_USING_UART1 == 1)

void uart1_init() {
   register uint32_t i;

   // reset local variables
   memset(&uart1_vars,0,sizeof(uart_vars_t));

   // wait some time before initializing UART, since don't want the
   // OpenMoteCC2538 to start generating data before the FTDI chip on the
   // OpenBase or XBee Explorer has fully initialized
   for(i=0;i<320000;i++);

   // Disable UART function
   UARTDisable(DBG_UART1_BASE);
   // Disable all UART module interrupts
   UARTIntDisable(DBG_UART1_BASE, 0x1FFF);

   UARTClockSourceSet(DBG_UART1_BASE, UART_CLOCK_PIOSC);

	IOCPinConfigPeriphOutput (DBG_UART1_BUS_BASE, DBG_UART1_TXD, DBG_MUX_UART1_TXD);
	GPIOPinTypeUARTOutput    (DBG_UART1_BUS_BASE, DBG_UART1_TXD);
	IOCPinConfigPeriphInput  (DBG_UART1_BUS_BASE, DBG_UART1_RXD, DBG_MUX_UART1_RXD);
	GPIOPinTypeUARTInput     (DBG_UART1_BUS_BASE, DBG_UART1_RXD);

   UARTConfigSetExpClk(DBG_UART1_BASE, SysCtrlIOClockGet(), 115200,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

   // Enable UART hardware
   UARTEnable(DBG_UART1_BASE);

   // Disable FIFO as we only one 1byte buffer
   UARTFIFODisable(DBG_UART1_BASE);

   // Raise interrupt at end of tx (not by fifo)
   UARTTxIntModeSet(DBG_UART1_BASE, UART_TXINT_MODE_EOT);

   // Register isr in the nvic and enable isr at the nvic
   UARTIntRegister(DBG_UART1_BASE, uart1_isr_private);

   IntEnable(DBG_INT_UART1);

}
void uart1_enableInterrupts(){
    UARTIntEnable(DBG_UART1_BASE, UART_INT_RX | UART_INT_TX);
}

void uart1_disableInterrupts(){
    UARTIntDisable(DBG_UART1_BASE, UART_INT_RX | UART_INT_TX);
}

void uart1_clearRxInterrupts(){
    UARTIntClear(DBG_UART1_BASE, UART_INT_RX);
}

void uart1_clearTxInterrupts(){
    UARTIntClear(DBG_UART1_BASE, UART_INT_TX);
}

void  uart1_writeByte(uint8_t byteToWrite){
	UARTCharPut(DBG_UART1_BASE, byteToWrite);
}

uint8_t uart1_readByte(){
	 int32_t i32Char;
     i32Char = UARTCharGet(DBG_UART1_BASE);
	 return (uint8_t)(i32Char & 0xFF);
}

#if (DBG_USING_UART1 == 1)

kick_scheduler_t uart1_tx_isr() {
   uart1_clearTxInterrupts(); // TODO: do not clear, but disable when done
   if (uart1_vars.txCb != NULL) {
       uart1_vars.txCb();
   }
   return DO_NOT_KICK_SCHEDULER;
}

kick_scheduler_t uart1_rx_isr() {
   uart1_clearRxInterrupts(); // TODO: do not clear, but disable when done
   if (uart1_vars.txCb != NULL) {
       uart1_vars.rxCb();
   }
   return DO_NOT_KICK_SCHEDULER;
}

void uart1_setCallbacks(uart_tx_cbt txCb, uart_rx_cbt rxCb) {
    uart1_vars.txCb = txCb;
    uart1_vars.rxCb = rxCb;
}
#endif

static void uart1_isr_private(void)
{
	uint32_t reg;
	debugpins_isr_set();

	// Read interrupt source
	reg = UARTIntStatus(DBG_UART1_BASE, true);

	// Clear UART interrupt in the NVIC
	IntPendClear(DBG_INT_UART1);

	// Process TX interrupt
	if(reg & UART_INT_TX){
	     uart1_tx_isr();
	}

	// Process RX interrupt
	if(reg & (UART_INT_RX )) {
		uart1_rx_isr();
	}

}



#endif

//=========================== SENS_ITF routines ==============================
//

#if (DBG_USING_UART1 == 0)

void uart1_init() {
   // reset local variables
   //memset(&uart_vars,0,sizeof(uart_vars_t));

   UARTDisable(OSENS_UART_BASE);

   UARTIntDisable(OSENS_UART_BASE, 0x1FFF);

   UARTClockSourceSet(OSENS_UART_BASE, UART_CLOCK_PIOSC);

	IOCPinConfigPeriphOutput (OSENS_UART_BUS_BASE, OSENS_UART_TXD, OSENS_MUX_UART_TXD);
	GPIOPinTypeUARTOutput    (OSENS_UART_BUS_BASE, OSENS_UART_TXD);
	IOCPinConfigPeriphInput  (OSENS_UART_BUS_BASE, OSENS_UART_RXD, OSENS_MUX_UART_RXD);
	GPIOPinTypeUARTInput     (OSENS_UART_BUS_BASE, OSENS_UART_RXD);

   UARTConfigSetExpClk(OSENS_UART_BASE, SysCtrlIOClockGet(), 9600,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

   UARTEnable(OSENS_UART_BASE);

   UARTIntRegister(OSENS_UART_BASE, uart1_isr_private);
   UARTFIFODisable(OSENS_UART_BASE);
   //UARTTxIntModeSet(OSENS_UART_BASE, UART_TXINT_MODE_EOT);
   //uart1_clearTxInterrupts();
   //uart1_clearRxInterrupts();      // clear possible pending interrupts
   uart1_enableInterrupts();

   //IntEnable(OSENS_INT_UART);


}

void uart1_setCallbacks(uart_tx_cbt txCb, uart_rx_cbt rxCb) {
    uart_vars.txCb = txCb;
    uart_vars.rxCb = rxCb;
}

void uart1_enableInterrupts(){
    // UARTIntEnable  (BSP_UART_BASE, (UART_INT_RX | UART_INT_RT | UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE));
    //UARTIntEnable(OSENS_UART_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
    UARTIntEnable(OSENS_UART_BASE, UART_INT_RX);

}

void uart1_disableInterrupts(){
    UARTIntDisable(OSENS_UART_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
}

void uart1_clearRxInterrupts(){
    UARTIntClear(OSENS_UART_BASE, UART_INT_RX | UART_INT_RT);
}

void uart1_clearTxInterrupts(){
    UARTIntClear(OSENS_UART_BASE, UART_INT_TX);
}

void  uart1_writeByte(uint8_t byteToWrite){
	UARTCharPut(OSENS_UART_BASE, byteToWrite);
}

uint8_t uart1_readByte(){
	 int32_t i32Char;
     i32Char = UARTCharGet(OSENS_UART_BASE);
	 return (uint8_t)(i32Char & 0xFF);
}

#endif

//=========================== SENS_ITF Interrupt handlers ==============================



