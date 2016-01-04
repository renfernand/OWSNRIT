#ifndef __BOARD_H
#define __BOARD_H

/**
\addtogroup BSP
\{
\addtogroup board
\{

\brief Cross-platform declaration "board" bsp module.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, February 2012.
*/

#include "board_info.h"
#include "toolchain_defs.h"

#define SINK 1

//enable the IEEE802154E Assyncronous RIT Protocol
#define IEEE802154E_RIT   1

#if (IEEE802154E_RIT == 1)
/* enable the CSMA_CA in the transmission */
#define ENABLE_CSMA_CA  1
#else
#define ENABLE_CSMA_CA  0
#endif

#define FORCETOPOLOGY 0 //atencao!!! aqui eu devo preencher a tabela pois somente tem alguns motes.

//define o nr de msg de tx que sera enviada antes de ter um pedido de um rx para o openvisualizer
#define OPENBRIDGE_MAX_TX_TO_RX 2

//usa o novo esquema de bridge para o dagroot (1) ou o antigo (0).
//lembrando que o novo (1) esta com problemas no ping e coap
#define NEW_DAG_BRIDGE 0

/* define a UART0 para o DAG ROOT
 * DBG_USING_UART1
 * Define se usará a UART1 pinos P1.18(Rx) P1.20(Tx) para debug.
 * Esta porta eh utilizada por padrao pelo MOTE para comunicacao com SENS_ITF
 * porem aqui ela eh usada pelo SINK como debug...
 * Tambem a porta de comunicacao do sink com o OpenVisualizer eh a UART0. Neste caso
 * somente frames especificos de debug sao passados para a interface 1.
 * Como o mote nao usa a UART0, esta porta eh o debug por default.
 */
#if SINK
#define ENABLE_BRIDGE_UART0       1     //Quando DAGROOT ele fica enviando os logs comuns na serial...isso atrapalha o debug
#define ENABLE_DEBUG_RFF          1     //imprime na serial o debug 0x11
#define DBG_USING_UART1           0     //quando 1 indica que o debug 0x11 vai para UART1 (somente util qdo dagROOT)
#define DAGROOT_ENABLE_ONSTARTUP  1
#define DAGROOT                   1
#define SINK_SIMULA_COAP          0
#else
#define ENABLE_BRIDGE_UART0       0     //Quando DAGROOT ele fica enviando os logs comuns na serial...isso atrapalha o debug
#define ENABLE_DEBUG_RFF          1     //imprime na serial o debug 0x11
#define DBG_USING_UART1           0     //quando 1 indica que o debug 0x11 vai para UART1 (somente util qdo dagROOT)
#define DAGROOT_ENABLE_ONSTARTUP  0
#define DBG_USING_UART1           0
#define SINK_SIMULA_COAP          0
#endif

#if ENABLE_DEBUG_RFF
#define DEBUG_LOG_RIT   1
#define DBG_IEEE802_TX  1
#define DBG_IEEE802_RX  1
#define DBG_RPL         1
#define DBG_FORWARDING  1
#define DBG_APP_1       0     //debug aplicacao - arquivo bsp\osens_itf_mote

#else
#define DEBUG_LOG_RIT  0
#endif
//enable the log using UART0 - cannot be used for DAG ROOT
#define DEBUG_VIA_SERIAL 0

#define OSENS_UART_INT  0


/*
 * HABILITA OS SENSORES JA UTILIZADOS NESTA PLACA CC2538EM
 * MYLINKXS_LIGHT_CONTROL - Renato Fernandes - sem necessidade de placa sensora.
 * MYLINKXS_REMOTE_CONTROL - Renato Fernandes - com arduino acionando controle remoto
 * OSENS - Original do marcelo Barros com placa remota Freescale
 *
 **/

#if (SINK == 1)
#define MYLINKXS_SENSORS 0
#else
#define MYLINKXS_SENSORS 1
#endif

//ATENCAO!!!!! SOMENTE DEVE SER ESCOLHIDO UM SENSOR ABAIXO
#if  (MYLINKXS_SENSORS == 1)
	#define SENSOR_ACCEL            0     // accelerometer sensor for the smartrf06 evaluation board
	#define SONOMA14                0     // accelerometer sensor for the smartrf06 evaluation board
	#define MYLINKXS_REMOTE_CONTROL 0   // Este habilita a comunicacao serial via arduino usando UART1
	#define MYLINKXS_LIGHT_CONTROL  1   // Este habilita o controle de uma lampada na propria placa - sem necessidade de placa externa

    //define porta de comunicacao
	//define the SENS_ITF UART or SPI : SPI --> USE_SPI_INTERFACE = 1 or UART --> USE_SPI_INTERFACE = 0
	#if ((SENSOR_ACCEL == 1) || (SONOMA14 == 1))
	#define USE_SPI_INTERFACE  1
	#else
	#define USE_SPI_INTERFACE  0
	#endif


	#define BYTE_END_FRAME 'Z'   //usado pelo REMOTE_CONTROL


#else
#define USE_SPI_INTERFACE  0
#endif



//=========================== define ==========================================

typedef enum {
   DO_NOT_KICK_SCHEDULER,
   KICK_SCHEDULER,
} kick_scheduler_t;


//*****************************************************************************
//
// SPI defines (Common for LCD, SD reader and accelerometer)
//*****************************************************************************
#if (USE_SPI_INTERFACE == 1)
#define  USE_SPI_INTERRUPT 0

#define BSP_SPI_SSI_BASE        SSI0_BASE
//! Bitmask to enable SSI module.
#define BSP_SPI_SSI_ENABLE_BM   SYS_CTRL_PERIPH_SSI0
#define BSP_SPI_BUS_BASE        GPIO_A_BASE
#define BSP_SPI_SCK             GPIO_PIN_2      //!< PA2
#define BSP_SPI_FSS             GPIO_PIN_3      //!< PA3
#define BSP_SPI_MOSI            GPIO_PIN_4      //!< PA4
#define BSP_SPI_MISO            GPIO_PIN_5      //!< PA5

//! Default SPI clock speed. 8 MHz is supported by all SmartRF06EB peripherals.
#define BSP_SPI_CLK_SPD         1000000UL


#if SENSOR_ACCEL
// Board accelerometer defines
#define BSP_ACC_PWR_BASE        GPIO_D_BASE
#define BSP_ACC_PWR             GPIO_PIN_4      //!< PD4
#define BSP_ACC_INT_BASE        GPIO_D_BASE
#define BSP_ACC_INT             GPIO_PIN_2      //!< PD2
#define BSP_ACC_INT1_BASE       BSP_ACC_INT_BASE
#define BSP_ACC_INT1            BSP_ACC_INT     //!< ACC_INT1 == ACC_INT
#define BSP_ACC_INT2_BASE       GPIO_D_BASE
#define BSP_ACC_INT2            GPIO_PIN_2      //!< PD1
#define BSP_ACC_CS_BASE         GPIO_D_BASE
#define BSP_ACC_CS              GPIO_PIN_5      //!< PD5
#define BSP_ACC_SCK_BASE        BSP_SPI_BUS_BASE
#define BSP_ACC_SCK             BSP_SPI_SCK     //!< PA2
#define BSP_ACC_MOSI_BASE       BSP_SPI_BUS_BASE
#define BSP_ACC_MOSI            BSP_SPI_MOSI    //!< PA4
#define BSP_ACC_MISO_BASE       BSP_SPI_BUS_BASE
#define BSP_ACC_MISO            BSP_SPI_MISO    //!< PA5
#endif


#else
//*****************************************************************************
//
// UART interface
//*****************************************************************************

// UART backchannel defines
#if 0
#define BSP_UART_BASE           UART0_BASE
#define BSP_UART_ENABLE_BM      SYS_CTRL_PERIPH_UART0
#define BSP_UART_BUS_BASE       GPIO_A_BASE
#define BSP_UART_RXD_BASE       BSP_UART_BUS_BASE
#define BSP_UART_RXD            GPIO_PIN_0      //!< PA0
#define BSP_UART_TXD_BASE       BSP_UART_BUS_BASE
#define BSP_UART_TXD            GPIO_PIN_1      //!< PA1
#define BSP_UART_CTS_BASE       GPIO_B_BASE
#define BSP_UART_CTS            GPIO_PIN_0      //!< PB0
#define BSP_UART_RTS_BASE       GPIO_D_BASE
#define BSP_UART_RTS            GPIO_PIN_3      //!< PD3
#define BSP_UART_INT_BM         0xF0            //!< Interrupts handled by bsp uart
#define BSP_INT_UART            INT_UART0
#define BSP_MUX_UART_TXD        IOC_MUX_OUT_SEL_UART0_TXD
#define BSP_MUX_UART_RXD        IOC_UARTRXD_UART0
#else
#define BSP_UART_BASE           UART1_BASE
#define BSP_UART_ENABLE_BM      SYS_CTRL_PERIPH_UART1
#define BSP_UART_BUS_BASE       GPIO_A_BASE
#define BSP_UART_RXD_BASE       BSP_UART_BUS_BASE
#define BSP_UART_RXD            GPIO_PIN_5      //!< PA5
#define BSP_UART_TXD_BASE       BSP_UART_BUS_BASE
#define BSP_UART_TXD            GPIO_PIN_4      //!< PA4
#define BSP_UART_CTS_BASE       GPIO_B_BASE
#define BSP_UART_CTS            GPIO_PIN_0      //!< PB0
#define BSP_UART_RTS_BASE       GPIO_D_BASE
#define BSP_UART_RTS            GPIO_PIN_3      //!< PD3
#define BSP_UART_INT_BM         0xF0            //!< Interrupts handled by bsp uart
#define BSP_INT_UART            INT_UART1
#define BSP_MUX_UART_TXD        IOC_MUX_OUT_SEL_UART1_TXD
#define BSP_MUX_UART_RXD        IOC_UARTRXD_UART1
#endif

#endif

//end test rff
//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

void board_init(void);
void board_sleep(void);
void board_reset(void);
void bspserial(void);

void bspSpiInit(void);
/**
\}
\}
*/

#endif
