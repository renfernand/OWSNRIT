/**
\brief cc2538-specific definition of the "radiotimer" bsp module.

\author Xavier Vilajosana <xvilajosana@eecs.berkeley.edu>, August 2013.
*/


#include "stdio.h"
#include "string.h"
#include "radiotimer.h"
#include "debugpins.h"
#include "hw_rfcore_sfr.h"
#include "interrupt.h"
#include "sys_ctrl.h"
#include "hw_ints.h"
#include "hw_types.h"
#include "sys_ctrl.h"
#include "leds.h"
#include "ieee802154e.h"
#include "systick.h"
#if (ENABLE_CSMA_CA  == 1)
#include "hw_sys_ctrl.h"
#include "mac_csp_tx.h"
#include "opentimers.h"
#endif
#include "hw_memmap.h"

#if (USE_GPTIMER == 1)
#include "gptimer.h"
#include "hw_gptimer.h"
#endif

#define RADIOTIMER_32MHZ_TICS_PER_32KHZ_TIC 976  //32Mhz to 32Khz ratio

//=========================== variables =======================================

typedef struct {
   radiotimer_compare_cbt    overflow_cb;
   radiotimer_compare_cbt    compare_cb;
} radiotimer_vars_t;
#if (ENABLE_CSMA_CA == 1)
extern radio_csma_vars_t radio_csma_vars;
#endif

radiotimer_vars_t radiotimer_vars;
volatile uint32_t TimingDelay=0;
volatile uint32_t govfcount=0;
volatile uint32_t radiotimer_freerunnig=0;
volatile uint32_t ritTimingPeriodOvf=0;
uint32_t lastradiotimer_freerunnig=0;
static uint32_t countStopIrq=0;
void gptimer_init(void);
//=========================== prototypes ======================================
uint32_t macMcuOverflowCapture(void);
void radiotimer_isr_private(void);
port_INLINE uint16_t get_real_counter(void);
void timertickisr(void);
void SysTickSetup(void);

#if 1 //(TESTE_CSMA == 1)
uint32_t ritTimingPeriod=0;
bool ritTimerPeriodEnable=0;
uint32_t ritTimingSched;
uint32_t ritTimingclock=0;
uint32_t systickcount;
uint32_t lastsystickcount;
uint32_t countStopSystick;
bool ritTimerSchedEnable;
#endif
void gptimer_isr_private(void);
//=========================== public ==========================================

//===== admin
#if (USE_GPTIMER == 1)
void gptimer_init(void) {
	 INTERRUPT_DECLARATION();

	 DISABLE_INTERRUPTS();

	TimerDisable(GPTIMER0_BASE,GPTIMER_A);

	//configura o timer do RESET.
	HWREG(GPTIMER_TAILR) = 0x100;

    HWREG(GPTIMER0_BASE + GPTIMER_O_CTL) = 0x00;

	 //!< GPTIMER_CFG ( Configure Timer for 32 bit Timer )
	TimerConfigure(GPTIMER0_BASE, GPTIMER_CFG_PERIODIC );

	//! Timer Control Event TimerControlEvent(uint32_t ui32Base, uint32_t ui32Timer, uint32_t ui32Event)
	// TimerControlEvent(GPTIMER0_BASE,GPTIMER_B,GPTIMER_EVENT_POS_EDGE); //!< EDGES

	  //!< Timer Enable
	  //TimerEnable(GPTIMER0_BASE,GPTIMER_BOTH);

	  //!< Register a handler else systick will be called
	  TimerIntRegister(GPTIMER0_BASE,GPTIMER_BOTH, gptimer_isr_private);

	  //!< Generate an interrupt
	  TimerIntEnable(GPTIMER0_BASE,GPTIMER_TIMA_TIMEOUT);

	  //!< Enable the Timer0A interrupt on the processor (NVIC)
	  IntEnable(INT_TIMER0A);


	  //!< Timer Enable
	  TimerEnable(GPTIMER0_BASE,GPTIMER_BOTH);

      ENABLE_INTERRUPTS();

}
#endif

void radiotimer_init() {
   // clear local variables
   memset(&radiotimer_vars,0,sizeof(radiotimer_vars_t));
}

void radiotimer_setOverflowCb(radiotimer_compare_cbt cb) {
   radiotimer_vars.overflow_cb    = cb;
}

void radiotimer_setCompareCb(radiotimer_compare_cbt cb) {
   radiotimer_vars.compare_cb     = cb;
}

void radiotimer_setStartFrameCb(radiotimer_capture_cbt cb) {
   while(1);
}

void radiotimer_setEndFrameCb(radiotimer_capture_cbt cb) {
   while(1);
}


/*-------------------------------------------------------------------------------
*  Initialize MAC timer.
*/
#if (ENABLE_CSMA_CA == 0)

void radiotimer_start(PORT_RADIOTIMER_WIDTH period) {

    //set period on the timer to 976 tics
    HWREG(RFCORE_SFR_MTMSEL) = (0x02 << RFCORE_SFR_MTMSEL_MTMSEL_S) & RFCORE_SFR_MTMSEL_MTMSEL_M;

    HWREG(RFCORE_SFR_MTM0)=(RADIOTIMER_32MHZ_TICS_PER_32KHZ_TIC << RFCORE_SFR_MTM0_MTM0_S) & RFCORE_SFR_MTM0_MTM0_M;
    HWREG(RFCORE_SFR_MTM1)=((RADIOTIMER_32MHZ_TICS_PER_32KHZ_TIC>> 8) << RFCORE_SFR_MTM1_MTM1_S)& RFCORE_SFR_MTM1_MTM1_M;

    //set counter on the timer to 0 tics
    HWREG(RFCORE_SFR_MTMSEL) = (0x00 << RFCORE_SFR_MTMSEL_MTMSEL_S) & RFCORE_SFR_MTMSEL_MTMSEL_M;

    HWREG(RFCORE_SFR_MTM0)=(0x00<< RFCORE_SFR_MTM0_MTM0_S) & RFCORE_SFR_MTM0_MTM0_M;
    HWREG(RFCORE_SFR_MTM1)=(0x00<< RFCORE_SFR_MTM1_MTM1_S) & RFCORE_SFR_MTM1_MTM1_M;

    //now overflow increments once every 1 32Khz tic.

    //select period register in the selector so it can be modified -- use OVF  so we have 24bit register
	HWREG(RFCORE_SFR_MTMSEL) = (0x02 << RFCORE_SFR_MTMSEL_MTMOVFSEL_S)& RFCORE_SFR_MTMSEL_MTMOVFSEL_M;
	//set the period now -- low 8bits
	HWREG(RFCORE_SFR_MTMOVF0) = (period << RFCORE_SFR_MTMOVF0_MTMOVF0_S)& RFCORE_SFR_MTMOVF0_MTMOVF0_M;
	//set the period now -- middle 8bits
	HWREG(RFCORE_SFR_MTMOVF1) = ((period >> 8) << RFCORE_SFR_MTMOVF1_MTMOVF1_S)& RFCORE_SFR_MTMOVF1_MTMOVF1_M;
	//set the period now -- high 8bits
	HWREG(RFCORE_SFR_MTMOVF2) = ((period >> 16) << RFCORE_SFR_MTMOVF2_MTMOVF2_S)& RFCORE_SFR_MTMOVF2_MTMOVF2_M;

	//select counter register in the selector so it can be modified -- use OVF version so we can have 24bit register
	HWREG(RFCORE_SFR_MTMSEL) = (0x00<< RFCORE_SFR_MTMSEL_MTMOVFSEL_S) & RFCORE_SFR_MTMSEL_MTMOVFSEL_M;
	//set the period now -- low 8bits
	HWREG(RFCORE_SFR_MTMOVF0) = (0x00 << RFCORE_SFR_MTMOVF0_MTMOVF0_S) & RFCORE_SFR_MTMOVF0_MTMOVF0_M;
	//set the period now -- middle 8bits
	HWREG(RFCORE_SFR_MTMOVF1) = (0x00 << RFCORE_SFR_MTMOVF1_MTMOVF1_S) & RFCORE_SFR_MTMOVF1_MTMOVF1_M;
	//set the period now -- high 8bits
	HWREG(RFCORE_SFR_MTMOVF2) = (0x00 << RFCORE_SFR_MTMOVF2_MTMOVF2_S) & RFCORE_SFR_MTMOVF2_MTMOVF2_M;

    //enable period interrupt - ovf
    HWREG(RFCORE_SFR_MTIRQM) = RFCORE_SFR_MTIRQM_MACTIMER_OVF_PERM;//RFCORE_SFR_MTIRQM_MACTIMER_OVF_PERM|RFCORE_SFR_MTIRQM_MACTIMER_PERM

    //register and enable the interrupt at the nvic
    IntRegister(INT_MACTIMR, radiotimer_isr_private);
    //clear all interrupts

    //active sync with 32khz clock and start the timer.
    HWREG(RFCORE_SFR_MTIRQF)=0x00;

    //enable,synch with 32khz and dont latch 3bytes on ovf counter so we have 24bit timer on the ovf.
    HWREG(RFCORE_SFR_MTCTRL)|=RFCORE_SFR_MTCTRL_RUN|RFCORE_SFR_MTCTRL_SYNC;

    while(!( HWREG(RFCORE_SFR_MTCTRL) & RFCORE_SFR_MTCTRL_STATE));//wait until stable.

    IntEnable(INT_MACTIMR);
}

#else   //ENABLE CSMACA == 1

void radiotimer_start(PORT_RADIOTIMER_WIDTH period) {

	/* set timer rollover */
	HWREG(RFCORE_SFR_MTMSEL) = T2M_T2_PER;    //MAC_MCU_T2_ACCESS_PERIOD_VALUE();

	/* TODO!!!! aqui eh a escolha entre o tick do OpenWSN e o TIMAC
	 * OpenWSN  = 976 ticks = 30us. (32Mhz to 32Khz ratio)
	 * TIMAC    = 10240 ticks = 320us */
#if 1
	HWREG(RFCORE_SFR_MTM0) = MAC_RADIO_TIMER_TICKS_PER_BACKOFF() & 0xFFUL;
	HWREG(RFCORE_SFR_MTM1) = MAC_RADIO_TIMER_TICKS_PER_BACKOFF() >> 8UL;
#else
    HWREG(RFCORE_SFR_MTM0) =  (RADIOTIMER_32MHZ_TICS_PER_32KHZ_TIC << RFCORE_SFR_MTM0_MTM0_S) & RFCORE_SFR_MTM0_MTM0_M;
    HWREG(RFCORE_SFR_MTM1) = ((RADIOTIMER_32MHZ_TICS_PER_32KHZ_TIC>> 8) << RFCORE_SFR_MTM1_MTM1_S)& RFCORE_SFR_MTM1_MTM1_M;
#endif


	//TODO!!!! nao sei se esta correto o timer...Atualmente estou gerando uma interrupcao de Overrun a cada 320us...
    //e isso que eh o aUnitBackoffPeriod do CSMA-CA...
    //aparentemente nao preciso de gerar a interrupcao pois eh interno do CSP...

#if 1
    //set counter on the timer to 0 tics
    HWREG(RFCORE_SFR_MTMSEL) = (0x00 << RFCORE_SFR_MTMSEL_MTMSEL_S) & RFCORE_SFR_MTMSEL_MTMSEL_M;

    HWREG(RFCORE_SFR_MTM0)=(0x00<< RFCORE_SFR_MTM0_MTM0_S) & RFCORE_SFR_MTM0_MTM0_M;
    HWREG(RFCORE_SFR_MTM1)=(0x00<< RFCORE_SFR_MTM1_MTM1_S) & RFCORE_SFR_MTM1_MTM1_M;

    //now overflow increments once every 1 32Khz tic.

    //select period register in the selector so it can be modified -- use OVF  so we have 24bit register
	HWREG(RFCORE_SFR_MTMSEL) = (0x02 << RFCORE_SFR_MTMSEL_MTMOVFSEL_S)& RFCORE_SFR_MTMSEL_MTMOVFSEL_M;
	//set the period now -- low 8bits
	HWREG(RFCORE_SFR_MTMOVF0) = (period << RFCORE_SFR_MTMOVF0_MTMOVF0_S)& RFCORE_SFR_MTMOVF0_MTMOVF0_M;
	//set the period now -- middle 8bits
	HWREG(RFCORE_SFR_MTMOVF1) = ((period >> 8) << RFCORE_SFR_MTMOVF1_MTMOVF1_S)& RFCORE_SFR_MTMOVF1_MTMOVF1_M;
	//set the period now -- high 8bits
	HWREG(RFCORE_SFR_MTMOVF2) = ((period >> 16) << RFCORE_SFR_MTMOVF2_MTMOVF2_S)& RFCORE_SFR_MTMOVF2_MTMOVF2_M;

	//select counter register in the selector so it can be modified -- use OVF version so we can have 24bit register
	HWREG(RFCORE_SFR_MTMSEL) = (0x00 << RFCORE_SFR_MTMSEL_MTMOVFSEL_S) & RFCORE_SFR_MTMSEL_MTMOVFSEL_M;
	//set the period now -- low 8bits
	HWREG(RFCORE_SFR_MTMOVF0) = (0x00 << RFCORE_SFR_MTMOVF0_MTMOVF0_S) & RFCORE_SFR_MTMOVF0_MTMOVF0_M;
	//set the period now -- middle 8bits
	HWREG(RFCORE_SFR_MTMOVF1) = (0x00 << RFCORE_SFR_MTMOVF1_MTMOVF1_S) & RFCORE_SFR_MTMOVF1_MTMOVF1_M;
	//set the period now -- high 8bits
	HWREG(RFCORE_SFR_MTMOVF2) = (0x00 << RFCORE_SFR_MTMOVF2_MTMOVF2_S) & RFCORE_SFR_MTMOVF2_MTMOVF2_M;
#endif

#if (TIMERSCHED_USECOMPARE2 == 1)
 	//enable period interrupt - ovf
 	HWREG(RFCORE_SFR_MTIRQM) =  RFCORE_SFR_MTIRQM_MACTIMER_PERM | RFCORE_SFR_MTIRQM_MACTIMER_OVF_PERM | RFCORE_SFR_MTIRQM_MACTIMER_OVF_COMPARE1M | RFCORE_SFR_MTIRQM_MACTIMER_OVF_COMPARE2M;
#else
 	HWREG(RFCORE_SFR_MTIRQM) =  RFCORE_SFR_MTIRQM_MACTIMER_PERM | RFCORE_SFR_MTIRQM_MACTIMER_OVF_PERM | RFCORE_SFR_MTIRQM_MACTIMER_OVF_COMPARE1M ;
#endif

 	/* Enable latch mode and T2 SYNC start.
	* The SYNC start msut be on when POWER_SAVING is on for this design to work.
	*/
	//HWREG(RFCORE_SFR_MTCTRL) |= (LATCH_MODE | TIMER2_SYNC);

	/* start timer */
	// MAC_RADIO_TIMER_WAKE_UP();
	while ( !((HWREG(SYS_CTRL_CLOCK_STA)) & (SYS_CTRL_CLOCK_STA_XOSC_STB))) ;
	HWREG(RFCORE_SFR_MTCTRL) |= (TIMER2_RUN | TIMER2_SYNC);
	while(!(HWREG(RFCORE_SFR_MTCTRL) & TIMER2_STATE));

	IntPrioritySet(INT_MACTIMR, HAL_INT_PRIOR_MAC);

	//register and enable the interrupt at the nvic
	IntRegister(INT_MACTIMR, radiotimer_isr_private);

	//active sync with 32khz clock and start the timer.
    HWREG(RFCORE_SFR_MTIRQF)=0x00;

	/* enable timer interrupts */
	IntEnable(INT_MACTIMR);

}

#endif


//===== direct access

/*
 * Aqui ele pega o valor atual do timer...
 *
 */
PORT_RADIOTIMER_WIDTH radiotimer_getValue(void) {
	 PORT_RADIOTIMER_WIDTH value=0;
	 //INTERRUPT_DECLARATION();

#if (ENABLE_CSMA_CA == 0)
	 //select period register in the selector so it can be read
	 HWREG(RFCORE_SFR_MTMSEL) = (0x00 << RFCORE_SFR_MTMSEL_MTMOVFSEL_S) & RFCORE_SFR_MTMSEL_MTMOVFSEL_M;
	 // compute value by adding m0 and m1 registers
	 value = HWREG(RFCORE_SFR_MTMOVF0);
	 value+=(HWREG(RFCORE_SFR_MTMOVF1)<<8);
     value+=(HWREG(RFCORE_SFR_MTMOVF2)<<16);
#else
	//DISABLE_INTERRUPTS();

	value = radiotimer_getfreerunning();

	//ENABLE_INTERRUPTS();
#endif

	 return value;
}

/*
 * Verifico se a interrupcao do MAC Timer esta travada...nao sei por que ainda trava...
 * olhando pelo freerunning...neste caso....reseto a interrupcao....
 */
uint8_t checkinterrupt(void){

	uint8_t ret=0;

	//verifica MACTIMER se ele travou...
	if (lastradiotimer_freerunnig == radiotimer_freerunnig) {
		countStopIrq++;

		if (countStopIrq > 3) {
			countStopIrq = 0;
			IntDisable(INT_MACTIMR);
			ret = TRUE;
			//considero que travou a interrupcao. Neste caso reseto ela...
			IntPrioritySet(INT_MACTIMR, HAL_INT_PRIOR_MAC);

			//register and enable the interrupt at the nvic
			IntRegister(INT_MACTIMR, radiotimer_isr_private);

			//Clear Pending Interrupts
			HWREG(RFCORE_SFR_MTIRQF)=0x00;

			/* enable timer interrupts */
			IntEnable(INT_MACTIMR);
		}
    }
	else
		countStopIrq = 0;

  lastradiotimer_freerunnig = radiotimer_freerunnig;


  //verifico Systick timer se travou
/*
  if (lastsystickcount == systickcount) {
	countStopSystick++;
	if (countStopSystick > 100) {
		countStopSystick = 0;
		SysTickDisable();
		SysTickPeriodClear();
		//radiotimer_schedule();
	}
  }
  else
	  countStopSystick = 0;

  lastsystickcount = systickcount;
*/


  return ret;
}


void radiotimer_clearValue(void) {
// INTERRUPT_DECLARATION();

// DISABLE_INTERRUPTS();
 radiotimer_freerunnig=0;
// ENABLE_INTERRUPTS();

}
/*
 * Seta periodo do timer...tempo de entrada em ms
 *
 */
void radiotimer_setPeriod_ms(PORT_RADIOTIMER_WIDTH nTime) {
    INTERRUPT_DECLARATION();
	//DISABLE_INTERRUPTS();

#if (ENABLE_CSMA_CA  == 1)
	  ritTimingPeriod = nTime/TICK_IN_MS;
	  ritTimingclock  = 0;
	  ritTimerPeriodEnable = TRUE;
#endif
	//ENABLE_INTERRUPTS();
}



void radiotimer_setPeriod(PORT_RADIOTIMER_WIDTH period) {
#if (ENABLE_CSMA_CA  == 0)
	//select period register in the selector so it can be modified -- use OVF  so we have 24bit register
	HWREG(RFCORE_SFR_MTMSEL) = (0x02 << RFCORE_SFR_MTMSEL_MTMOVFSEL_S)& RFCORE_SFR_MTMSEL_MTMOVFSEL_M;
	//set the period now -- low 8bits
	HWREG(RFCORE_SFR_MTMOVF0) = (period << RFCORE_SFR_MTMOVF0_MTMOVF0_S)& RFCORE_SFR_MTMOVF0_MTMOVF0_M;
	//set the period now -- middle 8bits
	HWREG(RFCORE_SFR_MTMOVF1) = ((period >> 8) << RFCORE_SFR_MTMOVF1_MTMOVF1_S)& RFCORE_SFR_MTMOVF1_MTMOVF1_M;
	//set the period now -- high 8bits
	HWREG(RFCORE_SFR_MTMOVF2) = ((period >> 16) << RFCORE_SFR_MTMOVF2_MTMOVF2_S)& RFCORE_SFR_MTMOVF2_MTMOVF2_M;

    IntEnable(INT_MACTIMR);
#else
	#if (TIMERPERIOD_FREERUNNING == 1)
		ritTimingPeriod      = period/RADIOTIMER_TICS_MS;
		ritTimingPeriodOvf   = radiotimer_freerunnig + ritTimingPeriod+1;  //checar o estouro
		ritTimerPeriodEnable = TRUE;
	#else
		opentimers_start(period, TIMER_ONESHOT, TIME_MS, (opentimers_cbt) radiotimer_vars.overflow_cb);
	#endif
#endif

}

/*
 * obtem o period restante para o estouro do timer
 */
PORT_RADIOTIMER_WIDTH radiotimer_getPeriod() {
	PORT_RADIOTIMER_WIDTH period=0;
#if (ENABLE_CSMA_CA == 0)
	//select period register in the selector so it can be read
	HWREG(RFCORE_SFR_MTMSEL) = (0x02 << RFCORE_SFR_MTMSEL_MTMOVFSEL_S) & RFCORE_SFR_MTMSEL_MTMOVFSEL_M;
	// compute period by adding m0 and m1 registers
	period = HWREG(RFCORE_SFR_MTMOVF0);
	period+=(HWREG(RFCORE_SFR_MTMOVF1)<<8);
	period+=(HWREG(RFCORE_SFR_MTMOVF2)<<16);
#else
	//period = ritTimingPeriod;
	period = (ritTimingPeriodOvf - radiotimer_freerunnig)*RADIOTIMER_TICS_MS;
#endif
    return period;
}


void radiotimer_schedule(PORT_RADIOTIMER_WIDTH offset) {

	uint32_t period;
	uint32_t timeticks;

#if (ENABLE_CSMA_CA  == 0)
	//===== compare
	
//select ovf cmp1 register in the selector so it can be modified
	HWREG(RFCORE_SFR_MTMSEL) = (MTOVF_CMP1 << RFCORE_SFR_MTMSEL_MTMOVFSEL_S) & RFCORE_SFR_MTMSEL_MTMOVFSEL_M;
	//set value
	HWREG(RFCORE_SFR_MTMOVF0) = (offset << RFCORE_SFR_MTMOVF0_MTMOVF0_S)& RFCORE_SFR_MTMOVF0_MTMOVF0_M;
	//set the period now -- middle 8bits
    HWREG(RFCORE_SFR_MTMOVF1) = ((offset >> 8) << RFCORE_SFR_MTMOVF1_MTMOVF1_S) & RFCORE_SFR_MTMOVF1_MTMOVF1_M;
	//set the period now -- high 8bits
	HWREG(RFCORE_SFR_MTMOVF2) = ((offset >> 16) << RFCORE_SFR_MTMOVF2_MTMOVF2_S)& RFCORE_SFR_MTMOVF2_MTMOVF2_M;
    // enable cmp1 ovf interrupt (this also cancels any pending interrupts)
	//clear interrupts
	HWREG(RFCORE_SFR_MTIRQF)=~RFCORE_SFR_MTIRQM_MACTIMER_OVF_COMPARE1M;
	//enable the timer compare interrupt
	HWREG(RFCORE_SFR_MTIRQM)|=RFCORE_SFR_MTIRQM_MACTIMER_OVF_COMPARE1M;
	IntEnable(INT_MACTIMR);
#else

    //pego atual tempo restante da janela de rit e coloco uma margem de seguranca para sempre haver o endslot
	period = radiotimer_getPeriod();

	if (period > offset) {
	  period = offset;
	}

    if (period > 1) {
		timeticks = SYSTICK_MS*period;

		SysTickPeriodSet(timeticks);
		SysTickEnable();
		//SysTickPeriodClear();
		//SysTickIntEnable();
	    //leds_error_on();
	}
    else {
		SysTickPeriodSet(SYSTICK_MS);
		SysTickEnable();
    }


#endif

}

/*
 * Esta funcao reprograma o schedule com um acrescimo do offset
 * porem ele deve estar ainda dentro do periodo do timer...senao ele reprograma para o periodo
 */
void radiotimer_reschedule(PORT_RADIOTIMER_WIDTH offset) {

#if (ENABLE_CSMA_CA  == 0)
	//select ovf cmp1 register in the selector so it can be modified
	HWREG(RFCORE_SFR_MTMSEL) = (MTOVF_CMP1 << RFCORE_SFR_MTMSEL_MTMOVFSEL_S) & RFCORE_SFR_MTMSEL_MTMOVFSEL_M;
	//set value
	HWREG(RFCORE_SFR_MTMOVF0) = (offset << RFCORE_SFR_MTMOVF0_MTMOVF0_S)& RFCORE_SFR_MTMOVF0_MTMOVF0_M;
	//set the period now -- middle 8bits
    HWREG(RFCORE_SFR_MTMOVF1) = ((offset >> 8) << RFCORE_SFR_MTMOVF1_MTMOVF1_S) & RFCORE_SFR_MTMOVF1_MTMOVF1_M;
	//set the period now -- high 8bits
	HWREG(RFCORE_SFR_MTMOVF2) = ((offset >> 16) << RFCORE_SFR_MTMOVF2_MTMOVF2_S)& RFCORE_SFR_MTMOVF2_MTMOVF2_M;
    // enable cmp1 ovf interrupt (this also cancels any pending interrupts)
	//clear interrupts
	HWREG(RFCORE_SFR_MTIRQF)=~RFCORE_SFR_MTIRQM_MACTIMER_OVF_COMPARE1M;
	//enable the timer compare interrupt
	HWREG(RFCORE_SFR_MTIRQM)|=RFCORE_SFR_MTIRQM_MACTIMER_OVF_COMPARE1M;
	IntEnable(INT_MACTIMR);
#else
	//usa o systick como compare
	uint32_t timeticks = offset/TICK_IN_MS;

	ritTimingSched = timeticks + ritTimingclock;
    if (ritTimingSched > ritTimingPeriod)
    	ritTimingSched = ritTimingPeriod;
#endif

}

void radiotimer_cancel() {
#if (ENABLE_CSMA_CA  == 0)
	//select ovf cmp1 register in the selector so it can be modified
	HWREG(RFCORE_SFR_MTMSEL) = (MTOVF_CMP1 << RFCORE_SFR_MTMSEL_MTMOVFSEL_S) & RFCORE_SFR_MTMSEL_MTMOVFSEL_M;
	//set value
	HWREG(RFCORE_SFR_MTMOVF0) = (0x00 << RFCORE_SFR_MTMOVF0_MTMOVF0_S)& RFCORE_SFR_MTMOVF0_MTMOVF0_M;
	//set the period now -- middle 8bits
	HWREG(RFCORE_SFR_MTMOVF1) = (0x00 << RFCORE_SFR_MTMOVF1_MTMOVF1_S) & RFCORE_SFR_MTMOVF1_MTMOVF1_M;
	//set the period now -- high 8bits
	HWREG(RFCORE_SFR_MTMOVF2) = (0x00 << RFCORE_SFR_MTMOVF2_MTMOVF2_S)& RFCORE_SFR_MTMOVF2_MTMOVF2_M;
	// disable cmp1 interrupt
	HWREG(RFCORE_SFR_MTIRQM)&=~RFCORE_SFR_MTIRQM_MACTIMER_OVF_COMPARE1M;
	//clear interrupts
	HWREG(RFCORE_SFR_MTIRQF)=~RFCORE_SFR_MTIRQM_MACTIMER_OVF_COMPARE1M;
#else

	SysTickDisable();
	SysTickPeriodClear();

	//usa o systick como compare
	//ritTimerSchedEnable = 0;
#endif
}

//===== capture

port_INLINE PORT_RADIOTIMER_WIDTH radiotimer_getCapturedTime() {
	 volatile PORT_RADIOTIMER_WIDTH value=0;
#if 1
	 //select period register in the selector so it can be read
	 HWREG(RFCORE_SFR_MTMSEL) = (0x00 << RFCORE_SFR_MTMSEL_MTMOVFSEL_S) & RFCORE_SFR_MTMSEL_MTMOVFSEL_M;
	 // compute value by adding m0 and m1 registers
	 value = HWREG(RFCORE_SFR_MTMOVF0);
	 value+=(HWREG(RFCORE_SFR_MTMOVF1)<<8);
     value+=(HWREG(RFCORE_SFR_MTMOVF2)<<16);
#else
     //value = SysTickValueGet();
 	value = radiotimer_getfreerunning();
#endif
     return value;
}


//=========================== private =========================================

port_INLINE uint16_t get_real_counter(void){
	uint16_t value=0;
	 //select period register in the selector so it can be read
	HWREG(RFCORE_SFR_MTMSEL) = (0x00 << RFCORE_SFR_MTMSEL_MTMSEL_S)& RFCORE_SFR_MTMSEL_MTMSEL_M;
		 // compute value by adding m0 and m1 registers
    value = HWREG(RFCORE_SFR_MTM0) + (HWREG(RFCORE_SFR_MTM1)<<8);
    return value;
}


void timertickisr(void)
{
/*
  if(ritTimingclock >= ritTimingPeriod)
  {
      //sempre programo o timer para RX...se precisar o Tx ele reprograma.
	  ritTimingPeriod = RX_RIT_PERIOD_TICKS;
	  ritTimingclock  = 0;

	  isr_ieee154e_newSlot();
  }
*/
  if(ritTimerSchedEnable){
	  if(ritTimingclock >= ritTimingSched)
	  {
		  ritTimerSchedEnable = 0;
		  isr_ieee154e_timer();
	  }
  }

  //leds_sync_toggle();
  ritTimingclock++;

}



void delay_ms(uint32_t nTime)
{
  TimingDelay = nTime;
  while(TimingDelay != 0);
}

void TimingDelay_Decrement(void)
{
  if(TimingDelay != 0x00)	TimingDelay--;
}


void SysTickIntHandler(void)
{
   if (radiotimer_vars.compare_cb!=NULL) {
	   // call the callback
	   radiotimer_vars.compare_cb();
	 // kick the OS
	   //return KICK_SCHEDULER;
	}

   //leds_error_toggle();
   systickcount++;
	//timertickisr();
	//TimingDelay_Decrement();
}

void RITTimer_Init(void) {
	int8_t i;

	ritTimingPeriod=0;
	ritTimerPeriodEnable = 0;
	ritTimingSched=0;
	ritTimingclock=0;
	systickcount=0;
	lastsystickcount=0;
	countStopSystick=0;
	ritTimerSchedEnable=0;

	SysTickSetup();
	//TICK_MAC_RITMC_PERIOD
	radio_startTimer(10);
	radiotimer_setPeriod(10);

#if (ENABLE_CSMA_CA == 1)
	radio_csma_vars.capturedTime0 = 0;
	radio_csma_vars.capturedTime1 = 0;
	radio_csma_vars.deltaMax = 0;
	radio_csma_vars.deltapos = 0;
	for (i=0;i<MAX_DELTA;i++)
		radio_csma_vars.delta[i] = 0;
#endif
}

#if (ENABLE_CSMA_CA == 1)

/**************************************************************************************************
 * @fn          macMcuOverflowCapture
 *
 * @brief       Returns the last capture of the overflow counter.  A special hardware feature
 *              captures the overflow counter when the regular hardware timer is captured.
 *
 * @param       none
 *
 * @return      last capture of overflow count
 **************************************************************************************************
 */
uint32_t macMcuOverflowCapture(void)
{
  uint32_t  overflowCapture;
  uint32_t  period;
  uint16_t  realcounter;

  //period = radiotimer_getPeriod();
  //overflowCapture = radiotimer_getCapturedTime();
  //realcounter = get_real_counter();

  period *= govfcount;

  //leds_debug_toggle();
/*

  //HWREG(RFCORE_SFR_MTCTRL) &= ~LATCH_MODE;
  HWREG(RFCORE_SFR_MTMSEL) = T2M_T2OVF_CAP;

  ((uint8_t *)&overflowCapture)[0] = T2MOVF0;
  ((uint8_t *)&overflowCapture)[1] = T2MOVF1;
  ((uint8_t *)&overflowCapture)[2] = T2MOVF2;
  ((uint8_t *)&overflowCapture)[3] = 0;
  //HAL_EXIT_CRITICAL_SECTION(s);
*/
  return (period);
}
#endif

//=========================== interrupt handlers ==============================
/*
 * Retorna a um contador de ticks do RadioMACTimer que eh multipo de 320us
 *
 */
uint32_t radiotimer_getfreerunning(void)
{
	return (radiotimer_freerunnig);
}


#if ( ENABLE_CSMA_CA == 0)
void radiotimer_isr_private(){
	debugpins_isr_set();
	radiotimer_isr();
	debugpins_isr_clr();
}

/*
 * The interrupt flags are given in the RFCORE_SFR_MTIRQF registers. The interrupt flag bits are set only
by hardware and are cleared only by writing to the SFR register.
Each interrupt source can be masked by its corresponding mask bit in the RFCORE_SFR_MTIRQM
register. An interrupt is generated when the corresponding mask bit is set; otherwise, the interrupt is not
generated. The interrupt flag bit is set, however, regardless of the state of the interrupt mask bit.
 *
 */
kick_scheduler_t radiotimer_isr() {

   uint8_t t2irqm;
   uint8_t t2irqf;

   t2irqm = HWREG(RFCORE_SFR_MTIRQM);
   t2irqf = HWREG(RFCORE_SFR_MTIRQF);

   IntPendClear(INT_MACTIMR);

   if ((t2irqf & RFCORE_SFR_MTIRQM_MACTIMER_OVF_COMPARE1M)& t2irqm){ // compare 1
	   debugpins_isr_toggle();
  	   //clear interrupt
  	   HWREG(RFCORE_SFR_MTIRQF)=~RFCORE_SFR_MTIRQM_MACTIMER_OVF_COMPARE1M;

  	   if (radiotimer_vars.compare_cb!=NULL) {
              // call the callback
              radiotimer_vars.compare_cb();
            // kick the OS
              return KICK_SCHEDULER;
           }
     } else if((t2irqf & RFCORE_SFR_MTIRQM_MACTIMER_OVF_PERM) & t2irqm){ // timer overflows
    	 debugpins_isr_toggle();
	     //clear interrupt
	     HWREG(RFCORE_SFR_MTIRQF)=~RFCORE_SFR_MTIRQM_MACTIMER_OVF_PERM;

	     if (radiotimer_vars.overflow_cb!=NULL) {
               // call the callback
             radiotimer_vars.overflow_cb();
               // kick the OS
             return KICK_SCHEDULER;
          }
   }
   return DO_NOT_KICK_SCHEDULER;
}
#else
/**************************************************************************************************
 * @fn          macMcuTimer2Isr
 *
 * @brief       Interrupt service routine for timer2, the MAC timer.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void radiotimer_isr_private(void)
{
  volatile uint32_t capturedTime;
  uint8_t t2irqm;
  uint8_t t2irqf;
  t2irqm = HWREG(RFCORE_SFR_MTIRQM);
  t2irqf = HWREG(RFCORE_SFR_MTIRQF);

  IntPendClear(INT_MACTIMR);

  /*------------------------------------------------------------------------------------------------
   *  Overflow compare interrupt - triggers when then overflow counter is
   *  equal to the overflow compare register.
   */
  if ((t2irqf & RFCORE_SFR_MTIRQM_MACTIMER_OVF_COMPARE2M)& t2irqm){ // compare 2
	/* clear overflow compare interrupt flag */
	HWREG(RFCORE_SFR_MTIRQF)=~RFCORE_SFR_MTIRQM_MACTIMER_OVF_COMPARE2M;

	radiotimer_vars.compare_cb();

  }

  /*------------------------------------------------------------------------------------------------
   *  Overflow compare interrupt - triggers when then overflow counter is
   *  equal to the overflow compare register.
   */
  else if ((t2irqf & TIMER2_OVF_PERF) & t2irqm)
  {
    /* call function for dealing with the timer compare interrupt */
    //macBackoffTimerPeriodIsr();
     //capturedTime = (uint32_t) macMcuOverflowCapture();

    /* clear overflow compare interrupt flag */
    T2IRQF = ~TIMER2_OVF_PERF;
    //timertickisr();
  }

  /*------------------------------------------------------------------------------------------------
   *  Overflow Timer Period interrupt - triggers when then overflow Timer period counter is
   *  equal to the overflow compare register.
   *  esta interrupcao somente eh gerada para teste...este eh o tempo de UnitBackoffperiod
   */
  else if ((t2irqf & TIMER2_PERF) & t2irqm)
  {
    /* call function for dealing with the timer compare interrupt */
    //macBackoffTimerPeriodIsr();
    radiotimer_freerunnig++;
    /* clear overflow compare interrupt flag */
    T2IRQF = ~TIMER2_PERF;
    //timertickisr();
#if (TIMERPERIOD_FREERUNNING == 1)
    //checa timer...
    if (ritTimerPeriodEnable){
        if (radiotimer_freerunnig > ritTimingPeriodOvf){
   	     if (radiotimer_vars.overflow_cb!=NULL) {
                radiotimer_vars.overflow_cb();
          }
        }
    }
#endif
  }

  //return DO_NOT_KICK_SCHEDULER;
  //CLEAR_SLEEP_MODE();
}

#endif
#if (USE_GPTIMER == 1)
void gptimer_isr_private(void)
{
	TimerIntClear(GPTIMER0_BASE,GPTIMER_TIMA_TIMEOUT);
}

#endif
