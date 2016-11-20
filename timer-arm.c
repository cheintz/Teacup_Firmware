
/** \file
  \brief Timer management, ARM specific part.

  To be included from timer.c.
*/

#if defined TEACUP_C_INCLUDE && defined __ARMEL__

#include "cmsis-core_cm0.h"
#include "clock.h"
#include "pinio.h"
#include "dda_queue.h"


/** Timer initialisation.

  Initialise timer and enable system clock interrupt. Step interrupt is
  enabled later, when we start using it.

  For the system clock, we use SysTickTimer. This timer is made for exactly
  such purposes.

  Other than AVRs, Cortex-M doesn't allow reentry of interupts. To deal with
  this we use another interrupt, PendSV, with slightly lower priority and
  without an interrupt source. The possibly lengthy operation in dda_clock()
  goes into there and at the end of the SysTick interrupt we simply set PendSV
  pending. This way PendSV is executed right after SysTick or, if PendSV is
  already running, ignored.
*/
void timer_init() {

  /**
    Initialise the system tick timer.

    We enable the system tick timer with interrupts. A similar function is
    SysTick_Config(uint32_t ticks) in cmsis-core_cm0.h

    Register name mapping from LPC111x User Manual to CMSIS headers:

      chap. 24.5     cmsis-core_cm0.h  description

      SYST_CSR       SysTick->CTRL     System Timer Control and status register.
      SYST_RVR       SysTick->LOAD     System Timer Reload value register.
      SYST_CVR       SysTick->VAL      System Timer Current value register.
      SYST_CALIB     SysTick->CALIB    System Timer Calibration value register.
  */
  NVIC_SetPriority(SysTick_IRQn, 0);              // Highest priority.

  // SysTick defined in cmsis-core_cm0.h.
  SysTick->LOAD  = TICK_TIME - 1;
  SysTick->VAL   = 0;                             // Actually load LOAD.

  SysTick->CTRL  = SysTick_CTRL_ENABLE_Msk        // Enable the ticker.
                 | SysTick_CTRL_TICKINT_Msk       // Enable interrupt.
                 | SysTick_CTRL_CLKSOURCE_Msk;    // Run at full CPU clock.

  /**
    Initialise PendSV for dda_clock().
  */
  NVIC_SetPriority(PendSV_IRQn, 1);               // Almost highest priority.

  /**
    Initialise the stepper timer. On ARM we have the comfort of hardware
    32-bit timers, so we don't have to artifically extend the timer to this
    size. We use match register 0 of the first 32-bit timer, CT32B0.

    We run the timer all the time, just turn the interrupt on and off, to
    allow interrupts equally spaced, independent from processing time. See
    also description of timer_set().
  */
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 9);          // Turn on CT32B0 power.

  LPC_TMR32B0->TCR    = (1 << 0);                 // Enable counter.
  //LPC_TMR32B0->PR   = 0; ( = reset value)       // Prescaler off.
  //LPC_TMR32B0->MCR is handled by timer_set() and IRQ handler.
  //LPC_TMR32B0->CCR  = 0; ( = reset value)       // No pin capture.
  //LPC_TMR32B0->EMR  = 0; ( = reset value)       // No external matches.
  //LPC_TMR32B0->CTCR = 0; ( = reset value)       // Timer mode.
  //LPC_TMR32B0->PWMC = 0; ( = reset value)       // All PWM off.

  NVIC_SetPriority(TIMER_32_0_IRQn, 0);           // Also highest priority.
  NVIC_EnableIRQ(TIMER_32_0_IRQn);                // Enable interrupt generally.
}

/** System clock interrupt.

  Happens every TICK_TIME. Must have the same name as in
  cmsis-startup_lpc11xx.s
*/
void SysTick_Handler(void) {

  clock_tick();

  SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;             // Trigger PendSV_Handler().
}

/** System clock interrupt, slow part.

  Here we do potentially lengthy calculations. Must have the same name as in
  cmsis-startup_lpc11xx.s
*/
void PendSV_Handler(void) {
  dda_clock();
}

/** Step interrupt.

  Happens for every stepper motor step. Must have the same name as in
  cmsis-startup_lpc11xx.s
*/
void TIMER32_0_IRQHandler(void) {

  #ifdef DEBUG_LED_PIN
    WRITE(DEBUG_LED_PIN, 1);
  #endif

  /**
    Turn off interrupt generation, timer counter continues. As this interrupt
    is the only use of this timer, there's no need for a read-modify-write.
  */
  LPC_TMR32B0->MCR = 0;

  /**
    We have to "reset" this interrupt, else it'll be triggered over and over
    again.
  */
  LPC_TMR32B0->IR = (1 << 0);                     // Clear match on channel 0.

  queue_step();

  #ifdef DEBUG_LED_PIN
    WRITE(DEBUG_LED_PIN, 0);
  #endif
}

/** Timer reset.

  Reset the timer, so step interrupts scheduled at an arbitrary point in time
  don't lead to a full round through the timer counter.

  On ARM we actually do something, such a full round through the timer is
  2^32 / F_CPU = 44 to 90 seconds.
*/
void timer_reset() {
  LPC_TMR32B0->TC = 0;
  LPC_TMR32B0->MR0 = 0;
}

/** Stop timers.

  This means to be an emergency stop.
*/
void timer_stop() {
  SysTick->CTRL = 0;
}

#endif /* defined TEACUP_C_INCLUDE && defined __ARMEL__ */
