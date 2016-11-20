
/** \file
  \brief Timer management, AVR and simulator specific part.

  To be included from timer.c.

	Teacup uses timer1 to generate both step pulse clock and system clock.

	We achieve this by using the output compare registers to generate the two clocks while the timer free-runs.

	Teacup has tried numerous timer management methods, and this is the best so far.
*/

#if defined TEACUP_C_INCLUDE && (defined __AVR__ || defined SIMULATOR)

#include	"config_wrapper.h"
#include "pinio.h"
#include "clock.h"
#include "cpu.h"
#include "memory_barrier.h"

#ifdef	MOTHERBOARD
#include	"dda_queue.h"
#endif


/**
  Time until next step, as output compare register is too small for long
  step times.
*/
uint32_t	next_step_time;

#ifdef ACCELERATION_TEMPORAL
/// Unwanted extra delays, ideally always zero.
uint32_t	step_extra_time = 0;
#endif /* ACCELERATION_TEMPORAL */


/** System clock interrupt.

  Comparator B is the system clock, happens every TICK_TIME.
*/
ISR(TIMER1_COMPB_vect) {
  static volatile uint8_t busy = 0;

	// set output compare register to the next clock tick
	OCR1B = (OCR1B + TICK_TIME) & 0xFFFF;

  clock_tick();

  /**
    Lengthy calculations ahead! Make sure we didn't re-enter, then allow
    nested interrupts.
  */
  if ( ! busy) {
    busy = 1;
    sei();

    dda_clock();

    busy = 0;
  }
}

#ifdef	MOTHERBOARD

/** Step interrupt.

  Comparator A is the step timer. It has higher priority then B.
*/
ISR(TIMER1_COMPA_vect) {
	// Check if this is a real step, or just a next_step_time "overflow"
	if (next_step_time < 65536) {
		// step!
		#ifdef DEBUG_LED_PIN
			WRITE(DEBUG_LED_PIN, 1);
		#endif

		// disable this interrupt. if we set a new timeout, it will be re-enabled when appropriate
		TIMSK1 &= ~MASK(OCIE1A);

		// stepper tick
		queue_step();

		// led off
		#ifdef DEBUG_LED_PIN
			WRITE(DEBUG_LED_PIN, 0);
		#endif

		return;
	}

	next_step_time -= 65536;

  // Similar algorithm as described in timer_set() below.
	if (next_step_time < 65536) {
		OCR1A = (OCR1A + next_step_time) & 0xFFFF;
	} else if(next_step_time < 75536){
		OCR1A = (OCR1A - 10000) & 0xFFFF;
		next_step_time += 10000;
	}
	// leave OCR1A as it was
}
#endif /* ifdef MOTHERBOARD */

/** Timer initialisation.

  Initialise timer and enable system clock interrupt. Step interrupt is
  enabled later, when we start using it.
*/
void timer_init() {
	// no outputs
	TCCR1A = 0;
	// Normal Mode
	TCCR1B = MASK(CS10);
	// set up "clock" comparator for first tick
	OCR1B = TICK_TIME & 0xFFFF;
	// enable interrupt
	TIMSK1 = MASK(OCIE1B);
#ifdef SIMULATOR
  // Tell simulator
  sim_timer_set();
#endif
}

#ifdef	MOTHERBOARD
/** Timer reset.

  Reset the timer, so step interrupts scheduled at an arbitrary point in time
  don't lead to a full round through the timer counter.

  On AVR we simply do nothing, such a full round through the timer is just
  2^16 / F_CPU = 3 to 4 milliseconds.
*/
void timer_reset() {
}

/** Stop timers.

  This means to be an emergency stop.
*/
void timer_stop() {
	// disable all interrupts
	TIMSK1 = 0;
  #ifdef SIMULATOR
    // Tell simulator
    sim_timer_stop();
  #endif
}
#endif /* ifdef MOTHERBOARD */

#endif /* defined TEACUP_C_INCLUDE && (defined __AVR__ || defined SIMULATOR) */
