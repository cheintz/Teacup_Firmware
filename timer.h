#ifndef	_TIMER_H
#define	_TIMER_H

#include	<stdint.h>
#include "arduino.h"  // For F_CPU on ARM.

// time-related constants
#define	US	* (F_CPU / 1000000)
#define	MS	* (F_CPU / 1000)

/// How often we overflow and update our clock.
/// With F_CPU = 16MHz, max is < 4.096ms (TICK_TIME = 65535).
#define TICK_TIME (2 MS)

/// Convert back to ms from cpu ticks so our system clock runs
/// properly if you change TICK_TIME.
#define TICK_TIME_MS (TICK_TIME / (F_CPU / 1000))


void timer_init(void);

static uint8_t timer_set(int32_t, uint8_t) __attribute__((always_inline));
#if defined __AVR__ || SIMULATOR
#include "pinio.h"
#include  "memory_barrier.h"
extern uint32_t  next_step_time;
#ifdef  MOTHERBOARD
/** Specify how long until the step timer should fire.

  \param delay Delay for the next step interrupt, in CPU ticks.

  \param check_short Tell whether to check for impossibly short requests. This
         should be set to 1 for calls from the step interrupt. Short requests
         then return 1 and do not schedule a timer interrupt. The calling code
         usually wants to handle this case.

         Calls from elsewhere should set it to 0. In this case a timer
         interrupt is always scheduled.

  \return A flag whether the requested time was too short to allow scheduling
          an interrupt. This is meaningful for ACCELERATION_TEMPORAL, where
          requested delays can be zero or even negative. In this case, the
          calling code should repeat the stepping code immediately and also
          assume the timer to not change his idea of when the last step
          happened.

  Strategy of this timer is to schedule timer interrupts not starting at the
  time of the call, but starting at the time of the previous timer interrupt
  fired. This ignores the processing time taken in the step interrupt so far,
  offering smooth and even step distribution. Flipside of this coin is,
  one has to call timer_reset() before scheduling a step at an arbitrary time.

  This enables the step interrupt, but also disables interrupts globally.
  So, if you use it from inside the step interrupt, make sure to do so
  as late as possible. If you use it from outside the step interrupt,
  do a sei() after it to make the interrupt actually fire.
*/
inline uint8_t timer_set(int32_t delay, uint8_t check_short) {
  uint16_t step_start = 0;
  #ifdef ACCELERATION_TEMPORAL
  uint16_t current_time;
  #endif /* ACCELERATION_TEMPORAL */

  // An interrupt would make all our timing calculations invalid,
  // so stop that here.
  cli();
  CLI_SEI_BUG_MEMORY_BARRIER();

  // Assume all steps belong to one move. Within one move the delay is
  // from one step to the next one, which should be more or less the same
  // as from one step interrupt to the next one. The last step interrupt happend
  // at OCR1A, so start delay from there.
  step_start = OCR1A;
  next_step_time = delay;

  #ifdef ACCELERATION_TEMPORAL
    if (check_short) {
      current_time = TCNT1;

      // 200 = safe number of cpu cycles after current_time to allow a new
      // interrupt happening. This is mostly the time needed to complete the
      // current interrupt.
      if ((current_time - step_start) + 200 > delay)
        return 1;
    }
  #endif /* ACCELERATION_TEMPORAL */

  // From here on we assume the requested delay is long enough to allow
  // completion of the current interrupt before the next one is about to
  // happen.

  // Now we know how long we actually want to delay, so set the timer.
  if (next_step_time < 65536) {
    // set the comparator directly to the next real step
    OCR1A = (next_step_time + step_start) & 0xFFFF;
  }
  else if (next_step_time < 75536) {
    // Next comparator interrupt would have to trigger another
    // interrupt within a short time (possibly within 1 cycle).
    // Avoid the impossible by firing the interrupt earlier.
    OCR1A = (step_start - 10000) & 0xFFFF;
    next_step_time += 10000;
  }
  else {
    OCR1A = step_start;
  }

  // Enable this interrupt, but only do it after disabling
  // global interrupts (see above). This will cause push any possible
  // timer1a interrupt to the far side of the return, protecting the
  // stack from recursively clobbering memory.
  TIMSK1 |= MASK(OCIE1A);
  #ifdef SIMULATOR
    // Tell simulator
    sim_timer_set();
  #endif

  return 0;
}
#endif
#elif defined __ARMEL__
/** Specify how long until the step timer should fire.

  \param delay Delay for the next step interrupt, in CPU ticks.

  \param check_short Tell whether to check for impossibly short requests. This
         should be set to 1 for calls from the step interrupt. Short requests
         then return 1 and do not schedule a timer interrupt. The calling code
         usually wants to handle this case.

         Calls from elsewhere should set it to 0. In this case a timer
         interrupt is always scheduled.

  \return A flag whether the requested time was too short to allow scheduling
          an interrupt. This is meaningful for ACCELERATION_TEMPORAL, where
          requested delays can be zero or even negative. In this case, the
          calling code should repeat the stepping code immediately and also
          assume the timer to not change his idea of when the last step
          happened.

  Strategy of this timer is to schedule timer interrupts not starting at the
  time of the call, but starting at the time of the previous timer interrupt
  fired. This ignores the processing time taken in the step interrupt so far,
  offering smooth and even step distribution. Flipside of this coin is,
  schedules issued at an arbitrary time can result in drastically wrong delays.
  See also discussion of parameter check_short and the return value.

  This enables the step interrupt, but also disables interrupts globally.
  So, if you use it from inside the step interrupt, make sure to do so
  as late as possible. If you use it from outside the step interrupt,
  do a sei() after it to make the interrupt actually fire.

  On ARM we have the comfort of hardware 32-bit timers, so we don't have to
  artifically extend the timer to this size. We use match register 0 of the
  first 32-bit timer, CT32B0.
*/
inline uint8_t timer_set(int32_t delay, uint8_t check_short) {

  #ifdef ACCELERATION_TEMPORAL
    if (check_short) {
      /**
        100 = safe number of cpu cycles after current_time to allow a new
        interrupt happening. This is mostly the time needed to complete the
        current interrupt.

        LPC_TMR32B0->TC  = timer counter = current time.
        LPC_TMR32B0->MR0 = last timer match = time of the last step.
      */
      if ((LPC_TMR32B0->TC - LPC_TMR32B0->MR0) + 100 > delay)
        return 1;
    }
  #endif /* ACCELERATION_TEMPORAL */

  /**
    Still here? Then we can schedule the next step. Off of the previous step.
    If there is no previous step, TC and MR0 should have been reset to zero
    by calling timer_reset() shortly before we arrive here.
  */
  LPC_TMR32B0->MR0 += delay;

  /**
    Turn on the stepper interrupt. As this interrupt is the only use of this
    timer, there's no need for a read-modify-write.
  */
  LPC_TMR32B0->MCR = (1 << 0);                    // Interrupt on MR0 match.

  return 0;
}
#endif

void timer_reset(void);

void timer_stop(void);

#endif	/* _TIMER_H */
