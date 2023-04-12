
/** @file		main.c
 * 	@brief		Run this routine once for each new WISP. Generates and stores
 * 				a table of random values to use in quickly producing RN16, and
 * 				also generates a unique ID for each WISP.
 *
 * 	@author		Aaron Parks, Justin Reina, Sensor Systems Lab, University of Washington
 */

#include "wisp-base.h"


/**

 */



 /*
  * Pulse LED at a given rate for a given number of times
  * @param count number of times to pulse
  * @param delay btwn pulses and after last pulse
 */
void ledBlinks (uint8_t count, uint16_t duration) {

	while(count--) {
		// Stay on for ~1ms, then wait for specified duration
		BITSET(PLEDOUT,PIN_LED);
		Timer_LooseDelay(32);
		BITCLR(PLEDOUT,PIN_LED);
		Timer_LooseDelay(duration);
	}
	return;
}

/**
 * Generates and stores a table of random numbers into Info Mem segment B/C.
 *  These are used by the WISP as a unique ID and for RN16 generation in Aloha protocol.
 */
void main (void) {

  WISP_init();



  // Blink
  while(FOREVER) {
      ledBlinks(1,200);
  }

}


