#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>

#define DELAY_TICK_FREQUENCY_US 1000000   /* = 1MHZ -> microseconds delay */
#define DELAY_TICK_FREQUENCY_MS 1000      /* = 1kHZ -> milliseconds delay */

void delay_10ms(unsigned int);
void delay_us(unsigned int);

/*
 *   Declare Functions
 */
extern void Delay_ms(uint16_t nTime);
extern void Delay_us(uint16_t nTime);

#endif
