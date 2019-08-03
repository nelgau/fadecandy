
#ifndef BITBAND_WS2812B_H_
#define BITBAND_WS2812B_H_

#include "WProgram.h"

extern "C" void getPixel(uint32_t strip, uint32_t index, uint8_t *out);

void bitband_init(void);
void bitband_show(void);
bool bitband_busy(void);

#endif
