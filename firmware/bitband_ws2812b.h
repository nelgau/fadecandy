
#ifndef BITBAND_WS2812B_H_
#define BITBAND_WS2812B_H_

#include "WProgram.h"

extern "C" void drawChunk(uint8_t *pChunk, uint32_t startIndex, uint32_t length);

void bitband_init(void);
void bitband_show(void);
bool bitband_busy(void);

#endif
