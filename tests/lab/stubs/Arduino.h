#pragma once
#include <stdint.h>
#include <stddef.h>
extern uint32_t fakeMillis;
inline uint32_t millis(){return fakeMillis;}
inline void delay(uint32_t n){fakeMillis+=n;}
