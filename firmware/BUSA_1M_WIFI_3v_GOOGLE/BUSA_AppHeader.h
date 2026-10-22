#pragma once
#include <stdint.h>
#include <stddef.h>
// ESP image header (24), segment header (8), esp_app_desc magic (4).
inline bool busaAppHeaderValid(const uint8_t* p,size_t n){
  return p && n>=36 && p[0]==0xe9 && p[1]>=1 && p[1]<=16 &&
    p[12]==9 && p[13]==0 && p[32]==0x32 && p[33]==0x54 &&
    p[34]==0xcd && p[35]==0xab;
}
