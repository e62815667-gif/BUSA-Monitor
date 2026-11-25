#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdlib>
#include <chrono>
using PGM_P=const char*;
inline uint32_t millis(){return uint32_t(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());}
