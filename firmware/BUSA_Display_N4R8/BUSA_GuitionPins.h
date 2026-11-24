#pragma once
#include <stdint.h>
namespace busa_guition {
// Guition-JC/Guition, components/board/guition_boards/jc4827w543/
// board_devices.yaml and board_peripherals.yaml; resistive touch variant.
// Exact initialization remains sourced from the original device image.
constexpr int LCD_CS=45, LCD_SCK=47, LCD_D0=21, LCD_D1=48, LCD_D2=40, LCD_D3=39;
constexpr int BACKLIGHT=1;
constexpr int SHARED_SPI_SCK=12, SHARED_SPI_MOSI=11, SHARED_SPI_MISO=13;
constexpr int TOUCH_CS=38, SD_CS=10;
constexpr uint32_t LCD_CLOCK_HZ=30000000;
// XPT2046_Touchscreen v1.4 owns its fixed 2 MHz SPI transaction setting.
static_assert(TOUCH_CS!=SD_CS,"SPI chip-select collision");
}
