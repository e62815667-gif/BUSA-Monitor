#pragma once
// V15 board contract: verified pin allocation and assertions.
// DS18B20 T1-T4: four separate direct 1-Wire buses, each with its own 4.7k pull-up to 3.3V.
// Old GPIO38-41 are no longer used for temperature sensors.
constexpr uint8_t TPIN[4]={16,17,18,21};
constexpr uint8_t KPIN[4]={7,6,5,4}; // Reference only: never configured as outputs
constexpr uint8_t BUS_SDA=8,BUS_SCL=9;
constexpr uint8_t TFT_CS=10,TFT_MOSI=11,TFT_SCK=12,TFT_DC=13,TFT_RST=14;
constexpr uint8_t FLOW_GPIO=15; // INPUT ONLY, signal level must be <=3.3 V
constexpr bool RELAY_PHYSICAL_OUTPUTS=false,LED_PHYSICAL_OUTPUTS=false;
static_assert(!RELAY_PHYSICAL_OUTPUTS && !LED_PHYSICAL_OUTPUTS,"Physical outputs must stay locked");
static_assert(FLOW_GPIO==15 && TPIN[0]==16 && TPIN[1]==17 && TPIN[2]==18 && TPIN[3]==21 && BUS_SDA==8 && BUS_SCL==9,"Temperature GPIO mapping changed unexpectedly");
