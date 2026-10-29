#pragma once
// BUSA 1M V15.2 / ESP32-S3 N16R8 / Flash 16 MB / OPI PSRAM 8 MB.
// Board pin map is a wiring contract, not proof of a physical wiring test.
// DS18B20 T1-T4: four separate direct 1-Wire buses, each with its own 4.7k pull-up to 3.3V.
// Old GPIO38-41 are no longer used for temperature sensors.
constexpr uint8_t TPIN[4]={16,17,18,21};
constexpr uint8_t KPIN[4]={7,6,5,4}; // Reference only: never configured as outputs
constexpr uint8_t BUS_SDA=8,BUS_SCL=9;
constexpr uint8_t TFT_CS=10,TFT_DC=11,TFT_RST=12,TFT_MOSI=13,TFT_SCK=14; // New owner-approved 7-pin TFT connector order
constexpr uint8_t FLOW_GPIO=15; // INPUT ONLY, signal level must be <=3.3 V
constexpr bool RELAY_PHYSICAL_OUTPUTS=true,LED_PHYSICAL_OUTPUTS=false;
static_assert(RELAY_PHYSICAL_OUTPUTS && !LED_PHYSICAL_OUTPUTS,"Four sound-annunciator outputs only, LED power stage stays off");
static_assert(FLOW_GPIO==15 && TPIN[0]==16 && TPIN[1]==17 && TPIN[2]==18 && TPIN[3]==21 && BUS_SDA==8 && BUS_SCL==9,"Temperature GPIO mapping changed unexpectedly");

static_assert(TFT_CS==10 && TFT_DC==11 && TFT_RST==12 && TFT_MOSI==13 && TFT_SCK==14,"TFT 7-pin wiring differs from approved mapping");
