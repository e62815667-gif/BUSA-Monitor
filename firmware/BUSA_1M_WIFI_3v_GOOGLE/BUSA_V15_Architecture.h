#pragma once
// V15 architecture boundary. Legacy physical outputs remain disconnected.
// New channel driver requires independent safe-hardware validation.
namespace BusaArchitectureV15 {
constexpr uint8_t kTemperatureChannels=4;
constexpr uint8_t kLogicalRelayChannels=4;
constexpr bool kPhysicalRelayDriverImplemented=false;
static_assert(kTemperatureChannels==4 && kLogicalRelayChannels==4,"V15 channels");
}
String v15ArchitectureJSON(){
  String s; s.reserve(340);
  s="{\"revision\":\"V15.0\",\"platform\":\"ESP32-S3-N8R2\",";
  s+="\"modules\":[\"board\",\"sensors\",\"wifi\",\"google\",\"ui\",\"relay_model\",\"runtime\"],";
  s+="\"physical_relay_driver_implemented\":false,";
  s+="\"mains_and_valve_outside_project\":true}";
  return s;
}
