#include "BUSA_SoundAlarmLogic.h"
#include <cstdlib>
#include <iostream>

using BusaSound::Routed;
using BusaSound::evaluateRouted;

void require(bool ok, const char* message) {
  if (!ok) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}
void sample(Routed& route, const bool enabled[4], float t3, float t4, uint32_t now) {
  evaluateRouted(route, enabled, true, t3, 80.0f, 75.0f,
                 true, t4, 80.0f, 75.0f, now, 90000UL);
}

void test_t4_on_while_k3_running() {
  Routed route;
  const bool enabled[4] = {true, true, true, true};
  sample(route, enabled, 70, 70, 100);
  sample(route, enabled, 81, 70, 200);
  require(route.k[0] && route.k[1] && !route.k[2], "T3 ON starts K1 and K2");
  sample(route, enabled, 74, 70, 300);
  require(!route.k[0] && route.k[2], "T3 OFF silences K1 and starts K3");
  sample(route, enabled, 74, 81, 400);
  require(route.t4OnEvent && route.k[1] && route.k[2] && route.k[3],
          "T4 ON starts K2 while K3 runs; K4 remains on");
  sample(route, enabled, 74, 81, 500);
  require(route.k[1] && route.k2.start_ms == 400, "K2 does not retrigger without an event");
  sample(route, enabled, 74, 81, 90400);
  require(!route.k[1] && !route.k[2] && route.k[3], "both pulses expire after 90 seconds");
}

void test_t3_on_while_k3_running() {
  Routed route;
  const bool enabled[4] = {true, true, true, true};
  sample(route, enabled, 70, 70, 100);
  sample(route, enabled, 81, 70, 200);
  sample(route, enabled, 74, 70, 300);
  require(route.k[2], "T3 OFF starts K3");
  sample(route, enabled, 81, 70, 400);
  require(route.t3OnEvent && route.k[1] && route.k[0] && route.k[2],
          "T3 ON starts K2 and K1 without cancelling K3");
}

void test_k2_not_cut_short_on_off_event() {
  Routed route;
  const bool enabled[4] = {true, true, true, true};
  sample(route, enabled, 70, 70, 100);
  sample(route, enabled, 81, 70, 200);
  require(route.k[1], "K2 starts at T3 ON");
  sample(route, enabled, 74, 70, 300);
  require(route.t3OffEvent && route.k[2] && route.k[1],
          "T3 OFF starts K3 but does not stop the K2 90-second pulse");
  sample(route, enabled, 74, 70, 90199);
  require(route.k[1] && route.k[2], "K2 is on just before its 90-second limit");
  sample(route, enabled, 74, 70, 90200);
  require(!route.k[1] && route.k[2], "K2 expires, independently of K3");
  sample(route, enabled, 74, 70, 90300);
  require(!route.k[1] && !route.k[2], "K3 expires after its own 90 seconds");
}

void test_simultaneous_t3_on_t4_off() {
  Routed route;
  const bool enabled[4] = {true, true, true, true};
  sample(route, enabled, 70, 70, 100);
  sample(route, enabled, 70, 81, 200);
  require(route.k[3] && route.k[1], "T4 ON starts K4 and K2");
  sample(route, enabled, 70, 81, 90300);
  require(!route.k[1] && route.k[3], "initial K2 expired, K4 remains active");
  sample(route, enabled, 81, 74, 90500);
  require(route.t3OnEvent && route.t4OffEvent, "both events detected in one poll");
  require(route.k[0] && !route.k[3] && route.k[1] && route.k[2],
          "simultaneous T3 ON and T4 OFF independently start K2 and K3");
}

void test_t3_off_still_silences_k1() {
  Routed route;
  const bool enabled[4] = {true, true, true, true};
  sample(route, enabled, 70, 70, 100);
  sample(route, enabled, 81, 70, 200);
  require(route.k[0], "K1 starts on T3 ON");
  sample(route, enabled, 74, 70, 300);
  require(!route.k[0], "T3 OFF silences K1");
}

int main() {
  test_t4_on_while_k3_running();
  test_t3_on_while_k3_running();
  test_k2_not_cut_short_on_off_event();
  test_simultaneous_t3_on_t4_off();
  test_t3_off_still_silences_k1();
  std::cout << "PASS: overlap, full 90-second pulses, and simultaneous events\n";
}
