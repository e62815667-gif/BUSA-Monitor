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
  evaluateRouted(route, enabled,
                 true, t3, 80.0f, 75.0f,
                 true, t4, 80.0f, 75.0f, now, 90000UL);
}

void test_t4_on_while_k3_running() {
  Routed route;
  const bool enabled[4] = {true, true, true, true};
  sample(route, enabled, 70.0f, 70.0f, 100);
  sample(route, enabled, 81.0f, 70.0f, 200);
  require(route.k[0] && route.k[1] && !route.k[2], "T3 ON starts K1 and K2");
  sample(route, enabled, 74.0f, 70.0f, 300);
  require(!route.k[0] && !route.k[1] && route.k[2], "T3 OFF silences K1 and starts K3");
  sample(route, enabled, 74.0f, 81.0f, 400);
  require(route.t4OnEvent, "T4 ON event detected during K3");
  require(route.k[1], "K2 must start on T4 ON even while K3 runs");
  require(route.k[2], "Existing K3 pulse must remain active");
  require(route.k[3], "T4 ON enables K4");
  sample(route, enabled, 74.0f, 81.0f, 500);
  require(route.k[1], "K2 remains on during its 90-second pulse");
  require(route.k2.start_ms == 400, "K2 pulse is not retriggered without a new event");
  sample(route, enabled, 74.0f, 81.0f, 90400);
  require(!route.k[1] && !route.k[2] && route.k[3], "Pulses stop after 90 seconds; K4 remains on");
}

void test_t3_on_while_k3_running() {
  Routed route;
  const bool enabled[4] = {true, true, true, true};
  sample(route, enabled, 70.0f, 70.0f, 100);
  sample(route, enabled, 81.0f, 70.0f, 200);
  sample(route, enabled, 74.0f, 70.0f, 300);
  require(route.k[2], "K3 started by T3 OFF");
  sample(route, enabled, 81.0f, 70.0f, 400);
  require(route.t3OnEvent, "T3 ON event detected during K3");
  require(route.k[1], "K2 must start on T3 ON even while K3 runs");
  require(route.k[0], "K1 resumes on T3 ON");
  require(route.k[2], "K3 pulse is unchanged");
}

int main() {
  test_t4_on_while_k3_running();
  test_t3_on_while_k3_running();
  std::cout << "PASS: overlap and 90-second pulse checks\n";
}
