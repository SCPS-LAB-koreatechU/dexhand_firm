// Desktop-only adapter for exercising the real firmware with the Python SDK.
// Compile once per hand using -I firmware/DexHandLeft or DexHandRight.
#include <cstdio>
#include "BoardConfig.h"
#include "HandController.h"
using namespace hand_config;

struct Output {
  void off() { std::puts("OFF"); }
  bool apply(const uint16_t p[15]) {
    std::printf("PWM");
    for (int i = 0; i < 15; ++i) std::printf(" %u", p[i]);
    std::puts("");
    return true;
  }
};
struct Reporter {
  bool send(uint32_t id, const uint8_t d[8]) {
    std::printf("CAN %u", id);
    for (int i = 0; i < 8; ++i) std::printf(" %u", d[i]);
    std::puts("");
    return true;
  }
};
unsigned nibble(char c) { return c <= '9' ? c - '0' : c - 'a' + 10; }
int main() {
  Output out;
  Reporter reporter;
  uint8_t tag[4];
  for (int i = 0; i < 4; ++i) tag[i] = (nibble(kProfileSha256[2*i]) << 4) | nibble(kProfileSha256[2*i+1]);
  HandController<Output, Reporter> hand(out, reporter, kNodeId, kNodeId, (6U << 7) | kNodeId,
      kMinUs, kMaxUs, kAssemblyTimeoutMs, kWatchdogMs, 500, tag);
  hand.begin();
  hand.setBootId(0x12345678);
  hand.setPairCalibration(kOpenUs, kCloseUs);
  std::puts("END"); std::fflush(stdout);
  unsigned id, len, value;
  uint32_t now = 0;
  while (std::scanf("%u %u", &id, &len) == 2) {
    if (len > 8) return 2;
    uint8_t data[8] = {};
    for (unsigned i = 0; i < len; ++i) {
      if (std::scanf("%u", &value) != 1 || value > 255) return 3;
      data[i] = static_cast<uint8_t>(value);
    }
    hand.receive(id, false, false, static_cast<uint8_t>(len), data, ++now);
    hand.service(now);
    std::puts("END"); std::fflush(stdout);
  }
}
