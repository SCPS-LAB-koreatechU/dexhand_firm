// Local test configuration. Edit calibration.json, then run python update_config.py.
// Thumb endpoints below are user-confirmed assignments; GPIOs are unchanged.
#pragma once
#include <stdint.h>
namespace hand_config {
constexpr uint8_t kNodeId = 61;
constexpr int kCanTx = 1;
constexpr int kCanRx = 3;
constexpr uint32_t kWatchdogMs = 1000;
constexpr uint32_t kAssemblyTimeoutMs = 20;
// GPIO34/35 are input-only. User confirmed output-capable GPIO32/25.
constexpr uint8_t kIndexUpperGpio = 32;
constexpr uint8_t kIndexLowerGpio = 25;
constexpr uint8_t kPins[15] = {19, 21, 23, 26, 33, 22, kIndexUpperGpio, kIndexLowerGpio, 16, 17, 4, 27, 13, 14, 18};
constexpr uint8_t kThumbRightIndex = 14;  // channel 15, GPIO18: right pull
constexpr uint8_t kThumbLeftIndex = 12;   // channel 13, GPIO13: left pull
constexpr uint8_t kThumbTipIndex = 13;    // channel 14, GPIO14: thumb tip
// User-confirmed endpoints; edit JSON and regenerate this header together with SDK calibration.
constexpr uint16_t kThumbLeftOpenUs = 2100;
constexpr uint16_t kThumbLeftCloseUs = 700;
constexpr uint16_t kThumbTipOpenUs = 700;
constexpr uint16_t kThumbTipCloseUs = 1900;
constexpr uint16_t kThumbRightOpenUs = 2300;
constexpr uint16_t kThumbRightCloseUs = 700;
constexpr uint16_t minUs(uint16_t a, uint16_t b) { return a < b ? a : b; }
constexpr uint16_t maxUs(uint16_t a, uint16_t b) { return a > b ? a : b; }
constexpr uint16_t kHardwareMaxUs = 2300;
constexpr uint16_t kMinUs[15] = {850, 800, 850, 800, 825, 850, 700, 700, 700, 700, 750, 800, minUs(kThumbLeftOpenUs, kThumbLeftCloseUs), minUs(kThumbTipOpenUs, kThumbTipCloseUs), minUs(kThumbRightOpenUs, kThumbRightCloseUs)};
constexpr uint16_t kMaxUs[15] = {kHardwareMaxUs, kHardwareMaxUs, kHardwareMaxUs, kHardwareMaxUs, kHardwareMaxUs, kHardwareMaxUs, kHardwareMaxUs, kHardwareMaxUs, kHardwareMaxUs, kHardwareMaxUs, kHardwareMaxUs, kHardwareMaxUs, kHardwareMaxUs, kHardwareMaxUs, kHardwareMaxUs};
constexpr uint16_t kOpenUs[15] = {850, 1700, 850, 1700, 1750, 850, 1750, 700, 700, 700, 2000, 2000, kThumbLeftOpenUs, kThumbTipOpenUs, kThumbRightOpenUs};
constexpr uint16_t kCloseUs[15] = {1775, 800, 1725, 800, 825, 1750, 700, 1725, 2100, 2000, 750, 800, kThumbLeftCloseUs, kThumbTipCloseUs, kThumbRightCloseUs};
constexpr bool isServoOutputGpio(uint8_t p) {
  return p <= 5 || (p >= 12 && p <= 19) || (p >= 21 && p <= 23) ||
         (p >= 25 && p <= 27) || p == 32 || p == 33;
}
static_assert(isServoOutputGpio(kIndexUpperGpio) && isServoOutputGpio(kIndexLowerGpio),
              "Index GPIO34/35 are INPUT ONLY: connect upper/lower to GPIO32/25.");
constexpr bool validServoPins() {
  for (uint8_t i = 0; i < 15; ++i) {
    if (!isServoOutputGpio(kPins[i]) || kPins[i] == kCanTx || kPins[i] == kCanRx) return false;
    for (uint8_t j = 0; j < i; ++j) if (kPins[i] == kPins[j]) return false;
  }
  return true;
}
static_assert(validServoPins(), "Invalid/duplicate PWM GPIO or conflict with CAN GPIO1/3");
constexpr char kProfileSha256[] = "bdad07a440931ef8c5a5b53debdbad9e77b1bb9a9b758b6cee89741049dfc354";
}  // namespace hand_config
