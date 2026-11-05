#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "../firmware/DexHandCan/BoardConfig.h"
#include "../firmware/DexHandCan/HandController.h"

using namespace hand_config;

struct FakeOutput {
  unsigned writes = 0;
  unsigned stops = 0;
  bool enabled = false;
  bool fail = false;
  uint16_t command[15] = {};
  void off() { ++stops; enabled = false; }
  bool apply(const uint16_t pose[15]) {
    ++writes;
    if (fail) return false;
    enabled = true;
    for (uint8_t i = 0; i < 15; ++i) command[i] = pose[i];
    return true;
  }
};

struct NullReporter { bool send(uint32_t, const uint8_t[8]) { return true; } };
using Controller = HandController<FakeOutput, NullReporter>;
NullReporter reporter;
const uint8_t tag[4] = {};
constexpr uint32_t poseId = (6U << 7) | kNodeId;

static void packet(uint8_t seq, uint8_t group, const uint16_t pose[15], uint8_t data[8]) {
  data[0] = seq;
  data[1] = 0x10 | group;
  for (uint8_t i = 0; i < 3; ++i) {
    data[2 + i * 2] = pose[group * 3 + i] & 255;
    data[3 + i * 2] = pose[group * 3 + i] >> 8;
  }
}

static void sendGroup(Controller& hand, uint8_t seq, uint8_t group, uint32_t now,
                      const uint16_t* pose = kMinUs) {
  uint8_t data[8];
  packet(seq, group, pose, data);
  hand.receive(poseId, false, false, 8, data, now);
}

static void full(Controller& hand, uint8_t seq, uint32_t now) {
  for (uint8_t group = 0; group < 5; ++group) sendGroup(hand, seq, group, now + group);
}

static void mode(Controller& hand, uint8_t value) {
  const uint8_t data[2] = {value, kNodeId};
  hand.receive(kNodeId, false, false, 2, data, 0);
}

int main() {
  FakeOutput output;
  Controller hand(output, reporter, kNodeId, kNodeId, poseId, kMinUs, kMaxUs, 20, 1000, 500, tag);
  hand.begin();
  assert(!output.enabled && output.stops == 1 && output.writes == 0);

  // All fifteen outputs change only on the final valid group, right inside receive().
  for (uint8_t group = 0; group < 4; ++group) sendGroup(hand, 7, group, 100);
  assert(output.writes == 0);
  sendGroup(hand, 7, 4, 100);
  assert(output.enabled && output.writes == 1);
  for (uint8_t i = 0; i < 15; ++i) assert(output.command[i] == kMinUs[i]);

  // Bad headers, extended/RTR frames, foreign IDs and wrong control targets
  // cannot change the output or inhibit the controller.
  uint8_t data[8];
  packet(8, 0, kMinUs, data);
  hand.receive(poseId, true, false, 8, data, 110);
  hand.receive(poseId, false, true, 8, data, 110);
  hand.receive(poseId + 1, false, false, 8, data, 110);
  hand.receive(poseId, false, false, 7, data, 110);
  uint8_t wrongControl[2] = {0x00, static_cast<uint8_t>(kNodeId + 1)};
  hand.receive(kNodeId, false, false, 2, wrongControl, 110);
  full(hand, 8, 120);
  assert(output.enabled && output.writes == 2);

  // A bad range poisons that sequence and preserves the last complete output.
  uint16_t bad[15];
  for (uint8_t i = 0; i < 15; ++i) bad[i] = kMinUs[i];
  bad[0]--;
  sendGroup(hand, 9, 0, 130, bad);
  full(hand, 9, 131);
  assert(output.writes == 2 && output.enabled);
  full(hand, 10, 140);
  assert(output.writes == 3);

  // HOLD preserves output. RESUME discards old partial sets and permits a new sequence.
  mode(hand, 0x02);
  full(hand, 11, 150);
  assert(output.writes == 3 && output.enabled);
  mode(hand, 0x13);
  full(hand, 0, 160);
  assert(output.writes == 4 && output.enabled);
  mode(hand, 0x00);
  full(hand, 1, 170);
  assert(!output.enabled && output.writes == 4);
  mode(hand, 0x01);
  assert(!output.enabled);
  full(hand, 0, 200);
  assert(output.enabled && output.writes == 5);

  // Duplicates/partial sets never extend the communication watchdog.
  full(hand, 0, 900);  // duplicate of completed set
  sendGroup(hand, 1, 0, 1000);
  assert(!hand.checkWatchdog(1203));
  assert(hand.checkWatchdog(1204));
  assert(output.enabled && output.writes == 5);
  assert(!hand.checkWatchdog(1205));
  full(hand, 0, 1210);  // automatic resumption, without a serial/CAN ARM step
  assert(output.writes == 6);

  // CAN recovery cannot combine pre-recovery fragments with new fragments.
  sendGroup(hand, 1, 0, 1300);
  hand.invalidatePose();
  for (uint8_t group = 1; group < 5; ++group) sendGroup(hand, 1, group, 1301);
  assert(output.writes == 6 && output.enabled);
  sendGroup(hand, 1, 0, 1301);
  assert(output.writes == 7);

  // A failed PWM application disables output and requires an explicit retry.
  output.fail = true;
  full(hand, 2, 1400);
  assert(output.writes == 8 && !output.enabled);
  output.fail = false;
  full(hand, 3, 1410);
  assert(output.writes == 8 && !output.enabled);
  mode(hand, 0x13);
  full(hand, 0, 1420);
  assert(output.writes == 9 && output.enabled);

  // Timeout arithmetic also works across 32-bit millis wrap.
  hand.invalidatePose();
  full(hand, 0, 0xFFFFFFF0U);
  const uint32_t completedAt = 0xFFFFFFF4U;
  assert(!hand.checkWatchdog(completedAt + 999U));
  assert(hand.checkWatchdog(completedAt + 1000U));
  assert(output.enabled);

  // Routing must use the configured full IDs rather than recomputing from node.
  FakeOutput customOutput;
  Controller custom(customOutput, reporter, kNodeId, 0x123, 0x456, kMinUs, kMaxUs, 20, 1000, 500, tag);
  custom.begin();
  full(custom, 0, 0);  // old formula-derived pose ID is ignored
  assert(customOutput.writes == 0);
  for (uint8_t g = 0; g < 5; ++g) {
    packet(0, g, kMinUs, data);
    custom.receive(0x456, false, false, 8, data, g);
  }
  assert(customOutput.writes == 1 && customOutput.enabled);
  const uint8_t off[2] = {0x00, kNodeId};
  custom.receive(kNodeId, false, false, 2, off, 10);  // old control ID ignored
  assert(customOutput.enabled);
  custom.receive(0x123, false, false, 2, off, 10);
  assert(!customOutput.enabled);
  puts("PASS: battery boot, immediate complete-pose output, frame filters, range rejection, controls, watchdog, CAN reset and output failure");
}
