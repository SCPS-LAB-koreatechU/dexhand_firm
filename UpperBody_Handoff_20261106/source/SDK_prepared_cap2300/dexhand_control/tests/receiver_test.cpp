#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include "../firmware/DexHandCan/BoardConfig.h"
#include "../firmware/DexHandCan/PoseReceiver.h"

using R = PoseReceiver::Result;
using namespace hand_config;

static void packet(uint8_t seq, uint8_t group, const uint16_t pose[15], uint8_t data[8]) {
  data[0] = seq;
  data[1] = 0x10 | group;
  for (uint8_t i = 0; i < 3; ++i) {
    data[2 + i * 2] = pose[group * 3 + i] & 255;
    data[3 + i * 2] = pose[group * 3 + i] >> 8;
  }
}

static R group(PoseReceiver& rx, uint8_t seq, uint8_t g, uint32_t now,
               uint16_t out[15], const uint16_t* pose = kMinUs) {
  uint8_t data[8];
  packet(seq, g, pose, data);
  return rx.accept(data, now, out);
}

static void full(PoseReceiver& rx, uint8_t seq, uint32_t now, uint16_t out[15]) {
  for (uint8_t g = 0; g < 5; ++g)
    assert(group(rx, seq, g, now + g, out) == (g == 4 ? R::Complete : R::Partial));
}

int main() {
  uint16_t out[15];
  for (auto& value : out) value = 9999;
  PoseReceiver rx(kMinUs, kMaxUs, 20);

  // Nothing is committed before the complete set, even with duplicate groups.
  assert(group(rx, 7, 0, 0, out) == R::Partial);
  assert(group(rx, 7, 0, 1, out) == R::Partial);
  for (auto value : out) assert(value == 9999);
  for (uint8_t g = 1; g < 5; ++g)
    assert(group(rx, 7, g, 1 + g, out) == (g == 4 ? R::Complete : R::Partial));
  for (uint8_t i = 0; i < 15; ++i) assert(out[i] == kMinUs[i]);
  assert(group(rx, 7, 4, 8, out) == R::Ignored);
  assert(group(rx, 6, 0, 9, out) == R::Ignored);

  // A new sequence replaces a missing old set without mixing its groups.
  assert(group(rx, 8, 0, 10, out) == R::Partial);
  assert(group(rx, 9, 1, 11, out) == R::Partial);
  assert(group(rx, 8, 2, 12, out) == R::Ignored);
  assert(group(rx, 9, 0, 12, out) == R::Partial);
  assert(group(rx, 9, 2, 13, out) == R::Partial);
  assert(group(rx, 9, 4, 14, out) == R::Partial);
  assert(group(rx, 9, 3, 15, out) == R::Complete);

  // Expired sets cannot be resurrected using the same sequence.
  assert(group(rx, 10, 0, 20, out) == R::Partial);
  assert(group(rx, 10, 1, 41, out) == R::Rejected);
  assert(group(rx, 10, 0, 42, out) == R::Ignored);
  full(rx, 11, 43, out);

  // Values outside a particular motor's limits poison that entire sequence.
  uint16_t bad[15];
  for (uint8_t i = 0; i < 15; ++i) bad[i] = kMinUs[i];
  bad[0] = kMinUs[0] - 1;
  assert(group(rx, 12, 0, 50, out, bad) == R::Rejected);
  assert(group(rx, 12, 0, 51, out) == R::Ignored);
  full(rx, 13, 52, out);

  // Conflicting duplicates cannot combine two different poses.
  assert(group(rx, 14, 0, 60, out) == R::Partial);
  bad[0] = kMinUs[0] + 1;
  assert(group(rx, 14, 0, 61, out, bad) == R::Rejected);
  assert(group(rx, 14, 1, 62, out) == R::Ignored);

  // Invalid headers poison that sequence and report once.
  uint8_t data[8];
  packet(15, 0, kMinUs, data);
  data[1] = 0x20;
  assert(rx.accept(data, 63, out) == R::Rejected);
  data[1] = 0x15;
  assert(rx.accept(data, 63, out) == R::Ignored);
  assert(group(rx, 15, 0, 64, out) == R::Ignored);
  full(rx, 16, 65, out);

  // Sequence wrap, millis wrap, and explicit PC-session reset.
  rx.reset();
  full(rx, 254, 100, out);
  full(rx, 255, 110, out);
  full(rx, 0, 120, out);
  assert(group(rx, 255, 0, 130, out) == R::Ignored);
  assert(group(rx, 128, 0, 130, out) == R::Ignored);
  rx.reset();
  full(rx, 200, 0xFFFFFFFEU, out);
  rx.reset();
  full(rx, 0, 0, out);
  puts("PASS: complete sets, duplicates, missing/out-of-order groups, limits, deadlines, sequence and timer wrap");
}

