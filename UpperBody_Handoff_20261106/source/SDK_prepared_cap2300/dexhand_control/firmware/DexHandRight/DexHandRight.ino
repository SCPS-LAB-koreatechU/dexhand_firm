// Standalone right-hand test copy, node 61.
// Edit this folder's calibration.json, then run: python update_config.py
// Confirmed index wiring: upper GPIO32 / lower GPIO25. GPIO34/35 cannot output PWM.
// Thumb: GPIO13=left pull 2100->700, GPIO14=tip 700->1900, GPIO18=right pull 2300->700.
/* Right hand receive-only CAN commands -> latest-goal mailbox -> PWM worker.
 * No application PONG/STATUS/STATE/BOOT/ACK frames. CAN hardware ACK remains.
 * A valid complete pose is accepted by core 1. Core 0 alone owns LEDC updates.
 * First output requires a complete deliberate target; boot never starts PWM.
 */
#include <Arduino.h>

// CAN_CONFIG_BEGIN
constexpr int CAN_TX_GPIO = 1;          // ESP32 -> transceiver TXD/CTX
constexpr int CAN_RX_GPIO = 3;          // ESP32 <- transceiver RXD/CRX
constexpr uint8_t CAN_NODE_ID = 61;        // 오른손 장치 번호
constexpr uint32_t CAN_CONTROL_ID = 0x03D; // START / HOLD / RESUME / OFF
constexpr uint32_t CAN_POSE_ID = 0x33D;    // 손가락 목표 프레임
constexpr uint32_t CAN_PING_ID = 0x23D;    // PING / 마지막 목표 조회
constexpr uint32_t CAN_PONG_ID = 0x1BD;    // ESP32 → PC 장치 응답
constexpr uint32_t CAN_STATUS_ID = 0x2BD;  // ESP32 → PC 적용 결과
constexpr uint32_t CAN_STATE_ID = 0x43D;   // ESP32 → PC 마지막 PWM 목표
constexpr uint32_t CAN_BOOT_ID = 0x4BD;    // ESP32 → PC 부팅 식별
constexpr uint32_t HEARTBEAT_MS = 500;
// CAN_CONFIG_END

#include "driver/twai.h"
#include "esp_log.h"
#include "esp_arduino_version.h"
#include "esp_system.h"
#include "BoardConfig.h"
#include "HandController.h"
#include "ServoOutput.h"
#include "StatusReport.h"

#if !defined(CONFIG_IDF_TARGET_ESP32) || ESP_ARDUINO_VERSION_MAJOR < 3 || CONFIG_FREERTOS_UNICORE || ARDUINO_RUNNING_CORE != 1
#error Use dual-core ESP32-WROOM, Arduino-ESP32 3.x, Arduino loop core 1 (15 LEDC channels required).
#endif

using namespace hand_config;
static_assert(kNodeId == 61, "Wrong CAN node for DexHandRight");
static_assert(CAN_TX_GPIO == kCanTx && CAN_RX_GPIO == kCanRx, "CAN pins differ from profile");
static_assert(CAN_NODE_ID >= 1 && CAN_NODE_ID <= 127, "CAN node must be 1..127");
static_assert(CAN_CONTROL_ID <= 0x7FF && CAN_POSE_ID <= 0x7FF, "Use standard 11-bit CAN IDs");
static_assert(CAN_CONTROL_ID != CAN_POSE_ID, "Control and pose IDs must differ");
static_assert(CAN_PING_ID == hand_status::canId(hand_status::kFunctionPing, CAN_NODE_ID), "ping id");
static_assert(CAN_PONG_ID == hand_status::canId(hand_status::kFunctionPong, CAN_NODE_ID), "pong id");
static_assert(CAN_STATUS_ID == hand_status::canId(hand_status::kFunctionStatus, CAN_NODE_ID), "status id");
static_assert(CAN_STATE_ID == hand_status::canId(hand_status::kFunctionState, CAN_NODE_ID), "state id");
static_assert(CAN_BOOT_ID == hand_status::canId(hand_status::kFunctionBoot, CAN_NODE_ID), "boot id");

constexpr uint8_t hexNibble(char c) {
  return c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : c - 'A' + 10;
}
constexpr uint8_t kProfileTag[4] = {
    static_cast<uint8_t>(hexNibble(kProfileSha256[0]) << 4 | hexNibble(kProfileSha256[1])),
    static_cast<uint8_t>(hexNibble(kProfileSha256[2]) << 4 | hexNibble(kProfileSha256[3])),
    static_cast<uint8_t>(hexNibble(kProfileSha256[4]) << 4 | hexNibble(kProfileSha256[5])),
    static_cast<uint8_t>(hexNibble(kProfileSha256[6]) << 4 | hexNibble(kProfileSha256[7]))};

// Application-level TX is intentionally absent. Normal CAN hardware ACK remains.
// No PONG, STATUS, STATE or BOOT data frames are sent by this firmware.
struct CanReporter {
  bool running=false;
  bool send(uint32_t,const uint8_t[8]) { return false; }
};

ServoOutput servoOutput;
CanReporter reporter;
HandController<ServoOutput, CanReporter> hand(servoOutput, reporter, CAN_NODE_ID, CAN_CONTROL_ID,
                                              CAN_POSE_ID, kMinUs, kMaxUs, kAssemblyTimeoutMs,
                                              kWatchdogMs, HEARTBEAT_MS, kProfileTag);
bool driverInstalled = false;
bool canRunning = false;
bool everStarted = false;
uint32_t lastCanAttemptMs = 0;

void setCanRunning(bool running) {
  canRunning = running;
  reporter.running = running;
}

void maintainCan(uint32_t now) {
  if (!driverInstalled) {
    if (static_cast<uint32_t>(now - lastCanAttemptMs) < 1000) return;
    lastCanAttemptMs = now;
    twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
        static_cast<gpio_num_t>(CAN_TX_GPIO), static_cast<gpio_num_t>(CAN_RX_GPIO), TWAI_MODE_NORMAL);
    general.tx_queue_len = 16;        // state snapshot (7), status and boot; never block PWM
    general.rx_queue_len = 64;
    general.alerts_enabled = TWAI_ALERT_RX_QUEUE_FULL;
    twai_timing_config_t timing = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t filter = {};
    filter.single_filter = true;
    // Match the bits shared by every configured ID. Software checks each full ID.
    const uint32_t sharedMask =
        0x7FFU & ~((CAN_CONTROL_ID ^ CAN_POSE_ID) | (CAN_CONTROL_ID ^ CAN_PING_ID));
    filter.acceptance_code = (CAN_CONTROL_ID & sharedMask) << 21;
    filter.acceptance_mask = ~(sharedMask << 21);
    driverInstalled = twai_driver_install(&general, &timing, &filter) == ESP_OK;
    if (!driverInstalled) return;
  }

  twai_status_info_t status;
  if (twai_get_status_info(&status) != ESP_OK) {
    setCanRunning(false);
    hand.invalidatePose();
    return;
  }
  hand.setErrorCounters(status.tx_error_counter, status.rx_error_counter);
  if (status.state == TWAI_STATE_BUS_OFF) {
    if (canRunning) hand.invalidatePose();
    setCanRunning(false);
    twai_initiate_recovery();           // completion is checked on later loop iterations
    return;
  }
  if (status.state == TWAI_STATE_RECOVERING) {
    setCanRunning(false);
    return;
  }
  if (status.state == TWAI_STATE_STOPPED) {
    setCanRunning(twai_start() == ESP_OK);
    if (canRunning) {
      twai_clear_receive_queue();
      twai_clear_transmit_queue();
      hand.invalidatePose();
      if (everStarted) hand.noteCanRecovered();
      everStarted = true;
    }
  } else {
    setCanRunning(status.state == TWAI_STATE_RUNNING);
  }
}

void setup() {
  // GPIO1/3 are CAN pins. Do not start UART0 or use Serial logging.
  // This does not electrically isolate the onboard USB-UART chip or ROM logs.
  Serial.end();
  esp_log_level_set("*", ESP_LOG_NONE);
  servoOutput.begin();
  hand.begin();
  hand.setBootId(esp_random());
  hand.setPairCalibration(kOpenUs, kCloseUs, kThumbRightIndex, kThumbLeftIndex);
  lastCanAttemptMs = millis() - 1000U;
  maintainCan(millis());
}

void loop() {
  const uint32_t now = millis();
  maintainCan(now);
  if (hand.checkWatchdog(now)) {
    // Keep the last PWM. No guessed home pose or motion on timeout.
    // A fresh complete pose automatically resumes, as requested for standalone use.
    if (canRunning) twai_clear_receive_queue();
  }
  if (canRunning) {
    uint32_t alerts = 0;
    if (twai_read_alerts(&alerts, 0) == ESP_OK && (alerts & TWAI_ALERT_RX_QUEUE_FULL)) {
      hand.invalidatePose();
      hand.noteRxOverflow();
      twai_clear_receive_queue();
    }
    twai_message_t message;
    for (uint8_t n = 0; n < 16 && twai_receive(&message, 0) == ESP_OK; ++n) {
      if(message.identifier==CAN_PING_ID)continue;  // queries have no response in receive-only mode
      hand.receive(message.identifier, message.extd, message.rtr,
                   message.data_length_code, message.data, millis());
    }
    // TX-only diagnostics/heartbeats are disabled; the board only receives.
  }
  delay(1);                          // bounded CAN work; hardware PWM continues independently
}
