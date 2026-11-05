#pragma once
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "BoardConfig.h"
#include "LatestPoseMailbox.h"

// CAN runs on Arduino core 1; only this task touches LEDC on core 0.
// LEDC's internal wait is retained. It can no longer block the CAN receive loop.
class ServoOutput {
 public:
  bool begin() {
    const BaseType_t result=xTaskCreatePinnedToCore(taskEntry,"hand_pwm",4096,this,2,&task_,0);
    portENTER_CRITICAL(&mux_);ready_=result==pdPASS;portEXIT_CRITICAL(&mux_);
    return result==pdPASS;
  }
  void off() {
    portENTER_CRITICAL(&mux_);mailbox_.off();portEXIT_CRITICAL(&mux_);
    wake();
  }
  void hold() {
    portENTER_CRITICAL(&mux_);mailbox_.hold();portEXIT_CRITICAL(&mux_);
    wake();
  }
  // Acceptance into a latest-only mailbox, not a physical/PWM completion ACK.
  bool apply(const uint16_t pose[15]) {
    portENTER_CRITICAL(&mux_);
    bool accepted=ready_&&!failed_;
    if(accepted)mailbox_.publish(pose);
    portEXIT_CRITICAL(&mux_);
    if(accepted)wake();return accepted;
  }
 private:
  static void taskEntry(void* context) { static_cast<ServoOutput*>(context)->run(); }
  void wake() { if(task_)xTaskNotifyGive(task_); }
  bool canceled(uint32_t generation) {
    portENTER_CRITICAL(&mux_);bool result=mailbox_.canceled(generation);portEXIT_CRITICAL(&mux_);return result;
  }
  void hardwareOff() {
    for(uint8_t i=0;i<15;++i) {
      if(attached_[i])ledcDetach(hand_config::kPins[i]);
      attached_[i]=false;written_[i]=false;
      pinMode(hand_config::kPins[i],OUTPUT);digitalWrite(hand_config::kPins[i],LOW);
    }
  }
  void fail() {
    portENTER_CRITICAL(&mux_);failed_=true;mailbox_.off();portEXIT_CRITICAL(&mux_);
    hardwareOff();
  }
  void run() {
    for(;;) {
      bool off;uint16_t pose[15];uint32_t generation=0;bool pending=false;
      portENTER_CRITICAL(&mux_);
      off=mailbox_.takeOff();
      if(!off)pending=mailbox_.take(pose,generation);
      portEXIT_CRITICAL(&mux_);
      if(off) { hardwareOff();continue; }
      if(!pending) { ulTaskNotifyTake(pdTRUE,portMAX_DELAY);continue; }
      for(uint8_t i=0;i<15;++i) {
        if(canceled(generation))break;
        if(!attached_[i]) {
          if(!ledcAttachChannel(hand_config::kPins[i],50,16,i)) { fail();break; }
          attached_[i]=true;
        }
        if(canceled(generation))break;
        if(written_[i]&&lastCommandUs_[i]==pose[i])continue;
        uint32_t duty=(static_cast<uint32_t>(pose[i])*65536U+10000U)/20000U;
        if(!ledcWrite(hand_config::kPins[i],duty)) { fail();break; }
        written_[i]=true;lastCommandUs_[i]=pose[i];
      }
      // Allow core-0 idle/system tasks to run between updates.
      vTaskDelay(1);
    }
  }
  TaskHandle_t task_=nullptr;portMUX_TYPE mux_=portMUX_INITIALIZER_UNLOCKED;
  LatestPoseMailbox mailbox_;bool ready_=false,failed_=false;
  bool attached_[15]={},written_[15]={};uint16_t lastCommandUs_[15]={};
};
