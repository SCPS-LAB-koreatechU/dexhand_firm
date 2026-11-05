#pragma once
#include <stdint.h>
#include <string.h>
// Caller holds a short critical section. No hardware operation runs under it.
class LatestPoseMailbox {
 public:
  void publish(const uint16_t pose[15]) { memcpy(pose_,pose,sizeof(pose_));pending_=true; }
  void hold() { ++generation_;pending_=false; }
  void off() { hold();offPending_=true; }
  bool takeOff() { bool result=offPending_;offPending_=false;return result; }
  bool take(uint16_t pose[15],uint32_t& generation) {
    if(!pending_)return false;
    memcpy(pose,pose_,sizeof(pose_));generation=generation_;pending_=false;return true;
  }
  bool canceled(uint32_t generation) const { return generation!=generation_; }
 private:
  uint16_t pose_[15]={};uint32_t generation_=0;bool pending_=false,offPending_=false;
};
