#include <cassert>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <vector>
#include <mutex>
#include <condition_variable>
#include "ServoOutput.h"
#include "HandController.h"
using namespace hand_config;
std::mutex hardware;std::condition_variable changed;bool blocked=false,released=false;
std::vector<std::pair<unsigned,unsigned>> writes;
bool ledcAttachChannel(uint8_t,int,int,int){return true;}
bool ledcDetach(uint8_t){return true;}
void pinMode(uint8_t,int){}
void digitalWrite(uint8_t,int){}
bool ledcWrite(uint8_t pin,uint32_t duty){
 std::unique_lock<std::mutex> lock(hardware);
 if(!blocked){blocked=true;changed.notify_all();changed.wait(lock,[]{return released;});}
 writes.emplace_back(pin,duty);changed.notify_all();return true;
}
struct Reporter {bool send(uint32_t,const uint8_t[8]){return false;}};
int main(){
 auto out=new ServoOutput;assert(out->begin());Reporter reporter;uint8_t tag[4]={};
 auto hand=new HandController<ServoOutput,Reporter>(*out,reporter,kNodeId,kNodeId,(6u<<7)|kNodeId,kMinUs,kMaxUs,20,1000,500,tag);
 hand->begin();hand->setPairCalibration(kOpenUs,kCloseUs,kThumbRightIndex,kThumbLeftIndex);
 uint16_t pose[15];memcpy(pose,kOpenUs,sizeof pose);
 auto send=[&](uint8_t sequence,uint32_t now){
  auto start=std::chrono::steady_clock::now();
  for(uint8_t g=0;g<5;++g){uint8_t d[8]={sequence,static_cast<uint8_t>(0x10|g)};
   for(unsigned i=0;i<3;++i){auto p=pose[g*3+i];d[2+2*i]=p&255;d[3+2*i]=p>>8;}
   hand->receive((6u<<7)|kNodeId,false,false,8,d,now+g);
  }
  return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
 };
 assert(send(1,10)<20);
 {std::unique_lock<std::mutex> lock(hardware);assert(changed.wait_for(lock,std::chrono::seconds(2),[]{return blocked;}));}
 pose[0]=900;double receiveMs=send(2,20);assert(receiveMs<20); // PWM task is still blocked
 uint8_t hold[3]={0x02,kNodeId,1};hand->receive(kNodeId,false,false,3,hold,30);
 {std::lock_guard<std::mutex> lock(hardware);released=true;changed.notify_all();}
 std::this_thread::sleep_for(std::chrono::milliseconds(20));
 {std::lock_guard<std::mutex> lock(hardware);assert(writes.size()==1);}
 uint8_t resume[3]={0x13,kNodeId,2};hand->receive(kNodeId,false,false,3,resume,40);
 pose[0]=1000;assert(send(3,50)<20);
 {std::unique_lock<std::mutex> lock(hardware);assert(changed.wait_for(lock,std::chrono::seconds(2),[]{return writes.size()>=16;}));
  for(auto w:writes)assert(!(w.first==kPins[0]&&w.second==(900u*65536u+10000u)/20000u));
 }
 LatestPoseMailbox m;uint16_t a[15]={},b[15]={},r[15];uint32_t gen;
 a[0]=100;b[0]=200;m.publish(a);m.publish(b);assert(m.take(r,gen)&&r[0]==200);assert(!m.take(r,gen));
 m.publish(a);m.hold();assert(!m.take(r,gen));assert(m.canceled(gen));
 m.publish(a);m.off();assert(m.takeOff());assert(!m.take(r,gen));
 printf("PASS: CAN receive %.3fms while PWM is blocked; HOLD cancels pending/in-flight remainder; stale goal never replays; latest-only mailbox; explicit resume\n",receiveMs);
}
