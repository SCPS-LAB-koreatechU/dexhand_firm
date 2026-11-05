#include <cassert>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include "../firmware/DexHandRight/BoardConfig.h"
#include "../firmware/DexHandRight/PoseReceiver.h"

int main() {
  using namespace hand_config;
  static_assert(kHardwareMaxUs==2300,"Expected requested ceiling");
  static_assert(kCloseUs[5]==1750 && kOpenUs[5]==850,"Do not stretch calibrated motion");
  for (int channel=0;channel<15;++channel) {
    assert(kMaxUs[channel]==2300);
    for (int pulse: {2300,2301}) {
      uint16_t pose[15],out[15];
      for(int i=0;i<15;++i) {pose[i]=kOpenUs[i];out[i]=9999;}
      pose[channel]=pulse;
      PoseReceiver receiver(kMinUs,kMaxUs,kAssemblyTimeoutMs);
      bool rejected=false,complete=false;
      for(uint8_t group=0;group<5;++group) {
        uint8_t packet[8]={1,static_cast<uint8_t>(0x10|group)};
        for(int j=0;j<3;++j) {packet[2+2*j]=pose[group*3+j]&255;packet[3+2*j]=pose[group*3+j]>>8;}
        auto result=receiver.accept(packet,group,out);
        rejected|=result==PoseReceiver::Result::Rejected;
        complete|=result==PoseReceiver::Result::Complete;
      }
      assert((pulse==2300 && complete && !rejected) || (pulse==2301 && rejected && !complete));
      if(pulse==2301)for(auto value:out)assert(value==9999);
    }
  }
  puts("PASS: all 15 channels accept raw 2300, atomically reject 2301; calibrated middle close remains 1750");
}
