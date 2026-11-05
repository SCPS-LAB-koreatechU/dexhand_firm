# 오른손 ESP32 수신 전용 · PWM 작업 분리

2026-11-05. 사용자 제공 DexHandRight.zip을 기준으로 수정했습니다.
새끼손가락 자체 불량을 전제로 핀/끝점을 바꾸지 않고 공통 수신·PWM 경로를 수정했습니다.

## 변경

- 오른손 ESP32는 PONG/STATUS/STATE/BOOT/명령 ACK 데이터 프레임을 송신하지 않습니다.
- CAN 정상 수신에 필요한 하드웨어 ACK 비트와 GPIO1/3 배선은 유지합니다. TX 선을 빼지 마세요.
- CAN 수신은 Arduino core 1, LEDC 출력은 별도 FreeRTOS 작업(core 0)에서 처리합니다.
- LEDC 드라이버 내부의 이전 PWM 완료 대기는 우회하지 않습니다. 기다리는 동안 CAN 수신 루프가 계속 실행될 수 있도록 작업을 분리했습니다.
- 출력 중 목표는 진행 중 1개와 대기 중 최신 1개만 보관합니다. 오래된 목표를 순서대로 재생하지 않습니다.
- 동일한 PWM은 다시 쓰지 않습니다. 파형은 LEDC 하드웨어가 유지합니다.
- HOLD/수신 시간 초과/연결 복구는 대기 목표와 이후 채널 쓰기를 취소합니다.
  이미 실행 중인 ledcWrite 한 개는 완료될 수 있습니다. OFF는 같은 PWM 작업에서 처리하여 동시에 LEDC를 조작하지 않습니다.
- PWM 50Hz, 15개 핀, CAN 1Mbps, 노드61 및 프로토콜3의 명령 형식은 유지합니다.
- 엄지: GPIO13 왼쪽 2100→700, GPIO14 TIP 700→1900, GPIO18 오른쪽 2300→700µs.
  검지 위 GPIO32/아래 GPIO25를 포함한 전체 GPIO는 유지합니다.

## 파일

오른손 펌웨어: dexhand_control/firmware/DexHandRight/DexHandRight.ino
같은 폴더의 헤더를 모두 유지하고 폴더 전체를 Arduino IDE에서 엽니다.
대상: classic 듀얼코어 ESP32-WROOM / ESP32 Dev Module, Arduino-ESP32 3.3.12,
Arduino loop core 1. S2/S3/C3용 스케치가 아닙니다.

새 SDK 모드 설정: humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk/config/dexhand_oneway.yaml
오른손 feedback:false이며 왼손은 기존 모드를 유지합니다.
기존 dexhand.yaml은 응답 기반 모드의 설정으로 보관했습니다.
새 릴리스의 gui_hand.sh/simple_hand.sh/test_hand.sh는 dexhand_oneway.yaml을 기본으로 읽습니다.
ROS/직접 SDK API를 사용한다면 config 경로도 dexhand_oneway.yaml로 지정해야 합니다.

## 휴머노이드 PC 실행 순서

1. 먼저 새 오른손 펌웨어를 업로드합니다. SDK만 바꾸어도 보드 내부 코드는 바뀌지 않습니다.
   이번 작업에서 실물 업로드와 모터 구동은 하지 않았습니다.
2. 기존 손 GUI를 종료한 뒤 다음을 실행합니다.

```bash
cd ~/dexhand_control_oneway
bash gui_hand.sh --hand right --channel can1
```

기존 ~/humanoid_sdk 링크는 변경하지 않았습니다. 새 펌웨어 업로드 전에
기존 GUI와 새 단방향 GUI를 섞어 실행하지 않도록 별도 경로에 준비했습니다.
새 경로는 ~/humanoid_sdk_oneway 입니다.

CAN 소켓 접속 버튼은 로컬 CAN 인터페이스 접속만 확인합니다. 보드가 켜져 있는지,
프로필이 일치하는지, 목표를 받았는지 확인할 수 없습니다.
송신 성공/SENT는 OS 송신 큐에 넣었다는 뜻이며 APPLIED 또는 실제 도달 확인이 아닙니다.
처음 연결/재연결에는 보드 PWM을 읽을 수 없으므로 명시적인 손 펼치기부터 시작합니다.
같은 연결에서 HOLD 후에는 마지막 송신값 재개를 선택할 수 있으나 이것도 보드 readback이 아닙니다.
자동 재개는 없습니다. 엄지 캘리브레이션 UI는 추가하지 않았습니다.

## 검증과 한계

원본 SDK 호환성 검사와 새 단방향 SDK 시험을 수행했습니다.
모의 GUI에서 미응답을 오류로 취급하지 않는지, 보드 확인값을 만들지 않는지,
재접속 후 자동 출력/이전 목표 복구가 없는지 확인했습니다.
Native C++ 시험에서는 LEDC 쓰기를 의도적으로 막은 채 CAN receive가 반환하는지,
최신 목표 1개 보관, HOLD 시 대기 목표 취소와 오래된 목표 미재생을 검증했습니다.
이 시험은 실제 MCU 지연/손가락 위치 측정 결과가 아닙니다.
CAN 무응답 모드이므로 보드 오류·전류·온도·PWM 적용 ACK 로그는 받을 수 없습니다.
PC 목표/송신값/송신 오류/입력 시각 로그는 계속 ~/bhl_logs/hand_gui/*.jsonl에 기록합니다.

공유된 구형 firmware/DexHandCan 템플릿으로 오른손을 재생성하면 비동기 변경을 잃습니다.
prepare_firmware.py는 이 오른손을 덮어쓰지 못하도록 차단했습니다.
보정값만 변경할 때는 해당 DexHandRight 폴더의 calibration.json과 update_config.py를 사용합니다.
