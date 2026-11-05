# 현재 사용할 코드: 양손 ESP32와 팔·손 SDK 보완본

**양손 CAN은 기존 납땜 배선에 맞춰 TX=GPIO1 / RX=GPIO3으로 수정했다.** [GPIO1/3 배포·업로드 안내](CAN_WIRING_GPIO1_3.md)를 따른다. 이전 GPIO5/34 통합본과 프로필 해시가 다르므로 양손 펌웨어와 PC 설정을 함께 갱신한다.

GUI 사용: [손가락·위치별 제어 / 카메라 / 캘리브레이션](HAND_GUI.md).

처음 손의 CAN 동작을 확인하려면 [간단한 손 CAN·USB 카메라 테스트](HAND_TEST_QUICKSTART.md)를 사용한다. ROS 없이 손 하나를 시험할 수 있다.

2026-11-04. **현재 버전은 손 CAN 프로토콜 3이다.** 받은 v2 추가분을 기존 전체 SDK에 통합한 뒤 검토에서 확인한 문제를 보완했다. 이전 V2 원본과 백업은 사용자 요청으로 삭제했으며 현재 실행본은 아래 경로를 사용한다.

**SDK 0.2.0 추가:** [양팔·양손 공통 API 가이드](UPPER_BODY_SDK_v020.md). `upper_body:=true`로 prepare/commit·전체 목표·순번·TTL·부위별 결과 경로를 켠다. 실물 팔은 검증 기록이 있는 고정 영점이 필요하다. 아래 개별 target/grip 예제는 `upper_body:=false` 벤치 모드용이며 통합 모드에서는 차단된다. 손 CAN 바이트 형식과 GPIO는 바뀌지 않았다.

## 바로 사용할 파일

프로젝트 루트: `C:/Users/alstj/Desktop/project_humanoid`.

| 대상 | 열거나 복사할 경로 |
|---|---|
| 왼손 ESP32 업로드 | `dexhand_control/firmware/DexHandLeft/DexHandLeft.ino` |
| 오른손 ESP32 업로드 | `dexhand_control/firmware/DexHandRight/DexHandRight.ino` |
| 휴머노이드 PC SDK 전체 | `humanoid_sdk-main/` |
| PC 개발자용 바이트 명세 | [PROTOCOL_v3.md](PROTOCOL_v3.md) |
| 양손 배선·보정 원본 | `dexhand_control/profiles/left_hand.json`, `right_hand.json` |
| 오프라인 검증 | `dexhand_control/tools/verify.py` |

각 `.ino`는 **자신이 들어 있는 폴더 전체**가 필요하다. 옆의 BoardConfig.h, HandController.h, PoseReceiver.h, ServoOutput.h, StatusReport.h를 함께 둔다. 두 `.ino`를 한 스케치 폴더에 합치지 않는다. 공통 `DexHandCan.ino`는 생성용 원본이다.

게인·토크 상한·전류 제한을 임의로 높이지 않았다.

## ESP32 업로드와 배터리 기동

1. 모터·보드 전원을 끄고 GPIO1/3과 트랜시버 연결을 분리한다. Arduino IDE에서 **ESP32 Dev Module**, Arduino-ESP32 **3.3.12**를 사용한다. ESP32-WROOM의 15개 LEDC 채널을 사용하므로 C3/S3용 스케치가 아니다.
2. 왼손 보드를 연결하여 왼손 `.ino`를 열고 해당 COM 포트에 업로드한다.
3. 오른손 보드를 연결하여 오른손 `.ino`를 열고 그 보드의 COM 포트에 업로드한다. COM 번호 자체에 손 구분 정보가 있지는 않다.
4. 업로드 후 USB·전원을 분리한 상태에서 기존 CAN 배선을 다시 연결하고 정상 전원을 공급하면 자동 실행된다. 운전 중 ESP32 USB와 UART0 시리얼 모니터를 사용하지 않는다. Wi-Fi·BLE 연결을 기다리지 않는다.
5. 전원 투입 직후 PWM은 꺼져 있다. 첫 정상 15개 목표 세트가 도착하면 바로 출력한다. ESP32는 기본 펼침 자세를 스스로 출력하지 않는다.

왼손 GPIO 순서: **32, 33, 25, 26, 21, 22, 23, 19, 16, 17, 4, 27, 13, 14, 18**. 오른손 GPIO 순서(revision 2): **19, 21, 23, 26, 33, 22, 32, 25, 16, 17, 4, 27, 13, 14, 18**. 마지막 GPIO18은 양손 모두 엄지 끝이다. 오른손 검지 위/아래는 사용자의 GPIO34/35 대신 출력 가능한 GPIO32/25로 정정했으며 실제 두 선도 옮겨야 한다. [오른손 배선표](RIGHT_HAND_PINMAP.md)를 따른다. 양손 펌웨어와 PC 설정을 함께 갱신한다. CAN TX=GPIO1, RX=GPIO3, 1Mbps, 표준 11비트 CAN이다. 외부 CAN 트랜시버가 필요하다.

배터리 원전압을 ESP32 GPIO나 서보 전원에 직접 연결하지 않는다. 기존에 설계한 전압 변환·모터 전원·보드 전원을 사용한다. 소프트웨어가 전원 용량이나 모터의 과전류 보호를 대신하지는 않는다.

각 스케치 맨 위에 CAN_NODE_ID와 **CONTROL/POSE/PING/PONG/STATUS/STATE/BOOT ID 상수**를 모았다. 기본 노드는 왼손 60, 오른손 61이다. 정상 통합 상태에서는 생성된 ID·GPIO·보정값을 그대로 사용한다.

## PC SDK 설치·실행

로봇 PC에는 `humanoid_sdk-main` 전체를 복사한다. 이름에 V2가 붙은 추가분 폴더만 복사하면 팔 코드가 빠진다. 아래는 프로젝트 전체를 `~/project_humanoid`에 복사한 경우다.

```bash
cd ~/project_humanoid/humanoid_sdk-main/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to bhl_bringup
source install/setup.bash

# 먼저 하드웨어를 열지 않는 실행
ros2 launch bhl_bringup robot.launch.py hands:=both dry_run:=true
```

실물 실행은 팔을 받치고 CAN·전원·종단저항·영점을 확인한 뒤 `dry_run:=false`로 바꾼다. `can0`는 별도로 1Mbps로 올라와 있어야 한다. `robot.launch.py`와 `demo.launch.py` 모두 `hands:=none|left|right|both`, `hand_config:=...`를 받는다. 기본값은 hands=none, dry_run=true다.

팔·왼손·오른손은 각각 별도 프로세스와 CAN 소켓을 사용한다. 함께 실행하면 서로의 ROS 상태를 감시한다. 팔 enable은 요청한 손 노드의 정상 상태를 기다리며, 손 enable은 팔 노드 상태가 있어야 한다. 노드 상태 발행은 모터 enable 전부터 시작하므로 활성화 순서가 순환 대기를 만들지는 않는다.

### 전원 직후 처음 손을 활성화할 때

**PWM을 아직 출력하지 않는 손은 초기 목표를 명시해야 한다.** 아래 open은 사용자가 의도적으로 펼침을 선택하는 명령이다. 실제 서보 위치를 읽을 수 없어 첫 출력은 현재 자세에서 보간할 수 없다. 물체를 잡은 상태에서 아래 open 절차를 무조건 반복하지 않는다.

```bash
ros2 service call /hand_left/open std_srvs/srv/Trigger "{}"
ros2 service call /hand_right/open std_srvs/srv/Trigger "{}"
ros2 service call /hand_left/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /hand_right/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /arm_controller/enable std_srvs/srv/SetBool "{data: true}"
```

**이미 PWM으로 자세를 유지하던 손에 PC SDK만 재접속한 경우에는 open을 먼저 호출하지 않는다.** enable은 PING → HOLD → 마지막 PWM 목표 조회 → RESUME → 같은 목표 전송 순으로 진행한다. 명시적으로 새 목표를 정하지 않았다면 현재 명령을 유지한다. 새 목표가 있으면 읽어 온 명령에서 PC 속도 제한을 거쳐 이동한다. 조회에 실패하면 임의의 펼침 값을 보내지 않는다.

### 목표 보내기

손 토픽은 손마다 15개 당김 비율이며 **관절 각도가 아니다**. 0=해당 모터의 풀림 보정값, 1=당김 보정값이다. 엄지 양쪽 당김인 13·14번 합은 1 이하여야 한다.

```bash
# 의도적인 한 번의 grip 설정: 수동 목표는 갱신하지 않아도 유지
ros2 param set /hand_left grip_level 0.2
ros2 param set /hand_left grip pinch

# 상위 제어 프로그램의 스트림: 예시는 전체 풀림 목표를 50Hz로 송신
ros2 topic pub --rate 50 /hand_left/target std_msgs/msg/Float64MultiArray \
  "{data: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}"
```

스트림 토픽은 **0.5초 동안 새 입력이 없으면 HOLD로 정지**한다. ESP32에 50Hz CAN이 계속 오더라도 상위 프로그램 입력 만료를 따로 검사한다. 수동 parameter/grip 명령은 의도적인 유지 목표이며 같은 TTL을 적용하지 않는다. `/target_us`에는 보정 범위 안의 정수 µs 15개를 보낼 수 있다.

한 손에서 `/target`, `/target_us`, 수동 grip을 동시에 사용하지 않는다. 스트림이 제어권을 가진 동안 다른 입력 경로는 거절한다. 전환하려면 HOLD 후 명시적으로 재개한다. 같은 목표 토픽에 ROS 송신자가 여럿이면 정지하며, 입력 큐는 최신 목표 위주로 depth=1을 사용한다. ROS 그래프 감지는 즉시성·인증 보장이 아니다. 운영상 상위 명령 프로그램도 하나로 둔다.

팔의 `/arm_controller/target`은 JointState의 **position=관절 rad**, `target.<joint>` 파라미터는 **deg**다. 이름·길이·유한값을 세트 전체 검사하고, timestamp가 있으면 오래된 목표를 거절한다. 토픽 스트림도 0.5초 만료 정책을 쓴다. 실제 VLA/IK 운용 전에 고정 motor_zero를 측정하여 `zero: config`와 `require_fixed_zero_for_streaming: true`를 적용한다. 현재 `zero: start`는 벤치 상대 좌표다.

이름 있는 power/pinch/point는 검증된 파지 동작이 아니라 시작용 목표다. 개별 보정 범위와 엄지 제약이 실제 힘·간섭을 보장하지는 않는다.

## 정지와 복구

| 명령·사건 | 현재 동작 |
|---|---|
| 손 HOLD / 상위 입력 만료 | 마지막 PWM 유지, 새 목표 무시, 명시적 재개 필요 |
| 손 OFF | PWM 중지. 다음 활성화에는 새 초기 목표 필요 |
| 손 CAN 목표 1초 단절 | ESP32 자체가 마지막 PWM 유지. 정상 완성 세트 수신 시 복귀하는 기본 동작 유지 |
| SDK 응답 연속 실패·재부팅·잘못된 목표 | 호스트 스트리밍 정지, 고장 표시. 예전 목표 자동 재생 없음 |
| 팔 HOLD / TRACKING | 갱신된 피드백과 마지막 확인 POSITION에 따라 유지. 새 이동은 래치로 차단 |
| 팔 SDO 진단만 실패 | 경고. 그 이유만으로 위치 유지 토크를 해제하지 않음. HOLD 시 기존 목표를 동결 |
| 팔 모드 이탈·피드백 유실 | 이상 관절은 재활성화하지 않음. 설정에 따라 정상 관절도 이동 정지 |
| 팔 E-STOP / disable | 기존 DAMPING 정책 유지. 정지 자세를 고정하는 브레이크가 아님 |
| 팔 컨트롤러 종료 | IDLE 요청. 계속 토크로 지지하는 명령이 아님 |
| 필수 팔·손 노드 상태가 1.5초간 끊김 | 남은 노드가 이동 정지를 래치. 자동 복구 없음 |

팔 HOLD도 힘 부족·기어 미끄러짐·전원 단절 상태에서 낙하를 막는 물리 브레이크가 아니다. 유지 중에도 추종 오차가 다시 크게 벌어지면 해당 관절을 DAMPING으로 전환한다. 명시적 E-STOP을 위치 유지로 임의 변경하지 않았다.

복구는 먼저 상위 목표 송신을 중단하고 실제 원인을 해결한 뒤 수행한다. 손의 `reset_estop`은 PONG과 보정 프로필을 확인한 다음 고장·정지 래치를 해제하며 PWM을 바꾸지 않는다. 팔이 HOLD 상태라면 정상 피드백을 확인하고 resume한다. 이어 손을 resume하고 새 목표를 준다. 상태 토픽 갱신을 확인하며 진행한다.

```bash
ros2 service call /hand_left/reset_estop std_srvs/srv/Trigger "{}"
ros2 service call /hand_right/reset_estop std_srvs/srv/Trigger "{}"
ros2 service call /arm_controller/resume std_srvs/srv/Trigger "{}"
ros2 service call /hand_left/resume std_srvs/srv/Trigger "{}"
ros2 service call /hand_right/resume std_srvs/srv/Trigger "{}"
```

손 자체가 재부팅해 PWM이 꺼졌다면 위 resume만으로 임의 목표가 출력되지 않는다. 새 초기 목표를 선택해야 한다. 팔의 모드 이탈·보드 오류·유실 관절이 남아 있으면 resume은 거절된다. E-STOP 후의 reset/enable, 보드 오류 조치, 재영점 설정은 별도다. 보드 오류를 SDK가 자동 삭제하거나 전류 제한을 높이지 않는다.

손만 따로 시험하려면 `ros2 launch bhl_hand_sdk hand_sdk.launch.py ...`를 사용하고, 팔 연동이 불필요한 독립 노드 실행에서는 `follow_arm_estop:=false`를 노드 파라미터로 지정한다. 팔과 함께 실행할 때는 연동 기본값을 유지한다.

## 팔 SDK에서 보완한 부분

- 모터별 4ms 응답 대기를 합산하는 대신, 같은 버스에 목표를 먼저 보내고 **공통 마감 시간** 안에 응답을 수집한다. 실제 OS·CAN 지연의 상한 보장은 아니며 주기 초과 횟수와 최대 step 시간이 status에 나온다.
- 기존 MODE·ERROR·기어비·전류 상한·게인 읽기 검사를 보존했다. enable 중에도 기존 활성 관절의 heartbeat를 서비스한다.
- 속도 제한에 가속도 제한을 추가하고, 지연 후 큰 dt가 한 번에 큰 목표 이동을 만들지 않게 제한한다. 기본 게인·토크 상한은 유지했다.
- 현재 프로필의 TRACKING 동작은 healthy HOLD다. 고장 종류에 따라 HOLD와 기존 E-STOP/DAMPING을 구분한다. 재개할 때 자동 재영점이나 자동 POSITION 재진입을 하지 않는다.
- 피드백 유효 여부·수신 경과 시간, MODE/ERROR 조회 시각, 운전 중 버스 전압, 제어 주기 통계를 추가했다. 실제 최신 피드백이 없는 관절은 새 arm joint_states 메시지에서 제외한다.
- 팔 각도는 **모터 엔코더 값을 감속비로 환산한 값**이다. 출력 기어가 미끄러지는 경우 실제 팔 각도를 보장하지 못한다. `/joint_states` 집계·RViz 표시만으로 최신성을 판단하지 말고 status의 feedback_valid/feedback_age_s도 본다.
- 같은 OS 사용자가 같은 CAN 노드를 쓰는 SDK 프로세스를 중복 실행하는 것을 로컬 파일 잠금으로 차단한다. 외부 기기나 임의 CAN 송신자를 인증하는 기능은 아니다. 별도 SDO 설정 프로그램과 동시 운전하지 않는다.

## 로그와 재생성

로봇 PC에서 다음 읽기 전용 기록기를 별도 실행하면 팔·양손 status가 시간과 함께 순환 로그로 저장된다. 기본 5MB×4개 파일이다. 제어 프로세스에서 파일을 쓰지 않는다.

```bash
ros2 run bhl_arm_sdk arm_trace
# 기본: ~/.local/state/bhl/upper_body.jsonl
```

보정값을 바꿀 때는 원본 JSON을 고친 뒤 아래 도구로 **ESP32 코드와 PC YAML을 함께** 생성한다. 받은 ZIP의 make_sketches.py는 사용하지 않는다.

```bash
cd ~/project_humanoid/dexhand_control
python3 tools/generate_board_config.py
python3 tools/prepare_firmware.py
python3 tools/generate_sdk_config.py
python3 tools/sync_release_docs.py
python3 tools/verify.py
```

검증 도구는 CAN을 열지 않는다. pytest, python-can, PyYAML과 C++17 컴파일러(g++/clang++ 또는 Python ziglang)가 필요하다. ROS 메시지 전달과 실물 동작 검사는 별도로 필요하다.

## 확인 결과와 남은 실측

2026-11-04 오프라인 검증: **Python 179개 + 68개 subtest 통과**, 네이티브 수신기·제어기 검사와 응답 검사 17개 사례 통과. SDK 0.2.0의 통합 트랜잭션·실제 노드 callback 연결(ROS 대체 객체)·고정 영점 도구 검사도 포함한다. Python SDK가 **양손의 실제 C++ 펌웨어 클래스**와 부팅·명령·재접속·거부 프레임을 주고받는 검사도 포함한다.

양손 스케치 모두 Arduino-ESP32 **3.3.12 / esp32:esp32:esp32** 컴파일 성공. 각각 flash **276,471 B / 1,310,720 B(21%)**, 전역 **22,544 B / 327,680 B(6%)**. 업로드·실제 CAN·실제 ROS 2 노드 실행·모터 동작은 이 작업에서 수행하지 않았다.

다음은 자료나 실측이 필요하여 자동 적용하지 않았다.

- DexHand·손목·브래킷 질량/무게중심에 따른 중력 보상과 게인·연속 토크 검증.
- 기어·축·인서트 미끄러짐, 배선 전압 강하, 모터 발열, 실제 전류·보드 22A 설정 확인.
- 고정 영점 측정 및 로봇 기하 모델로 검증한 몸통·좌우 팔·손의 전신 충돌 회피.
- 손가락 실제 위치·힘 피드백, 손목 SCS2332 구동.

따라서 이 보완본은 코드상 결함과 제어 인터페이스를 개선한 결과이며, 실제 팔의 드르륵거림·낙하가 해결됐다는 실기 판정은 아니다.
