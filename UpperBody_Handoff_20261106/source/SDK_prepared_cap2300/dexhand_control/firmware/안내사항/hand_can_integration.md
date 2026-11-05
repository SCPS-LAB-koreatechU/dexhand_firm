# SDK에서 팔과 DexHand를 함께 제어하는 가이드

> 2026-11-04 이후의 현재 코드는 [보완본 안내](CURRENT_SETUP_20261104.md), [프로토콜 3 명세](PROTOCOL_v3.md)를 따릅니다. 이 문서는 이전 버전의 설명 또는 비교·검증 기록입니다.

작성일: 2026-11-03. 대상은 Berkeley Humanoid Lite V1 양팔 10개 관절과 DexHand 양손이다. 왼손과 오른손 ES3352 15개씩의 PC 프로필과 별도 업로드 스케치 `DexHandLeft`, `DexHandRight`가 있다. GPIO 배선 확인·실물 업로드/검증과 SCS2332 손목 구현이 필요하다.

## 1. 동시 제어에 대한 현재 상태

**팔과 손이 같은 1Mbps CAN 버스를 공유하도록 ID와 프로토콜을 구분했다. SDK의 손 ROS 노드·launch·공통 정지 연결은 아직 구현되지 않았다.** 이 문서는 그 통합을 위한 구현 계약이다. 별도 폴더의 `HandSender`와 ESP32 펌웨어는 준비되어 있다.

| 항목 | 현재 상태 |
|---|---|
| 팔 노드 1~10, `can0`, 50Hz 위치 제어 | 구현됨 |
| 왼손 노드 60, PWM 목표 5프레임 송신 클래스 | 구현됨: `dexhand_control/pc/hand_can.py` |
| ESP32의 프레임 조립·검사·PWM 15채널 출력 | 구현·컴파일 검증됨 |
| 오른손 노드 61 | 사용자 보정값의 `profiles/right_hand.json`과 `firmware/DexHandRight` 준비됨. 실물 배선 확인·업로드 필요 |
| 팔+손 ROS 노드 동시 실행 launch | 추가 구현 필요 |
| 팔·손 공통 명령 소유권·정지 경로 | 추가 구현 필요 |
| 양팔·양손 동시 실물 운전·지연 측정 | 미검증 |
| SCS2332 손목 | CAN 명령 및 ESP32 UART 반이중 처리 추가 필요 |

`robot.launch.py model:=dexhand`는 URDF 모델을 선택한다. 이것만으로 손 송신 드라이버가 실행되는 것은 아니다. 기존 `joint_state_publisher`가 손 관절에 채우는 0은 손 실측값이 아니다.

## 2. 권장 실행 구조

```text
GUI / 동작 시퀀스 / VLA
         ↓ 공통 동작 계획: 팔 목표 + 손 목표 + 실행 시각
전체 동작 관리자 (명령 소유권, 시작/정지, 궤적·시간 관리)
         ├─ 기존 arm_controller 프로세스
         │     └─ 전용 RecoilBus 소켓 → can0 → 팔 모터 1~10
         │
         ├─ 새 hand_left_controller 프로세스 (50Hz)
         │     └─ 전용 송신 소켓 → can0 → ESP32 노드 60 → PWM 15개
         │
         └─ 새 hand_right_controller 프로세스 (50Hz, 오른손 준비 후)
               └─ 전용 송신 소켓 → can0 → ESP32 노드 61 → PWM 15개

물리 CAN: PC USB-CAN ── 팔 모터들 ── 왼손/오른손 CAN 트랜시버
```

새 손 노드는 같은 `can0`에 별도 SocketCAN 소켓을 연다. 기존 팔 `RecoilBus` 객체나 그 `recv()`를 다른 실행 스레드와 공유하지 않는다. 팔 응답은 팔 프로세스가 전담한다. 같은 인터페이스를 여러 소켓에서 사용하는 것은 [Linux SocketCAN이 지원하는 구조](https://docs.kernel.org/networking/can.html)다.

현재 팔은 10개 PDO 교환을 순서대로 수행한다. 모두 무응답이면 대기만 최대 약 `10×4ms=40ms`이며, 상태 조회 대기와 실행 오버헤드가 더해진다. 그 뒤에 손 송신을 붙이면 손도 늦어진다. 손 송신은 별도 프로세스/루프로 운전한다. 동일 프로세스에 배치할 경우 실행기·콜백 그룹·송신 잠금까지 별도 설계해야 하므로 초기 통합은 프로세스 분리를 권장한다.

동시에 실행된다는 것은 팔과 손이 병행하여 명령을 받는다는 의미다. 하나의 CAN 선에서는 프레임이 순서대로 전송된다. 현재 프로토콜은 모든 팔·양손이 동일한 하드웨어 시각에 목표를 적용하는 기능을 제공하지 않는다.

## 3. CAN ID와 데이터 형식

표준 11비트 Classical CAN, 1Mbps. CAN ID는 `(function << 7) | node_id`이다.

| 대상 | 노드 | 제어 ID | 목표 ID | 목표 내용 |
|---|---:|---:|---:|---|
| 팔 | 1~10 | 0x001~0x00A | 0x301~0x30A | 기존 PDO2: float32 위치·속도 |
| 왼손 ESP32 | 60 | 0x03C | 0x33C | 순번·그룹·uint16 PWM 3개 |
| 오른손 ESP32 | 61 | 0x03D | 0x33D | 오른손 프로필/스케치, 실물 업로드 필요 |

팔 목표는 기존 SDK가 각도를 모터 단위로 변환한다. 손 목표는 PC가 실제 PWM 펄스폭 µs로 변환한다. 각 손가락 모터를 독립 CAN 노드로 취급하지 않는다. ESP32 하나가 손 전체를 담당한다.

손을 `bhl_arms.yaml` 관절 목록에 추가하지 않는다. 손은 팔의 Ping/SDO/PDO 응답을 제공하지 않으므로 기존 팔 활성화 절차로 구동되지 않는다. 팔의 Mode 열거형을 손 동작 의미로 해석하지 말고 `HandSender.start/hold/resume/off`를 사용한다.

## 4. SDK 손 입력 계약 — 새 노드에 구현할 규칙

아래 인터페이스는 **제안이며 현재 ROS 토픽·서비스로 등록되어 있지 않다.**

| 이름 | 형식 | 내용 |
|---|---|---|
| `/hand_left/target_us` | `std_msgs/msg/UInt16MultiArray` | 모터 1~15 순서의 정수 µs 15개 |
| `/hand_right/target_us` | 동일 | 오른손 프로필 및 실제 배선·펌웨어 준비 후 제공 |
| `/hand_left/hold` | `std_srvs/srv/Trigger` | 주기 송신 정지, CAN HOLD, SDK 정지 상태 유지 |
| `/hand_left/resume` | 동일 | 새 목표 승인 후 CAN RESUME 및 주기 송신 허용 |
| `/hand_left/off` | 동일 | 주기 송신 정지, CAN OFF |
| `/hand_left/status` | `std_msgs/msg/String` JSON | SDK 명령·송신 시각·오류·정지 상태 |
| `/whole_body/estop` | `std_srvs/srv/Trigger` | 팔과 손 모두에 정지 요청하는 상위 서비스 |

오른손 서비스도 같은 규칙으로 별도 제공한다. 배열 메시지에는 시각이 없으므로 수신 시 SDK 내부 단조 시각을 기록한다. 다관절 동작의 예약 실행은 상위 동작 관리자가 담당한다. 실제 구현에서 이름 기반 입력이 필요하면 별도 명시적 메시지로 확장하며 순서를 추측하지 않는다.

손 배열 순서는 소지 위/아래, 약지 위/아래, 중지 위/아래, 검지 위/아래, 검지 끝, 중지 끝, 약지 끝, 소지 끝, 엄지 오른쪽 당김, 엄지 왼쪽 당김, 엄지 끝이다. 핀·풀림·당김 값은 `dexhand_control/profiles/left_hand.json`, `right_hand.json`이 원본이다. 오른손 기존 시험 번호는 검지 1·2~소지 7·8이므로 CAN 배열 순서와 다르다. `right_hand.json`의 `calibration_motor_id`와 `RIGHT_HAND_PINMAP.md`에 대응을 보존했다.

입력을 받을 때 길이 15, 정수 여부, 모터별 범위를 검사한다. 범위를 벗어나면 세트 전체를 거절한다. PC가 0..1 당김 비율을 사용한다면 `fractions_to_us()`로 변환한다. 이 값은 힘줄 서보의 명령 범위 비율이며 실제 손가락 관절각과 동일하지 않다. VLA의 라디안 출력을 그대로 넣지 않는다.

## 5. 매 20ms 송신 규칙

1. 손 노드는 해당 손의 유일한 송신 소유자가 된다. GUI·VLA·시험 스크립트는 손 CAN에 직접 동시에 송신하지 않는다.
2. 입력 콜백은 최신 목표를 교체한다. 과거 목표를 무제한 FIFO에 쌓지 않는다.
3. 독립적인 50Hz 루프가 PC 궤적에서 현재 실행 시각의 PWM 목표 15개를 계산한다.
4. 같은 순번의 5개 프레임을 `write_us()` 한 번으로 연속 전송한다. 프레임 사이에 20ms sleep을 넣지 않는다.
5. 다음 주기에 순번을 증가시킨다. 동일 자세를 유지할 때도 새 순번으로 보낸다. 같은 순번 반복은 ESP32 watchdog을 갱신하지 않는다.
6. 루프가 밀리면 과거 주기를 몰아서 재생하지 않는다. 현재 시각으로 진행하여 최신 궤적 목표를 보낸다.
7. 전송 실패 시 해당 손을 SDK 송신 오류 상태로 표시한다. 불완전 세트를 다른 목표와 같은 순번으로 재사용하지 않는다. `HandSender`는 실패해도 순번을 소비하도록 구현되어 있다.

`start()`는 새 송신 세션 시작 시 한 번 호출한다. 매 tick 호출하면 PWM과 순번이 반복 초기화된다. 정상 운전 중에는 `write_us()`만 반복한다.

현재 `PythonCanTransport`는 프레임별 `send(timeout=0.02)`를 사용한다. 5개 송신이 항상 20ms 안에 끝난다는 보장은 없다. 실제 노드에서는 짧은 프레임 송신 타임아웃과 세트 전체 마감을 구현하고, 큐 적체 시 오래된 세트를 계속 보내지 않도록 한다. 예를 들어 2ms 프레임 대기를 초기 검토값으로 둘 수 있으나 USB-CAN 드라이버에서 실측 후 조정한다. ESP32 조립 마감은 첫 프레임부터 20ms이다.

버스 송신 큐에 이미 들어간 프레임은 최신 목표 변수만 교체해서 취소되지 않는다. 큐를 과도하게 늘려 지연을 숨기지 않는다. 혼잡·버스 오류는 중단 사유와 지연으로 기록한다.

## 6. 기존 송신 클래스 호출 예제

실제 ROS 노드가 호출할 함수 예제다. 이 함수 정의만으로 모터를 구동하지 않으며, `open_hand()` 등을 호출하면 실제 CAN 연결/명령이 발생한다.

Linux 배치 경로에서 `dexhand_control/pc`를 모듈 경로에 추가한다.

```bash
export DEXHAND_CONTROL_DIR=/실제/경로/project_humanoid/dexhand_control
export PYTHONPATH="$DEXHAND_CONTROL_DIR/pc:$PYTHONPATH"
```

```python
import can
from hand_can import HandSender, PythonCanTransport, load_profile, validate_pulses

def open_hand(profile_path, channel="can0"):
    profile = load_profile(profile_path)
    # Arm controller owns a different socket on the same physical interface.
    bus = can.Bus(interface="socketcan", channel=channel,
                  bitrate=1_000_000, receive_own_messages=False)
    sender = HandSender(PythonCanTransport(bus), profile)
    return bus, sender

def begin_hand_session(sender, approved_first_us):
    # Approve the first complete pose before enabling this session's stream.
    pulses = validate_pulses(sender.profile, approved_first_us)
    sender.start()
    sender.write_us(pulses)

def hand_tick(sender, current_trajectory_us):
    # Call from the independent 50 Hz hand worker, after checking its stop latch.
    sender.write_us(current_trajectory_us)

def hold_hand(sender):
    # The worker must first inhibit future ticks; serialize this with hand_tick.
    sender.hold()
```

위 코드는 송신 API 호출 예제다. 정지 상태 유지, 입력 유효 시간 확인, 프레임·세트 송신 마감 처리는 실제 손 노드에서 추가해야 한다.

## 7. 팔과 손의 동작 시간 맞추기

상위 동작 관리자는 하나의 동작 계획에 팔 목표, 손 목표, 시작 시각과 지속 시간을 함께 둔다. 각 제어기는 같은 실행 시간축에서 자기 목표를 꺼내 실행한다. 이 방법으로 팔을 뻗으며 손을 펴고, 물체 근처에서 손을 닫는 순서를 조정한다.

팔과 손의 목표를 ROS 토픽으로 연달아 한 번 보냈다고 정확한 동시 시작이 보장되지는 않는다. 현재 손 CAN 패킷의 순번은 세트 조립용이며 실행 시각이 아니다. 엄격한 적용 시각 동기화가 필요해지면 팔·손 양쪽 프로토콜과 펌웨어가 시간 예약을 지원하도록 확장해야 한다.

VLA 추론은 50Hz 송신 루프와 분리한다. 새 계획이 없는 동안 승인된 궤적 구간은 계속 실행할 수 있다. 입력 유효 시간은 VLA 주기와 궤적 구간 길이에 맞춰 정하고, 유효 시간이 지난 목표를 새 CAN 순번으로 무한 반복하여 상위 제어 단절을 숨기지 않는다. 유효 구간이 끝나면 SDK가 HOLD 상태로 전환하고 명시적 재개를 요구하도록 설계한다.

## 8. 팔·손 공통 정지 처리

현재 `/arm_controller/estop`만 호출하면 손 HOLD/OFF는 전달되지 않는다. 상위 동작 관리자는 다음 절차를 구현해야 한다.

1. 전체 정지 상태를 먼저 고정하고 GUI/VLA의 새 목표 승인을 차단한다.
2. 각 손 worker에서 미래 주기 송신을 막고, 진행 중 호출과 직렬화하여 HOLD를 보낸다.
3. 동시에 기존 팔 estop 서비스를 요청한다. 팔의 기존 동작은 DAMPING이며 중력에 대한 자세 유지를 보장하지 않는다.
4. 장치별 요청 결과·전송 오류·최종 명령 시각을 기록한다. 일부 정지 요청 실패를 전체 성공으로 표시하지 않는다.
5. 재개 시 이전 목표와 오래된 궤적을 버리고 새로운 자세를 승인한다. 정지 해제와 동작 재개는 구분한다.

팔 상태의 `estop`, `TRACKING`, `LOST`, `MODE_LOST`와 손 송신 오류를 상위 관리자에서 감시하고 전체 정지 정책으로 연결한다. 어떤 정지 원인이 다른 팔·손까지 정지시킬지는 정책으로 명시해야 한다. 초기 통합에서는 이러한 구동 실패를 전체 동작 중단 사유로 다루는 것을 권장한다.

손 HOLD는 마지막 PWM 유지다. OFF는 PWM 중단이며 서보 전원 차단이나 토크 해제 보장은 아니다. ESP32에 응용 계층 응답이 없으므로 SDK는 HOLD가 실제 적용됐다고 확인할 수 없다. 공통 정지는 소프트웨어 요청이며 하드웨어 비상정지 동작을 뜻하지 않는다.

PC가 사라지면 각 ESP32는 1초 뒤 마지막 PWM을 유지한다. 수신 타임아웃 이후 새 완성 목표는 자동 적용된다. 따라서 SDK 쪽 정지 래치와 송신 소유권 검사가 필요하다.

## 9. CAN 대역폭 산정

계산 가정: 양팔 10개·양손 각각 15개 PWM 서보, 모두 50Hz, 1Mbps, 정상 프레임만 계산. 손목 프레임·초기 설정·진단 도구·오류 재전송은 제외한다. 모든 프레임을 간격 포함 135bit로 근사하는 보수적인 계산을 사용한다.

| 트래픽 | 프레임/초 | 근사 버스 점유율 |
|---|---:|---:|
| 팔 PDO2: 10개 × 요청/응답 2개 × 50Hz | 1000 | 13.50% |
| 양손: 2개 × 5프레임 × 50Hz | 500 | 6.75% |
| 팔 heartbeat: 10개 / 0.2초 | 50 | 0.675% |
| 팔 상태 SDO: 주기당 1회, 요청/응답 | 100 | 1.35% |
| 합계 | 1650 | 약 22.3% |

짧은 heartbeat도 135bit로 계산하여 실제 정상 점유율보다 크게 잡았다. 이는 용량 검토용 계산이며 실제 지연·데드라인의 보장은 아니다. 팔 목표 ID가 손 목표 ID보다 낮아 버스 경쟁 시 우선될 수 있다. USB-CAN 큐와 트래픽 집중 구간도 실제 수신 지연에 영향을 준다.

평균 대역폭 기준으로 한 버스 공유를 검토할 수 있는 구성이다. 한편 PC가 팔 응답을 기다리는 시간은 버스 점유율에 비례하지 않으므로 별도 손 루프가 필요하다. `can0`를 운전 중 down/up하거나 비트레이트를 재설정하지 않는다. 단일 CAN 버스는 통신 장애 영향도 공유하므로 실측 결과에 따라 손을 별도 CAN 인터페이스로 분리할 수 있다.

## 10. 통합 시 추가할 파일과 검증

추가 구현 대상은 다음과 같다. 아래 경로는 현재 존재하는 패키지로 오해하지 않도록 제안으로 표시한다.

```text
ros2_ws/src/bhl_hand_sdk/         # 새 패키지: 손 노드, 설치 가능한 hand_can, 프로필
  bhl_hand_sdk/node.py
  bhl_hand_sdk/controller.py     # 최신 목표, 궤적, 정지 래치, 송신 마감
  config/left_hand.json
  config/right_hand.json        # 기존 dexhand_control 오른손 프로필을 설치

ros2_ws/src/bhl_bringup/
  launch/robot.launch.py        # 손 노드 선택 실행 인자 추가
  ...                          # 전체 동작 관리자/공통 정지 구현 위치 결정
```

배포 시 `PYTHONPATH` 수동 설정에 의존하기보다 `hand_can`과 프로필을 새 ROS 패키지에 설치한다. 모터 보정·핀 설정 원본과 펌웨어 빌드의 프로필 SHA256 일치도 기록한다. `robot.launch.py`에는 `hand_left`, `hand_right`, 각 프로필·채널 인자를 추가하고 전역 dry_run을 모든 드라이버에 전달한다. 한 노드만 실물 송신하는 혼합 dry_run을 기본 동작으로 만들지 않는다.

실물 운전 전에 가상 CAN에서 팔·왼손·오른손 ID 분리, 팔 무응답 중 손 송신 주기, 송신 실패·큐 적체·구간 만료 시 HOLD, 정지 후 목표 차단, 두 송신자 중복 실행 차단을 확인한다. 실물에서는 CAN 프레임 시각, 목표 세트 완료 간격, 전체 세트 수신 마감 초과, PWM 파형과 실제 동작을 따로 측정한다.

손 상태에는 `commanded_us`, 마지막 큐 등록 시각, 입력 유효 시간, 송신 오류 수, 정지 상태를 표시한다. `HandSender.write_us()` 반환은 손에서의 적용 확인이나 실제 위치 측정이 아니다. 손 명령을 실측 `/joint_states`로 발표하지 않는다. 현재 상태 응답이 없는 ESP32의 연결 상태는 미확인으로 표시한다.

이 문서 작성에서는 SDK 런타임 코드·launch를 변경하거나 실제 CAN을 송신하지 않았다. 기존 손 송신 예제·상세 패킷 규약은 같은 폴더의 [SDK 송신 코드 사용법](SDK_INTEGRATION.md)과 [CAN 프로토콜](PROTOCOL.md)을 참고한다.
