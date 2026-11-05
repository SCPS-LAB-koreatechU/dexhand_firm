# 휴머노이드 PC SDK에서 손에 명령 보내기

> 2026-11-04 이후의 현재 코드는 [보완본 안내](CURRENT_SETUP_20261104.md), [프로토콜 3 명세](PROTOCOL_v3.md)를 따릅니다. 이 문서는 이전 버전의 설명 또는 비교·검증 기록입니다.

2026-11-03: **[팔·양손 동시 제어 통합 가이드](hand_can_integration.md)**를 추가했습니다. 같은 CAN 버스를 공유하는 ID·패킷 구성은 준비되어 있으며, 손 ROS 노드·launch·공통 정지 연결은 추가 구현 대상입니다. 아래는 현재 송신 클래스의 사용법입니다.

대상 SDK는 같은 상위 폴더의 `humanoid_sdk-main/ros2_ws/src/bhl_arm_sdk`입니다. 기존 SDK는 팔 ID 1~10을 `RecoilBus`로 제어하며 양팔 `can0`, 1Mbps, 50Hz를 사용합니다. 손 송신 코드는 이 CAN 버스에 표준 ID 0x33C의 손 전용 데이터를 추가합니다.

## 역할

PC의 GUI/VLA/동작 코드 → PC 손 제어기(궤적·속도·조합 제한) → `HandSender` → CAN → ESP32 → PWM.

ESP32는 전원으로 자동 실행하고 첫 정상 목표에 자동 출력합니다. USB/시리얼 연결은 운전에 필요하지 않습니다. PC는 15개 모터의 실제 µs 값만 보내면 됩니다. VLA가 사용하는 값은 이름·순서·단위가 정해진 SDK 명령으로 변환한 후 전송합니다.

## 의존성

이 폴더에서 실행합니다.

```bash
python3 -m pip install -r requirements.txt
```

SocketCAN `can0`는 기존 SDK와 동일하게 1Mbps로 활성화되어 있어야 합니다. 기존 SDK의 `bhl_bringup/scripts/can_up.sh can0` 또는 이미 설정한 systemd-networkd 구성을 사용할 수 있습니다. 팔이 구동 중인 버스를 재설정하는 명령은 실행하지 않습니다.

## 기존 RecoilBus에 연결 — API 호환 예제

아래 방식은 `send()` 형식의 호환성을 보여 줍니다. 팔·손 동시 운전의 권장 구성은 다음 절의 **별도 손 노드·별도 SocketCAN 소켓**입니다. 기존 `RecoilBus`는 PDO/SDO 응답 대기 중 잠금을 유지하므로 그 객체를 공유하면 손 송신도 지연될 수 있습니다. 팔 수신 소켓을 다른 스레드에서 같이 읽지 않습니다.

프로젝트를 SDK 패키지에 설치한 상태는 아니므로, 우선 이 폴더의 `pc`를 Python 모듈 검색 경로에 추가합니다. Linux에서 실제 배치 경로에 맞춰 지정합니다.

```bash
export DEXHAND_CONTROL_DIR=/실제/경로/project_humanoid/dexhand_control
export PYTHONPATH="$DEXHAND_CONTROL_DIR/pc:$PYTHONPATH"
```

이미 SDK가 쓰는 `RecoilBus` 객체를 전달하는 예입니다.

```python
from hand_can import HandSender, load_profile

profile = load_profile("/실제/경로/dexhand_control/profiles/left_hand.json")
hand = HandSender(existing_recoil_bus, profile)
hand.start()  # 송신 세션 시작 때 한 번. 이 손의 출력·순번을 초기화.

# PC 제어기의 20ms tick에서 호출. 반드시 모터 1~15 순서의 완전한 목표.
hand.write_us(current_target_us)

# PC가 0..1 목표를 사용하는 경우, 캘리브레이션 변환도 PC에서 처리 가능.
hand.write_fractions(current_target_fractions)

# 정지 경로 중 목적에 맞는 하나를 호출.
hand.hold()    # 마지막 출력 유지, 새 목표 차단
hand.resume()  # 마지막 출력 유지, 새 목표 허용
hand.off()     # PWM 출력 중단
```

예제의 `existing_recoil_bus`, `current_target_us`, `current_target_fractions`는 SDK 제어기가 제공하는 객체·목표입니다. 두 write 메서드는 대안이며 매 tick에 둘 다 호출할 필요는 없습니다. 실제 운영에서는 하나를 선택합니다.

`HandSender`는 `send()`만 사용합니다. 손에 `ping()`, `read_f32()`, `exchange_pdo_2()`를 호출하지 않습니다. 이 펌웨어는 응답하지 않고, 손 PDO의 데이터는 팔의 `<ff>`가 아니라 `<BBHHH>`이기 때문입니다. 손을 `bhl_arms.yaml`의 팔 관절 목록에 추가하면 팔 활성화 로직에서 `NO_REPLY`가 됩니다.

## 별도 손 노드로 SocketCAN 연결

팔과 분리된 ROS 2 손 노드를 추가할 때는 별도 SocketCAN 소켓으로 같은 `can0`에 송신할 수 있습니다.

```python
import can
from hand_can import HandSender, PythonCanTransport, load_profile

bus = can.Bus(interface="socketcan", channel="can0", bitrate=1_000_000)
hand = HandSender(PythonCanTransport(bus), load_profile())
hand.start()
# 손 노드의 50Hz 타이머: hand.write_us(target_us)
# 종료 시: hand.hold(); bus.shutdown()
```

실제 ROS 노드와 launch 추가는 아직 수행하지 않았습니다. 이 폴더에는 SDK가 호출할 수 있는 송신 클래스와 독립 실행 예제가 들어 있습니다.

손 노드는 기존 `arm_controller`의 단일 실행 루프와 분리하여 운전합니다. 같은 `can0`를 사용하는 것과 같은 Python 버스 객체를 공유하는 것은 별개의 선택입니다. 현재 송신 클래스의 프레임별 대기 시간 20ms만으로는 전체 세트의 20ms 수신 마감을 보장할 수 없습니다. 통합 노드에서는 짧은 프레임 타임아웃·세트 전체 마감·큐 적체 처리도 구현해야 합니다.

## 독립 송신 예제

먼저 장치를 열지 않고 데이터를 확인합니다. 예제 자세는 사용자의 풀림 값입니다.

```bash
python3 pc/send_hand.py --targets examples/left_open_us.json
```

실제 전송은 `--send`를 추가합니다. 다음 명령은 첫 완성 자세부터 모터를 움직일 수 있습니다. 2초간 50Hz 전송 후 HOLD로 끝납니다.

```bash
python3 pc/send_hand.py --channel can0 --targets examples/left_open_us.json --seconds 2 --send
```

`--targets` 파일은 모터 1~15 순서의 정수 15개 JSON 배열입니다. 개별 모터 한계를 벗어나면 첫 프레임을 보내기 전에 거부합니다. 실제 모터 속도 제한·궤적 계산은 PC 손 제어기에서 수행합니다. 송신 클래스는 목표를 그대로 인코딩하며 자동으로 부드럽게 이동시키지 않습니다.

## 전체 시스템에 연결할 때

- 팔 제어의 비상정지·TRACKING 오류·노드 종료가 손의 HOLD/OFF에도 전달되도록 공통 정지 경로를 추가해야 합니다. 기존 팔의 estop 서비스만 호출하면 이 손이 자동 정지하는 구조는 아닙니다.
- 팔의 10개 PDO 응답 대기는 무응답 시 주기가 늘어날 수 있습니다. 손 송신은 별도 노드/실행 루프로 운전하거나 공통 스케줄러에서 지연을 관리합니다.
- GUI와 VLA 중 누가 명령을 생성하는지 PC에서 선택하고, 동일 손에 여러 송신 세션이 동시에 개입하지 않게 합니다.
- PC의 50Hz 실행은 VLA 추론과 분리합니다. 과거 목표를 큐에 길게 쌓지 않고 최신 실행 시점의 목표를 보냅니다.
- 처음 연결할 때 출력 펄스는 첫 목표를 직접 적용합니다. 실제 모터 위치를 읽지 못하므로 현재 위치에서의 자동 초기 보간을 가정하지 않습니다.
- 이 펌웨어는 CAN 상태 응답과 손 실측값이 없습니다. SDK의 손 명령값을 실측 `/joint_states`로 위장하지 않습니다. 관측/학습 로그에서는 `commanded_us`와 실제 측정값을 구분합니다.
- 오른손은 `profiles/right_hand.json`의 노드 61과 업로드 스케치 `firmware/DexHandRight/DexHandRight.ino`를 사용합니다. 왼손은 노드 60과 `DexHandLeft`입니다. 사용자 제공 보정값과 같은 위치별 GPIO를 적용했습니다. 기존 모터 번호와 CAN 순서는 `RIGHT_HAND_PINMAP.md`에서 확인합니다. 실제 배선 확인·보드 업로드는 별도 수행해야 합니다.
