# bhl_hand_sdk — 양손 ESP32 CAN 제어

[손 GUI: 개별 모터·카메라·캘리브레이션](../../../../dexhand_control/firmware/안내사항/HAND_GUI.md). 프로젝트의 `dexhand_control`에서 `bash gui_hand.sh`를 실행합니다. 설치된 `hand_gui` 명령도 지원하며 `--project`로 전체 프로젝트 루트를 지정해야 합니다.

간단한 단독 테스트: [CAN·USB 카메라 테스트 가이드](../../../../dexhand_control/firmware/안내사항/HAND_TEST_QUICKSTART.md). ROS 없이 `dexhand_control/test_hand.sh`로 ping → 제한된 접기/펴기 → USB 카메라 따라 하기를 실행합니다. 설치된 패키지의 `hand_test` 명령도 같은 기능입니다.

SDK 0.2.0: [양팔·양손 공통 명령](../bhl_upper_body_sdk/README.md). 통합 모드에서는 아래 개별 target/grip 입력을 차단하고 UpperBodyClient를 사용합니다. status의 last_applied_command는 응답받은 PWM과 원래 PC 명령 식별값이며 실제 위치가 아닙니다. measured_position/physical_reached는 null입니다.

현재 보완본: **2026-11-04 / CAN 프로토콜 3**. 팔 SDK와 같은 can0에서 손마다 별도 프로세스·소켓을 사용합니다. ES3352 15개이며 SCS2332 손목은 포함하지 않습니다.

- [현재 설치·업로드·팔/손 동시 실행·복구 절차](../../../../dexhand_control/firmware/안내사항/CURRENT_SETUP_20261104.md)
- [PC 개발자용 프로토콜 3](firmware/PROTOCOL_v3.md)
- 설정: `config/dexhand.yaml` (원본은 dexhand_control/profiles의 JSON)

## 인터페이스

| 입력/출력 | 형식 | 의미 |
|---|---|---|
| `~/target` 입력 | std_msgs/Float64MultiArray | 15개 당김 비율 0..1, 스트림 TTL 0.5초 |
| `~/target_us` 입력 | std_msgs/UInt16MultiArray | 15개 µs, 보정 범위 및 엄지 제약, 스트림 TTL 0.5초 |
| grip / grip_level 파라미터 | 문자열 / 실수 | 의도적인 유지 목표, grip_level=0..1 |
| `~/enable` | std_srvs/SetBool | true: 확인·동기화 후 스트리밍, false: HOLD |
| `~/hold`, `~/resume`, `~/off` | std_srvs/Trigger | 유지·명시적 재개·PWM 중지 |
| `~/estop`, `~/reset_estop` | std_srvs/Trigger | 래치 정지, 통신/프로필 확인 후 고장 인정 |
| `~/ping`, `~/clear_flags`, `~/open` | std_srvs/Trigger | 조회·이력 플래그 해제·명시적 펼침 목표 |
| `~/delivery` 출력 | std_msgs/String JSON | 세트별 APPLIED / NO_REPLY / REJECT 등 |
| `~/command_us` 출력 | std_msgs/UInt16MultiArray | 명령 PWM, 실제 위치가 아님 |
| `~/status` 출력 | std_msgs/String JSON | 상태·고장·최근 응답·부팅 식별·통계 |
| `/diagnostics` 출력 | diagnostic_msgs/DiagnosticArray | 상태 진단 |

초기 PWM이 꺼진 손은 사용자가 초기 목표를 정해야 enable됩니다. 이미 PWM이 켜진 손은 protocol 3의 마지막 명령 스냅샷을 읽어 같은 목표에서 재개합니다. 새 프로세스가 임의로 펼침을 출력하지 않습니다. 첫 냉기동의 실제 위치는 모르므로 첫 목표까지의 실제 자세 보간은 불가능합니다.

손은 실제 관절 각도·힘을 보고하지 않습니다. APPLIED는 ESP32가 검사를 통과한 PWM 목표를 기록했다는 의미입니다. 이름 있는 grip은 실물 검증 전 시작용 목표입니다.

## Python API

```python
from bhl_hand_sdk.config import load_config
from bhl_hand_sdk.controller import HandController

config = load_config('/path/to/dexhand.yaml')
hand = HandController(config, 'left', dry_run=True)  # 실물은 준비 후 false
hand.set_grip('open')  # 냉기동에서 의도적으로 정하는 초기 목표
result = hand.enable()
assert result.ok, result.message
hand.set_grip('pinch', 0.2)
# PC 주기 루프에서 hand.step(actual_dt)를 호출합니다.
# 종료할 때 hand.close(): 유지 후 소켓 해제
```

이미 물체를 잡은 손에 재접속할 때는 위의 set_grip('open')을 반복하지 말고 초기 목표를 미지정한 enable을 사용합니다. 실제 버스에서는 한 손에 한 드라이버만 실행합니다. 로컬 CAN 노드 잠금과 ROS 송신자 중복 감지는 운영 충돌 방지 기능입니다.

`set_goal(values, streaming=True, source='planner')`는 스트리밍 입력 만료와 입력 경로 제어권을 함께 사용합니다. ROS 목표 토픽은 자동으로 이 경로를 사용합니다. CLI와 ROS 노드를 같은 손에 동시에 실행하지 않습니다.

팔과 함께 robot.launch.py hands:=both로 실행하면 양방향 고장·상태 만료 연동을 사용합니다. 수신한 고장이나 연결 복구만으로 움직임을 자동 재개하지 않습니다. 상세 복구 순서는 통합 안내를 따릅니다.

CLI: `ros2 run bhl_hand_sdk hand_cli left ping`, `hand_cli left grip pinch --level 0.2`, `hand_cli left hold` 등. 실물 cold grip은 사용자가 지정한 첫 목표를 바로 적용할 수 있으므로 첫 동작을 의도해서 선택합니다. `--dry-run`을 붙이면 CAN을 열지 않습니다.

## 검증

프로젝트의 `dexhand_control/tools/verify.py`가 프로필 일치, 양손 C++ 수신기·상태기, Python SDK와 실제 C++ 클래스 간 프레임 왕복을 검사합니다. 실제 ROS 2 실행과 물리 CAN/모터 시험은 별도입니다.

오른손 손가락 배선 revision 2: GPIO 순서 19,21,23,26,33,22,32,25,16,17,4,27,13,14,18. 검지 위/아래는 GPIO32/25이며 GPIO34/35는 PWM 출력에 사용할 수 없습니다. 양손 CAN은 기존 배선의 TX=GPIO1 / RX=GPIO3입니다. 프로필 태그는 왼손 a5b23746 / 오른손 8f317a7f입니다. 양손 펌웨어와 config/dexhand.yaml을 함께 반영하세요. CAN 순서와 풀림·당김 값은 유지했습니다.
