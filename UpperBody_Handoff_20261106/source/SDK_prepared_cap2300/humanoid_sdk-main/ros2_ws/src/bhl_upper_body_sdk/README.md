# 양팔·양손 통합 SDK 0.2.0

2026-11-04. 상위 프로그램의 명령 하나를 팔 제어기와 두 손 제어기에 전달하는 PC용 SDK다. SDK JSON 계약은 `bhl.upper_body/1`, 손 CAN 프로토콜은 기존 **3**을 사용한다. 이 두 버전은 서로 다른 계층이다. ESP32는 계속 CAN 목표 수신과 PWM 출력을 담당한다.

## 구성과 책임

| 구성 | 담당 |
|---|---|
| `UpperBodyClient` | 공통 목표 제출, 순번별 결과, 최신 상태 조회 |
| `upper_body_controller` | 전체 명령 검사, prepare/commit 조정, 입력 만료·실패 시 HOLD 요청 |
| `bhl_arm_sdk` | rad→모터 좌표 변환, 속도·가속도 제한, 팔 CAN, 피드백·모드·오류 감시 |
| `bhl_hand_sdk` | 15개 당김 비율→보정 PWM, 속도 제한, 손 CAN, 적용 응답·재접속 |
| 왼손·오른손 ESP32 | 완성된 15개 PWM 목표 세트 검사·반영, CAN 응답, 단절 시 마지막 PWM 유지 |

ROS 2 Humble의 `std_msgs/String`에 크기가 제한된 JSON을 담는다. 별도 메시지 생성 패키지 없이 설치 가능하다. JSON 명령에 단위·버전·정확한 필드를 고정했으며 잘못된 값과 중복 키는 거절한다. GUI, VLA 모델, 역기구학/모션 플래너는 이 SDK 위의 프로그램이다.

## 빌드와 실행

받은 `humanoid_sdk V2` 추가분이나 원격 저장소를 새로 내려받은 코드 대신, 현재 `humanoid_sdk-main` **전체**를 로봇 PC에 복사한다. 프로젝트 전체를 `~/project_humanoid`에 복사한 예:

```bash
cd ~/project_humanoid/humanoid_sdk-main/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to bhl_bringup
source install/setup.bash

# CAN을 열지 않는 양팔·양손 통합 경로
ros2 launch bhl_bringup robot.launch.py hands:=both upper_body:=true dry_run:=true
```

`upper_body:=false`는 기존 개별 팔/손 벤치 인터페이스를 유지한다. `true`에서는 각 노드의 `upper_body_only=true`가 되고, 개별 target 토픽·팔 home/목표 파라미터·운전 중 손 grip/open 명령을 차단한다. 단독 경로와 통합 경로를 동시에 사용하지 않는다. 통합 제어기는 CAN 소켓을 추가로 열지 않는다.

초기 목표 선택과 활성화는 명시적으로 한다. 아래 open은 **PWM이 꺼진 손을 의도적으로 펼침에서 시작할 때만** 사용한다. 이미 물체를 잡고 PWM을 출력하던 손에 PC만 재접속할 때는 open을 보내지 않고 enable의 스냅샷 동기화를 사용한다.

```bash
# 다른 터미널에서도 ROS와 install/setup.bash를 source한다.
ros2 service call /hand_left/open std_srvs/srv/Trigger "{}"
ros2 service call /hand_right/open std_srvs/srv/Trigger "{}"
ros2 service call /hand_left/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /hand_right/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /arm_controller/enable std_srvs/srv/SetBool "{data: true}"
ros2 topic echo /upper_body/state
```

실물에서는 고정 영점 설정 파일을 먼저 준비하고 `config:=/absolute/path/arms_fixed.yaml dry_run:=false`로 실행한다. 현재 벤치 기본값 `zero: start`로는 실물 통합 목표가 거절된다. 위치 유지는 기계 브레이크가 아니며 E-STOP/DAMPING과 다르다.

## Python API와 명령 단위

```python
from bhl_upper_body_sdk.client import UpperBodyClient

client = UpperBodyClient(my_rclpy_node)
# my_rclpy_node는 executor에서 계속 spin해야 송수신된다.
# 준비된 planner/teleoperation의 목표를 전달한다. 이 예시는 자동 enable을 하지 않는다.
session, sequence = client.send(
    arms_rad=joint_targets,     # 활성 팔 관절 10개: {관절 이름: rad}
    hand_left=left_pulls,       # 모터 번호 1..15 순서의 실수 15개
    hand_right=right_pulls,     # 각 값 0=풀림, 1=당김
    ttl_s=0.25,
)
# 나중 executor 반복에서 확인한다. 즉시 응답을 기다리며 제어 루프를 막지 않는다.
result = client.get_result(sequence)  # 아직 결과가 없으면 None, 최근 128개 보관
state = client.get_state(max_age_s=0.5)  # 없거나 오래되면 RuntimeError
```

손은 실제 관절 각도가 아닌 **텐던 모터별 당김 비율**이다. 각 손 13·14번 엄지 반대편 당김 합은 1 이하여야 한다. 동일 비율이어도 양손의 실제 자세가 같다고 보장되지 않는다. SDK가 각 손의 캘리브레이션에 따라 µs로 변환한다. VLA가 CAN ID나 PWM 보정값을 직접 생성할 필요는 없다.

기본 양팔·양손 실행에서는 세 부분 `arms`, `hand_left`, `hand_right`가 모두 필요하다. arms에는 활성 관절 전체를 넣는다. 팔을 빼고 실행하거나 손 한 개만 실행했다면 해당 실행에서 지정한 부분만 정확히 보낸다. 모터 순서는 `config/dexhand.yaml`과 양손 업로드 폴더의 BoardConfig.h에 명시되어 있다.

설치된 `share/bhl_upper_body_sdk/examples/stream_current_pose.py`는 **dry_run 전용**으로 시뮬레이션 현재 명령을 50Hz로 재전송한다. live 상태에는 전송하지 않는다. 실행:

```bash
python3 "$(ros2 pkg prefix bhl_upper_body_sdk)/share/bhl_upper_body_sdk/examples/stream_current_pose.py"
```

## 공통 명령 계약

입력 `/upper_body/target`:

```json
{
  "schema": "bhl.upper_body/1",
  "session": "0123456789abcdef0123456789abcdef",
  "sequence": 123,
  "stamp_s": 1793760000.0,
  "ttl_s": 0.25,
  "targets": {
    "arms": {"실제_활성_관절_이름_전부": 0.0},
    "hand_left": [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    "hand_right": [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
  }
}
```

위 JSON의 관절 이름과 시간은 구조 설명용이며 그대로 실행할 수 없다. `UpperBodyClient`가 실제 ROS 시간·새 세션 UUID·증가 순번을 생성한다.

- `stamp_s`: 목표를 **생성한 ROS 시각**, 미래 실행 예약 시각이 아니다. 재전송한다고 오래된 센서/정책 출력을 새 것으로 취급하지 않는다. 정책 출력이 만료됐으면 새 목표를 생성하거나 HOLD한다.
- `ttl_s`: 0.05~0.5초. stamp+TTL까지 유효하다. 미래 시각은 최대 20ms 오차만 허용한다. PC 사이 ROS 시계를 맞춘다. 실물에서 use_sim_time을 켜지 않는다.
- `session`: 한 제어 프로그램의 UUID. 다른 프로그램으로 제어권을 넘기려면 HOLD·명시적 복구 후 새 세션을 쓴다. 같은 세션 순번은 증가해야 한다.
- 유효 숫자·전체 부위·전체 활성 팔 관절·관절 범위·손 엄지 제약·활성 상태·고정 영점을 모두 검사한다. 일부만 조용히 적용하거나 범위를 몰래 잘라서 통합 목표를 수락하지 않는다.
- 명령 최대 16KiB. 상위 프로그램은 하나만 송신한다. 중복 송신자/코디네이터 감지는 ROS 그래프 기반 운영 보호이며 인증·실시간 보장은 아니다.

통합 제어기는 `/upper_body/transaction`으로 PREPARE를 보낸다. 팔·손은 움직이지 않고 검증한 뒤 `/upper_body/receipt`로 응답한다. 모두 준비되면 COMMIT을 보내고 각 제어기가 기존 속도 제한 루프의 목표를 갱신한다. 부분 거절·응답 유실·만료 시 전체 HOLD를 요청한다. 제어기 내부에서도 단조 시계로 TTL을 검사하므로 코디네이터 종료나 ROS 시각 정지 후 목표가 무기한 유지되며 이동하지 않는다.

한 번에 트랜잭션 하나만 진행한다. 진행 중 다음 입력은 `BUSY`로 거절하며 대기열에 오래된 목표를 쌓지 않는다. 다음 제어 주기에 새 목표를 제출한다. 단계별 제한은 80ms이지만 응답이 모이면 즉시 다음 단계로 넘어가므로 항상 80ms를 기다리지 않는다. 50Hz 수락률은 실제 ROS·CAN 환경에서 측정해야 한다.

**prepare/commit은 물리적 동시 시작이나 전체 모터의 원자적 적용을 보장하지 않는다.** COMMIT이 한쪽에만 도착한 뒤 통신이 끊기면 일부가 잠깐 움직일 수 있다. HOLD 전파와 로컬 TTL로 제한하며 이미 발생한 움직임을 되돌리지는 않는다. 관절 간 정확한 시간 궤적 추종이 필요하면 별도의 시간 기반 궤적 실행기를 추가해야 한다.

## 결과와 관측값

| 값 | 정확한 의미 |
|---|---|
| `/upper_body/result`: ACCEPTED | 모든 PC 제어기가 해당 목표를 저장했다고 응답. CAN 적용/도달 보장 아님 |
| PREPARE receipt | 해당 제어기가 검사 통과. 목표는 아직 바뀌지 않음 |
| COMMIT receipt | 해당 제어기가 새 목표를 저장. 실제 보간/출력은 이후 제어 주기 |
| 손 `last_delivery.result=APPLIED` | 그 CAN 순번의 PWM 명령을 ESP32가 반영했다고 응답 |
| 손 `last_applied_command` | 응답받은 PWM·그때의 PC 명령 session/sequence·CAN 순번·관측 경과 시간 |
| 손 `command`/`command_us` | SDK가 전송하려는 보간된 명령. 응답이 없으면 반영 여부 불명 |
| 손 `command_settled` | SDK 보간 명령이 목표까지 도달. 손가락 실제 도달 아님 |
| 손 `measured_position`/`measured_force`/`physical_reached` | 센서가 없으므로 null |
| 팔 `feedback_valid`, `feedback_age_s` | 피드백의 유효 여부와 수신 경과 시간 |
| 팔 `feedback_source=rotor_encoder_scaled` | 모터 엔코더/감속비 환산. 출력 기어 미끄러짐을 직접 측정하지 못함 |

`last_applied_command.source=simulated`는 dry_run 결과, `firmware_snapshot`은 재접속 때 읽은 마지막 PWM으로 PC 명령 연관값이 null이다. 응답이 유실되면 새 `command_us`가 있어도 마지막 확인 PWM을 새 값으로 바꾸지 않는다. 손 재부팅·OFF 시 확인 상태를 비운다. 실제 도달 판정은 추가 센서나 외부 관측을 사용하는 상위 프로그램의 책임이다.

`/upper_body/state`는 부위별 원본 상태, 수신 경과 시간, 최근 전체 결과, 준비 여부를 제공한다. `ready`는 현재 상태 기준 수락 가능 여부이지 이동 허가나 작업 성공 판정이 아니다. Client의 `get_state()`는 수신 시각과 발행 ROS 시각을 함께 확인한다.

## 정지와 제어권 복구

```bash
ros2 service call /upper_body/hold std_srvs/srv/Trigger "{}"
```

이는 HOLD **요청 발행**의 성공이다. 각 상태의 held/motion_stopped 및 오류를 확인해야 한다. 손은 마지막 PWM 유지, 팔은 기존 검증된 위치 유지 정책을 사용한다. 팔의 E-STOP은 기존 DAMPING이며 물리 브레이크가 아니다. 전원/기어/토크 부족 상황에서 낙하 방지는 보장되지 않는다.

명령 유효기간 만료, 부분 실패, 필수 노드 상태 유실은 자동 복구하지 않는다. 상위 송신을 멈추고 원인을 해결한 다음 기존 노드의 reset_estop/resume 절차를 수행한다. 각 노드가 정상·활성 상태로 돌아온 뒤:

```bash
ros2 service call /upper_body/clear_fault std_srvs/srv/Trigger "{}"
```

이 서비스는 코디네이터의 래치와 제어권만 비운다. 모터 enable, 영점 변경, 과거 목표 재생은 하지 않는다. 새 명령을 제출해야 이동한다. 코디네이터만 재시작해 제어권을 넘길 때도 먼저 upper_body/hold를 호출하여 로컬 endpoint의 이전 세션을 비운 뒤 복구한다. 전체 전원 재부팅으로 PWM이 꺼진 손에는 새 초기 목표가 필요하다.

## 고정 영점과 손 하중 기록

`upper_body_commission`은 **오프라인 파일 도구**이며 CAN을 읽거나 쓰지 않는다. 실제 기준 자세에서 각 관절의 독립적으로 확인한 각도와 모터 읽기값을 기록한다. SDK 운전 중 별도의 SDO 읽기 도구를 동시에 실행하지 않는다. 엔코더가 재부팅/회전 수 변화 후에도 같은 기준을 유지하는지 먼저 확인한다. 유지하지 못한다면 기준 센서·원점 복귀나 부팅별 재정렬 절차가 필요하며 숫자 저장만으로 해결되지 않는다.

측정 JSON 형식(실제 활성 관절 이름 10개를 모두 넣는다):

```json
{
  "reference_pose_verified": false,
  "encoder_reference_persistent_verified": false,
  "joints": {
    "실제_관절_이름": {"motor_position": null, "joint_rad": null}
  }
}
```

실측값을 넣고 검증이 완료된 두 플래그만 true로 바꾼 뒤:

```bash
ros2 run bhl_upper_body_sdk upper_body_commission make-fixed-zero \
  --arm-config /absolute/path/bhl_arms.yaml \
  --samples /absolute/path/measured_reference.json \
  --output /absolute/path/arms_fixed.yaml
```

`motor_zero = motor_position - direction × gear_ratio × joint_rad`로 계산한다. 게인·토크·전류 제한은 올리지 않는다. 원본/기존 출력 파일은 덮어쓰지 않는다. 새 설정에 zero:config, 스트리밍 고정 영점 요구, 측정 해시·검증 기록을 저장한다. 검증 플래그는 작업자 기록이지 자동 하드웨어 인증이 아니다.

`config/robot_model.yaml`에 실물 URDF 경로와 양손의 손목·브래킷 포함 질량(kg), 장착 프레임 이름, 그 프레임에서의 무게중심(m)을 기록한다. 초기값은 모두 미측정이다.

```bash
ros2 run bhl_upper_body_sdk upper_body_commission audit \
  --arm-config /absolute/path/arms_fixed.yaml \
  --model /absolute/path/robot_model.yaml --strict
```

누락값, 기준 검증 기록, URDF 관절 이름을 확인하고 strict에서 미완료면 종료 코드 1을 반환한다. URDF 링크 기하·관성의 정확성까지 검증하는 도구는 아니다. **중력 보상·충돌 회피·SCS2332 손목 제어는 구현하지 않았다.** 하중 값은 임의 추정으로 채우지 않으며, 이 파일을 작성했다고 중력 보상이 자동 활성화되지 않는다.

## 검증 범위

프로젝트의 `dexhand_control/tools/verify.py`는 기존 팔·손 검사와 공통 계약/트랜잭션/노드 callback 연결/영점 도구 검사를 실행한다. Python에서 양손의 실제 C++ 펌웨어 클래스와 통신하는 검사도 포함한다. callback 연결 검사는 ROS 대체 객체로 수행하며 실제 DDS·QoS·executor·50Hz 성능 확인을 대신하지 않는다.

실물 순서는 CAN 없이 실행 → 지지된 팔과 손의 전원·ID·설정 확인 → 고정 영점 검증 → 초기화/활성화 → 현재 자세 목표 → 작은 변화 → 명령 중단·일부 노드 종료·재접속 시험이다. 전원·기계 상태·위치 피드백·장치 로그와 함께 판단한다. 예전에 보고된 팔의 드르륵거림/낙하 원인이 해결됐다는 실기 판정은 아직 없다.

실제 ROS 런타임의 CAN 없는 검사 프로그램도 포함한다. ROS를 설치한 Linux PC에서 워크스페이스를 source한 다음 실행한다. 기본 별도 도메인 187에 기존 로봇 노드가 있으면 중단한다. 이 프로그램은 dry_run=true 노드만 실행하고 초기화·통합 수락·시뮬레이션 PWM 순번 연관·TTL 정지를 확인한 뒤 자신이 띄운 프로세스를 종료한다.

```bash
python3 "$(ros2 pkg prefix bhl_upper_body_sdk)/share/bhl_upper_body_sdk/examples/validate_ros_dry_run.py" \
  --domain 187 --report ros_dry_run_report.json
```

현재 Windows 작업 환경에는 ROS 2/WSL이 없어 이 실제 ROS 검사는 실행하지 않았다. 제공된 검사 프로그램 자체와 오프라인 callback 검사를 실제 DDS 시험 통과로 표기하지 않는다. CAN 단절·실제 모터·전원·하중 검증은 이 검사 프로그램의 범위에 포함되지 않는다.

이번 보완에는 전원 직후 BOOT 방송을 놓쳐도 스냅샷으로 부팅 식별을 먼저 확보하는 처리와, HOLD 후 resume 때 전송 유실된 예전 호스트 목표를 재생하지 않고 실제 마지막 PWM 명령에서 재개하는 처리도 포함한다. 두 경우 모두 실제 손가락 위치 측정을 뜻하지 않는다.
