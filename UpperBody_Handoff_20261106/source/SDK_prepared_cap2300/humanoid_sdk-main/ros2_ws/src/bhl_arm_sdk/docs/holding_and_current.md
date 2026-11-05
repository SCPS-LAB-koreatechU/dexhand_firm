# 팔 위치 유지, Berkeley 기준 설정, 22A 확인

> 아래는 2026-11-02 당시의 구현 기록입니다. 현재 TRACKING/HOLD 정책과 통신 구조는 [2026-11-04 통합 안내](../../../../../dexhand_control/firmware/안내사항/CURRENT_SETUP_20261104.md)에서 변경됐습니다. 게인·전류 상한의 단위 및 실측 필요 조건은 그대로입니다.

2026-11-02. 대상은 원래 Berkeley Humanoid Lite V1의 5010/M6C12 + 15:1 출력 감속기이다. 최신 RobStride 팔의 설정은 사용하지 않았다. 로컬 코드와 공개 원본을 비교했으며, 로봇에서 실행하거나 실제 전류·MODE를 측정한 결과는 아니다.

## 현재 증상과 확인된 결함

증상은 올라가다가 드르륵거린 뒤 힘이 풀린 것처럼 다시 떨어짐이다. 게인을 바꾸면 올라가는 경우도 있다. 이는 하중에 비해 토크가 부족하거나 진동하는 경우와, 보드가 POSITION을 이탈하는 경우 모두 가능하다. 소리만으로 기어 손상 또는 전류 부족을 확정할 수 없다.

기존 SDK도 POSITION을 켜고 목표 도달 후 같은 위치를 계속 보냈다. 위치 유지 제어가 없었던 것이 아니다. 다만 PDO 위치 응답만 확인하여 **보드는 DAMPING인데 SDK는 live=True인 상태**가 가능했다. 20° 추종 오차가 나면 이후 모든 관절을 DAMPING으로 바꿨다. 이 소프트웨어 경로는 가짜 모터로 재현되었지만 실물 고장의 원인 확정은 아니다.

DAMPING은 움직임을 저항하는 제어다. 정지 자세를 유지하는 브레이크가 아니며, 중력으로 팔이 내려갈 수 있다. 기존 E-STOP의 DAMPING 동작도 이 한계를 갖는다. 물리적으로 받친 상태에서 진단한다.

GUI에서 목표에 도달하는 것 자체는 출력을 해제하지 않는다. 반면 `arm_sequence` 기본 동작은 유지 시간 후 원점 복귀·DAMPING 해제까지 수행한다. 계속 유지하려면 `--no-release`가 필요하며, 이때에도 `arm_controller` 프로세스는 계속 실행되어야 한다. 컨트롤러 종료는 IDLE 명령을 보내고, 통신 중단은 보드 watchdog을 유발할 수 있다.

## 이번 코드 동작

- enable 전에 펌웨어 버전, 펌웨어 기어비, 전류 상한, 토크 상수, 시작 시 버스 전압, ERROR를 읽는다. 지원 레지스터 배치는 `0x20250226`이다. 같은 버전 번호를 사용하는 커스텀 빌드까지 동일하다고 보장하지는 않는다.
- 펌웨어 기어비가 설정의 `firmware_gear_ratio`와 다르면 `GEAR_MISMATCH`로 활성화를 거절한다. 펌웨어 기어비를 자동 변경하지 않는다. 현재 프로필은 보드 +1, 호스트 15를 전제로 한다.
- Kp/Kd/torque_limit을 쓴 뒤 읽어 비교한다. 불일치하면 POSITION을 켜지 않는다. POSITION 명령 후 실제 MODE도 읽는다.
- 제어 주기마다 MODE 또는 ERROR 하나를 추가로 읽는다. 관절 10개, 실제 50Hz일 때 각 값은 약 0.4초마다 갱신된다. 읽기 대기는 최대 2ms로 설정한다. 실시간 OS가 아니며 PDO 지연 등으로 주기가 밀리면 갱신도 늦어진다.
- 실제 MODE가 POSITION에서 벗어나면 해당 관절은 `MODE_LOST`로 표시하고 명령 전송을 중단한다. 다른 관절을 이 사유만으로 해제하지 않는다. 자동 재활성화하지 않는다. 기존 20° TRACKING 전체 정지와 PDO 연속 유실 보호는 유지한다.
- ERROR는 과거 오류가 남을 수도 있다. 운전 중 ERROR만 보고 곧바로 토크를 해제하지 않고 경고로 표시한다. 새로 enable할 때 ERROR가 0이 아니면 거절한다. 오류 비트를 자동 삭제하지 않는다.
- SDO 조회 실패는 `SDO_UNVERIFIED`로 표시한다. 그 자체만으로 위치 출력을 해제하지 않는다. `live`는 호스트가 명령을 보내는지를 뜻하고, 실제 확인 상태는 MODE와 조회 시각·경고를 함께 본다.
- GUI 상태 및 툴팁, `/arm_controller/status`, `/diagnostics`에 보드 상태를 표시한다. dry_run에서는 보드 값이 없으며 하드웨어 검증을 뜻하지 않는다.
- E-STOP 후에도 MODE/ERROR 읽기는 계속하여 오류를 확인할 수 있게 한다. 이때 목표와 하트비트는 보내지 않는다.

### SDO 사용 제약

Recoil SDO 응답에는 요청한 레지스터 주소가 없다. 이 SDK 실행 중에는 별도 설정 프로그램 또는 `recoil_audit.py snapshot`을 같이 실행하지 않는다. 수신 전용 `capture`는 가능하다. SDK 내부 조회는 직렬화한다. 조회 타임아웃이나 다른 SDO 클라이언트가 감지되면 해당 노드의 추가 SDO 조회를 차단하여 늦은 MODE 응답을 전류값으로 오인하지 않게 했다. 팔을 받치고 외부 클라이언트를 종료한 뒤 SDK를 다시 시작해야 재조회할 수 있다. 외부 클라이언트의 모든 경쟁 상황을 프로토콜상 완벽하게 식별할 수 있는 것은 아니다.

## Berkeley 기준 설정

원본 [run_teleop.py](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite/blob/984741a3623c93b0583ccfdc479f1f8b1c4d900e/scripts/teleop/run_teleop.py)는 `kp=30, kd=2, torque_limit=2`로 시작한다. [Bimanual 구현](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite-Lowlevel/blob/652777cc7c49884e7cd7ddfada758dc1979bf627/berkeley_humanoid_lite_lowlevel/robot/bimanual.py)과 펌웨어 단위를 함께 고려해야 한다. 숫자를 그대로 복사하면 안 된다.

물리 감속비 N=15, 보드 기어비 +1일 때 이상적인 무손실 환산은 다음과 같다.

```
출력 관절 Kp = 보드 Kp × N²
출력 관절 Kd = 보드 Kd × N²
출력 관절 토크 상한 = 보드 torque_limit × N
```

| 설정 | 보드 Kp | 보드 Kd | 보드 토크 상한 | 이상적 출력 Kp / Kd / 토크 상한 |
|---|---:|---:|---:|---|
| 기존 기본 | 0.3 | 0.01 | 0.5 | 67.5 / 2.25 / 7.5 Nm |
| 기존 왼 어깨 롤 ID3 | 1.0 | 0.01 | 1.2 | 225 / 2.25 / 18 Nm |
| 기존 오른 팔꿈치 피치 ID8 | 0.8 | 0.01 | 0.5 | 180 / 2.25 / 7.5 Nm |
| 원본 teleop 동등 비교값 | 0.1333333 | 0.0088889 | 0.1333333 | 30 / 2 / 2 Nm |

Kp 단위는 Nm/rad, Kd는 Nm·s/rad이다. 표는 단위 환산이며 출력 토크 측정치나 감속기의 허용 정격이 아니다. 전류 제한, 기어 효율, 마찰, 전압과 모터 보정에 따라 실제 출력은 달라진다.

비교 설정은 `config/bhl_arms_berkeley_reference.yaml`에 넣었다. 기존 `bhl_arms.yaml`의 게인을 갑자기 낮춰 자동 적용하지 않았다. 새 프로필의 5°/s는 여기서 정한 느린 비교용 속도이며 원본 설정이 아니다. 토크 상한이 기존보다 낮으므로 DexHand를 부착한 팔이 올라가거나 유지된다고 보장하지 않는다.

로봇 PC로 수정 소스를 옮긴 뒤 재빌드한다. 다음은 CAN을 열지 않는 비교용 실행이다.

```bash
cd ~/humanoid_sdk/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to bhl_arm_sdk
source install/setup.bash
ros2 launch bhl_arm_sdk arm_sdk.launch.py dry_run:=true rviz:=false \
  config:="$(ros2 pkg prefix bhl_arm_sdk)/share/bhl_arm_sdk/config/bhl_arms_berkeley_reference.yaml"
```

실물에서 기존 설정의 상태 확인은 수정 SDK를 기존 프로필로 실행하여 한다. 읽은 값은 다음에서 확인한다. `current_limit_a` 등 시작 시 값은 enable 사전 확인이 실행되어야 채워진다.

```bash
ros2 topic echo /arm_controller/status --once
ros2 topic echo /diagnostics --once
```

## 22A의 의미와 확인 결과

사용자가 전달받은 22A의 설정 위치는 모른다. 로컬 SDK에는 22A를 쓰는 코드가 없다. 모터 보드 Flash/설정 도구에 저장된 값일 수도 있고 전체 전원 제한일 수도 있으므로 현재 적용 여부를 확정하지 않았다.

보드 `current_limit_a`는 [전류 제어기](https://github.com/T-K-233/recoil-motor-controller-besc/blob/3571ab60a05951561abbab2e27c816b67b315bb0/Recoil-Motor-Controller-B-G431B-ESC1/Core/Src/current_controller.c)의 dq 전류 상한을 읽은 값이다. 이것이 22이면 해당 보드 전류 상한 22A를 확인한 것이며, 배터리 전류 22A나 연속 운전 허용 전류라는 뜻이 아니다. 이번 수정은 이 값을 읽기만 하고 변경하거나 Flash에 저장하지 않는다. 배터리/BMS/전원공급기 제한은 이 레지스터로 확인할 수 없다.

보드 기어비 +1 기준 대략 `허용 모터 토크 ≤ min(torque_limit, Kt × Iq_limit)`이다. 실제 출력은 그보다 작을 수 있다. 따라서 전류 상한이 22A여도 torque_limit에 먼저 막힐 수 있으며, Kp 증가가 곧 사용 가능한 최대 토크 증가를 뜻하지 않는다.

## DexHand 반영의 남은 조건

완성 손·손목·브래킷의 질량과 무게중심은 현재 미확인이다. 프로필에는 이를 `null`, `validated: false`로 명시했다. 메타데이터를 채우기만 해도 자동 중력 보상이 켜지는 구조는 아니다.

각 관절에 추가되는 정적 중력 토크는 손 무게만이 아니라 관절축에서 손 무게중심까지의 위치로 계산한다. 일반식은 `τ_i = axis_i · (r_i × m g)`이며, 어깨에는 팔꿈치보다 긴 지렛팔이 걸릴 수 있다. 팔 자체 하중, 가속도, 기어 효율·정격과 열 여유도 필요하다. `zero: start`는 임의 시작 자세를 0°로 삼으므로 중력 보상 전에 실제 기구 자세에 대한 절대 영점·축도 검증해야 한다.

지금 할 수 있는 개선은 MODE/ERROR로 토크 해제 여부를 먼저 가르고, 기어비·게인·전류 상한을 확인하는 것이다. MODE가 POSITION을 유지하면서 추종 오차가 커지면 실제 전류·전압·토크 제한 및 기계적 미끄러짐을 조사한다. 증상 시점의 로그 없이 “브레이크 부재” 또는 “22A 부족”으로 단정하지 않는다. 가짜 모터 테스트는 실제 감속기 떨림이나 하중 지지를 검증하지 못한다.

## 검증 기록

Windows의 Python 3.12 환경에서 진단 도구, SDK 코어와 bring-up 테스트 합계 **56개 통과**. 수정한 Python 파일의 구문 검사와 당시 수정 전 백업을 이용한 과거 결함 재현도 통과했다. 이번 변경을 ROS 2 실환경, CAN 어댑터 또는 실제 모터로 실행하지 않았다. 수정 전 백업과 과거 결함 재현 전용 파일은 이후 사용자 요청에 따른 프로젝트 정리에서 삭제했다. 현재 진단 도구는 `arm_diagnostics/`에 유지한다.
