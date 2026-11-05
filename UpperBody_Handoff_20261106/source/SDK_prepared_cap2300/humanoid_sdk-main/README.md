현재 오른손 수신 전용/PWM 작업 분리 안내는 릴리스 루트의 ONEWAY_PWM_안내.md를 보세요. 아래 내용은 이전 배포 기록입니다.

현재 오른손 엄지 값은 2026-11-05 THUMBFIX2_안내.md 및 profiles/right_hand.json을 참고하세요. 아래 내용은 이전 배포 기록입니다.

# humanoid_sdk

**현재 통합본: SDK 0.2.0 / 2026-11-04 / 손 CAN 프로토콜 3.** [양팔·양손 통합 API](ros2_ws/src/bhl_upper_body_sdk/README.md)와 [양손 업로드·팔/손 SDK 실행 가이드](../dexhand_control/firmware/안내사항/CURRENT_SETUP_20261104.md)를 먼저 읽으세요. 받은 V2 추가분은 현재 통합본에 반영했습니다.

Berkeley Humanoid Lite 상반신(양팔 recoil 모터, CAN)을 ROS 2 Humble에서 띄우고 다루는 워크스페이스입니다. 로봇에 실린 컴퓨터가 현재 상태를 내보내고, 나머지는 전부 그 상태를 보고 통신합니다.

| 패키지 | 역할 |
|---|---|
| [`bhl_bringup`](ros2_ws/src/bhl_bringup) | bring-up. `robot.launch.py`(로봇 컴퓨터: `arm_controller` + `joint_state_publisher` + `robot_state_publisher` → `/joint_states`, `/tf`, `/robot_description`, `/diagnostics`), `remote.launch.py`(원격 PC: RViz, 슬라이더), `demo.launch.py`(한 PC). 네트워크, CAN, 자동 시작 설정 |
| [`bhl_arm_sdk`](ros2_ws/src/bhl_arm_sdk) | 모터별 각도 제어 노드 `arm_controller`(CAN ↔ 관절 상태, 목표 토픽, 파라미터, 서비스, 안전 장치), 슬라이더 GUI, G1 예제식 시퀀스, 관절 맵 `config/bhl_arms.yaml` |
| [`bhl_hand_sdk`](ros2_ws/src/bhl_hand_sdk) | DexHand 양손 제어 노드 `hand_controller`(팔과 같은 `can0`, 손마다 별도 프로세스), 적용 응답·마지막 확인 PWM·PC 명령 연관값, HOLD/RESUME/OFF, 팔 정지 연동. [CAN 프로토콜 3](ros2_ws/src/bhl_hand_sdk/firmware/PROTOCOL_v3.md) |
| [`bhl_upper_body_sdk`](ros2_ws/src/bhl_upper_body_sdk) | 공통 Python API, 전체 목표 prepare/commit, 순번·시간·TTL·상태·제어권 관리, 고정 영점 생성 및 하중/모델 누락 검사 |

```
humanoid_sdk/
├── env.sh            source 하면 ROS 2, description 워크스페이스, 이 워크스페이스, ROS_DOMAIN_ID
├── humanoid.repos    URDF + 메시(berkeley_humanoid_lite_description)를 vcstool로 가져오는 목록
└── ros2_ws/src/
    ├── bhl_bringup/
    ├── bhl_arm_sdk/
    ├── bhl_hand_sdk/
    └── bhl_upper_body_sdk/
```

## 빠른 시작

```bash
# 현재 humanoid_sdk-main 전체를 로봇 PC의 ~/project_humanoid에 복사한 경우
cd ~/project_humanoid/humanoid_sdk-main/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to bhl_bringup
source install/setup.bash

ros2 launch bhl_bringup demo.launch.py                    # 연습 모드 + RViz + 슬라이더 (CAN 안 엶)
# 통합 API 경로: 개별 슬라이더 목표 대신 UpperBodyClient를 사용
ros2 launch bhl_bringup robot.launch.py hands:=both upper_body:=true dry_run:=true
```

RViz에 로봇을 그리고 `/tf`를 내려면 URDF가 필요합니다: `Berkeley-Humanoid-Lite-origin-`의 `ros2_ws`를 따로 빌드해 두거나(`env.sh`가 `~/Berkeley-Humanoid-Lite/ros2_ws`를 찾음), `vcs import src < ../humanoid.repos`로 이 워크스페이스에 가져옵니다. 없으면 팔 제어만 되고 RViz는 빠집니다.

로봇 컴퓨터와 원격 PC로 나누는 법, 토픽 계약, CAN 자동 설정, 실물 bring-up 순서는 [`bhl_bringup/README.md`](ros2_ws/src/bhl_bringup/README.md), 모터 제어 자체(각도 지정, 안전 장치, 설정값)는 [`bhl_arm_sdk/README.md`](ros2_ws/src/bhl_arm_sdk/README.md).

## 테스트 (ROS, CAN 없이)

```bash
python3 -m pytest -q ros2_ws/src/bhl_arm_sdk/test ros2_ws/src/bhl_bringup/test ros2_ws/src/bhl_hand_sdk/test ros2_ws/src/bhl_upper_body_sdk/test
```

## 관련 저장소

- [Berkeley-Humanoid-Lite-origin-](https://github.com/SCPS-LAB-koreatechU/Berkeley-Humanoid-Lite-origin-) — URDF, 메시, MoveIt 설정, PC 쪽 `hardware_bridge.py`
- [Berkeley-humanoid-lite `feature/onboard-x5`](https://github.com/SCPS-LAB-koreatechU/Berkeley-humanoid-lite/tree/feature/onboard-x5) — RDK X5용 `bhl_onboard`(unitree_sdk2식 `/lowcmd`, `/lowstate`). 그쪽 `/bhl/joint_states`도 `bhl_bringup`의 `joint_state_sources`로 받을 수 있습니다
