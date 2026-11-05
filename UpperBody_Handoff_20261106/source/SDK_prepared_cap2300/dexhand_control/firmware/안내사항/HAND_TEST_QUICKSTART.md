# 손 CAN 단독 테스트와 USB 카메라 따라 하기

휴머노이드 **Linux PC**에 USB-CAN 어댑터와 USB 카메라를 연결하는 기준이다. ROS를 실행하거나 colcon으로 빌드하지 않아도 된다. 현재 `dexhand_control`과 `humanoid_sdk-main`이 나란히 있어야 한다. 오른손은 손가락 배선 revision 2, 프로필 8f317a7f 기준이며 검지 위/아래 GPIO32/25를 사용한다. 양손 CAN은 TX=GPIO1 / RX=GPIO3이다. [기존 배선용 수정본 업로드 안내](CAN_WIRING_GPIO1_3.md)를 먼저 따른다.

## 바로 실행

처음에는 팔을 지지하고 기존 팔·손 SDK를 종료한 상태에서 손 하나만 시험한다. 같은 손의 hand_controller/다른 테스트가 실행 중이면 CAN 노드 잠금으로 실행을 거절한다. 이 도구는 선택한 손 노드 60 또는 61만 제어하며 팔 모터 명령을 보내지 않는다. 외부 CAN 송신자를 인증하거나 차단하는 기능은 아니다.

```bash
cd ~/project_humanoid/dexhand_control

# 처음 한 번: 전용 .venv에 의존성 설치 + 공식 손 인식 모델 다운로드
bash setup_hand_test.sh --camera

# 1. 움직임 없는 CAN 통신·프로토콜·프로필 확인
bash test_hand.sh right ping

# 2. 작은 접기/펴기 2회 후 HOLD
bash test_hand.sh right cycle --initial-open

# 3. USB 카메라 따라 하기
bash test_hand.sh right camera --initial-open
```

왼손은 `right`를 `left`로 바꾼다. 카메라가 나중에 필요하면 처음 설치에서 `--camera`를 빼면 CAN 검사에 필요한 두 패키지만 설치한다. 카메라 설치에는 인터넷이 필요하고, 이후에는 내려받은 모델을 사용한다. `.venv`는 이 테스트 전용이며 ROS 시스템 Python 패키지를 변경하지 않는다. Python 3.10 이상, Linux 64비트의 호환 패키지 설치 환경이 필요하다.

`--initial-open`은 **PWM이 꺼진 손을 의도적으로 풀림 목표에서 시작한다는 선택**이다. 빈손 상태에서 사용한다. 실제 위치 센서가 없어 첫 PWM을 현재 실제 위치에서 보간할 수 없다. 이미 PWM을 출력하던 손은 먼저 마지막 PWM을 읽어 같은 명령으로 동기화한다. `cycle`을 선택하면 그 후 명시적인 펼침/접힘 시험을 시작한다.

먼저 소프트웨어만 시험하려면:

```bash
bash test_hand.sh right cycle --initial-open --dry-run --cycles 1
```

dry-run의 SIMULATED/APPLIED는 실제 CAN 수신 증거가 아니다.

## 카메라 조작

화면에는 사람 손 하나만 보이게 한다. 기본은 **미리보기 상태**라 로봇 손이 움직이지 않는다.

1. 사람 손의 네 손가락을 편 상태로 카메라에 보여준다.
2. **Space**를 누르면 CAN 확인과 초기화 후 제어가 활성화된다.
3. 네 손가락을 접으면 로봇 손이 조금 접히고, 펴면 풀림 목표로 이동한다.
4. **H**는 HOLD, **Q/Esc** 또는 창 닫기는 종료하며 HOLD를 요청한다.
5. 손이 사라지거나 두 손이 보이거나 영상이 오래되면 HOLD한다. 다시 편 손을 보여주고 Space를 눌러야 재개한다.

이 버전은 **전체 손의 접기/펴기 두 상태를 따라 하는 테스트**다. 각 손가락/관절/엄지 방향을 정밀 복제하거나 힘을 제어하지 않는다. MediaPipe의 손가락 PIP/DIP 추정 각도로 상태를 분류하고, SDK의 보정된 power/open 목표로 변환한다.

기본 최대 당김은 **각 모터 보정 스트로크의 25%**, 이동 속도는 초당 스트로크의 25%다. 이는 힘이나 실제 관절 각도의 25%를 뜻하지 않는다. 초기 시험은 제한을 유지한다. 필요하면 `--level 0.15`, `--speed 0.15`처럼 더 작게 설정할 수 있다. 테스트 도구는 level 최대 0.5, speed 최대 1.0으로 제한한다. 엄지 반대편 두 모터를 동시에 최대로 당기지 않는다.

```bash
# 카메라 선택: 0 대신 1 또는 /dev/video2 등
bash test_hand.sh right camera --initial-open --camera 1

# SSH 등 창 없는 환경: 편 손을 0.5초 보여주면 한 번 활성화
bash test_hand.sh right camera --initial-open --headless --enable
```

창 없는 모드는 `--enable`을 명시해야 한다. 추적을 잃으면 자동 재개하지 않으며 프로그램을 다시 실행해야 한다. Ctrl+C는 HOLD를 요청하고 종료한다.

영상 읽기/추론은 별도 프로세스에서 실행하고 CAN 루프는 기본 50Hz로 동작한다. 프레임이 0.35초 이상 오래되면 사용하지 않는다. 추론이 멈춰도 마지막 목표를 계속 새 목표처럼 재전송하지 않는다. 일반 PC/Linux는 실시간 지연 상한을 보장하지 않으며, VideoCapture 드라이버 버퍼의 실제 촬영 시각까지 검증하는 것은 아니다.

## 통신 준비와 결과 해석

```bash
ip -details link show can0
```

기존 어댑터가 can0, UP, bitrate 1000000인지 확인한다. DOWN 상태이고 다른 제어기가 사용하지 않는 경우에만:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

다른 CAN 인터페이스는 `--channel can1`처럼 지정한다. 모터/ESP32/트랜시버 전원과 CAN-H/L/GND, 종단저항은 기존 배선을 사용한다. 오른손은 최신 펌웨어와 PC 프로필을 함께 사용해야 한다.

| 표시 | 뜻 |
|---|---|
| CAN PONG, protocol=3, profile 일치 | ESP32의 응답을 확인. 모터 움직임 증거는 아님 |
| APPLIED 증가 | ESP32가 PWM 목표 반영을 응답. 실제 손가락 도달/힘 측정은 아님 |
| NO_PONG | 보드/트랜시버 전원, CAN 배선·속도·노드 번호 등을 확인 |
| PROFILE_MISMATCH | 오른손/왼손 펌웨어와 PC dexhand.yaml이 다름. 억지로 우회하지 않고 함께 갱신 |
| already owned | 같은 손 SDK/다른 테스트 프로세스가 실행 중 |
| NO_REPLY / REJECT_* | 시험을 멈추고 HOLD 요청. 응답이 없으면 실제 출력 상태는 불명 |
| HOLD CONTROL_ACK | ESP32가 유지 명령에 응답. 힘/물리 브레이크 보장은 아님 |

시험 종료 시 PWM OFF로 풀지 않고 마지막 PWM을 유지한다. 전원·통신 단절 상황에서 HOLD 전달이나 파지 유지가 보장되는 것은 아니다. 기본 펌웨어는 정상 명령 단절 시 마지막 PWM을 유지한다.

## 참고와 검증 범위

- [Google 공식 Hand Landmarker Python API](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker/python)
- [Google 공식 모델 사용 예제](https://github.com/google-ai-edge/mediapipe-samples/blob/main/examples/hand_landmarker/python/hand_landmarker.ipynb)
- 공식 모델은 `tools/fetch_hand_model.py`가 고정 HTTPS 주소에서 받아 SHA256을 확인한다. 임의 모델을 자동 실행하거나 시작 중 다운로드하지 않는다.
- 의존성은 MediaPipe 1.2.2 / OpenCV contrib 5.0.0.93 / python-can 4.6.1 / PyYAML 6.0.3으로 고정했다. 보드 OS/CPU/Python에 맞는 wheel이 없으면 설치가 실패하며 임의 소스 빌드를 시도하지 않는다.
- 합성 랜드마크·가상 CAN·실제 C++ 펌웨어 클래스와의 송수신으로 테스트한다. 실제 모델 API 검사와 실제 USB 카메라/물리 CAN/모터 시험은 구분하여 기록한다. 현재 작업에서는 실제 카메라나 로봇 모터를 작동시키지 않았다.
# GUI로 실행

손가락·위치별 슬라이더, 카메라 영상, 캘리브레이션은 [손 GUI 안내](HAND_GUI.md)를 사용합니다.
`bash setup_hand_test.sh --gui --camera` 설치 후 `bash gui_hand.sh --dry-run`으로 시작합니다.
아래는 기존 CLI 방식입니다.

