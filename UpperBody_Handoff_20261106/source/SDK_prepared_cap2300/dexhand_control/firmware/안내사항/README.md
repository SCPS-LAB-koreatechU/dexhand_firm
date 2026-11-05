# DexHand CAN 손 제어

**현재 코드(2026-11-04)는 프로토콜 3입니다. [양손 업로드·SDK 실행 안내](CURRENT_SETUP_20261104.md)와 [PC CAN 명세](PROTOCOL_v3.md)를 먼저 읽으세요. 아래의 기존 v1 설명·송신 예제는 이력용이며 현재 동작 변경은 새 안내가 우선합니다.**

안내 문서는 이 폴더에 모았습니다.

- **[기존 CAN 배선 수정본: 양손 TX GPIO1 / RX GPIO3 · 업로드 순서](CAN_WIRING_GPIO1_3.md)**

- [GUI: 손가락·위치별 제어 / 카메라 / 캘리브레이션](HAND_GUI.md)

- [간단한 실행: 손 CAN 단독·USB 카메라 접기/펴기 테스트](HAND_TEST_QUICKSTART.md)

- [현재 양팔·양손 통합 SDK 0.2.0: API·결과·단위·영점·실물 검증](UPPER_BODY_SDK_v020.md)

- [기존 SDK·수신 v2 전체 비교와 수정 우선순위 (2026-11-04)](SDK_V2_COMPARISON_20261104.md)
- [휴머노이드 PC: 팔·양손 동시 제어 가이드](hand_can_integration.md)
- [PC 송신 코드 사용법](SDK_INTEGRATION.md)
- [CAN 프로토콜](PROTOCOL.md)
- [ESP32 업로드·재생성 안내](UPLOAD_GUIDE.md)
- [ESP32 코드 구성](FIRMWARE_STRUCTURE.md)
- [오른손 배선·캘리브레이션](RIGHT_HAND_PINMAP.md)
- [검증 기록](VERIFICATION.md)

`tools/`, `pc/`, `profiles/`, `firmware/`로 시작하는 소스 경로와 실행 명령은 **dexhand_control 프로젝트 루트** 기준입니다. 이 안내사항 폴더에서 두 단계 위입니다.

배터리로 켜지는 ESP32가 PC의 CAN 목표를 받아 손가락 ES3352 15개에 PWM을 출력하는 코드입니다. VLA, GUI, ROS 2는 PC에서 실행합니다. ESP32는 CAN 수신과 모터 출력을 담당합니다.

왼손과 오른손은 각각 **[DexHandLeft.ino](../DexHandLeft/DexHandLeft.ino)**, **[DexHandRight.ino](../DexHandRight/DexHandRight.ino)**를 업로드합니다. 손별 캘리브레이션과 CAN 노드 60/61이 적용된 별도 스케치입니다. [업로드·재생성 안내](UPLOAD_GUIDE.md)와 [오른손 배선표](RIGHT_HAND_PINMAP.md)를 참고하세요. 실물 배선·펌웨어 업로드는 미확인입니다. 손목 SCS2332는 이번 펌웨어에 포함하지 않았습니다. 이 폴더는 기존 `humanoid_sdk-main` 옆에 있습니다.

## 전원을 켰을 때

1. ESP32 플래시에 저장된 펌웨어가 자동 실행됩니다.
2. GPIO의 PWM은 꺼진 상태로 CAN 수신을 시작합니다. USB 연결, 시리얼 모니터, 터미널, Wi-Fi 또는 BLE 연결을 기다리지 않습니다.
3. PC에서 올바른 15개 PWM 목표를 모두 받으면 자동으로 모터 출력을 시작합니다. 별도 ARM 명령은 필수가 아닙니다.
4. 목표 세트가 1초 동안 오지 않으면 마지막 PWM 값을 유지합니다. 새 정상 세트가 도착하면 자동으로 다시 적용합니다.
5. 명시적인 HOLD/OFF 명령을 받으면 CAN의 START/RESUME 전까지 목표 프레임을 무시합니다.

전원만 켰을 때 손이 임의의 1500µs 또는 초기 자세로 움직이지 않습니다. 첫 출력은 첫 완성 목표를 직접 적용합니다. ES3352의 실제 위치를 읽을 수 없으므로, 첫 출력을 현재 실제 자세에서 보간하는 기능은 없습니다. 이후 이동 궤적과 속도는 PC가 계산해서 전송합니다.

## 폴더

```text
dexhand_control/
  profiles/left_hand.json              PC가 관리하는 원본 배선·보정값
  profiles/right_hand.json             오른손 보정값·제안 GPIO, CAN 노드 61
  tools/generate_board_config.py       핀과 허용 범위 헤더 생성
  tools/prepare_firmware.py            왼손·오른손 업로드 스케치 각각 생성
  firmware/DexHandLeft/                왼손 업로드용, 노드 60, 헤더 포함
  firmware/DexHandRight/               오른손 업로드용, 노드 61, 헤더 포함
  firmware/DexHandCan/
    DexHandCan.ino                     전원 투입 실행, CAN 드라이버·수신 루프
    HandController.h                  명령 구분·목표 반영·HOLD/OFF·타임아웃
    PoseReceiver.h                    5개 프레임 조립·검사
    ServoOutput.h                     LEDC 하드웨어 PWM 15채널
    BoardConfig.h                     JSON에서 생성된 핀·허용 범위
  pc/hand_can.py                       PC 보정 및 기존 SDK용 송신 클래스
  pc/send_hand.py                      Linux SocketCAN 송신/미리보기 예제
  examples/left_open_us.json           풀림 위치 예제: 자동 실행하지 않음
  examples/right_open_us.json          오른손 풀림 예제: 자동 실행하지 않음
  tests/                              PC 및 수신기 검사
  firmware/안내사항/
    README.md                         문서 목록·전체 사용법
    UPLOAD_GUIDE.md                   ESP32 업로드·재생성 방법
    hand_can_integration.md           팔·양손 동시 제어 통합 가이드
    SDK_INTEGRATION.md                휴머노이드 PC 송신 코드 사용법
    PROTOCOL.md                       CAN 메시지 형식
    FIRMWARE_STRUCTURE.md             ESP32 내부 코드 흐름과 파일별 역할
    RIGHT_HAND_PINMAP.md              오른손 GPIO·기존 번호/CAN 순서 대응
    VERIFICATION.md                   검증 기록
```

## 왼손 PWM 배선 (오른손은 RIGHT_HAND_PINMAP.md 참조)

| 모터 | 위치 | GPIO | 풀림 µs | 당김 µs |
|---:|---|---:|---:|---:|
| 1 | 소지 위 | 32 | 1775 | 700 |
| 2 | 소지 아래 | 33 | 700 | 1625 |
| 3 | 약지 위 | 25 | 1750 | 700 |
| 4 | 약지 아래 | 26 | 700 | 1675 |
| 5 | 중지 위 | 21 | 725 | 1675 |
| 6 | 중지 아래 | 22 | 1675 | 725 |
| 7 | 검지 위 | 23 | 725 | 1700 |
| 8 | 검지 아래 | 19 | 1750 | 875 |
| 9 | 검지 끝 | 16 | 2000 | 700 |
| 10 | 중지 끝 | 17 | 2000 | 700 |
| 11 | 약지 끝 | 4 | 700 | 2000 |
| 12 | 소지 끝 | 27 | 700 | 2000 |
| 13 | 엄지 오른쪽 당김 | 13 | 1000 | 1900 |
| 14 | 엄지 왼쪽 당김 | 14 | 2100 | 700 |
| 15 | 엄지 끝 | 18 | 700 | 1900 |

GPIO16을 사용하므로 PSRAM 없는 ESP32-WROOM-32E 계열을 기준으로 합니다. ESP32-S3/C3용 코드가 아닙니다.

## CAN과 전원 배선

현재 프로필은 기존 납땜 배선에 맞춰 **CAN TX GPIO1 / CAN RX GPIO3**, 1Mbps입니다. GPIO5/34로 되어 있던 이전 통합본을 수정했습니다. 이전 오른손 TIP 시험과 같은 물리 CAN 핀을 사용하지만, 통합 펌웨어는 표준 11비트 프로토콜 3을 사용하므로 새 PC 설정과 양손 펌웨어를 함께 배포합니다.

- ESP32 CAN TX → 3.3V 호환 CAN 트랜시버의 TXD.
- ESP32 CAN RX ← 트랜시버의 RXD.
- 트랜시버 CAN-H/L → 팔과 같은 CAN 버스. GND는 공통으로 연결합니다.
- ESP32 개발보드는 해당 보드의 전원 입력 규격에 맞는 레귤레이터 출력을 사용합니다. 24V 배터리를 ESP32 5V/3V3 핀에 직접 넣지 않습니다.
- 서보 전원은 서보용 컨버터에서 공급합니다. ESP32 GPIO는 신호만 담당합니다.
- CAN 종단저항은 버스 양 끝에만 구성합니다.

펌웨어는 정상 CAN 모드로 수신하며 ACK를 처리하고 PONG·STATUS·STATE 응답도 송신합니다. 트랜시버의 TXD와 RXD 연결이 모두 필요합니다. GPIO1/3의 USB-UART 공유와 ROM 부팅 로그의 전기적 영향은 [기존 CAN 배선 수정본 안내](CAN_WIRING_GPIO1_3.md)를 따릅니다. Serial 비활성화가 USB-UART 칩을 전기적으로 분리하지는 않습니다.

## 빌드와 최초 업로드

Arduino-ESP32 **3.3.12**, 일반 ESP32 보드 FQBN `esp32:esp32:esp32` 기준입니다. 외부 서보 라이브러리는 사용하지 않고 LEDC 하드웨어 PWM 15채널을 사용합니다.

dexhand_control 프로젝트 루트를 현재 디렉터리로 두고 실행합니다.

```bash
python tools/prepare_firmware.py
python tools/prepare_firmware.py --check
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/DexHandLeft
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/DexHandRight
```

최초 펌웨어 업로드에는 USB 또는 외부 프로그래머가 한 번 필요합니다. 예를 들어 업로드 포트가 COM5인 Windows에서는 다음과 같습니다. COM5는 예시이므로 실제 포트로 바꿉니다.

```powershell
arduino-cli upload --fqbn esp32:esp32:esp32 --port COM5 firmware/DexHandLeft
```

오른손은 실제 오른손 보드의 포트와 `firmware/DexHandRight`를 지정합니다.

업로드가 끝나면 USB 없이 배터리 전원만으로 실행됩니다. 이번 작성·검증에서는 보드 업로드나 실제 모터 구동을 수행하지 않았습니다.

`open_us/close_us`는 PC가 펄스폭을 계산할 때 사용합니다. 생성 헤더에는 GPIO와 수치상 최소·최대만 들어가며, ESP32는 범위 검사만 합니다. 프로필에서 GPIO 또는 허용 범위를 바꾸면 헤더를 다시 생성하고 펌웨어를 다시 업로드합니다. 자동 생성 파일을 따로 편집하지 않습니다.

## 출력과 정지의 의미

- 5개 프레임의 목표가 모두 정상일 때 같은 소프트웨어 갱신에서 15개 출력값을 변경합니다. 모든 PWM 에지가 완전히 일치하거나 실제 모터 도착 시간이 같다는 의미는 아닙니다.
- 50Hz PWM의 펄스폭을 갱신하며, 5개 프레임이 모이기를 기다리는 블로킹 루프는 없습니다. 별도의 20ms 대기 루프도 없습니다.
- 통신 타임아웃은 마지막 PWM 유지입니다. 목표 프레임을 새로 받으면 자동 재개합니다. 완성 세트만 타이머를 갱신하며 팔 트래픽·반복 순번·불완전 세트는 갱신하지 않습니다.
- HOLD는 마지막 명령 펄스 유지이며 팔 모터처럼 능동 댐핑을 수행하지 않습니다.
- OFF는 PWM 펄스 중단입니다. 모터 전원 차단이나 확실한 토크 해제를 보장하는 명령은 아닙니다.
- 개별 모터의 펄스 범위 검사는 여러 힘줄 조합의 충돌·장력·힘 제한을 검증하지 않습니다. 조합 제약과 이동 경로는 PC 제어 계층에서 처리합니다.
- 이 펌웨어에는 ES3352 실측 각도·전류·힘 피드백이 없습니다. `ServoOutput`의 `lastCommandUs_`는 출력 명령 기록입니다.

## 검사

```bash
python -m pip install -r requirements.txt
python -B -m unittest discover -s tests -v
python tools/generate_board_config.py --check
python tools/prepare_firmware.py --check
```

`tests/receiver_test.cpp`는 펌웨어와 같은 `PoseReceiver.h`를 네이티브 C++ 컴파일러에서 실행하는 검사입니다.
`tests/controller_test.cpp`는 실제 `HandController.h`를 사용하여 CAN 프레임에서 출력 호출까지 검사합니다.

```bash
c++ -std=c++17 tests/receiver_test.cpp -o receiver_test
./receiver_test
c++ -std=c++17 tests/controller_test.cpp -o controller_test
./controller_test
```

검증 결과와 실물 미검증 범위는 이 폴더의 `VERIFICATION.md`에 기록합니다.

