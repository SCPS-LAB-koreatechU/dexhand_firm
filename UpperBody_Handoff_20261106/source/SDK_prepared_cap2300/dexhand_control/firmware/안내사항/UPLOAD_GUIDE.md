# 왼손·오른손 업로드 파일

> 2026-11-04 이후의 현재 코드는 [보완본 안내](CURRENT_SETUP_20261104.md), [프로토콜 3 명세](PROTOCOL_v3.md)를 따릅니다. 이 문서는 이전 버전의 설명 또는 비교·검증 기록입니다.

각 손의 스케치는 별도 폴더에 있으며 다른 폴더 없이도 컴파일할 수 있도록 필요한 헤더를 모두 포함한다.

| 업로드 대상 | Arduino IDE에서 열 파일 | 노드 | 목표 CAN ID | PC 프로필 |
|---|---|---:|---|---|
| 왼손 ESP32 | [DexHandLeft/DexHandLeft.ino](../DexHandLeft/DexHandLeft.ino) | 60 | 0x33C | `profiles/left_hand.json` |
| 오른손 ESP32 | [DexHandRight/DexHandRight.ino](../DexHandRight/DexHandRight.ino) | 61 | 0x33D | `profiles/right_hand.json` |

양손의 손가락 GPIO 배치, 펄스 허용 범위와 CAN 노드는 다르다. 엄지 끝은 양손 GPIO18이며 CAN은 기존 배선의 TX GPIO1 / RX GPIO3이다. 오른손 기존 시험 번호와 CAN 배열 순서는 [오른손 배선표](RIGHT_HAND_PINMAP.md)에서 확인한다. [GPIO1/3 업로드·배포 안내](CAN_WIRING_GPIO1_3.md)를 먼저 따른다.

## CAN ID 상수 위치

각 `.ino` 상단의 `CAN_CONFIG_BEGIN` 블록에서 지정한다. 이 값이 실제 수신 처리와 하드웨어 필터에 연결되어 있다.

| 상수 | 왼손 | 오른손 |
|---|---|---|
| `CAN_NODE_ID` | 60 | 61 |
| `CAN_CONTROL_ID` | 0x03C | 0x03D |
| `CAN_POSE_ID` | 0x33C | 0x33D |

장치 번호는 제어 프레임 두 번째 바이트 검사에도 쓰인다. 현재 PC `HandSender`는 프로필의 장치 번호와 기존 ID 공식으로 송신하므로 값을 바꾸면 PC 송신 설정도 일치시켜야 한다. 재생성 도구는 왼손 60/오른손 61의 기본 상수를 다시 생성한다. `.ino`에서 수동 변경한 ID는 재생성 시 기본값으로 돌아간다.

## 업로드

1. 모든 전원을 끄고 업로드할 ESP32의 GPIO1/3과 트랜시버 연결을 분리한 뒤 ESP32만 USB로 연결한다. 서보 전원은 끈 상태로 작업한다.
2. 위 표에서 맞는 `.ino`를 연다. 같은 폴더의 다섯 개 `.h` 파일도 필요하다.
3. 보드는 **ESP32 Dev Module**, 코어는 **esp32 by Espressif Systems 3.3.12**를 선택한다.
4. 연결한 보드의 실제 COM 포트를 선택해 업로드한다. 양손을 한 번에 연결했다면 어떤 포트가 어느 보드인지 확인한다.
5. 업로드 후 USB·전원을 분리한 상태에서 기존 배선을 다시 연결하고 규격에 맞는 배터리 전원을 공급하면 자동 실행한다. 첫 정상 CAN 목표가 오면 PWM을 시작한다. 운전 중 ESP32 USB와 UART0 시리얼 모니터를 사용하지 않는다.

최초 업로드에는 USB가 필요하다. 운전 중에는 USB와 Serial 모니터를 기다리지 않는다. 이 코드에는 자동 손가락 시험 움직임이나 임의 초기 자세가 없다. SCS2332 손목은 이번 스케치의 범위에 포함하지 않는다.

## 수정과 재생성

업로드 폴더 두 개는 공통 코드를 복사하고 손별 프로필을 적용해 생성했다. 공통 동작을 바꾸려면 `firmware/DexHandCan` 폴더의 원본을 수정한다. GPIO/범위는 `profiles/left_hand.json`, `profiles/right_hand.json`을 수정한다. 이 소스 경로와 아래 명령은 안내사항 폴더가 아닌 dexhand_control 프로젝트 루트 기준이다.

```bash
python tools/prepare_firmware.py
python tools/prepare_firmware.py --check
```

한 손만 재생성하려면 `--hand left` 또는 `--hand right`를 지정한다. 생성된 업로드 폴더를 직접 수정하면 재생성 시 덮어쓰므로 공통 원본과 프로필을 수정한다.

각 `.ino`에는 예상 노드에 대한 컴파일 검사가 있어 오른손 헤더를 왼손 스케치에 넣는 실수를 잡는다. 프로필 SHA256은 각 `BoardConfig.h`에 기록된다. 실제 GPIO 배선과 기구 상태가 올바른지는 별도 확인이 필요하다.

`DexHandCan`은 기존 왼손 호환 스케치이자 공통 소스다. 새로 업로드할 때는 위의 `DexHandLeft`, `DexHandRight`를 사용한다.

## CLI 컴파일

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/DexHandLeft
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/DexHandRight
```

PC에서도 전송 대상에 맞는 JSON 프로필을 선택한다. 오른손은 `--profile profiles/right_hand.json`을 지정해야 한다. 컴파일과 소프트웨어 검사 결과는 [검증 기록](VERIFICATION.md)에 기록한다.
