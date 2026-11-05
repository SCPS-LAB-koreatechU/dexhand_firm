# ESP32 손 제어 코드 구성

> 2026-11-04 이후의 현재 코드는 [보완본 안내](CURRENT_SETUP_20261104.md), [프로토콜 3 명세](PROTOCOL_v3.md)를 따릅니다. 이 문서는 이전 버전의 설명 또는 비교·검증 기록입니다.

2026-11-03. 대상: ESP32-WROOM-32E 계열, 왼손 ES3352 15개. PC는 PWM 목표를 계산하고, ESP32는 CAN 수신·검사·출력을 처리한다.

왼손·오른손 업로드 파일은 각각 `firmware/DexHandLeft/DexHandLeft.ino`, `firmware/DexHandRight/DexHandRight.ino`이다. 아래 설명은 양손이 공유하는 원본 `firmware/DexHandCan`의 코드 구조다. 손별 노드·GPIO·범위는 각 업로드 폴더의 `BoardConfig.h`에 들어 있다. 공통 소스나 프로필을 바꾸면 `tools/prepare_firmware.py`로 업로드 폴더를 다시 생성한다.

실제 운전에 사용하는 CAN 장치 번호·제어 ID·목표 ID는 각 `.ino` 맨 위의 `CAN_NODE_ID`, `CAN_CONTROL_ID`, `CAN_POSE_ID` 상수에서 지정한다. `BoardConfig.h`의 노드 번호는 생성 프로필의 메타데이터이자 좌우 헤더 혼용 검사에 사용한다. CAN 수신 필터와 `HandController`는 `.ino`의 상수 값을 사용한다.

## 코드 흐름

```text
배터리 전원 → setup()
                ├─ PWM 출력 OFF
                └─ CAN 1Mbps 수신 시작

loop()
  ├─ CAN 드라이버 상태/버스 복구 확인
  ├─ 완성 목표가 1초간 없으면 수신 상태 초기화, 마지막 PWM 유지
  └─ CAN 메시지를 최대 16개씩 꺼내 처리
       ↓
     HandController.receive()
       ├─ ID/프레임 종류/길이 검사
       ├─ START / HOLD / RESUME / OFF 처리
       └─ PoseReceiver.accept()
            ├─ 같은 순번의 그룹 0~4를 임시 저장
            ├─ 모터별 허용 범위·중복·순번·조립 마감 검사
            └─ 15개 목표가 완성되면 ServoOutput.apply() 즉시 호출
                 ↓
               GPIO 15개 LEDC PWM 갱신
```

## 파일별 역할

| 파일 | 역할 |
|---|---|
| `firmware/DexHandCan/DexHandCan.ino` | Arduino 진입점, TWAI CAN 드라이버, 버스 복구, 프레임 수신 루프 |
| `firmware/DexHandCan/HandController.h` | 손 명령 구분, 완성 목표 적용, 출력 허용 상태와 통신 타임아웃 |
| `firmware/DexHandCan/PoseReceiver.h` | 프레임 5개 조립, 순번·범위 검사, 완성된 15개 값 반환 |
| `firmware/DexHandCan/ServoOutput.h` | GPIO에 LEDC 15채널 설정, µs→duty 변환, PWM 중단 |
| `firmware/DexHandCan/BoardConfig.h` | JSON 프로필에서 생성한 노드 번호·CAN 핀·서보 핀·허용 범위 |

설정 원본은 `profiles/left_hand.json`이다. 핀이나 허용 범위를 변경하면 `tools/generate_board_config.py`로 헤더를 재생성한다. 왼손 노드는 60, CAN TX는 GPIO1, RX는 GPIO3이다. 양손 스케치는 `tools/prepare_firmware.py`로 재생성한다. 서보 순서는 README 표의 모터 1~15이며 엄지 끝은 GPIO18이다.

## 목표를 적용하는 시점

하나의 CAN 목표 프레임에는 세 모터의 uint16 PWM 펄스폭(µs)이 들어간다. 같은 순번의 다섯 프레임이 모여야 한 자세가 완성된다. `PoseReceiver`는 완성 전에 출력 배열을 갱신하지 않으며, `HandController`는 `Complete` 반환 시 바로 `ServoOutput.apply()`를 호출한다.

조립 제한 20ms는 다섯 프레임이 모일 수 있는 최대 시간이다. 완성 후 추가로 20ms를 기다리지 않는다. 다음 송신 주기는 PC가 관리한다. ESP32는 별도의 50Hz 목표 적용 타이머 없이 새 완성 목표를 받는 즉시 처리한다.

PWM 펄스는 LEDC 하드웨어가 50Hz로 계속 생성한다. CPU는 새 목표가 들어오면 duty를 바꾼다. 15개 duty는 같은 함수 호출에서 순서대로 갱신되며 물리적인 PWM 에지 또는 모터 도착 시각을 동기화하는 기능은 아니다. 정상 루프 끝에는 `delay(1)`이 있으며 실제 지연에는 OS 스케줄링·USB-CAN·버스 경쟁·PWM 주기·서보 응답이 더해진다. 지연 측정은 아직 하지 않았다.

## 부팅·정지·복구

- 전원 투입 시 PWM을 끄고 CAN을 받는다. USB, Serial, Wi-Fi, BLE 연결을 기다리지 않는다.
- 첫 완성 목표가 도착하면 PWM을 자동 시작한다. 임의 초기 자세를 출력하지 않는다.
- 누락·범위 위반·충돌하는 중복 그룹·조립 마감 초과가 있는 세트는 반영하지 않는다. 이전 출력은 유지된다.
- 1초 동안 새 완성 목표가 없으면 조립 상태와 수신 큐를 초기화한다. 마지막 PWM은 유지한다. 새 완성 목표를 받으면 자동 재개한다.
- HOLD는 PWM을 유지하며 목표 수신 적용을 차단한다. OFF는 PWM을 중단하며 목표 수신 적용을 차단한다. START/RESUME로 다시 허용한다.
- LEDC 설정 또는 갱신 실패 시 PWM을 중단하고 START/RESUME까지 적용을 차단한다.
- CAN bus-off 복구 또는 수신 큐 초과 시 미완성 세트를 폐기한다. 이 사유만으로 임의 자세로 이동하지 않는다.

CAN ID, 패킷 바이트, 제어 명령은 `PROTOCOL.md`에 있다. 기존 PC의 `HandSender.write_us()`와 그대로 호환된다. ESP32 응용 프로그램은 상태 응답을 보내지 않으며 CAN 하드웨어 ACK는 동작한다. 손목 SCS2332 처리는 현재 범위에 포함하지 않는다.

## 빌드와 업로드

Arduino IDE에서는 대상 손에 맞는 `DexHandLeft.ino` 또는 `DexHandRight.ino`를 열면 같은 폴더의 헤더가 함께 포함된다. 보드는 일반 **ESP32 Dev Module**, 코어는 **Arduino-ESP32 3.3.12**를 기준으로 한다.

```bash
python tools/prepare_firmware.py --check
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/DexHandLeft
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/DexHandRight
```

최초 업로드에는 USB 또는 외부 프로그래머가 필요하다. 업로드 이후에는 배터리 전원만으로 실행한다. 이번 작업에서는 업로드와 실물 구동을 수행하지 않았다. 검증 결과는 `VERIFICATION.md`를 참고한다.
