# 오른손 rightfix1 펌웨어: SDK 담당자 수정 안내

2026-11-04. **이번에는 펌웨어와 이 안내 문서만 수정했다. SDK 코드·YAML·GUI 파일은 수정하지 않았다.**
PC 담당자는 아래 설정과 의미를 함께 반영해야 한다. 프로필 해시 검사만 우회하거나
YAML 값만 바꾸고 엄지 채널 의미를 그대로 두면 올바른 손 제어가 되지 않는다.

배포 파일: `DexHand_Firmware_20261104_rightfix1.zip`.
양손 스케치 폴더 전체, 양손 JSON 프로필, 이 안내와 배선표, 펌웨어 검사 매니페스트가 들어 있다.
SDK는 이 ZIP에 포함하지 않는다. 이전 전체 SDK ZIP에는 이번 오른손 수정이 반영되어 있지 않다.

## 1. 확정된 배선과 변경

- 양손 CAN: **TX GPIO1 / RX GPIO3**, 1Mbps, 표준 11비트 프로토콜 3 유지.
- 노드: 왼손 60, 오른손 61 유지. CAN ID와 패킷 바이트 형식은 변경하지 않았다.
- 오른손 검지: 사용자가 확인한 **위 GPIO32 / 아래 GPIO25** 유지. GPIO34/35는 사용하지 않는다.
- 오른손 엄지: GPIO와 해당 GPIO의 PWM 끝점은 그대로 두고 실제 동작에 맞춰 이름을 바꿨다.
- 오른손 약지: 위/아래에 잘못 기록한 **풀림·당김 값 쌍 전체를 서로 교환**했다.
- 왼손의 GPIO·보정값·모터 의미는 그대로다.

| 오른손 CAN 채널(1부터) | 실제 의미 | GPIO | 풀림 µs | 당김 µs |
|---:|---|---:|---:|---:|
| 3 | 약지 위 | 23 | **850** | **1725** |
| 4 | 약지 아래 | 26 | **1700** | **800** |
| 7 | 검지 위 | 32 | 1750 | 700 |
| 8 | 검지 아래 | 25 | 700 | 1725 |
| 13 | **엄지 왼쪽 당김** | **13** | 2300 | 700 |
| 14 | **엄지 끝** | **14** | 2100 | 700 |
| 15 | **엄지 오른쪽 당김** | **18** | 700 | 1900 |

전체 오른손 표는 [오른손 핀·보정 표](RIGHT_HAND_PINMAP.md)를 참고한다.
실제 GPIO 변경은 펌웨어의 JSON 프로필 수정·재생성·재업로드로 적용한다.
PC SDK는 GPIO 번호를 모터에 보내지 않고, 정해진 CAN 배열 순서의 목표를 보낸다.
이번에는 GPIO 배열이 그대로이므로 SDK에서 다른 GPIO로 재배치하면 안 된다.
엄지 보정값을 물리적으로 다시 측정한 것은 아니다. 사용자가 요청한 대로
기존 GPIO별 PWM 값은 유지하고 이름만 바로잡았다. 새 값 확인이 필요하면 개별 캘리브레이션한다.

## 2. SDK YAML과 프로필 해시

대상: `humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk/config/dexhand.yaml`.
아래는 `hands.right`에 들어갈 값이다. `control`, 왼손, CAN 인터페이스 이름은 기존 설정을 유지한다.

```yaml
hands:
  right:
    node: 61
    profile_tag: b09f98b0
    open_us: [850, 1700, 850, 1700, 1750, 850, 1750, 700, 700, 700, 2000, 2000, 2300, 2100, 700]
    close_us: [1775, 800, 1725, 800, 825, 1750, 700, 1725, 2100, 2000, 750, 800, 700, 700, 1900]
```

왼손 `profile_tag`는 **a5b23746**, 오른손은 **b09f98b0**이다.
`8f317a7f`는 이번 수정 전 오른손 태그다. 새 펌웨어는 새 태그로 PONG에 응답한다.
태그는 JSON 전체의 canonical SHA256 앞 8자리이며 단순 PWM 값뿐 아니라 모터 이름도 포함한다.

현재 프로젝트의 `dexhand_control/profiles/right_hand.json`과 ZIP의 같은 프로필에는
수정값이 들어 있다. SDK 담당자가 아래 명령으로 YAML을 생성할 수도 있다.

```bash
cd ~/project_humanoid/dexhand_control
python tools/generate_sdk_config.py
```

**이번 작업에서는 이 명령을 실행하지 않았다.** 이 명령도 YAML만 생성하므로 아래 엄지
제한·자세 변환 코드는 SDK 담당자가 별도로 반영해야 한다. GUI가 JSON 프로필을
읽는 경우 동봉된 수정 JSON을 함께 배포한다. 펌웨어만 새로 올린 상태에서 기존 SDK가
`PROFILE_MISMATCH`로 거절하는 것은 예상되는 동작이다. 검사를 제거하지 않는다.

## 3. 손마다 다른 엄지 채널과 당김 제한

| 손 | CAN 13 | CAN 14 | CAN 15 | 반대 방향 당김 쌍 |
|---|---|---|---|---|
| 왼손 | 오른쪽 당김 | 왼쪽 당김 | 엄지 끝 | **13·14** |
| 오른손 | 왼쪽 당김 | 엄지 끝 | 오른쪽 당김 | **13·15** |

`당김 비율 = (목표µs - 풀림µs) / (당김µs - 풀림µs)`로 계산한다.
반대 방향 당김 쌍의 비율 합은 1 이하여야 한다. 오른손 **14번은 끝 모터이므로
좌우 당김 제한의 합에 넣지 않는다.** 펌웨어는 이 쌍을 이미 반영했으며
SDK를 거치지 않은 CAN 명령도 검사한다.

현재 SDK의 `control.antagonist_pairs: [[13, 14]]`는 모든 손에 공통이어서
그대로 오른손에 사용하면 잘못된 모터 쌍을 검사한다. 글로벌 값을 `[[13,15]]`로만
바꾸면 왼손이 틀어지므로 **컨트롤러별 설정을 분리**해야 한다.

대상: `bhl_hand_sdk/bhl_hand_sdk/controller.py`의 `HandController.__init__`.
아래는 현재 로봇 배치에 맞춘 최소 변경 예시다. `copy`는 이 파일에서 이미 import한다.
기존 생성자의 `self.config = config`, `self.spec = config.hand(hand)` 부분을 대체한다.

```python
self.config = copy.deepcopy(config)
self.spec = self.config.hand(hand)
if hand == "right":
    self.config.antagonist_pairs = [(13, 15)]
    # 공통 grip 템플릿의 엄지 순서는 [오른쪽, 왼쪽, 끝].
    # 오른손 실제 CAN 순서는 [왼쪽, 끝, 오른쪽].
    self.config.grips = {
        name: list(g[:12]) + [g[13], g[14], g[12]]
        for name, g in self.config.grips.items()
    }
```

같은 생성자의 `open_pose = list(config.grips["open"])`도
`open_pose = list(self.config.grips["open"])`로 바꾼다.
공통 `config` 객체나 YAML의 공통 `grips`를 제자리에서 회전시키지 않는다.
양손을 같은 프로세스에서 만들 때 오른손 변환이 왼손에 전파되면 안 된다.

이렇게 손별 복사본을 사용하면 `set_goal`, `_send_set`, 재접속 시 유지 PWM 검증,
`HandAdapter.validate`가 각각 올바른 손의 제한을 사용한다.
더 일반적인 구현은 손별 `antagonist_pairs`와 `channel_names`를 `HandSpec`에 넣고
각 호출에서 선택한 손의 규칙을 사용하는 방식이다. 어느 방식을 택하든 위 표와 같아야 한다.

## 4. 이름 있는 자세와 카메라

`power`, `pinch`, `point` 같은 공통 grip 템플릿이 기존의 [오른쪽, 왼쪽, 끝] 순서를
유지한다면 **오른손에 적용할 때 마지막 3개를 [왼쪽, 끝, 오른쪽]으로 변환**한다.
위 생성자 예시가 이 변환을 포함한다. `set_grip()`에서 다시 변환하면 안 된다.

예: 공통 템플릿의 엄지 `[1, 0, 1]`은 오른손 실제 CAN에서 `[0, 1, 1]`이어야 한다.
즉 GPIO13(왼쪽)은 풀고 GPIO14(끝)와 GPIO18(오른쪽)을 당긴다.
현재 카메라 데모는 `set_grip("power", level)`을 호출하므로 이 변환이 반영되어야 한다.

반면 SDK `set_goal()` / ROS `target` / `target_us` / CAN raw 배열은 **실제 CAN 채널 순서**다.
이미 오른손 실제 배열을 만든 입력을 또 회전시키면 안 된다. GUI의 개별 모터 슬라이더는
JSON의 이름과 고정된 채널/GPIO를 사용하면 된다. GUI에서 엄지 이름이 GPIO13=왼쪽,
GPIO14=끝, GPIO18=오른쪽으로 보이는지 확인한다.

`config.py`의 공통 `CHANNEL_NAMES`를 외부 SDK API나 화면에서 사용한다면 오른손 이름
배열도 별도로 선택해야 한다. 오른손 마지막 세 영문 이름은
`thumb_pull_left`, `thumb_tip`, `thumb_pull_right`이며 왼손은 기존 순서다.

## 5. 양팔·양손 통합 계약 검사

대상: `bhl_upper_body_sdk/bhl_upper_body_sdk/contract.py`의 `validate()`.
현재 `values[12] + values[13]`을 모든 손에 적용하는 부분도 수정해야 한다.
SDK와 CAN 배열은 위 표의 손별 실제 순서라는 전제로 다음처럼 검사한다.

```python
a, b = (12, 14) if part == "hand_right" else (12, 13)  # Python은 0부터
if values[a] + values[b] > 1 + 1e-9:
    raise ValueError("opposing thumb pull sum exceeds 1")
```

`hand_right`에 보내는 원시 15개 목표의 마지막 3개는 [왼쪽, 끝, 오른쪽]이다.
VLA나 상위 프로그램이 양손의 엄지 순서를 동일하다고 가정하면 여기서 변환해야 한다.
팔 관절 명령 형식, CAN ID와 전송 주기는 이 수정으로 변경할 필요가 없다.

## 6. SDK 적용 후 확인 항목

1. PONG의 오른손 태그가 `b09f98b0`이며 SDK 설정과 일치하는지 확인한다.
2. GPIO23 약지 위가 850→1725, GPIO26 약지 아래가 1700→800인지 확인한다.
3. 오른손 13·15번을 합 1 초과로 당기는 목표는 PC와 펌웨어 모두 거절해야 한다.
4. 오른손 13·14번 또는 14·15번 조합은 끝 모터를 이유로 잘못 거절하면 안 된다.
5. 왼손의 13·14번 제한과 기존 동작은 유지되어야 한다.
6. `power`/카메라 목표가 오른손에서 좌우 당김 두 모터를 혼동하지 않는지 확인한다.
7. 기존 일부 SDK 테스트도 오른손 당김 쌍을 13·15로 바꾸고, 끝 모터 독립 동작과
   양손 설정 분리 테스트를 추가한다. 펌웨어 wire harness는 실제 생성된 쌍을 사용하도록 갱신했다.
8. ROS 설치본은 다시 빌드하여 설치된 YAML과 Python 코드까지 갱신한다.

## 7. 업로드 및 검사 범위

각 손의 `.ino`는 같은 폴더의 `.h` 파일 5개를 유지한 채 Arduino IDE에서 연다.
ESP32 Dev Module / Arduino-ESP32 3.3.12를 사용한다. GPIO1/3은 USB 업로드 UART와
공유되므로 모든 전원을 끄고 외부 CAN 트랜시버 연결을 분리한 뒤 업로드한다.
USB·전원을 끈 뒤 기존 배선을 복구하고 배터리로 실행한다. Serial 로그를 껐다고
보드의 USB-UART 회로가 전기적으로 분리되는 것은 아니다.

새 펌웨어의 수신기·출력 상태기계·17개 ACK 사례와 양손 실제 프로필의 엄지 쌍을
native C++로 검사한다. 실제 보드 업로드·CAN 왕복·모터 동작은 별도 검증이 필요하다.
이번 양손 스케치는 Arduino-ESP32 3.3.12에서 각각 flash 276531바이트,
전역 메모리 22552바이트로 컴파일 통과했다. 펌웨어 native 검사 5개 묶음도 통과했다.
SDK 폴더 95개 파일은 수정 전 SHA256과 모두 일치한다.
이번 펌웨어 전용 검사 명령은 프로젝트에서 다음과 같다.

```bash
python dexhand_control/tools/verify_firmware_only.py
```

기존 전체 `verify.py`는 아직 수정하지 않은 SDK YAML과 새 펌웨어의 불일치를
검출할 수 있으므로, SDK 담당자가 위 변경을 완료한 다음 전체 검사를 다시 실행한다.
