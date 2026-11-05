# 오른손 엄지 값 수정 · 2026-11-05

이번 수정은 사용자가 확인한 기존 풀림→당김 값 쌍을 옮긴 것입니다. GPIO와 모터 이름은 유지했습니다.

| 실제 채널 | 의미 | GPIO | 풀림 µs | 당김 µs |
|---:|---|---:|---:|---:|
| 13 | 엄지 왼쪽 당김 | 13 | 2100 | 700 |
| 14 | 엄지 TIP/끝 | 14 | 700 | 1900 |
| 15 | 엄지 오른쪽 당김 | 18 | 2300 | 700 |

검지 위 GPIO32 / 아래 GPIO25, 약지 및 나머지 손가락 값, 왼손 설정은 유지했습니다.
엄지 왼쪽·오른쪽 당김 쌍은 계속 채널13·15입니다. TIP은 이 쌍에 포함하지 않습니다.

SDK YAML, 오른손 JSON, 직접 CAN 전송 예제의 전체 펼침 배열을 같은 값으로 갱신했습니다.
기본 GUI와 손가락 테스트 CLI는 공통 SDK 설정을 읽으므로 같은 끝점을 사용합니다.
엄지 전용 캘리브레이션 버튼·창·모듈은 제거했습니다. CAN 연결/재연결/HOLD/현재 자세 이어받기는 유지했습니다.

휴머노이드 PC 실행:
```bash
cd ~/humanoid_sdk/../dexhand_control
bash gui_hand.sh --hand right --channel can1
# 통신 확인만 하는 CLI (출력 시작 안 함)
bash test_hand.sh right ping --channel can1
```

먼저 연결 확인 후 현재 PWM 이어받기 또는 의도적인 손 펼치기를 선택합니다.
현재 PWM 이어받기는 보드에 저장된 자세를 그대로 읽습니다. 설정이 바뀌었다고 자동으로 새 풀림 값으로 이동하지 않습니다.

## 펌웨어 반영 필요

새 오른손 프로필 태그는 `79610de6`입니다. 이전 rightfix1 태그 `b09f98b0`와 다릅니다.
현재 ESP32의 펌웨어가 이전 값이면 PROFILE_MISMATCH로 출력 시작이 차단됩니다.
태그 검사만 바꾸거나 우회하면 보정 방향과 허용 범위가 서로 달라집니다.
새 `dexhand_control/profiles/right_hand.json`의 값을 기존 rightfix1 지원 펌웨어에 반영하고 재업로드해야 합니다.
SDK 파일 복사만으로 ESP32 내부 값이 바뀌지 않습니다. 이번 작업에서 업로드나 모터 동작은 수행하지 않았습니다.
펌웨어에 전달할 전체 값과 태그는 `deployment/right_hand_firmware_values.json`에 정리했습니다.
이 SDK에 들어 있던 구형 `firmware/DexHandRight` 스케치는 rightfix1 이전 사본입니다. 업로드하지 마세요.
이전 배포 문서의 엄지 값은 과거 기록입니다. 현재 설정은 이 안내 및 profiles/right_hand.json을 기준으로 합니다.

## 검증

로컬 SDK 190개, CAN 전송 도구 17개 테스트 통과.
모의 GUI에서 캘리브레이션 UI 제거, GPIO 유지, 새 TIP 목표 계산, 다른 14개 목표 유지,
HOLD/이어받기, 재연결 후 자동 출력 없음 검증 통과.
GUI 로그: ~/bhl_logs/hand_gui/*.jsonl
