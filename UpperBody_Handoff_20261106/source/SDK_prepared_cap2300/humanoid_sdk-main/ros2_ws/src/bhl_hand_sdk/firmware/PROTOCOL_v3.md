# DexHand CAN 프로토콜 3 — PC 개발자용

2026-11-04. 현재 생성된 DexHandLeft/Right와 통합 PC SDK가 사용하는 명세다. 설치·실행은 프로젝트의 `dexhand_control/firmware/안내사항/CURRENT_SETUP_20261104.md`를 함께 읽는다. 이 명세는 SDK에도 같은 내용으로 복사된다. 프로토콜 2 원본은 이력용이다.

표준 11비트 CAN, **1Mbps**, ID=`(function << 7) | node`, 정수는 little-endian. 왼손 node=60(0x3C), 오른손 node=61(0x3D). ES3352 손가락 15개만 제어한다.

| 방향 | 종류 | function | 왼손 ID | 오른손 ID | DLC |
|---|---|---:|---:|---:|---:|
| PC→손 | CONTROL | 0 | 0x03C | 0x03D | 3 권장 / 2 호환 |
| PC→손 | POSE | 6 | 0x33C | 0x33D | 8 |
| PC→손 | PING / QUERY | 4 | 0x23C | 0x23D | 1 / 2 |
| 손→PC | PONG | 3 | 0x1BC | 0x1BD | 8 |
| 손→PC | STATUS | 5 | 0x2BC | 0x2BD | 8 |
| 손→PC | STATE | 8 | 0x43C | 0x43D | 8 |
| 손→PC | BOOT | 9 | 0x4BC | 0x4BD | 8 |

## PC→손

POSE 한 프레임: `[sequence_u8, 0x10|group, us0_u16, us1_u16, us2_u16]`. group=0..4이며 채널 1..15를 3개씩 싣는다. 같은 순번의 5개 그룹을 첫 그룹부터 20ms 이내에 보낸다. 다섯째 정상 그룹을 받으면 즉시 목표를 확정한다. 추가 20ms를 기다리지 않는다.

채널 순서: 소지 위·아래, 약지 위·아래, 중지 위·아래, 검지 위·아래, 검지 끝, 중지 끝, 약지 끝, 소지 끝, 엄지 오른쪽 당김, 엄지 왼쪽 당김, 엄지 끝.

순번은 세트마다 증가하고 255 다음 0이다. 새 순번과 이전 순번의 차이를 8비트로 계산해 1..127이면 새 세트, 128..255면 과거/모호한 세트로 무시한다. 완성·거부된 순번은 다시 적용하지 않는다. 같은 그룹의 다른 값, 범위·형식·마감 오류는 해당 순번을 폐기한다. 프레임 일부만 빠지고 후속 프레임도 없으면 거부 응답 없이 미완성으로 남는다.

각 채널은 보정 범위를 검사한다. 13·14번은 각각 `fraction=(us-open_us)/(close_us-open_us)`로 환산한 합이 1 이하여야 한다. 정수 µs 반올림 오차만 허용한다. 이 제한은 실제 힘 센서나 전체 기구 충돌 검사를 대신하지 않는다.

CONTROL 권장: `[mode, node, nonce_u8]`. STATUS의 CONTROL_ACK는 byte0에 같은 nonce, byte3에 mode를 반환한다. 보내기 전에 큐에 남은 과거 응답을 처리하고 새 nonce를 사용하여 제한된 시간 안에 일치하는 응답을 기다린다. 기존 `[mode,node]`도 처리하며 이 경우 응답 byte0은 0이다.

| mode | 의미 |
|---:|---|
| 0x00 OFF | PWM 끄기, 새 POSE 억제, 마지막 유효 목표 표시 해제 |
| 0x01 START | PWM 끄고 조립 순번 초기화, 새 POSE 허용. 기존 파지 중 재접속에는 사용하지 않음 |
| 0x02 HOLD | PWM 유지, 새 POSE 억제 |
| 0x13 RESUME | PWM 유지, 조립 순번 초기화, 새 POSE 허용 |
| 0x03 CLEAR_FLAGS | 넘침·CAN 복구 이력 플래그만 해제. PWM·억제·조립 상태 유지 |

PING: `[0xCA]`. QUERY: `[0xCB, nonce_u8]`이며 아래 STATE 스냅샷을 요청한다. PING·QUERY는 PWM을 바꾸거나 watchdog을 연장하지 않는다.

## 손→PC

PONG: `[0xCA, 3, node, flags, profile_tag_4bytes]`.
현재 태그는 왼손 **a5b23746**, 오른손 **8f317a7f** (손가락 배선 revision 2, 양손 CAN TX GPIO1 / RX GPIO3). JSON 프로필 SHA-256의 앞 4바이트다. SDK는 버전·태그가 다르면 활성화를 거절한다.

STATUS: `[sequence_or_control_nonce, result, flags, detail, applied_count_u16, tx_errors_u8, rx_errors_u8]`.

| result | 의미 / detail |
|---:|---|
| 0x00 APPLIED | 완성 목표 검사 통과, PWM 기록 성공 / 0xFF |
| 0x01 REJECT_RANGE | 보정 범위 위반 / 채널 0..14 |
| 0x02 REJECT_DEADLINE | 조립 20ms 초과 / 0xFF |
| 0x03 REJECT_INHIBITED | HOLD/OFF 상태 / 0xFF |
| 0x04 REJECT_CONFLICT | 같은 그룹에 다른 값 / 채널 0..14 |
| 0x05 REJECT_FORMAT | 버전 니블·그룹 오류 / 0xFF |
| 0x06 OUTPUT_FAIL | LEDC 기록 실패, PWM 중지·출력 억제 / 0xFF |
| 0x07 REJECT_PAIR | 엄지 양쪽 당김 합 제한 / 12(채널13) |
| 0x10 CONTROL_ACK | 제어 명령 처리 / mode |
| 0x20 HEARTBEAT | 유휴 상태 응답 / 0xFF |

유효한 결과는 같은 POSE 순번당 한 번이다. HOLD/OFF 중에는 연속해서 수신하는 같은 순번에 대해 INHIBITED를 한 번 보낸다. 잘못된 DLC·외부 노드·확장/RTR·미지 모드 등은 조용히 무시할 수 있으므로 항상 REJECT가 돌아온다고 가정하지 않는다.

| flag bit | 의미 |
|---:|---|
| 0 | PWM_ON |
| 1 | INHIBITED |
| 2 | WATCHDOG_EXPIRED: 마지막 적용 후 통신 만료, 새 정상 적용으로 해제 |
| 3 | RX_OVERFLOW: CLEAR_FLAGS까지 유지 |
| 4 | CAN_RECOVERED: 실제 시작 이후 복구 이력, CLEAR_FLAGS까지 유지 |
| 5 | HAVE_POSE: 마지막 유효 PWM 목표 읽기 가능 |

applied_count는 16비트로 순환한다. STATUS 송신은 비차단·best effort이며 큐에 넣었다고 PC 도착이 보장되지는 않는다. 넘침·복구 플래그는 큐 등록 성공만으로 해제하지 않는다. 500ms 동안 STATUS를 큐에 넣지 못했거나 유휴 상태면 heartbeat를 시도한다.

BOOT: `[boot_id_u32, uptime_ms_u32]`. 초기 통신과 약 1초마다 보낸다. boot_id는 ESP32 기동 시 생성한 32비트 난수다. 다른 boot_id를 받으면 기존 미확인 목표·횟수 기준을 폐기하고 고장을 래치한다. 예전 목표를 자동 재생하지 않는다. BOOT가 유실될 수도 있으므로 불가능한 적용 횟수 점프도 재동기화 사유로 처리한다.

STATE는 **한 QUERY 처리 시점의 마지막 명령 스냅샷**이며 다음 프레임을 순서대로 큐에 넣는다.

| tag(byte1) | payload 전체 |
|---|---|
| 0xF0 | `[nonce, 0xF0, boot_id_u32, applied_count_u16]` |
| 0xF1 | `[nonce, 0xF1, flags, last_applied_sequence, 0,0,0,0]` |
| 0..4 | `[nonce, group, us0_u16, us1_u16, us2_u16]` |

HAVE_POSE가 없으면 metadata 두 개만 보낸다. 있으면 총 7개다. 송신 큐가 차면 일부만 오거나 전혀 안 올 수 있다. 같은 nonce의 필수 부분이 모두 모이고 일관성 검사를 통과했을 때만 사용한다. 불완전한 스냅샷으로 기본값을 채우거나 목표를 추정하지 않는다.

**APPLIED·STATE 모두 실제 위치·힘·파지 성공이 아니다.** 손가락 센서 응답이 없는 ES3352의 마지막 PWM 명령에 대한 정보다.

## SDK의 필수 상태 처리

1. 이미 PWM_ON인 손에 새 SDK가 연결되면 PING/태그 확인 → HOLD 확인 → QUERY 완성·검사 → RESUME 확인 → 같은 목표 전송. 새 목표는 그 뒤 PC에서 보간한다.
2. PWM이 꺼진 손은 사용자가 명시한 새 초기 목표가 있어야 START와 첫 세트를 보낸다. 냉기동의 실제 자세는 읽을 수 없다.
3. 보낸 미확인 순번·송신 시각·적용 횟수 기준을 관리한다. 이 SDK는 0.5초 이내 미확인 순번만 확인하며, 중복·만료·보내지 않은 APPLIED는 LOST 카운터를 지우지 못한다.
4. 순번 8비트·횟수 16비트의 순환과 재부팅을 구분한다. 큰 역방향/비현실적 점프를 수만 개 손실로 누적하지 않는다. 통신은 인증된 세션 프로토콜이 아니므로 악의적 프레임 재생을 막는다는 보장은 없다.
5. 기본 50Hz, 15개 목표를 세트 단위로 송신한다. SDK의 응답 대기 기본값은 5ms, 연속 실패 한도는 10세트다. 실제 CAN·OS 지연을 측정해 조정한다.
6. 상위 스트리밍 입력이 0.5초 만료되면 별도로 HOLD한다. CAN 송신 루프가 살아 있는 것과 AI 목표가 유효한 것은 다르다.
7. 손 CAN 목표가 1초 끊기면 ESP32는 마지막 PWM을 유지한다. 안전한 목표 없이 임의로 펼치거나 PWM을 재출력하지 않는다. 물리적 전원 단절·출력 실패는 별개의 사건이다.

프로토콜 1·2 호환 모드는 SDK에 남겼지만 **프로토콜 2는 알 수 없는 기존 파지 자세를 읽을 수 없어 새 프로세스의 복구를 거절**한다. 현재 양손 펌웨어와 SDK는 protocol=3으로 함께 사용한다.

팔과 손의 주소는 현재 겹치지 않으며, 같은 버스의 별도 소켓에서 실행한다. 동시 제어는 가능하지만 여러 프레임의 도착 시간과 모든 모터의 실제 도달 시간이 같다는 뜻은 아니다.

## PC 통합 SDK 0.2.0과의 관계

상위 PC의 bhl.upper_body/1 session/sequence/timestamp/TTL/prepare-commit은 ROS 계층의 계약이다. 이 CAN 패킷에 새 필드를 추가한 것이 아니며 손 펌웨어 프로토콜은 3을 유지한다. PC가 각 CAN 세트의 송신 당시 PC 명령 식별값을 보관하고, APPLIED 응답을 받으면 last_applied_command에 PWM·CAN 순번·PC 식별값을 연결한다. ESP32는 실제 손가락 위치나 힘을 측정/보고하지 않는다. 마지막 PWM 스냅샷도 실제 위치 측정값이 아니다.
