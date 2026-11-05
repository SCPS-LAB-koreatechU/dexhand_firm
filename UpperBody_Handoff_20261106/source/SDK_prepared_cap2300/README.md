현재 오른손 수신 전용/PWM 작업 분리 안내는 릴리스 루트의 ONEWAY_PWM_안내.md를 보세요. 아래 내용은 이전 배포 기록입니다.

현재 오른손 엄지 값은 2026-11-05 THUMBFIX2_안내.md 및 profiles/right_hand.json을 참고하세요. 아래 내용은 이전 배포 기록입니다.

# 휴머노이드 팔·DexHand 현재 코드

**CAN 기존 배선 수정본: 양손 TX=GPIO1(TX0), RX=GPIO3(RX0).** [업로드·배포 순서와 UART 공유 주의사항](dexhand_control/firmware/안내사항/CAN_WIRING_GPIO1_3.md)을 먼저 확인하세요. 양손 펌웨어와 PC 설정을 함께 갱신해야 합니다.

[손 GUI: 개별 모터·카메라·캘리브레이션](dexhand_control/firmware/안내사항/HAND_GUI.md): `dexhand_control`에서 `bash setup_hand_test.sh --gui --camera`로 설치한 뒤 `bash gui_hand.sh --dry-run`으로 화면을 확인합니다. 실제 연결은 `bash gui_hand.sh --hand right`입니다.

[간단한 손 CAN·USB 카메라 테스트](dexhand_control/firmware/안내사항/HAND_TEST_QUICKSTART.md): `dexhand_control`에서 `bash setup_hand_test.sh --camera`, `bash test_hand.sh right ping`으로 시작합니다.

2026-11-04 / **SDK 0.2.0** 보완본. 휴머노이드 PC 코드는 **humanoid_sdk-main 전체**를 사용합니다.

- [현재 업로드·실행·복구 가이드](dexhand_control/firmware/안내사항/CURRENT_SETUP_20261104.md)
- [양팔·양손 공통 API·단위·시간·고정 영점·검증 가이드](dexhand_control/firmware/안내사항/UPPER_BODY_SDK_v020.md)
- [PC 개발자용 CAN 프로토콜 3](dexhand_control/firmware/안내사항/PROTOCOL_v3.md)
- [왼손 업로드 스케치](dexhand_control/firmware/DexHandLeft/DexHandLeft.ino)
- [오른손 업로드 스케치](dexhand_control/firmware/DexHandRight/DexHandRight.ino)
- [휴머노이드 PC 전체 SDK](humanoid_sdk-main/README.md)

각 Arduino 스케치는 폴더 전체를 유지하세요. ESP32-WROOM / Arduino-ESP32 3.3.12 기준입니다. 배터리로 켜지면 CAN 수신을 자동 시작하며, 정상 목표 전에는 PWM을 출력하지 않습니다.

프로젝트에는 현재 SDK, 양손 펌웨어, 실행·개발 문서와 관련 도구를 유지합니다. 이전 V2 원본, 백업, 중복 압축 해제본과 구버전 ZIP은 사용자 요청으로 삭제했습니다. `arm_diagnostics/`는 팔 상태 수집·Recoil 진단 도구입니다.

ROS 2 실환경·실제 CAN·실물 모터 검증은 별도이며, 중력 보상·전신 충돌 회피·SCS2332 손목 구동은 이 보완본에 포함하지 않았습니다.

오른손 손가락 배선 revision 2: 검지 위/아래는 GPIO32/25입니다. GPIO34/35는 PWM 출력 불가입니다. CAN은 양손 모두 TX=GPIO1 / RX=GPIO3입니다. 휴머노이드 PC로 옮길 압축파일은 UpperBody_SDK_0.2.0_20261104_pinrev2_handdemo1_gui1_can13.zip입니다. 옆의 .zip.sha256은 무결성 확인용입니다. 양손 펌웨어와 PC 프로필을 함께 갱신하세요.
