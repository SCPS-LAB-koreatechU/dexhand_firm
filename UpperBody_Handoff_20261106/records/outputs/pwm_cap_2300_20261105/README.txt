오른손 PWM 수신 상한 2300us 패치 (2026-11-05)

요청 범위: 수신 허용 상한만 2300us. GPIO, 기존 open_us/close_us 15쌍, 방향, 카메라 0~100% 목표는 유지한다.
예: 중지 아래 GPIO22의 자동 추종은 계속 850~1750us다. 2300us는 자동 접힘 목표가 아니다. 기존 채널별 최소값도 유지한다.
프로필 식별값: 79610de6 -> bdad07a4. ESP32용 RX-only 펌웨어와 SDK 패치를 함께 사용한다.
현재 실물: 새 펌웨어는 아직 업로드하지 않았고 실행 중 RAM SDK도 아직 전환하지 않았다.

적용 순서
1. 서보 전원을 끄고 DexHandRight_RXOnly_PWMCap2300_20261105.zip을 해제한다.
2. DexHandRight/DexHandRight.ino를 ESP32-WROOM 듀얼코어 보드에 업로드한다. 기존 GPIO1/3 CAN 및 GPIO32/25 검지 배선은 유지한다.
3. 손 제어 GUI와 시험 프로그램을 종료한 상태에서 SDK 패치를 적용한다. 이 스크립트는 CAN/시리얼을 열거나 모터를 움직이지 않는다.
   python3 apply_sdk_patch.py --root <dexhand_control과 humanoid_sdk-main이 있는 SDK 루트> --check
   python3 apply_sdk_patch.py --root <SDK 루트>
현재 RAM 세션을 갱신할 경우 --root /run/user/1000/bhl_hand_camera_session/project --data-dir /run/user/1000/bhl_hand_camera_session/data 를 사용한다.
--data-dir 지정 시 같은 PWM 종점을 쓰는 오른손 대응표의 식별값만 옮긴다. 기록된 모터값/학습 자세는 바꾸지 않는다. 원본은 .pwm_cap2300_backup_*에 보존한다.
4. GUI를 다시 실행한다. 직접 파일 복사 대신 스크립트를 사용하면 체크섬/원본 차이를 확인하고 오류 시 되돌린다.

검증
SDK/PC Python 검사 151개 통과. C++ 실제 수신기 로직으로 15개 채널의 2300us 허용/2301us 거부 확인. ESP32 Arduino 컴파일 통과. 실물 2300us 명령은 전송하지 않았다.

관찰 결과
ZED는 현재 로봇 손 RGB 관찰용이며 제어 입력으로 사용하지 않는다. 자동 손 랜드마크가 검출되지 않아 로봇 관절각/장력의 측정값은 없다. 영상은 소프트웨어 프레임 획득 시각과 송신 PWM을 연결한 기록이다.
사용자가 중지를 펼칠 때 옆 손가락에 실제로 걸린다고 확인했다. 상한 확장은 이 기계적 간섭을 해결하지 않는다.
