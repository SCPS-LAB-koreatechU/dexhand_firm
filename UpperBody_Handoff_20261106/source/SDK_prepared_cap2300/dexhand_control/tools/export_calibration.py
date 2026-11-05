"""Export matched PWM profiles, complete sketches and SDK YAML into a NEW folder.

Never rewrites an active profile or transmits CAN. Reflash the selected ESP32
and deploy dexhand.yaml together before connecting with the new calibration.
"""
import argparse
import copy
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pc'))
sys.path.insert(0, str(ROOT / 'tools'))
from hand_can import load_profile, profile_hash, validate_profile
from prepare_firmware import expected_files


def export_bundle(output, profiles):
    output = Path(output).resolve()
    project = ROOT.parent.resolve()
    if output == project or output in project.parents:
        raise ValueError('Export must not replace the current project or its parents')
    if output.exists():
        raise ValueError('Choose a NEW export folder; existing folders are never overwritten')
    right_names = [m['name'] for m in profiles['right']['motors'][-3:]]
    if right_names == ['엄지 왼쪽 당김', '엄지 끝', '엄지 오른쪽 당김']:
        raise ValueError('rightfix1 펌웨어 원본 ZIP을 반영하기 전에는 구형 템플릿으로 내보낼 수 없습니다. SDK_수정안내_rightfix1.md를 따르세요.')
    prepared = {}
    config = yaml.safe_load((project / 'humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk/config/dexhand.yaml').read_text(encoding='utf-8'))
    for side in ('left', 'right'):
        profile = copy.deepcopy(profiles[side])
        validate_profile(profile)
        if profile['hand'] != side or profile['device_id'] != (60 if side == 'left' else 61):
            raise ValueError('Wrong hand/node in calibration')
        prepared.update({p.relative_to(ROOT): content for p, content in expected_files(side, profile=profile).items()})
        import json
        prepared[Path(f'profiles/{side}_hand.json')] = json.dumps(profile, ensure_ascii=False, indent=2)+'\n'
        config['hands'][side].update(profile_tag=profile_hash(profile)[:8],
                open_us=[m['open_us'] for m in profile['motors']],
                close_us=[m['close_us'] for m in profile['motors']])
    prepared[Path('dexhand.yaml')] = yaml.safe_dump(config, allow_unicode=True, sort_keys=False)
    prepared[Path('README.md')] = '''# 캘리브레이션 적용 파일

GUI에서 기록한 값으로 생성했습니다. 모터의 실제 도달 위치는 측정하지 않습니다.
이 폴더를 만들었다고 현재 ESP32나 실행 중 SDK의 설정이 바뀌지는 않습니다.

1. 값을 변경한 손의 `firmware/DexHandLeft/DexHandLeft.ino` 또는
   `firmware/DexHandRight/DexHandRight.ino`를 폴더 전체와 함께 열어 업로드합니다.
2. 손 제어 프로그램을 종료한 뒤, 두 `profiles/*_hand.json`을 프로젝트의
   `dexhand_control/profiles/`에 복사합니다.
3. `dexhand.yaml`을 프로젝트의
   `humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk/config/dexhand.yaml`에 복사합니다.
   ROS 설치본은 다시 빌드하거나 설치된 설정도 함께 갱신합니다.
4. GUI를 다시 실행하고 CAN 연결을 확인합니다. 프로필 불일치면 출력이 차단됩니다.

기존 허용 범위 밖으로 입력한 끝점은 현재 펌웨어로 시험한 값이 아닙니다.
그 범위를 사용하려면 이 새 펌웨어를 올린 뒤 작은 증분으로 다시 확인하세요.
'''
    # Validate everything before creating any deployment output.
    output.mkdir(parents=True, exist_ok=False)
    for relative, content in prepared.items():
        destination = output / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(content, encoding='utf-8', newline='\n')
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--left', type=Path, default=ROOT / 'profiles/left_hand.json')
    parser.add_argument('--right', type=Path, default=ROOT / 'profiles/right_hand.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(export_bundle(args.output, {side: load_profile(getattr(args, side)) for side in ('left', 'right')}))


if __name__ == '__main__':
    main()
