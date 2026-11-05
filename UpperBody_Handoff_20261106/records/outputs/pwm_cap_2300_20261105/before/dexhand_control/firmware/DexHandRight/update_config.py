"""Update ONLY this folder's BoardConfig.h and SDK_설정값.md from calibration.json.

No CAN/serial access. No SDK files are written. Existing endpoints are retained
until the operator measures and edits them; this script never invents values.
"""
import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
OUTPUT_GPIOS = {0,1,2,3,4,5,12,13,14,15,16,17,18,19,21,22,23,25,26,27,32,33}
THUMB_NAMES = ('엄지 왼쪽 당김', '엄지 끝', '엄지 오른쪽 당김')


def render(profile):
    if profile.get('hand') != 'right' or profile.get('device_id') != 61 or profile.get('profile_version') != 1:
        raise ValueError('This folder is for right hand node 61 only')
    if profile.get('can') != {'tx_gpio': 1, 'rx_gpio': 3, 'bitrate': 1000000}:
        raise ValueError('Keep existing CAN TX GPIO1 / RX GPIO3 at 1 Mbps')
    if profile.get('pwm_frequency_hz') != 50:
        raise ValueError('PWM frequency must remain 50 Hz')
    motors = profile['motors']
    if len(motors) != 15 or [m['id'] for m in motors] != list(range(1,16)):
        raise ValueError('Exactly 15 motors in CAN channel order are required')
    pins = [m['gpio'] for m in motors]
    for pin in pins:
        if type(pin) is not int or pin not in OUTPUT_GPIOS:
            raise ValueError(f'GPIO{pin} cannot output servo PWM: GPIO34..39 are INPUT ONLY. Index upper/lower use GPIO32/25.')
    if len(set(pins)) != 15 or 1 in pins or 3 in pins:
        raise ValueError('Duplicate PWM GPIO or conflict with CAN GPIO1/3')
    if pins[12:] != [13,14,18] or [m['name'] for m in motors[12:]] != list(THUMB_NAMES):
        raise ValueError('Keep thumb mapping: GPIO13=left, GPIO14=tip, GPIO18=right')
    for motor in motors:
        for key in ('open_us','close_us'):
            value = motor[key]
            if type(value) is not int or not 500 <= value <= 2500:
                raise ValueError(f"Channel {motor['id']}: {key} must be integer 500..2500 us")
        if motor['open_us'] == motor['close_us']:
            raise ValueError('Open/close endpoints must differ')
    assembly, watchdog = profile['assembly_timeout_ms'], profile['watchdog_ms']
    if type(assembly) is not int or type(watchdog) is not int or not 1 <= assembly < watchdog <= 60000:
        raise ValueError('Invalid assembly/watchdog timing')
    tag = hashlib.sha256(json.dumps(profile, ensure_ascii=False, sort_keys=True, separators=(',', ':')).encode('utf-8')).hexdigest()
    def array(key):
        return ', '.join(str(m[key]) for m in motors[:12])
    thumb_vars = ('Left', 'Tip', 'Right')
    def full(key):
        suffix = 'OpenUs' if key == 'open_us' else 'CloseUs'
        return array(key)+', '+', '.join('kThumb'+name+suffix for name in thumb_vars)
    low = ', '.join(str(min(m['open_us'],m['close_us'])) for m in motors[:12])
    high = ', '.join(str(max(m['open_us'],m['close_us'])) for m in motors[:12])
    low += ', '+', '.join(f'minUs(kThumb{x}OpenUs, kThumb{x}CloseUs)' for x in thumb_vars)
    high += ', '+', '.join(f'maxUs(kThumb{x}OpenUs, kThumb{x}CloseUs)' for x in thumb_vars)
    thumb = '\n'.join(f"constexpr uint16_t kThumb{x}{suffix}Us = {m[key]};" for x,m in zip(thumb_vars,motors[12:])
            for suffix,key in (('Open','open_us'),('Close','close_us')))
    pin_prefix = ', '.join(str(p) for p in pins[:6])
    pin_suffix = ', '.join(str(p) for p in pins[8:])
    header = f'''// Local test configuration. Edit calibration.json, then run python update_config.py.
// Thumb endpoints below are user-confirmed assignments; GPIOs are unchanged.
#pragma once
#include <stdint.h>
namespace hand_config {{
constexpr uint8_t kNodeId = 61;
constexpr int kCanTx = 1;
constexpr int kCanRx = 3;
constexpr uint32_t kWatchdogMs = {watchdog};
constexpr uint32_t kAssemblyTimeoutMs = {assembly};
// GPIO34/35 are input-only. User confirmed output-capable GPIO32/25.
constexpr uint8_t kIndexUpperGpio = {pins[6]};
constexpr uint8_t kIndexLowerGpio = {pins[7]};
constexpr uint8_t kPins[15] = {{{pin_prefix}, kIndexUpperGpio, kIndexLowerGpio, {pin_suffix}}};
constexpr uint8_t kThumbRightIndex = 14;  // channel 15, GPIO18: right pull
constexpr uint8_t kThumbLeftIndex = 12;   // channel 13, GPIO13: left pull
constexpr uint8_t kThumbTipIndex = 13;    // channel 14, GPIO14: thumb tip
// User-confirmed endpoints; edit JSON and regenerate this header together with SDK calibration.
{thumb}
constexpr uint16_t minUs(uint16_t a, uint16_t b) {{ return a < b ? a : b; }}
constexpr uint16_t maxUs(uint16_t a, uint16_t b) {{ return a > b ? a : b; }}
constexpr uint16_t kMinUs[15] = {{{low}}};
constexpr uint16_t kMaxUs[15] = {{{high}}};
constexpr uint16_t kOpenUs[15] = {{{full('open_us')}}};
constexpr uint16_t kCloseUs[15] = {{{full('close_us')}}};
constexpr bool isServoOutputGpio(uint8_t p) {{
  return p <= 5 || (p >= 12 && p <= 19) || (p >= 21 && p <= 23) ||
         (p >= 25 && p <= 27) || p == 32 || p == 33;
}}
static_assert(isServoOutputGpio(kIndexUpperGpio) && isServoOutputGpio(kIndexLowerGpio),
              "Index GPIO34/35 are INPUT ONLY: connect upper/lower to GPIO32/25.");
constexpr bool validServoPins() {{
  for (uint8_t i = 0; i < 15; ++i) {{
    if (!isServoOutputGpio(kPins[i]) || kPins[i] == kCanTx || kPins[i] == kCanRx) return false;
    for (uint8_t j = 0; j < i; ++j) if (kPins[i] == kPins[j]) return false;
  }}
  return true;
}}
static_assert(validServoPins(), "Invalid/duplicate PWM GPIO or conflict with CAN GPIO1/3");
constexpr char kProfileSha256[] = "{tag}";
}}  // namespace hand_config
'''
    open_values = [m['open_us'] for m in motors]
    close_values = [m['close_us'] for m in motors]
    table = '\n'.join(f"| {m['id']} | {m['name']} | {m['gpio']} | {m['open_us']} | {m['close_us']} |" for m in motors)
    doc = f'''# 이 폴더의 오른손 SDK 설정값

`calibration.json`에서 자동 생성했다. SDK 파일 자체는 수정하지 않았다.
실제 GPIO 변경은 이 펌웨어 재업로드로 적용한다. SDK는 CAN 순서의 목표를 전송한다.

```yaml
hands:
  right:
    node: 61
    feedback: false
    profile_tag: {tag[:8]}
    open_us: {open_values}
    close_us: {close_values}
```

CAN TX=GPIO1 / RX=GPIO3, 1Mbps, 프로토콜 3을 유지한다.
오른손 엄지 채널 13/14/15는 **왼쪽 당김 / 끝 / 오른쪽 당김**이다.
SDK의 반대 당김 검사는 오른손 **13·15**를 사용해야 하며 왼손은 기존 **13·14**다.
공통 grip 템플릿의 엄지 순서가 [오른쪽, 왼쪽, 끝]이라면 오른손에 적용할 때
[왼쪽, 끝, 오른쪽]으로 변환해야 한다. 이미 실제 CAN 순서인 원시 목표는 재변환하지 않는다.

| CAN 채널 | 손가락·위치 | GPIO | 풀림 µs | 당김 µs |
|---:|---|---:|---:|---:|
{table}

이 표의 엄지 값은 사용자가 확정한 값 쌍이다. 이번 수정에서 실물 끝점을 새로 측정하지는 않았다.
새 값을 측정해 JSON을 고친 뒤 update_config.py를 실행하고 펌웨어와 SDK 설정을 함께 갱신한다.
예전 JSON이나 BoardConfig.h를 덮어씌우면 설정이 되돌아간다. 단방향에서는 보드 프로필을 조회하지 못한다. 새 펌웨어와 dexhand_oneway.yaml을 한 쌍으로 사용한다.
'''
    return header, doc


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    profile = json.loads((ROOT/'calibration.json').read_text(encoding='utf-8'))
    header, doc = render(profile)  # validate all data before writing either output
    for name, content in (('BoardConfig.h',header),('SDK_설정값.md',doc)):
        path = ROOT/name
        if args.check:
            if not path.exists() or path.read_text(encoding='utf-8') != content:
                raise SystemExit('Stale local output: '+name+'; run python update_config.py')
        else:
            path.write_text(content, encoding='utf-8', newline='\n')
    print('PASS: local pins/endpoints/hash and SDK reference match; no SDK files written')


if __name__ == '__main__':
    main()
