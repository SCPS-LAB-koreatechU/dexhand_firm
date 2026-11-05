"""Apply the matched 2300-us SDK patch. No CAN, serial, firmware upload or motion."""
import argparse,hashlib,json,os,shutil,time
from pathlib import Path


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def atomic(path,data):
    temporary=path.with_name(path.name+'.cap2300.tmp');temporary.write_bytes(data);temporary.replace(path)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True,help='SDK parent containing dexhand_control and humanoid_sdk-main')
    parser.add_argument('--data-dir',type=Path,help='Optional BHL_HAND_DATA_DIR; retag compatible right-hand correspondence metadata')
    parser.add_argument('--check',action='store_true',help='Read-only compatibility check')
    args=parser.parse_args();root=args.root.resolve();package=Path(__file__).resolve().parent
    manifest=json.loads((package/'manifest.json').read_text())
    profile_path=root/'dexhand_control/profiles/right_hand.json'
    profile=json.loads(profile_path.read_text())
    if profile['motors']!=manifest['motors'] or profile.get('device_id')!=61:
        raise ValueError('SDK baseline pins/endpoints differ; nothing written')
    planned=[]
    for entry in manifest['files']:
        source=(package/'payload'/entry['path']).resolve();target=(root/entry['path']).resolve()
        if not source.is_relative_to((package/'payload').resolve()) or not target.is_relative_to(root):
            raise ValueError('Path outside SDK or patch')
        if digest(source)!=entry['sha256']:raise ValueError('Patch checksum mismatch: '+entry['path'])
        if target.exists() and digest(target) not in {entry['sha256'],entry.get('before_sha256')}:
            raise ValueError('Unrecognized local edits: '+entry['path']+'; nothing written')
        planned.append((source,target,entry['path']))
    metadata=[]
    if args.data_dir:
        for path in (args.data_dir.resolve()/'calibration').glob('*.json'):
            data=json.loads(path.read_text())
            if data.get('hand')!='right' or data.get('format_version')!=1:continue
            if data.get('profile_tag') not in (manifest['before_tag'],manifest['after_tag']):continue
            if (data.get('open_us')!=[m['open_us'] for m in manifest['motors']]
                    or data.get('close_us')!=[m['close_us'] for m in manifest['motors']]):
                raise ValueError('Correspondence endpoints differ: '+str(path))
            if data['profile_tag']!=manifest['after_tag']:
                data['profile_tag']=manifest['after_tag'];metadata.append((path,data))
    if args.check:
        print(json.dumps({'compatible':True,'files':len(planned),'correspondence_retags':len(metadata),'writes':0,'motor_commands':0}))
        return
    backup=root/('.pwm_cap2300_backup_'+str(time.time_ns()));backup.mkdir()
    completed=[]
    try:
        for source,target,relative in planned:
            old=target.read_bytes() if target.exists() else None
            if old is not None:
                original=backup/relative;original.parent.mkdir(parents=True,exist_ok=True);original.write_bytes(old)
            target.parent.mkdir(parents=True,exist_ok=True);completed.append((target,old));atomic(target,source.read_bytes())
        for i,(path,data) in enumerate(metadata):
            old=path.read_bytes();(backup/f'correspondence_{i}.json').write_bytes(old)
            completed.append((path,old));atomic(path,(json.dumps(data,ensure_ascii=False,indent=2)+'\n').encode())
    except Exception:
        for target,old in reversed(completed):
            if old is None:target.unlink(missing_ok=True)
            else:atomic(target,old)
        raise
    print(json.dumps({'applied':True,'backup':str(backup),'profile_tag':manifest['after_tag'],
        'hardware_max_us':2300,'calibrated_targets_changed':False,'motor_commands':0,'gui_restart_required':True}))


if __name__=='__main__':main()
