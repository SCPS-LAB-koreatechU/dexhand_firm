"""Extract a snapshot locally for review. Never starts programs or contacts the robot."""
from pathlib import Path
import argparse,tarfile,shutil
root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--archive',type=Path,default=root/'runtime/robot_ram_snapshot_20261106.tar.gz');parser.add_argument('--out',type=Path,required=True);args=parser.parse_args()
target=args.out.resolve()
if target.exists():raise SystemExit('Choose a new output directory; existing files are never overwritten.')
with tarfile.open(args.archive,'r:gz') as archive:
 members=archive.getmembers()
 for m in members:
  p=(target/m.name).resolve()
  if not p.is_relative_to(target) or not (m.isdir() or m.isfile()):raise SystemExit('Unsupported or unsafe archive member: '+m.name)
 target.mkdir(parents=True)
 for m in members:
  p=target/m.name
  if m.isdir():p.mkdir(parents=True,exist_ok=True);continue
  p.parent.mkdir(parents=True,exist_ok=True)
  with archive.extractfile(m) as src,p.open('wb') as dst:shutil.copyfileobj(src,dst)
print('Extracted for review:',target)
print('Status/PWM values are historical snapshot records, not measurements of the current robot. No programs were started.')
