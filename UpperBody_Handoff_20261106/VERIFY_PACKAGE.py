"""Verify extracted handoff files. Standard library only; no network or robot access."""
from pathlib import Path
import hashlib,json,sys
root=Path(__file__).resolve().parent
manifest=json.loads((root/'MANIFEST.json').read_text(encoding='utf-8'))
errors=[]
for item in manifest['files']:
 path=(root/item['path']).resolve()
 if not path.is_relative_to(root):errors.append(item['path']+': unsafe path');continue
 if not path.is_file():errors.append(item['path']+': missing');continue
 h=hashlib.sha256()
 with path.open('rb') as stream:
  for chunk in iter(lambda:stream.read(1024*1024),b''):h.update(chunk)
 if h.hexdigest()!=item['sha256']:errors.append(item['path']+': checksum mismatch')
if errors:
 print('\n'.join(errors));sys.exit(1)
print(f"PASS: {len(manifest['files'])} files verified. No network connection or motor commands were made.")
