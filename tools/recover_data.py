#!/usr/bin/env python3
"""Recover checksum-verified legacy research assets into an external data directory."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'data')
    parser.add_argument('--prefix', default='', help='Recover only manifest paths starting with this prefix')
    args = parser.parse_args()
    manifest = json.loads((ROOT / 'docs/data-manifest.json').read_text())
    for entry in manifest['files']:
        relative = Path(entry['path'])
        if relative.is_absolute() or '..' in relative.parts: raise ValueError('invalid manifest path')
        if not entry['path'].startswith(args.prefix): continue
        data = subprocess.check_output(['git', 'show', manifest['revision'] + ':' + entry['path']], cwd=ROOT)
        if hashlib.sha256(data).hexdigest() != entry['sha256']: raise ValueError('checksum mismatch: ' + entry['path'])
        destination = args.output / relative
        if destination.exists(): raise FileExistsError(destination)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(data)
    print('Recovered assets to', args.output)

if __name__ == '__main__': main()
