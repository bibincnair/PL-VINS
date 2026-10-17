#!/usr/bin/env python3
"""Reject ROS 1 and old OpenCV dependencies in installed PL-VINS ELF files."""
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
forbidden = re.compile(r'lib(?:roscpp|rostime|rosconsole|rosserialization|tf)\.so|libopencv_\w+\.so\.[34](?:\.|\s)')
checked = 0
for path in (ROOT / 'install').rglob('*'):
    if not path.is_file() or path.open('rb').read(4) != b'\x7fELF': continue
    result = subprocess.run(['ldd', str(path)], text=True, capture_output=True)
    if 'not found' in result.stdout or forbidden.search(result.stdout):
        raise SystemExit(f'Invalid linkage: {path}\n{result.stdout}')
    checked += 1
if not checked: raise SystemExit('No installed ELF files found; build the project first')
print(f'Checked {checked} installed ELF files: no missing libraries, ROS 1, or OpenCV 3/4 dependencies')
