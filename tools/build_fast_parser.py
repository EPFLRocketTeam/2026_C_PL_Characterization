#!/usr/bin/env python3
"""Compile the optional C11 accelerator once. Portable GCC/Clang or MSVC."""
import os
import shutil
import subprocess
from pathlib import Path

root = Path(__file__).resolve().parent
source = root / 'fast_decode.c'
output = root / ('fast_decode.exe' if os.name == 'nt' else 'fast_decode')
compiler = shutil.which('gcc') or shutil.which('clang') or shutil.which('cc')
if compiler:
    command = [compiler, '-O3', '-std=c11', '-D_FILE_OFFSET_BITS=64',
               str(source), '-o', str(output)]
    if os.name != 'nt':
        command.append('-lm')
elif os.name == 'nt' and (cl := shutil.which('cl')):
    command = [cl, '/nologo', '/O2', str(source), '/Fe:' + str(output)]
else:
    raise SystemExit('No compiler found. Install GCC (MSYS2 on Windows), or run from a Visual Studio Developer Prompt.')
print('Building:', ' '.join(command))
subprocess.run(command, check=True, cwd=root)
print('Native decoder ready:', output)
