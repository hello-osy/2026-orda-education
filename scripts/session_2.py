"""Session 2 launcher: run with the project's virtual-environment Python."""
import multiprocessing
import os
from pathlib import Path
import runpy
import sys


def main():
    root = Path(__file__).resolve().parents[1]
    sources = [str(root / 'src' / name) for name in ('session_2', 'session_1')]
    sys.path[:0] = sources
    old_path = os.environ.get('PYTHONPATH')
    os.environ['PYTHONPATH'] = os.pathsep.join(sources + ([old_path] if old_path else []))
    os.environ['PYTHONUTF8'] = '1'
    os.environ['PYTHONIOENCODING'] = 'utf-8'
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, 'reconfigure'):
            stream.reconfigure(encoding='utf-8', errors='replace')
    os.chdir(root)
    multiprocessing.freeze_support()
    runpy.run_module('session_2.studio', run_name='__main__')


if __name__ == '__main__':
    main()
