import os
import sys
from pathlib import Path

# Ensure core and project root are always available on sys.path
_CORE_DIR = str(Path(__file__).resolve().parent)
_ROOT_DIR = str(Path(__file__).resolve().parent.parent)

for _p in [_CORE_DIR, _ROOT_DIR]:
    if _p not in sys.path:
        sys.path.insert(0, _p)