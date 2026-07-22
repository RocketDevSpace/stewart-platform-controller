# Root conftest.py — ensures imports resolve the same way under pytest
# regardless of what the clone directory is named: puts the repo root on
# sys.path so `from core.safety import ...` etc. work.
#
# (A `stewart_control` module alias used to live here too. It was dead
# scaffolding — no module imports via that prefix anymore, and keeping the
# alias would MASK a reintroduced `stewart_control.*` import, which hard
# constraint 6 forbids.)
import os
import sys

_repo_root = os.path.dirname(os.path.abspath(__file__))

if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)
