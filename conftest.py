# Root conftest.py — ensures imports resolve the same way under pytest
# regardless of what the clone directory is named.
#
# Two things are set up here:
#
# 1. Repo root on sys.path so `from core.safety import ...` etc. work.
#
# 2. A `stewart_control` module alias pointing to the repo root. The legacy
#    modules (e.g. kinematics/ik_solver.py) hardcode imports like
#    `from stewart_control.config import ...`. That path only resolves
#    naturally when the clone directory happens to be named `stewart_control`
#    AND its parent is on sys.path — which is true on some dev machines but
#    NOT on CI (where the checkout folder is `stewart-platform-controller`).
#    Registering the alias here makes the import environment-independent.
import os
import sys
import types

_repo_root = os.path.dirname(os.path.abspath(__file__))

if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)

if "stewart_control" not in sys.modules:
    _pkg = types.ModuleType("stewart_control")
    _pkg.__path__ = [_repo_root]  # type: ignore[attr-defined]
    sys.modules["stewart_control"] = _pkg
