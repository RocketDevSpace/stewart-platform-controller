# Root conftest.py — adds repo root to sys.path so that
# `from core.safety import ...` and `from settings import ...`
# resolve correctly when pytest is run from any working directory.
# Also adds the parent directory so that legacy `from stewart_control.config`
# imports resolve (kinematics/ik_solver.py uses this path).
import sys
import os

_repo_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _repo_root)
sys.path.insert(0, os.path.dirname(_repo_root))
