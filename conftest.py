# Root conftest.py — adds repo root to sys.path so that
# `from core.safety import ...` and `from settings import ...`
# resolve correctly when pytest is run from any working directory.
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
