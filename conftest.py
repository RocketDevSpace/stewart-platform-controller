"""
pytest conftest — worktree import shim.

The worktree directory IS the stewart_control package contents, but it is not
named 'stewart_control'. Register it in sys.modules under that name so that
absolute imports (e.g. `from stewart_control.settings import ...`) resolve
correctly during test runs without modifying any source file.
"""
import sys
import os
import types

_wt_root = os.path.dirname(os.path.abspath(__file__))

_pkg = types.ModuleType("stewart_control")
_pkg.__path__ = [_wt_root]
_pkg.__file__ = os.path.join(_wt_root, "__init__.py")
_pkg.__package__ = "stewart_control"
sys.modules.setdefault("stewart_control", _pkg)
