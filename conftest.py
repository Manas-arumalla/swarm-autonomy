"""Make the pure-core packages importable for ``pytest`` from a fresh clone, with no
``colcon build`` / ``pip install`` / ``PYTHONPATH`` setup. Mirrors ``sim/swarm_sim/_bootstrap.py``
so the documented quick-start (``python3 -m pytest -q ros2_ws/src/*/test sim/swarm_sim/test``)
just works.
"""
import sys
from pathlib import Path

_root = Path(__file__).parent
for _pkg in sorted((_root / "ros2_ws" / "src").glob("swarm_autonomy_*")):
    if _pkg.is_dir():
        sys.path.insert(0, str(_pkg))
sys.path.insert(0, str(_root / "sim"))
