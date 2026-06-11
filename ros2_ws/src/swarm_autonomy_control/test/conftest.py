"""Make the swarm_autonomy_* pure cores importable when running this package's tests directly
(pytest's rootdir stops at the package's setup.py, so the repo-root conftest is not loaded)."""
import sys
from pathlib import Path

_src = Path(__file__).resolve().parents[2]
for _pkg in sorted(_src.glob("swarm_autonomy_*")):
    if _pkg.is_dir():
        sys.path.insert(0, str(_pkg))
