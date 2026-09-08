"""Fail-closed controller heartbeat shared by the receiver and fusion node."""
import json
import math
import time
from pathlib import Path


def guard_healthy(path, map_id=None, now=None):
    if not path:
        return True
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        updated = float(data["updated_unix_s"])
        age = (time.time() if now is None else now) - updated
        return (data["healthy"] is True and math.isfinite(age) and 0 <= age <= 2.0
                and data.get("map_id") in (None, map_id))
    except (OSError, ValueError, TypeError, KeyError):
        return False
