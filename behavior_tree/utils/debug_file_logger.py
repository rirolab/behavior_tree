import json
import os
import re
import threading
from datetime import datetime
from pathlib import Path


_LOGGER_CACHE = {}
_LOGGER_CACHE_LOCK = threading.Lock()


def normalise_debug_value(value):
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if hasattr(value, "uuid"):
        try:
            return list(value.uuid)
        except TypeError:
            pass
    if hasattr(value, "tolist"):
        return value.tolist()
    if isinstance(value, bytes):
        return list(value)
    if isinstance(value, dict):
        return {
            str(key): normalise_debug_value(sub_value)
            for key, sub_value in value.items()
        }
    if isinstance(value, (list, tuple, set)):
        return [normalise_debug_value(sub_value) for sub_value in value]
    return str(value)


def make_debug_snapshot(value):
    return json.dumps(
        normalise_debug_value(value),
        ensure_ascii=True,
        sort_keys=True,
    )


def _slugify(value):
    slug = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value).strip())
    return slug.strip("._") or "debug"


class DebugFileLogger:
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._handle = self.path.open("a", encoding="utf-8", buffering=1)

    def log(self, event, **fields):
        payload = {
            "ts": datetime.now().isoformat(timespec="milliseconds"),
            "event": str(event),
        }
        for key, value in fields.items():
            payload[str(key)] = normalise_debug_value(value)
        line = json.dumps(payload, ensure_ascii=True, sort_keys=True)
        with self._lock:
            self._handle.write(line + "\n")
            self._handle.flush()


def get_debug_file_logger(subdir, prefix):
    cache_key = (str(subdir), str(prefix))
    with _LOGGER_CACHE_LOCK:
        if cache_key not in _LOGGER_CACHE:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = f"{_slugify(prefix)}_{timestamp}_pid{os.getpid()}.jsonl"
            log_root = Path(__file__).resolve().parents[2] / "logs" / str(subdir)
            _LOGGER_CACHE[cache_key] = DebugFileLogger(log_root / filename)
        return _LOGGER_CACHE[cache_key]
