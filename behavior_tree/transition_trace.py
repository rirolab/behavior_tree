import json
import os
from datetime import datetime
from pathlib import Path
import time


TRACE_FILE_ENV = "DRB_TRANSITION_TRACE_FILE"


def _default_trace_file():
    # Resolve the workspace log path from this package location.
    workspace_root = Path(__file__).resolve().parents[4]
    return workspace_root / "logs" / "transition_trace.jsonl"


def _trace_file():
    # Allow launch or shell overrides while keeping one stable default file.
    raw_path = os.environ.get(TRACE_FILE_ENV, "").strip()
    if raw_path:
        return Path(raw_path).expanduser()
    return _default_trace_file()


def _json_safe(value):
    # Convert diagnostic values into compact JSON-compatible payloads.
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, set):
        return sorted((_json_safe(item) for item in value), key=str)
    if isinstance(value, BaseException):
        return {"type": type(value).__name__, "message": str(value)}
    return str(value)


def elapsed_seconds(start_perf):
    # Report short elapsed timings with stable precision across processes.
    return round(time.perf_counter() - float(start_perf), 6)


def trace_event(process, event, **fields):
    # Append one timestamped transition event without writing to the terminal.
    payload = {
        "agent_log_marker": "[DebugLogGeneratedByCodingAgent]",
        "wall_time": datetime.now().astimezone().isoformat(timespec="microseconds"),
        "perf_counter": round(time.perf_counter(), 6),
        "pid": os.getpid(),
        "process": str(process),
        "event": str(event),
    }
    payload.update({str(key): _json_safe(value) for key, value in fields.items()})
    trace_file = _trace_file()
    trace_file.parent.mkdir(parents=True, exist_ok=True)
    with trace_file.open("a", encoding="utf-8") as file_handle:
        file_handle.write(json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n")
