from enum import Enum, auto


class BlendPhase(Enum):
    # Mark the two children inside the active blend pair.
    RUNNING_BLEND_FIRST = auto()
    RUNNING_BLEND_SECOND = auto()


class BlendProgressStatus(Enum):
    # Mark progress payload states reported by blend execution.
    EXECUTING = auto()
    HANDOFF = auto()
    SUCCEEDED = auto()
    ABORTED = auto()
    CANCELED = auto()
