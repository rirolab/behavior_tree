from enum import Enum, auto

class StepValidationResult(Enum):
    """
    Result of validating whether one step should reject the whole goal.
    """

    NOT_APPLICABLE = auto()
    ACCEPT_GOAL = auto()
    REJECT_GOAL = auto()