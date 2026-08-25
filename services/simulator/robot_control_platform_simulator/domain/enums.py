"""String enumerations for lifecycle, outcomes, artifacts, and control."""

from enum import StrEnum


class CanonicalUnit(StrEnum):
    """Units allowed on domain values: meters, radians, seconds, kilograms, newtons."""

    METERS = "meters"
    RADIANS = "radians"
    SECONDS = "seconds"
    KILOGRAMS = "kilograms"
    NEWTONS = "newtons"


class ExperimentStatus(StrEnum):
    DRAFT = "draft"
    QUEUED = "queued"
    RUNNING = "running"
    COMPLETED = "completed"
    COMPLETED_WITH_ERRORS = "completed_with_errors"
    CANCELLED = "cancelled"
    FAILED = "failed"


class RunStatus(StrEnum):
    QUEUED = "queued"
    CLAIMED = "claimed"
    RUNNING = "running"
    COMPLETED = "completed"
    COMPLETED_WITH_ERRORS = "completed_with_errors"
    CANCELLING = "cancelling"
    CANCELLED = "cancelled"
    LEASE_EXPIRED = "lease_expired"
    FAILED = "failed"


class TrialStatus(StrEnum):
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class TerminalOutcome(StrEnum):
    SYSTEM_ERROR = "system_error"
    COLLISION = "collision"
    MISSED_GRASP = "missed_grasp"
    DROPPED_OBJECT = "dropped_object"
    WRONG_BIN = "wrong_bin"
    SUCCESS = "success"


class ArtifactKind(StrEnum):
    INITIAL_RGB = "initial_rgb"
    PRE_GRASP_RGB = "pre_grasp_rgb"
    POST_GRASP_RGB = "post_grasp_rgb"
    PRE_RELEASE_RGB = "pre_release_rgb"
    TERMINAL_RGB = "terminal_rgb"
    TRAJECTORY = "trajectory"
    TRIAL_MANIFEST = "trial_manifest"


class ControllerState(StrEnum):
    RESET = "reset"
    OBSERVE = "observe"
    PLAN = "plan"
    APPROACH = "approach"
    GRASP = "grasp"
    VERIFY_GRASP = "verify_grasp"
    LIFT = "lift"
    TRANSFER = "transfer"
    RELEASE = "release"
    VERIFY_PLACE = "verify_place"
    RETRACT = "retract"
    TERMINAL = "terminal"


class EventType(StrEnum):
    STATE_START = "state_start"
    STATE_END = "state_end"
    STATE_FAILURE = "state_failure"
    ACTION = "action"
    OBSERVATION = "observation"
    CONTACT = "contact"
    TIMEOUT = "timeout"
