"""Health and versioned API route modules."""

from robot_control_platform_api.routes.annotations import router as annotations_router
from robot_control_platform_api.routes.artifacts import router as artifacts_router
from robot_control_platform_api.routes.experiments import router as experiments_router
from robot_control_platform_api.routes.health import router as health_router
from robot_control_platform_api.routes.policies import router as policies_router
from robot_control_platform_api.routes.runs import router as runs_router
from robot_control_platform_api.routes.scenario_sets import router as scenario_sets_router
from robot_control_platform_api.routes.trials import router as trials_router

__all__ = [
    "annotations_router",
    "artifacts_router",
    "experiments_router",
    "health_router",
    "policies_router",
    "runs_router",
    "scenario_sets_router",
    "trials_router",
]
