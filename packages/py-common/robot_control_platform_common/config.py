"""Shared process settings loaded from ``RCP_`` environment variables."""

from __future__ import annotations

from enum import StrEnum
from pathlib import Path
from typing import Self
from urllib.parse import quote

from pydantic import Field, SecretStr, ValidationError, field_validator, model_validator
from pydantic_settings import BaseSettings, SettingsConfigDict


class ConfigurationError(ValueError):
    """Raised when required ``RCP_`` settings are missing or invalid."""


class RuntimeEnv(StrEnum):
    DEVELOPMENT = "development"
    TEST = "test"
    DEMO = "demo"


class LogLevel(StrEnum):
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"


def _env_name(field_name: str) -> str:
    return f"RCP_{field_name.upper()}"


class Settings(BaseSettings):
    """Database, artifact, and shared runtime settings.

    Environment variables are the only configuration mechanism. Field names
    map to ``RCP_`` aliases such as ``RCP_DATABASE_HOST`` and
    ``RCP_ARTIFACT_ROOT``.
    """

    model_config = SettingsConfigDict(
        env_prefix="RCP_",
        env_file=None,
        extra="ignore",
        frozen=True,
        hide_input_in_errors=True,
        case_sensitive=False,
    )

    env: RuntimeEnv
    log_level: LogLevel
    database_host: str = Field(min_length=1)
    database_port: int = Field(ge=1, le=65535)
    database_name: str = Field(min_length=1)
    database_user: str = Field(min_length=1)
    database_password: SecretStr = Field(min_length=1)
    artifact_root: Path
    api_base_url: str = Field(min_length=1)
    worker_id: str = Field(min_length=1)
    run_lease_seconds: int = Field(default=30, gt=0)
    run_heartbeat_seconds: int = Field(default=10, gt=0)
    simulation_gui: bool = False

    @field_validator("artifact_root")
    @classmethod
    def artifact_root_must_be_absolute(cls, value: Path) -> Path:
        if not value.is_absolute():
            msg = "RCP_ARTIFACT_ROOT must be an absolute path"
            raise ValueError(msg)
        return value

    @model_validator(mode="after")
    def password_must_not_be_blank(self) -> Self:
        if self.database_password.get_secret_value().strip() == "":
            msg = "RCP_DATABASE_PASSWORD must not be blank"
            raise ValueError(msg)
        return self

    def database_dsn(self) -> SecretStr:
        """Return the PostgreSQL DSN. Never log this value."""

        user = quote(self.database_user, safe="")
        password = quote(self.database_password.get_secret_value(), safe="")
        return SecretStr(
            "postgresql://"
            f"{user}:{password}@{self.database_host}:{self.database_port}/"
            f"{self.database_name}"
        )


def load_settings() -> Settings:
    """Load settings from the process environment.

    Missing or invalid required configuration fails with a concise error that
    names ``RCP_`` variables and never includes secret values.
    """

    try:
        # BaseSettings reads RCP_ environment variables; fields are required at runtime.
        return Settings()  # type: ignore[call-arg]
    except ValidationError as exc:
        messages: list[str] = []
        for error in exc.errors():
            location = ".".join(str(part) for part in error["loc"])
            env_name = _env_name(location.split(".")[0]) if location else "RCP_SETTINGS"
            kind = error["type"]
            if kind == "missing":
                messages.append(f"missing {env_name}")
            else:
                messages.append(f"{env_name} is invalid")
        detail = "; ".join(messages) if messages else "invalid configuration"
        raise ConfigurationError(f"Invalid configuration: {detail}") from exc
