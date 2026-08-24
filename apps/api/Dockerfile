# syntax=docker/dockerfile:1
# python:3.12-slim-bookworm index digest recorded 2026-08-24.
FROM python:3.12-slim-bookworm@sha256:a116514e19457bcb7af7efe9c3dd0b9b71e85b317694e7882a1c52aa15a78134 AS builder

WORKDIR /app
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    UV_COMPILE_BYTECODE=1 \
    UV_LINK_MODE=copy \
    UV_NO_CACHE=1 \
    UV_NO_EDITABLE=1

RUN pip install --no-cache-dir uv==0.11.7

COPY pyproject.toml uv.lock ./
COPY packages/py-common ./packages/py-common
COPY apps/api ./apps/api
COPY services/simulator ./services/simulator

RUN uv sync --frozen --no-dev --no-editable --package robot-control-platform-api

FROM python:3.12-slim-bookworm@sha256:a116514e19457bcb7af7efe9c3dd0b9b71e85b317694e7882a1c52aa15a78134

WORKDIR /app
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PATH="/app/.venv/bin:${PATH}" \
    HOME=/tmp

RUN groupadd --system --gid 10001 app \
    && useradd --system --uid 10001 --gid 10001 --no-create-home --home-dir /tmp --shell /usr/sbin/nologin app \
    && mkdir -p /var/lib/robot-platform/artifacts \
    && chown 10001:10001 /var/lib/robot-platform/artifacts

COPY --from=builder /app/.venv /app/.venv

USER 10001:10001
EXPOSE 8000
CMD ["python", "-m", "robot_control_platform_api"]
