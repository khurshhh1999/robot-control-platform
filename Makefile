.PHONY: install format format-check lint typecheck test test-integration test-e2e \
	migration-check compose-build compose-up compose-down precommit ci \
	test-simulator-smoke test-camera-smoke test-scenario-smoke test-motion-smoke

UV ?= uv
PNPM ?= pnpm

install:
	$(UV) sync --group dev
	$(PNPM) install --frozen-lockfile

format:
	$(UV) run ruff format .
	$(UV) run ruff check --fix .

format-check:
	$(UV) run ruff format --check .

lint:
	$(UV) run ruff check .

typecheck:
	$(UV) run mypy
	$(PNPM) run typecheck

test:
	$(UV) run pytest

test-integration:
	$(UV) run pytest tests/integration

test-e2e:
	@echo "End-to-end tests are not implemented yet." >&2
	@exit 1

test-simulator-smoke:
	docker compose build simulator
	docker compose run --rm --no-deps simulator python -m robot_control_platform_simulator.physics.smoke

test-camera-smoke:
	@outdir="$${RCP_CAMERA_SMOKE_DIR:-$$(mktemp -d /tmp/rcp-camera-smoke.XXXXXX)}"; \
	mkdir -p "$$outdir" && \
	docker compose build simulator && \
	docker compose run --rm --no-deps \
		-v "$$outdir:/out" \
		simulator \
		python -m robot_control_platform_simulator.physics.camera_smoke \
		--output /out/review_rgb.png && \
	echo "wrote $$outdir/review_rgb.png"

test-scenario-smoke:
	docker compose build simulator
	docker compose run --rm --no-deps simulator python -m robot_control_platform_simulator.scenarios.smoke

test-motion-smoke:
	@test -f .project-private/motion-reliability.json || { \
		echo "motion reliability configuration is missing" >&2; \
		exit 1; \
	}
	docker compose build simulator
	docker compose run --rm --no-deps \
		-v "$(CURDIR)/.project-private/motion-reliability.json:/tmp/motion-reliability.json:ro" \
		simulator \
		python -m robot_control_platform_simulator.control.smoke \
		--config /tmp/motion-reliability.json

migration-check:
	@echo "Migration checks are not implemented yet." >&2
	@exit 1

compose-build:
	docker compose build

compose-up:
	docker compose up --build

compose-down:
	docker compose down

precommit: format-check lint typecheck

ci: format-check lint typecheck test
