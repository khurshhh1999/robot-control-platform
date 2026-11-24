# syntax=docker/dockerfile:1
# node:22.23.2-bookworm-slim index digest recorded 2026-11-02.
FROM node:22.23.2-bookworm-slim@sha256:48e4b67d85f87bd551df43704e24d252f56cc5f8e9718841aace50f19948f0f9 AS build

WORKDIR /src
RUN npm install --global pnpm@11.5.2

COPY package.json pnpm-lock.yaml pnpm-workspace.yaml ./
COPY apps/web/package.json apps/web/package.json
COPY packages/contracts/package.json packages/contracts/package.json
RUN pnpm install --frozen-lockfile --filter @robot-control-platform/web...

COPY apps/web apps/web
COPY packages/contracts packages/contracts
RUN pnpm --filter @robot-control-platform/web run build

# nginx:1.28-alpine index digest recorded 2026-08-24.
FROM nginx:1.28-alpine@sha256:a8b39bd9cf0f83869a2162827a0caf6137ddf759d50a171451b335cecc87d236

COPY apps/web/nginx.conf /etc/nginx/nginx.conf
COPY --from=build /src/apps/web/dist /usr/share/nginx/html

RUN mkdir -p /var/cache/nginx /var/run \
    && chown -R 101:101 /usr/share/nginx/html

USER 101:101
EXPOSE 8080
CMD ["nginx", "-g", "daemon off;"]
