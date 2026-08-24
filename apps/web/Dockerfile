# syntax=docker/dockerfile:1
# nginx:1.28-alpine index digest recorded 2026-08-24.
FROM nginx:1.28-alpine@sha256:a8b39bd9cf0f83869a2162827a0caf6137ddf759d50a171451b335cecc87d236

COPY nginx.conf /etc/nginx/nginx.conf
COPY index.html /usr/share/nginx/html/index.html

RUN mkdir -p /var/cache/nginx /var/run \
    && chown -R 101:101 /usr/share/nginx/html

USER 101:101
EXPOSE 8080
CMD ["nginx", "-g", "daemon off;"]
