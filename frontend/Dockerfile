# syntax=docker/dockerfile:1

ARG NODE_VERSION=24
ARG CADDY_VERSION=2

# ------------------------------------------------------------------------------------------
FROM node:${NODE_VERSION}-alpine AS deps
WORKDIR /app
COPY package.json package-lock.json ./
RUN --mount=type=cache,target=/root/.npm \
    npm ci --no-audit --no-fund

# ------------------------------------------------------------------------------------------
FROM deps AS build
# only what the build reads, so editing the Caddyfile or lint config reuses this layer
COPY index.html env.d.ts vite.config.ts pwa-assets.config.ts tsconfig*.json ./
COPY public public
COPY src src
RUN npm run build

# ------------------------------------------------------------------------------------------
# official binary carries cap_net_bind_service (port 80); we use 8080, and under cap_drop: ALL
# the kernel refuses file caps it cannot grant; a plain cp drops them
FROM caddy:${CADDY_VERSION}-alpine AS caddy
RUN cp /usr/bin/caddy /caddy

# ------------------------------------------------------------------------------------------
FROM gcr.io/distroless/static-debian13:nonroot AS runtime
LABEL org.opencontainers.image.title="vcortex-frontend" \
      org.opencontainers.image.description="vCortex frontend: Vue app behind Caddy" \
      org.opencontainers.image.source="https://github.com/NickGhignatti/vCortex" \
      org.opencontainers.image.licenses="GPL-3.0-only"

# caddy writes autosave and storage here; /tmp is writable (tmpfs in compose)
ENV XDG_CONFIG_HOME=/tmp/caddy XDG_DATA_HOME=/tmp/caddy

COPY --from=caddy /caddy /usr/local/bin/caddy
COPY Caddyfile /etc/caddy/Caddyfile
COPY --from=build /app/dist /srv

EXPOSE 8080
USER nonroot:nonroot
ENTRYPOINT ["/usr/local/bin/caddy"]
CMD ["run", "--config", "/etc/caddy/Caddyfile", "--adapter", "caddyfile"]
