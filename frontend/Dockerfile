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
# Only what the build reads, so editing the Caddyfile or lint config reuses this layer.
COPY index.html env.d.ts vite.config.ts pwa-assets.config.ts tsconfig*.json ./
COPY public public
COPY src src
RUN npm run build

# ------------------------------------------------------------------------------------------
# The official binary carries cap_net_bind_service (to bind port 80). We listen on 8080, and
# with cap_drop: ALL the kernel refuses to start a binary whose file capabilities it cannot
# grant. A plain cp drops the capability.
FROM caddy:${CADDY_VERSION}-alpine AS caddy
RUN cp /usr/bin/caddy /caddy

# ------------------------------------------------------------------------------------------
FROM gcr.io/distroless/static-debian13:nonroot AS runtime
LABEL org.opencontainers.image.title="vcortex-frontend" \
      org.opencontainers.image.description="vCortex frontend: Vue app behind Caddy" \
      org.opencontainers.image.source="https://github.com/NickGhignatti/vCortex" \
      org.opencontainers.image.licenses="GPL-3.0-only"

# Caddy writes its autosave and storage under these; /tmp is writable (tmpfs in compose).
ENV XDG_CONFIG_HOME=/tmp/caddy XDG_DATA_HOME=/tmp/caddy

COPY --from=caddy /caddy /usr/local/bin/caddy
COPY Caddyfile /etc/caddy/Caddyfile
COPY --from=build /app/dist /srv

EXPOSE 8080
USER nonroot:nonroot
ENTRYPOINT ["/usr/local/bin/caddy"]
CMD ["run", "--config", "/etc/caddy/Caddyfile", "--adapter", "caddyfile"]
