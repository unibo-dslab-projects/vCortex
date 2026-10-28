# vCortex

vCortex turns a point-cloud map of a building into a digital twin: walls, rooms and objects.
A C++ backend cleans the map and reasons about its geometry (the **model**), then writes the
result in several formats (the **connectors**: twin JSON, DXF, PLY). A Vue
frontend shows the raw cloud and the chosen output side by side, live.

Any source that speaks the ingest protocol can feed it. The first one is the Unitree Go2
bridge, which lives in the robot repository.

## Layout

```
compose.yaml            backend + frontend
backend/                C++17, CMake, no ROS
  CMakeLists.txt        targets: vcortex_core, vcortex_io, vcortex
  config/model.json     model parameters (start values)
  schema/               twin.schema.json: the twin JSON contract
  include/vcortex/      domain/ model/ connectors/ protocol/ io/
  src/                  one .cpp per header that holds code, plus main.cpp
frontend/               Vue 3 + TypeScript + Vite, PWA, TresJS (three.js)
  Caddyfile             serves the app, forwards /api and /ws to the backend
data/                   saved maps for offline mode (not in git)
```

The backend follows Clean Architecture: `domain` depends on the standard library only, `model`
adds PCL algorithms, `connectors` and `protocol` add nlohmann/json, and only `io` knows about
Crow, files and threads.

## Run

```sh
cp .env.example .env            # optional: set VCORTEX_INGEST_TOKEN
docker compose up --build       # or: podman compose up --build
```

Open <http://localhost:8080>. The robot bridge connects to port 9002 (`/ingest`).

## Develop

Backend, natively on Fedora. Fedora's `pcl-devel` pulls in VTK and Qt (about 2 GiB), so build
the few PCL modules we use once, with the same options as `backend/Dockerfile`:

```sh
sudo dnf install json-devel asio-devel flann-devel lz4-devel ninja-build clang-tools-extra

# PCL 1.15.1 into ~/.local/opt/pcl. Fedora ships Eigen 5, which PCL 1.15.1 rejects by version
# number only: apply Fedora's own patch (pcl-1.15.1-eigen5.patch from the pcl source RPM) first.
cmake -S pcl-1.15.1 -B pcl-build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$HOME/.local/opt/pcl -DPCL_FLANN_REQUIRED_TYPE=SHARED \
  <the -DPCL_* -DWITH_* -DBUILD_* options from backend/Dockerfile>
cmake --build pcl-build && cmake --install pcl-build

cd backend
cmake --preset dev              # writes build/compile_commands.json for clangd (Zed: .zed/)
cmake --build --preset dev
build/vcortex --port 9002
```

Frontend, with hot reload. Vite forwards `/api` and `/ws` to `localhost:9002`
(override with `VCORTEX_BACKEND`):

```sh
cd frontend
npm install
npm run dev
```

After a change to `backend/schema/twin.schema.json`, run `npm run generate-types` in
`frontend/` to rebuild `src/types/twin.ts`.

## Images

| Image | Base | Contents |
|---|---|---|
| `vcortex-backend` | `gcr.io/distroless/static-debian13:nonroot` | one static binary (PCL built from source without VTK) + `config/` |
| `vcortex-frontend` | `gcr.io/distroless/static-debian13:nonroot` | the Caddy binary + the built app |

Both run as a non-root user, with no shell, a read-only root file system and every Linux
capability dropped (see `compose.yaml`).

## License

GPL-3.0. See `LICENSE`.
