# syntax=docker/dockerfile:1

ARG DEBIAN_VERSION=trixie
ARG PCL_VERSION=1.15.1
ARG PCL_SHA256=e1d862c7b6bd27a45884a825a2e509bfcbd4561307d5bfe17ce5c8a3d94a6c29
ARG CROW_VERSION=1.3.4
ARG CROW_SHA256=6be759b3a648b41f0adc3a8408a53fd0243775217b4e32e8101d69e4aadf48cc

# ------------------------------------------------------------------------------------------
FROM debian:${DEBIAN_VERSION}-slim AS toolchain

# keep downloaded .debs in a cache mount, not the image layer
RUN rm -f /etc/apt/apt.conf.d/docker-clean
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update \
 && apt-get install -y --no-install-recommends \
      g++ cmake ninja-build ccache \
      libeigen3-dev libflann-dev liblz4-dev libboost-iostreams-dev \
      nlohmann-json3-dev libasio-dev

# ------------------------------------------------------------------------------------------
# pcl from source: debian's libpcl-dev is shared-only and pulls in vtk and qt (hundreds of mb);
# static, needed modules only, keeps the runtime image tiny
FROM toolchain AS pcl
ARG PCL_VERSION PCL_SHA256
ADD --checksum=sha256:${PCL_SHA256} \
    https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-${PCL_VERSION}.tar.gz /tmp/pcl.tar.gz
RUN mkdir /tmp/pcl \
 && tar -xzf /tmp/pcl.tar.gz -C /tmp/pcl --strip-components=1 \
 && cmake -S /tmp/pcl -B /tmp/pcl/build -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/opt/pcl \
      -DPCL_SHARED_LIBS=OFF \
      -DPCL_ONLY_CORE_POINT_TYPES=ON \
      -DPCL_FLANN_REQUIRED_TYPE=STATIC \
      -DWITH_OPENMP=OFF -DWITH_CUDA=OFF -DWITH_VTK=OFF -DWITH_QT=NO -DWITH_OPENGL=OFF \
      -DWITH_GLEW=OFF -DWITH_LIBUSB=OFF -DWITH_PNG=OFF -DWITH_PCAP=OFF -DWITH_QHULL=OFF \
      -DBUILD_tools=OFF -DBUILD_apps=OFF -DBUILD_examples=OFF -DBUILD_global_tests=OFF \
      -DBUILD_visualization=OFF -DBUILD_surface=OFF -DBUILD_registration=OFF \
      -DBUILD_keypoints=OFF -DBUILD_tracking=OFF -DBUILD_recognition=OFF -DBUILD_stereo=OFF \
      -DBUILD_people=OFF -DBUILD_outofcore=OFF -DBUILD_simulation=OFF \
 && cmake --build /tmp/pcl/build \
 && cmake --install /tmp/pcl/build \
 && rm -rf /tmp/pcl /tmp/pcl.tar.gz

# ------------------------------------------------------------------------------------------
FROM toolchain AS crow
ARG CROW_VERSION CROW_SHA256
ADD --checksum=sha256:${CROW_SHA256} \
    https://github.com/CrowCpp/Crow/archive/refs/tags/v${CROW_VERSION}.tar.gz /tmp/crow.tar.gz
RUN mkdir /opt/crow && tar -xzf /tmp/crow.tar.gz -C /opt/crow --strip-components=1

# ------------------------------------------------------------------------------------------
FROM toolchain AS build
# static archives only the final link needs (boost.iostreams references them)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update \
 && apt-get install -y --no-install-recommends zlib1g-dev libbz2-dev liblzma-dev libzstd-dev
COPY --from=pcl /opt/pcl /opt/pcl
COPY --from=crow /opt/crow /opt/crow
WORKDIR /src

# least to most often changed, so an edit in src/ reuses every layer above
COPY CMakeLists.txt ./
COPY config config
COPY include include
COPY src src

# ccache keeps objects between builds; -static-pie: one self-contained binary that keeps aslr
# .a suffix and Boost_USE_STATIC_LIBS make every lookup pick the static archive
RUN --mount=type=cache,target=/root/.cache/ccache \
    cmake -S . -B /build -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH=/opt/pcl \
      -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
      -DCMAKE_EXE_LINKER_FLAGS="-static-pie" \
      -DCMAKE_FIND_LIBRARY_SUFFIXES=.a \
      -DBoost_USE_STATIC_LIBS=ON \
      -DFETCHCONTENT_SOURCE_DIR_CROW=/opt/crow \
      -DFETCHCONTENT_FULLY_DISCONNECTED=ON \
 && cmake --build /build \
 && cmake --install /build --prefix /out --strip

# ------------------------------------------------------------------------------------------
FROM gcr.io/distroless/static-debian13:nonroot AS runtime
LABEL org.opencontainers.image.title="vcortex-backend" \
      org.opencontainers.image.description="vCortex backend: point cloud in, building twin out" \
      org.opencontainers.image.source="https://github.com/NickGhignatti/vCortex" \
      org.opencontainers.image.licenses="GPL-3.0-only"

WORKDIR /app
COPY --from=build /out/bin/vcortex /usr/local/bin/vcortex
COPY --from=build /out/share/vcortex/config ./config

EXPOSE 9002
USER nonroot:nonroot
ENTRYPOINT ["/usr/local/bin/vcortex"]
CMD ["--port", "9002"]
