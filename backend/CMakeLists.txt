cmake_minimum_required(VERSION 3.25)
project(vcortex VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # for clangd

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------

# Point-cloud algorithms (core) and PCD reading (io). PCL is an algorithm library here:
# domain types never contain PCL types.
find_package(PCL 1.14 REQUIRED COMPONENTS common io kdtree search sample_consensus filters segmentation)
# Static build (Docker): PCL's per-module targets leave FLANN out (it is only in PCL_LIBRARIES),
# and FLANN's pkg-config file names the shared liblz4.so. Link FLANN explicitly (below) and
# point it at the LZ4 archive.
if(TARGET FLANN::FLANN)
  get_target_property(_flann FLANN::FLANN ALIASED_TARGET)  # PCL makes it an alias of flann's own
  if(NOT _flann)
    set(_flann FLANN::FLANN)
  endif()
  get_target_property(_flann_type ${_flann} TYPE)
  if(_flann_type STREQUAL "STATIC_LIBRARY")
    find_library(LZ4_STATIC_LIBRARY NAMES liblz4.a REQUIRED)
    set_property(TARGET ${_flann} PROPERTY INTERFACE_LINK_LIBRARIES ${LZ4_STATIC_LIBRARY})
  endif()
endif()
find_package(nlohmann_json 3.11 REQUIRED)
find_package(Threads REQUIRED)

# HTTP + WebSocket server. Pinned: v1.3.4 fixed an authentication bypass in the WebSocket upgrade.
# The Docker build passes -DFETCHCONTENT_SOURCE_DIR_CROW=<dir> so nothing is downloaded there.
include(FetchContent)
FetchContent_Declare(Crow
  URL https://github.com/CrowCpp/Crow/archive/refs/tags/v1.3.4.tar.gz
  URL_HASH SHA256=6be759b3a648b41f0adc3a8408a53fd0243775217b4e32e8101d69e4aadf48cc
  DOWNLOAD_EXTRACT_TIMESTAMP ON
  SYSTEM)  # Crow headers are not ours: no warnings from them
set(CROW_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(Crow)

# ---------------------------------------------------------------------------
# Warnings
# ---------------------------------------------------------------------------

add_library(vcortex_warnings INTERFACE)
target_compile_options(vcortex_warnings INTERFACE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic -Wshadow -Wconversion>)

# ---------------------------------------------------------------------------
# vcortex_core: domain, model, connectors, protocol. Plain C++ + PCL algorithms. No Crow, no files.
# ---------------------------------------------------------------------------

add_library(vcortex_core STATIC
  src/model/clean.cpp
  src/model/floor.cpp
  src/model/align.cpp
  src/model/grid.cpp
  src/model/candidates.cpp
  src/model/walls.cpp
  src/model/rooms.cpp
  src/model/model.cpp
  src/connectors/connector.cpp
  src/connectors/twin_json.cpp
  src/connectors/dxf.cpp
  src/connectors/ply.cpp
  src/protocol/ingest.cpp
  src/protocol/raw_frame.cpp
)
target_include_directories(vcortex_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(vcortex_core SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_compile_definitions(vcortex_core PUBLIC ${PCL_DEFINITIONS})
target_compile_features(vcortex_core PUBLIC cxx_std_17)
target_link_libraries(vcortex_core
  PUBLIC
    pcl_common pcl_kdtree pcl_search pcl_sample_consensus pcl_filters pcl_segmentation
    $<TARGET_NAME_IF_EXISTS:FLANN::FLANN>
    nlohmann_json::nlohmann_json
  PRIVATE
    vcortex_warnings)

# ---------------------------------------------------------------------------
# vcortex_io: frameworks and drivers. Crow, PCD files, threads.
# ---------------------------------------------------------------------------

add_library(vcortex_io STATIC
  src/io/pcd_input.cpp
  src/io/map_store.cpp
  src/io/model_worker.cpp
  src/io/web_server.cpp
)
target_link_libraries(vcortex_io
  PUBLIC vcortex_core pcl_io Crow::Crow Threads::Threads
  PRIVATE vcortex_warnings)

# ---------------------------------------------------------------------------
# vcortex: the server
# ---------------------------------------------------------------------------

add_executable(vcortex src/main.cpp)
target_link_libraries(vcortex PRIVATE vcortex_io vcortex_warnings)

install(TARGETS vcortex RUNTIME DESTINATION bin)
install(DIRECTORY config/ DESTINATION share/vcortex/config)
