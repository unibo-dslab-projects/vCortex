cmake_minimum_required(VERSION 3.25)
project(vcortex VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # for clangd

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# --- dependencies ---

# point-cloud algorithms (core) and pcd reading (io); pcl is an algorithm library here,
# domain types never hold pcl types
find_package(PCL 1.14 REQUIRED COMPONENTS common io kdtree search sample_consensus filters segmentation)
# static build (docker): pcl module targets leave flann out, and flann's pkg-config names the
# shared liblz4.so, so link flann explicitly against the lz4 archive
if(TARGET FLANN::FLANN)
  get_target_property(_flann FLANN::FLANN ALIASED_TARGET)  # pcl makes it an alias of flann's own
  if(NOT _flann)
    set(_flann FLANN::FLANN)
  endif()
  get_target_property(_flann_type ${_flann} TYPE)
  if(_flann_type STREQUAL "STATIC_LIBRARY")
    find_library(LZ4_STATIC_LIBRARY NAMES liblz4.a REQUIRED)
    set_property(TARGET ${_flann} PROPERTY INTERFACE_LINK_LIBRARIES ${LZ4_STATIC_LIBRARY})
  endif()
endif()
# vectors, boxes and rotations for the domain and the model; no version asked: eigen 5 rejects
# a 3.x request, and pcl already needs some eigen 3.3+
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(nlohmann_json 3.11 REQUIRED)
find_package(Threads REQUIRED)

# http + websocket server, pinned: v1.3.4 fixed an auth bypass in the websocket upgrade
# docker passes -DFETCHCONTENT_SOURCE_DIR_CROW=<dir>, so nothing is downloaded there
include(FetchContent)
FetchContent_Declare(Crow
  URL https://github.com/CrowCpp/Crow/archive/refs/tags/v1.3.4.tar.gz
  URL_HASH SHA256=6be759b3a648b41f0adc3a8408a53fd0243775217b4e32e8101d69e4aadf48cc
  DOWNLOAD_EXTRACT_TIMESTAMP ON
  SYSTEM)  # crow headers are not ours: no warnings from them
set(CROW_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(Crow)

# --- warnings ---

add_library(vcortex_warnings INTERFACE)
target_compile_options(vcortex_warnings INTERFACE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic -Wshadow -Wconversion>)

# --- vcortex_core: domain, model, connectors, protocol; c++, eigen and pcl only, no crow, no files ---

add_library(vcortex_core STATIC
  src/geometry/points.cpp
  src/geometry/vectors.cpp
  src/geometry/grids.cpp
  src/domain/settings.cpp
  src/model/clean.cpp
  src/model/floor/floor.cpp
  src/model/align.cpp
  src/model/grid.cpp
  src/model/wall/walls.cpp
  src/model/rooms.cpp
  src/model/floor/free_space.cpp
  src/model/wall/outline.cpp
  src/model/model.cpp
  src/model/object/objects.cpp
  src/model/object/grid.cpp
  src/model/object/labels.cpp
  src/model/object/outline.cpp
  src/connectors/connector.cpp
  src/connectors/json.cpp
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
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
  PRIVATE
    vcortex_warnings)

# --- vcortex_io: frameworks and drivers (crow, pcd files, threads) ---

add_library(vcortex_io STATIC
  src/io/config.cpp
  src/io/offline.cpp
  src/io/store.cpp
  src/io/worker.cpp
  src/io/server.cpp
)
target_link_libraries(vcortex_io
  PUBLIC vcortex_core pcl_io Crow::Crow Threads::Threads
  PRIVATE vcortex_warnings)

# --- vcortex: the server ---

add_executable(vcortex src/main.cpp)
target_link_libraries(vcortex PRIVATE vcortex_io vcortex_warnings)

# --- tests: off in docker, which builds offline and copies no tests/ ---

include(CTest)
if(BUILD_TESTING)
  FetchContent_Declare(doctest
    URL https://github.com/doctest/doctest/archive/refs/tags/v2.4.12.tar.gz
    URL_HASH SHA256=73381c7aa4dee704bd935609668cf41880ea7f19fa0504a200e13b74999c2d70
    DOWNLOAD_EXTRACT_TIMESTAMP ON
    SYSTEM)
  set(DOCTEST_NO_INSTALL ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(doctest)

  add_executable(vcortex_tests
    tests/main.cpp
    tests/model/floor/free_space_test.cpp
    tests/model/wall/outline_test.cpp
    tests/model/wall/outline_steps_test.cpp
    tests/model/wall/walls_test.cpp
  )
  target_include_directories(vcortex_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  target_link_libraries(vcortex_tests PRIVATE vcortex_core doctest::doctest vcortex_warnings)
  add_test(NAME vcortex_tests COMMAND vcortex_tests)
endif()

install(TARGETS vcortex RUNTIME DESTINATION bin)
install(DIRECTORY config/ DESTINATION share/vcortex/config)
