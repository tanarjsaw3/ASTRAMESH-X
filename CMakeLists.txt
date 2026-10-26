cmake_minimum_required(VERSION 3.20)

project(AstroMeshX
    VERSION 0.1.0
    DESCRIPTION "Decentralized Collaborative Topological SLAM for Multi-Rover Exploration"
    LANGUAGES CXX
)

# Enforce standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable testing framework
include(CTest)

# Set build output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# Public include directory
set(ASTROMESH_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

message(STATUS "Configuring AstroMesh-X v${PROJECT_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Testing: ${BUILD_TESTING}")

# ==============================================================================
# UNIMPLEMENTED MODULES (PHASE 0 PLACEHOLDER STATUS)
# ==============================================================================
# In accordance with Phase 0 scientific and engineering rules:
# No fake source files or empty placeholder libraries are generated here.
#
# Concrete CMake targets will be added incrementally in upcoming phases:
# - Phase 1: Core Data Structures (Landmark Fingerprints, IBLT reconciliation, CBF baseline)
# - Phase 2: Topological Graph & Distributed Disjoint Set Union (DSU)
# - Phase 3: Continuous Riemannian Dynamics (CORD) distributed pose graph optimization
# - Phase 4: Delay-tolerant transport & communications layer
# - Phase 5: Simulation harness & multi-rover benchmark runners
# ==============================================================================
