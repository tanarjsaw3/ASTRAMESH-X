# ==============================================================================
# AstroMesh-X — Autonomous Multi-Rover Decentralized Topological SLAM
# Production Multi-Stage Container for Full-Stack C++ Engine & Mission Control
# ==============================================================================

# ------------------------------------------------------------------------------
# Stage 1: Build the C++20 sim_runner engine
# ------------------------------------------------------------------------------
FROM ubuntu:22.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive

# Install build toolchain
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ninja-build \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Copy build configuration and source tree
COPY CMakeLists.txt ./
COPY include/ ./include/
COPY src/ ./src/

# Configure and compile sim_runner CLI in Release mode
RUN cmake -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF \
    && cmake --build build --target sim_runner -j$(nproc)

# ------------------------------------------------------------------------------
# Stage 2: Minimal runtime image with Python backend and Mission Control UI
# ------------------------------------------------------------------------------
FROM python:3.11-slim AS runner

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PORT=8080 \
    HOST=0.0.0.0 \
    ASTROMESH_SIM_RUNNER=/app/bin/sim_runner

WORKDIR /app

# Install minimal runtime libraries (libstdc++ for C++ binary, curl for healthcheck)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libstdc++6 \
    ca-certificates \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install Python requirements
COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

# Copy compiled C++ simulation executable from builder stage
COPY --from=builder /workspace/build/bin/sim_runner /app/bin/sim_runner
RUN chmod +x /app/bin/sim_runner

# Copy application backend, static web UI, documentation, and benchmark datasets
COPY app/ ./app/
COPY docs/ ./docs/
COPY scratch/ ./scratch/
COPY benchmark_*.csv ./

# Ensure scratch directory exists and is writable
RUN mkdir -p /app/scratch && chmod 777 /app/scratch

# Service ports (8080 for HTTP Mission Control, 29999 for UDP Swarm Telemetry)
EXPOSE 8080
EXPOSE 29999/udp

# Healthcheck for container orchestrators (Render, Railway, Kubernetes, Docker)
HEALTHCHECK --interval=20s --timeout=5s --start-period=5s --retries=3 \
    CMD curl -f http://127.0.0.1:${PORT}/api/status || exit 1

# Start the AstroMesh-X Mission Control server
CMD ["python3", "app/server.py"]
