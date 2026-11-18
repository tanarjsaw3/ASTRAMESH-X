# 🚀 AstroMesh-X — Full-Stack Cloud Deployment Guide (Render & Docker)

This guide explains how to deploy the **complete AstroMesh-X platform** to the cloud with **zero components skipped**. The deployed service includes the live C++20 SLAM simulation engine (`sim_runner`), the Python backend server, and the 60 FPS interactive Mission Control dashboard.

---

## 🏗️ Architecture Overview

The production container uses a **multi-stage Docker build**:
1. **Builder Stage (`ubuntu:22.04`):** Compiles the C++20 `sim_runner` engine using CMake and Ninja in Release mode.
2. **Runner Stage (`python:3.11-slim`):** Runs the lightweight Python telemetry server (`app/server.py`) and serves the static Mission Control UI, IEEE paper viewer, and REST API.
3. **Live Execution (`/api/run`):** When a user or examiner triggers a simulation with custom parameters, the Python server executes the compiled C++ binary in real time.

```
┌────────────────────────────────────────────────────────┐
│             CLOUD CONTAINER (RENDER / DOCKER)          │
│                                                        │
│  [ Web Browser / Examiner ]                            │
│           │                                            │
│           ▼ (HTTP Port 8080)                           │
│  [ app/server.py (Python HTTP & Telemetry) ]           │
│     ├── Static UI & Paper Viewer (app/static/)         │
│     ├── REST API (/api/status, /api/demo/{id})         │
│     └── Live Subprocess Execution ───┐                 │
│                                      ▼                 │
│                       [ /app/bin/sim_runner (C++) ]    │
│                       • IBLT Set Reconciliation        │
│                       • Continuous Riemannian Dynamics │
│                       • Disjoint-Set Union Merging     │
└────────────────────────────────────────────────────────┘
```

---

## 🌐 Deploying to Render.com (Step-by-Step)

Render provides free hosting for Docker-based web services directly from your GitHub repository.

### Step 1: Commit and Push Changes to GitHub
Make sure all deployment files (`Dockerfile`, `docker-compose.yml`, `render.yaml`, `requirements.txt`, and updated `app/server.py`) are pushed to your repository:
```bash
git add Dockerfile docker-compose.yml render.yaml requirements.txt app/server.py docs/DEPLOYMENT_GUIDE_RENDER_DOCKER.md
git commit -m "feat(deploy): Add production Dockerfile and Render deployment blueprint"
git push origin <your-branch-name>
```

### Step 2: Create a Web Service on Render
1. Go to [https://dashboard.render.com](https://dashboard.render.com) and log in with your GitHub account.
2. Click the **"New +"** button in the top navigation bar and select **"Web Service"**.
3. Choose **"Build and deploy from a Git repository"** and click **Next**.
4. Connect and select your **`ASTRAMESH-X`** repository.

### Step 3: Configure and Launch
Render will automatically detect the `Dockerfile` and `render.yaml`:
- **Name:** `astromesh-x` (or your choice)
- **Region:** Any (e.g., Oregon, Frankfurt)
- **Branch:** Select your active branch (`feature/ui-refactor` or `main`)
- **Runtime:** `Docker` (Auto-detected)
- **Instance Type:** `Free`
- **Health Check Path:** `/api/status`

Click **"Deploy Web Service"**.

### Step 4: Access Your Live Mission Control
Render will:
1. Pull the Ubuntu base image and install GCC/CMake.
2. Compile the C++ `sim_runner` engine.
3. Install Python requirements (`pandas`).
4. Start the server and verify the `/api/status` healthcheck.
5. Provide a live public HTTPS URL (e.g., `https://astromesh-x.onrender.com`).

---

## 💻 Running Locally (During In-Person Viva / Defense)

For your viva examination or laptop demonstration, you don't even need Docker:

### Direct Native Execution (Fastest & Offline)
```bash
python app/server.py
```
- Open [http://localhost:8765](http://localhost:8765) in your web browser.
- Automatically connects to your local Windows `sim_runner.exe`.
- Runs with **0 ms network latency** and requires **zero internet connection**.

### Local Docker Execution (Optional)
If running on a machine with Docker Desktop installed:
```bash
docker compose up --build
```
- Open [http://localhost:8080](http://localhost:8080).

---

## 📋 Pre-Flight Verification Checklist

Before walking into your demonstration or viva examination, verify:
- [ ] `/api/status` reports `"status": "online"` and `"engine_available": true`.
- [ ] Scenario 1 (Alpha 2R) loads 81 frames with cyan consensus shockwave.
- [ ] Scenario 2 (Beta 4R) displays all 4 rovers and active mesh links.
- [ ] Fault Injection button `[⚡ Sever Radio]` initiates unaligned drift.
- [ ] Synchronization button `[🔗 1 KB IBLT Sync]` restores topological consensus.
- [ ] PDF export button downloads `ASTROMESH_X_RESEARCH_MANUSCRIPT.pdf`.
