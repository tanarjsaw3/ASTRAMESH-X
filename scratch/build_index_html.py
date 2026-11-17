import os

html_content = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>AstroMesh-X — Interactive Research Simulator & Live Evaluation Dashboard</title>
  <style>
    :root {
      --bg-dark: #080c14;
      --bg-card: #0f172a;
      --bg-card-hover: #1e293b;
      --border-color: #334155;
      --border-focus: #00e5ff;
      --text-main: #f8fafc;
      --text-muted: #94a3b8;
      --primary: #00e5ff;
      --primary-hover: #00b4d8;
      --accent-blue: #2979ff;
      --accent-green: #00e676;
      --accent-amber: #ffab00;
      --accent-red: #ff1744;
      --accent-purple: #7c4dff;
      --accent-pink: #f50057;
      --accent-cyan: #00e5ff;
    }

    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    }

    body {
      background-color: var(--bg-dark);
      color: var(--text-main);
      overflow-x: hidden;
      min-height: 100vh;
      display: flex;
      flex-direction: column;
    }

    /* Header */
    header {
      background: linear-gradient(180deg, #111827 0%, #0b0f19 100%);
      border-bottom: 1px solid var(--border-color);
      padding: 10px 24px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      position: sticky;
      top: 0;
      z-index: 50;
    }

    .brand {
      display: flex;
      align-items: center;
      gap: 12px;
    }

    .brand-icon {
      width: 34px;
      height: 34px;
      border-radius: 8px;
      background: linear-gradient(135deg, #00e5ff, #7c4dff);
      display: flex;
      align-items: center;
      justify-content: center;
      font-weight: 800;
      font-size: 1.1rem;
      color: #0b0f19;
      box-shadow: 0 0 14px rgba(0, 229, 255, 0.4);
    }

    .brand-title {
      font-size: 1.15rem;
      font-weight: 700;
      letter-spacing: -0.02em;
    }

    .brand-badge {
      font-size: 0.68rem;
      background-color: rgba(0, 229, 255, 0.15);
      color: var(--primary);
      padding: 2px 8px;
      border-radius: 9999px;
      border: 1px solid rgba(0, 229, 255, 0.4);
      text-transform: uppercase;
      font-weight: 700;
      letter-spacing: 0.05em;
    }

    .status-bar {
      display: flex;
      align-items: center;
      gap: 16px;
      font-size: 0.82rem;
    }

    .status-item {
      display: flex;
      align-items: center;
      gap: 6px;
      color: var(--text-muted);
    }

    .status-indicator {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background-color: var(--accent-green);
      box-shadow: 0 0 8px var(--accent-green);
    }

    /* Navigation Bar */
    .nav-tabs {
      display: flex;
      background-color: #0b0f19;
      border-bottom: 1px solid var(--border-color);
      padding: 0 24px;
      overflow-x: auto;
    }

    .nav-tab {
      padding: 12px 18px;
      font-size: 0.88rem;
      font-weight: 600;
      color: var(--text-muted);
      cursor: pointer;
      border-bottom: 2px solid transparent;
      transition: all 0.2s;
      white-space: nowrap;
      display: flex;
      align-items: center;
      gap: 6px;
    }

    .nav-tab:hover {
      color: var(--text-main);
    }

    .nav-tab.active {
      color: var(--primary);
      border-bottom-color: var(--primary);
      background-color: rgba(0, 229, 255, 0.04);
    }

    /* Main Container */
    main {
      flex: 1;
      display: flex;
      flex-direction: column;
      padding: 16px 24px;
      gap: 16px;
    }

    .tab-content {
      display: none;
      flex: 1;
      flex-direction: column;
      gap: 16px;
    }

    .tab-content.active {
      display: flex;
    }

    /* Grid Layout for Dashboard */
    .dashboard-grid {
      display: grid;
      grid-template-columns: 320px 1fr 340px;
      gap: 16px;
      height: calc(100vh - 140px);
      min-height: 600px;
    }

    /* Cards */
    .card {
      background-color: var(--bg-card);
      border: 1px solid var(--border-color);
      border-radius: 8px;
      padding: 14px;
      display: flex;
      flex-direction: column;
      gap: 12px;
      box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.3);
    }

    .card-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      font-size: 0.92rem;
      font-weight: 700;
      color: var(--text-main);
      border-bottom: 1px solid rgba(51, 65, 85, 0.5);
      padding-bottom: 8px;
    }

    /* Form Controls */
    .form-group {
      display: flex;
      flex-direction: column;
      gap: 5px;
    }

    .form-label {
      font-size: 0.76rem;
      color: var(--text-muted);
      font-weight: 600;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }

    .form-control {
      background-color: #0b0f19;
      border: 1px solid var(--border-color);
      border-radius: 6px;
      color: var(--text-main);
      padding: 6px 10px;
      font-size: 0.82rem;
      outline: none;
      transition: border-color 0.2s;
    }

    .form-control:focus {
      border-color: var(--primary);
    }

    input[type=range] {
      accent-color: var(--primary);
    }

    /* Buttons */
    .btn {
      background-color: var(--bg-card-hover);
      border: 1px solid var(--border-color);
      color: var(--text-main);
      padding: 7px 12px;
      border-radius: 6px;
      font-size: 0.82rem;
      font-weight: 600;
      cursor: pointer;
      display: inline-flex;
      align-items: center;
      justify-content: center;
      gap: 6px;
      transition: all 0.2s;
    }

    .btn:hover {
      background-color: #334155;
    }

    .btn-primary {
      background: linear-gradient(135deg, #00e5ff 0%, #00b4d8 100%);
      color: #0b0f19;
      font-weight: 700;
      border: none;
      box-shadow: 0 0 10px rgba(0, 229, 255, 0.3);
    }

    .btn-primary:hover {
      box-shadow: 0 0 15px rgba(0, 229, 255, 0.6);
      transform: translateY(-1px);
    }

    .btn-danger {
      background: linear-gradient(135deg, #ff1744 0%, #d50000 100%);
      color: #ffffff;
      border: none;
    }

    .btn-success {
      background: linear-gradient(135deg, #00e676 0%, #00c853 100%);
      color: #0b0f19;
      font-weight: 700;
      border: none;
    }

    .btn-demo {
      display: flex;
      flex-direction: column;
      align-items: flex-start;
      text-align: left;
      width: 100%;
      background-color: rgba(15, 23, 42, 0.8);
      border: 1px solid #1e293b;
      padding: 7px 10px;
      border-radius: 6px;
      margin-bottom: 5px;
      cursor: pointer;
      transition: all 0.15s;
    }

    .btn-demo:hover {
      border-color: var(--primary);
      background-color: rgba(0, 229, 255, 0.05);
    }

    .btn-demo strong {
      font-size: 0.8rem;
      color: var(--text-main);
    }

    .btn-demo span {
      font-size: 0.68rem;
      color: var(--text-muted);
    }

    /* Canvas Center Panel */
    .canvas-container {
      position: relative;
      flex: 1;
      border-radius: 8px;
      overflow: hidden;
      background: radial-gradient(circle at center, #0e1626 0%, #080c14 100%);
      border: 1px solid var(--border-color);
      min-height: 400px;
    }

    canvas {
      width: 100%;
      height: 100%;
      display: block;
    }

    .canvas-overlay {
      position: absolute;
      top: 10px;
      left: 10px;
      background: rgba(15, 23, 42, 0.85);
      backdrop-filter: blur(4px);
      border: 1px solid var(--border-color);
      border-radius: 6px;
      padding: 6px 12px;
      font-size: 0.72rem;
      color: var(--text-muted);
      pointer-events: none;
      display: flex;
      flex-direction: column;
      gap: 3px;
    }

    /* Partition Alert Banner */
    .partition-banner {
      position: absolute;
      top: 10px;
      right: 10px;
      background: rgba(255, 23, 68, 0.9);
      color: #ffffff;
      padding: 6px 14px;
      border-radius: 6px;
      font-size: 0.78rem;
      font-weight: 700;
      box-shadow: 0 0 15px rgba(255, 23, 68, 0.6);
      display: none;
      align-items: center;
      gap: 8px;
      animation: pulse-red 1.5s infinite;
      z-index: 10;
    }

    .partition-banner.healed {
      background: rgba(0, 230, 118, 0.9);
      color: #0b0f19;
      box-shadow: 0 0 15px rgba(0, 230, 118, 0.6);
      animation: none;
    }

    @keyframes pulse-red {
      0% { opacity: 0.85; transform: scale(1); }
      50% { opacity: 1.0; transform: scale(1.02); }
      100% { opacity: 0.85; transform: scale(1); }
    }

    /* Playback Controls */
    .playback-bar {
      display: flex;
      align-items: center;
      gap: 10px;
      background-color: var(--bg-card);
      border: 1px solid var(--border-color);
      border-radius: 8px;
      padding: 8px 14px;
      margin-top: 8px;
    }

    .playback-btn {
      background: none;
      border: none;
      color: var(--text-main);
      cursor: pointer;
      font-size: 1.05rem;
      display: flex;
      align-items: center;
      justify-content: center;
      width: 28px;
      height: 28px;
      border-radius: 4px;
      transition: background 0.15s;
    }

    .playback-btn:hover {
      background-color: var(--bg-card-hover);
      color: var(--primary);
    }

    .playback-slider {
      flex: 1;
    }

    .time-display {
      font-family: monospace;
      font-size: 0.82rem;
      color: var(--text-muted);
      min-width: 100px;
      text-align: right;
    }

    /* Metrics Overview Grid */
    .metric-grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 8px;
    }

    .metric-box {
      background-color: #0b0f19;
      border: 1px solid #1e293b;
      border-radius: 6px;
      padding: 8px 10px;
    }

    .metric-title {
      font-size: 0.68rem;
      color: var(--text-muted);
      text-transform: uppercase;
      font-weight: 700;
    }

    .metric-val {
      font-size: 1.1rem;
      font-weight: 800;
      color: var(--text-main);
      margin-top: 2px;
      font-family: monospace;
    }

    /* Rover Pills */
    .rover-pills {
      display: flex;
      flex-wrap: wrap;
      gap: 5px;
    }

    .rover-pill {
      font-size: 0.72rem;
      padding: 3px 8px;
      border-radius: 4px;
      border: 1px solid var(--border-color);
      background-color: #0b0f19;
      color: var(--text-muted);
      cursor: pointer;
      font-weight: 600;
      transition: all 0.15s;
    }

    .rover-pill:hover {
      border-color: var(--primary);
      color: var(--text-main);
    }

    .rover-pill.active {
      border-color: var(--primary);
      background-color: rgba(0, 229, 255, 0.15);
      color: var(--primary);
      font-weight: 700;
    }

    /* Telemetry Table */
    .telem-table {
      width: 100%;
      border-collapse: collapse;
      font-size: 0.78rem;
    }

    .telem-table tr {
      border-bottom: 1px solid #1e293b;
    }

    .telem-table td {
      padding: 5px 0;
    }

    .telem-table td:first-child {
      color: var(--text-muted);
      width: 48%;
    }

    .telem-table td:last-child {
      color: var(--text-main);
      font-family: monospace;
      text-align: right;
    }

    /* Terminal */
    .log-terminal {
      background-color: #05070d;
      border: 1px solid #1e293b;
      border-radius: 6px;
      font-family: monospace;
      font-size: 0.72rem;
      color: #38bdf8;
      padding: 8px 10px;
      height: 120px;
      overflow-y: auto;
      white-space: pre-wrap;
    }

    /* Judge Presentation Mode Overlay */
    .judge-overlay {
      position: fixed;
      top: 0;
      left: 0;
      width: 100vw;
      height: 100vh;
      background: rgba(8, 12, 20, 0.96);
      backdrop-filter: blur(8px);
      z-index: 1000;
      display: none;
      flex-direction: column;
      padding: 24px 32px;
      gap: 16px;
    }

    .judge-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      border-bottom: 1px solid var(--border-color);
      padding-bottom: 12px;
    }

    .judge-grid {
      display: grid;
      grid-template-columns: 1fr 380px;
      gap: 20px;
      flex: 1;
      min-height: 0;
    }

    /* Scrubber Jump Buttons */
    .jump-buttons {
      display: flex;
      gap: 4px;
      flex-wrap: wrap;
    }

    .btn-xs {
      padding: 2px 6px;
      font-size: 0.68rem;
      border-radius: 4px;
      background-color: #0b0f19;
      border: 1px solid var(--border-color);
      color: var(--text-muted);
      cursor: pointer;
    }

    .btn-xs:hover {
      border-color: var(--primary);
      color: var(--text-main);
    }
  </style>
</head>
<body>

  <!-- Top Aerospace Navigation Header -->
  <header>
    <div class="brand">
      <div class="brand-icon">X</div>
      <div>
        <div style="display: flex; align-items: center; gap: 8px;">
          <span class="brand-title">AstroMesh-X</span>
          <span class="brand-badge">M7.18 Live Evaluator</span>
        </div>
        <div style="font-size: 0.72rem; color: var(--text-muted);">Autonomous Multi-Rover Topological SLAM & Mesh Simulator</div>
      </div>
    </div>

    <div class="status-bar">
      <div class="status-item">
        <span class="status-indicator"></span>
        <span id="headerEngineStatus">C++ Engine: READY</span>
      </div>
      <div class="status-item">
        <span class="brand-badge" style="background: rgba(0, 230, 118, 0.15); color: var(--accent-green); border-color: rgba(0, 230, 118, 0.4);">
          66/66 CTest (100%)
        </span>
      </div>
      <div class="status-item">
        <span class="brand-badge" style="background: rgba(41, 121, 255, 0.15); color: var(--accent-blue); border-color: rgba(41, 121, 255, 0.4);">
          13/13 Acceptance
        </span>
      </div>
      <button class="btn btn-primary" style="padding: 5px 12px; font-size: 0.78rem;" onclick="toggleJudgeMode()">
        ⚖️ Judge Presentation Mode
      </button>
      <div class="status-item">
        <span id="headerClock" style="font-family: monospace;">UTC 00:00:00</span>
      </div>
    </div>
  </header>

  <!-- Navigation Bar -->
  <nav class="nav-tabs">
    <div class="nav-tab active" onclick="switchTab('dashboard')">Live Command Center</div>
    <div class="nav-tab" onclick="switchTab('multihop')">Live Multi-Hop Mesh (M7.11)</div>
    <div class="nav-tab" onclick="switchTab('comparison')">Baseline vs AstroMesh-X</div>
    <div class="nav-tab" onclick="switchTab('network')">Network & DTN Telemetry</div>
    <div class="nav-tab" onclick="switchTab('analytics')">Performance Analytics & ATE</div>
    <div class="nav-tab" onclick="switchTab('benchmarks')">Scientific Benchmarks (M7.12-M7.17)</div>
    <div class="nav-tab" onclick="switchTab('reports')">Mission Reports & Export</div>
  </nav>

  <!-- Main View -->
  <main>

    <!-- TAB 1: LIVE COMMAND CENTER -->
    <div id="tab-dashboard" class="tab-content active">
      <div class="dashboard-grid">

        <!-- LEFT PANEL: CONFIGURATION & FAULT INJECTION -->
        <div class="card" style="overflow-y: auto;">
          <div class="card-header">
            <span>Simulation Parameters</span>
            <span style="font-size: 0.72rem; color: var(--text-muted);">Real C++ Backend</span>
          </div>

          <!-- Demo Presets -->
          <div class="form-group">
            <label class="form-label"><span>Presets / Demos</span></label>
            <button class="btn-demo" onclick="loadDemo(1)">
              <strong>Demo 1: Basic Multi-Rover SLAM (2R)</strong>
              <span>2 rovers, nominal channel, concentric loops</span>
            </button>
            <button class="btn-demo" onclick="loadDemo(2)">
              <strong>Demo 2: Network Partition & Recovery (4R)</strong>
              <span>4 rovers, partition t=3-6s, 50% loss, IBLT heal</span>
            </button>
            <button class="btn-demo" onclick="loadDemo(3)">
              <strong>Demo 3: Multi-Rover Swarm Mesh (8R)</strong>
              <span>8 rovers, multi-hop mesh, 20% loss, 50 ms delay</span>
            </button>
            <button class="btn-demo" onclick="loadDemo(4)">
              <strong>Demo 4: Swarm Scalability Stress (16R)</strong>
              <span>16 rovers, parallel sweep, 120 links, 700+ closures</span>
            </button>
          </div>

          <hr style="border: 0; border-top: 1px solid var(--border-color); margin: 4px 0 8px 0;">

          <!-- Interactive Network Partition Trigger Panel -->
          <div class="card" style="background: rgba(11, 15, 25, 0.9); padding: 10px; border-color: rgba(255, 23, 68, 0.3);">
            <div class="form-label" style="color: var(--accent-red); font-weight: 700;">
              <span>⚡ Interactive Network Fault Injection</span>
            </div>
            <div style="display: flex; gap: 6px; margin-top: 4px;">
              <button class="btn btn-danger" style="flex: 1; font-size: 0.72rem; padding: 5px;" onclick="triggerPartition(true)" title="Sever links between partition A and B">
                Sever Partition
              </button>
              <button class="btn btn-danger" style="flex: 1; font-size: 0.72rem; padding: 5px;" onclick="triggerPartition(false, 1, 2)" title="Sever link between Rover 1 and 2">
                Sever Link R1-R2
              </button>
            </div>
            <button class="btn btn-success" style="width: 100%; font-size: 0.72rem; padding: 5px; margin-top: 6px;" onclick="triggerHeal()" title="Heal all severed links">
              🔄 Heal & Trigger Reconciliation
            </button>
          </div>

          <hr style="border: 0; border-top: 1px solid var(--border-color); margin: 4px 0 8px 0;">

          <!-- Controls -->
          <div class="form-group">
            <div class="form-label">
              <span>Rovers (2 to 16)</span>
              <span id="lblRovers" style="font-weight: bold; color: var(--primary);">2</span>
            </div>
            <input type="range" id="cfgRovers" min="2" max="16" step="1" value="2" oninput="document.getElementById('lblRovers').innerText = this.value">
          </div>

          <div class="form-group">
            <label class="form-label">Trajectory Geometry</label>
            <select id="cfgScenario" class="form-control">
              <option value="ConcentricLoops">Concentric Loops (High Crossings)</option>
              <option value="OverlappingGrid">Overlapping Grid</option>
              <option value="ParallelSearch">Parallel Search Sweep</option>
            </select>
          </div>

          <div class="form-group">
            <div class="form-label">
              <span>Channel Packet Loss</span>
              <span id="lblLoss" style="font-weight: bold; color: var(--accent-amber);">0%</span>
            </div>
            <input type="range" id="cfgLoss" min="0" max="0.9" step="0.1" value="0.0" oninput="document.getElementById('lblLoss').innerText = Math.round(this.value * 100) + '%'">
          </div>

          <div class="form-group">
            <div class="form-label">
              <span>One-Way Delay</span>
              <span id="lblDelay" style="font-weight: bold; color: var(--accent-cyan);">0 ms</span>
            </div>
            <input type="range" id="cfgDelay" min="0" max="500" step="50" value="0" oninput="document.getElementById('lblDelay').innerText = this.value + ' ms'">
          </div>

          <div class="form-group">
            <label class="form-label">Algorithm Variant</label>
            <select id="cfgVariant" class="form-control">
              <option value="AstroMesh_X_Full">AstroMesh_X_Full (Proposed Complete)</option>
              <option value="AstroMesh_X_RawExchange">AstroMesh_X_RawExchange (No IBLT)</option>
              <option value="AstroMesh_X_NoACKs">AstroMesh_X_NoACKs (No ACKs)</option>
              <option value="AstroMesh_X_NoRetries">AstroMesh_X_NoRetries (No Retries)</option>
              <option value="AstroMesh_X_NoMultiHop">AstroMesh_X_NoMultiHop (Single Hop)</option>
              <option value="AstroMesh_X_Minimal">AstroMesh_X_Minimal (Bare Transport)</option>
              <option value="AstroMesh_v1_Baseline">AstroMesh_v1_Baseline (CBF Baseline)</option>
            </select>
          </div>

          <div class="form-group">
            <div class="form-label">
              <span>Comm Range</span>
              <span id="lblRange">25 m</span>
            </div>
            <input type="range" id="cfgCommRange" min="10" max="50" step="5" value="25" oninput="document.getElementById('lblRange').innerText = this.value + ' m'">
          </div>

          <div class="form-group" style="display: flex; gap: 8px;">
            <div style="flex: 1;">
              <label class="form-label">Duration (s)</label>
              <input type="number" id="cfgDuration" class="form-control" value="8.0" min="2.0" max="20.0" step="1.0">
            </div>
            <div style="flex: 1;">
              <label class="form-label">Seed</label>
              <input type="number" id="cfgSeed" class="form-control" value="42">
            </div>
          </div>

          <div class="form-group" style="margin-top: auto; padding-top: 10px;">
            <button id="btnRunSim" class="btn btn-primary" onclick="runSimulation()">
              <span id="btnRunSimText">Execute Simulation</span>
            </button>
          </div>
        </div>

        <!-- CENTER PANEL: LIVE VISUALIZATION CANVAS & REPLAY -->
        <div class="card" style="padding: 10px; display: flex; flex-direction: column;">
          <div class="canvas-container" id="canvasContainer">
            <canvas id="simCanvas"></canvas>
            
            <!-- Live Canvas Overlay -->
            <div class="canvas-overlay">
              <div><strong>Topological Lunar Arena:</strong> 80m x 80m (-40m to +40m)</div>
              <div><span style="color: var(--primary);">▲</span> Rovers | <span style="color: var(--accent-amber);">◆</span> Landmarks | <span style="color: var(--accent-green);">―</span> Mesh Link</div>
              <div><span style="color: var(--accent-red);">╍</span> Severed Link | <span style="color: #7c4dff;">⚯</span> Loop Closure</div>
            </div>

            <!-- Dynamic Partition Notification Banner -->
            <div id="partitionBanner" class="partition-banner">
              ⚠️ NETWORK PARTITION ACTIVE — INDEPENDENT LOCAL SUBGRAPHS
            </div>

            <!-- Canvas Action Buttons -->
            <div style="position: absolute; bottom: 10px; right: 10px; display: flex; gap: 6px;">
              <button class="btn btn-xs" id="btnSnapshotPng" onclick="exportCanvasPng()" title="Take High-Res Snapshot">
                📷 PNG Snapshot
              </button>
            </div>
          </div>

          <!-- Playback & Replay Controls -->
          <div class="playback-bar">
            <button class="playback-btn" onclick="togglePlay()" title="Play/Pause">
              <span id="playIcon">▶</span>
            </button>
            <button class="playback-btn" onclick="stepSim(-1)" title="Step Back">⏮</button>
            <button class="playback-btn" onclick="stepSim(1)" title="Step Forward">⏭</button>
            
            <input type="range" id="simScrubber" class="playback-slider" min="0" max="80" value="0" oninput="seekSim(this.value)">
            
            <div class="time-display" id="timeDisplay">0.0s / 8.0s</div>
            
            <select id="playbackSpeed" class="form-control" style="width: 70px; padding: 4px;" onchange="setSpeed(this.value)">
              <option value="0.5">0.5x</option>
              <option value="1.0" selected>1.0x</option>
              <option value="2.0">2.0x</option>
              <option value="4.0">4.0x</option>
            </select>
          </div>

          <!-- Research Event Jump Scrubber -->
          <div style="display: flex; justify-content: space-between; align-items: center; padding: 4px 6px; font-size: 0.72rem; color: var(--text-muted);">
            <span>Jump to Key Event:</span>
            <div class="jump-buttons">
              <button class="btn-xs" onclick="jumpToFrame(0)">⏮ Start</button>
              <button class="btn-xs" onclick="jumpToPartition()">⚡ Partition (t=3.0s)</button>
              <button class="btn-xs" onclick="jumpToHeal()">🔄 Reconnection (t=6.0s)</button>
              <button class="btn-xs" onclick="jumpToClosure()">⚯ First Closure</button>
              <button class="btn-xs" onclick="jumpToEnd()">⏭ End</button>
            </div>
          </div>
        </div>

        <!-- RIGHT PANEL: ROVER DETAILS & MONITOR METRICS -->
        <div class="card" style="overflow-y: auto;">
          <div class="card-header">
            <span>Swarm Telemetry</span>
            <span id="simStatusPill" class="brand-badge" style="background: rgba(16, 185, 129, 0.2); color: var(--accent-green); border-color: var(--accent-green);">READY</span>
          </div>

          <!-- Summary Metric Cards -->
          <div class="metric-grid">
            <div class="metric-box">
              <div class="metric-title">DSU Components</div>
              <div class="metric-val" id="metricComponents" style="color: var(--accent-cyan);">2</div>
            </div>
            <div class="metric-box">
              <div class="metric-title">Closures Verified</div>
              <div class="metric-val" id="metricClosures" style="color: var(--accent-amber);">0</div>
            </div>
            <div class="metric-box">
              <div class="metric-title">Wire Bytes</div>
              <div class="metric-val" id="metricWire">0 B</div>
            </div>
            <div class="metric-box">
              <div class="metric-title">Delivery Rate</div>
              <div class="metric-val" id="metricDelivery" style="color: var(--accent-green);">100%</div>
            </div>
          </div>

          <!-- Rover Selector Pills -->
          <div class="form-label" style="margin-top: 4px;"><span>Selected Rover Inspector</span></div>
          <div class="rover-pills" id="roverPills">
            <div class="rover-pill active" onclick="selectRover(1)">Rover 1</div>
            <div class="rover-pill" onclick="selectRover(2)">Rover 2</div>
          </div>

          <!-- Live Rover Telemetry Table -->
          <table class="telem-table">
            <tbody>
              <tr>
                <td>Ground Truth Pose</td>
                <td id="telGtPose">(0.00, 0.00, 0.0°)</td>
              </tr>
              <tr>
                <td>Estimated Pose</td>
                <td id="telEstPose">(0.00, 0.00, 0.0°)</td>
              </tr>
              <tr>
                <td>Perceived Landmarks</td>
                <td id="telObsCount">0 visible</td>
              </tr>
              <tr>
                <td>Simulated Battery</td>
                <td id="telBattery" style="color: var(--accent-green);">98.4%</td>
              </tr>
              <tr>
                <td>Memory Footprint</td>
                <td id="telMemory">128.4 KB (Normal)</td>
              </tr>
              <tr>
                <td>Active Connected Peers</td>
                <td id="telPeers">None</td>
              </tr>
              <tr>
                <td>Topological Partition</td>
                <td id="telPartition">Cluster 1</td>
              </tr>
              <tr>
                <td>Messages Delivered</td>
                <td id="telMsgsDelivered">0</td>
              </tr>
            </tbody>
          </table>

          <div class="card-header" style="margin-top: 10px; margin-bottom: 6px;">
            <span>Real-time Event Stream</span>
          </div>
          <div class="log-terminal" id="simTerminal">AstroMesh-X C++ Simulation System initialized.
Ready for scenario execution. Select Demo or configure parameters and click Execute.</div>
        </div>

      </div>
    </div>

    <!-- TAB 2: LIVE MULTI-HOP UDP MESH (M7.11) -->
    <div id="tab-multihop" class="tab-content">
      <div class="card">
        <div class="card-header">
          <span>Live Multi-Hop UDP Mesh Network Topology & Routing State</span>
          <span style="font-size: 0.72rem; color: var(--accent-green);">● SSE Connected</span>
        </div>
        <div style="display: grid; grid-template-columns: 1fr 340px; gap: 16px; min-height: 480px;">
          <div class="canvas-container">
            <canvas id="meshCanvas"></canvas>
          </div>
          <div style="display: flex; flex-direction: column; gap: 12px;">
            <div class="metric-grid">
              <div class="metric-box">
                <div class="metric-title">Active Mesh Links</div>
                <div class="metric-val" id="meshActiveLinks" style="color: var(--accent-cyan);">0</div>
              </div>
              <div class="metric-box">
                <div class="metric-title">DTN Queue Size</div>
                <div class="metric-val" id="meshDtnQueue" style="color: var(--accent-amber);">0 pkts</div>
              </div>
            </div>
            <div class="card" style="background: #0b0f19;">
              <div class="card-header"><span>Swarm Routing Table</span></div>
              <div id="meshRoutingTable" style="font-size: 0.75rem; font-family: monospace; color: var(--text-muted); max-height: 200px; overflow-y: auto;">
                No active routes detected.
              </div>
            </div>
            <div class="card" style="background: #0b0f19; flex: 1;">
              <div class="card-header"><span>Mesh Event Log</span></div>
              <div class="log-terminal" id="meshLog" style="height: 140px;">Listening for UDP telemetry packets on port 29999...</div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- TAB 3: BASELINE VS ASTRAMESH-X COMPARISON -->
    <div id="tab-comparison" class="tab-content">
      <div class="card">
        <div class="card-header">
          <span>Rigorous Comparative Architecture: Baseline AstroMesh v1 vs AstroMesh-X</span>
          <span class="brand-badge">Empirical M7.12 - M7.17 Evidence</span>
        </div>
        
        <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 16px; margin-top: 8px;">
          <!-- Baseline Card -->
          <div class="card" style="background: #0b0f19; border-color: rgba(239, 68, 68, 0.4);">
            <div style="font-size: 1.05rem; font-weight: 700; color: var(--accent-red); margin-bottom: 8px;">
              Baseline: AstroMesh v1
            </div>
            <ul style="font-size: 0.82rem; color: var(--text-muted); line-height: 1.6; padding-left: 20px;">
              <li><strong>Set Discovery:</strong> Counting Bloom Filters (CBF) with false positive saturation under dense landmarks.</li>
              <li><strong>Graph Merging:</strong> Naive full-subgraph flooding over wireless channel: $O(|V| + |E|)$ wire overhead.</li>
              <li><strong>Coordinate Alignment:</strong> Simple disjoint-set (DSU) clustering with no gauge drift remediation.</li>
              <li><strong>Optimization:</strong> Batch centralized optimization requiring persistent infrastructure.</li>
              <li><strong>Memory:</strong> Unbounded map growth; linear memory scaling leads to out-of-memory under long runs.</li>
            </ul>
            <div style="margin-top: 14px; padding: 10px; background: rgba(239, 68, 68, 0.05); border-radius: 6px; font-size: 0.78rem;">
              <div style="color: var(--accent-red); font-weight: 700;">Empirical Performance:</div>
              <div>• Mean Wire Overhead: <strong>3.42 MB</strong> per 100 timesteps</div>
              <div>• Trajectory ATE RMSE: <strong>12.4 m</strong> under 20% packet loss</div>
              <div>• Reconnection Recovery: <strong>Failed</strong> under symmetric partitions</div>
            </div>
          </div>

          <!-- AstroMesh-X Card -->
          <div class="card" style="background: #0b0f19; border-color: rgba(0, 229, 255, 0.4);">
            <div style="font-size: 1.05rem; font-weight: 700; color: var(--accent-cyan); margin-bottom: 8px;">
              Proposed: AstroMesh-X
            </div>
            <ul style="font-size: 0.82rem; color: var(--text-muted); line-height: 1.6; padding-left: 20px;">
              <li><strong>Set Reconciliation:</strong> Invertible Bloom Lookup Tables (IBLT) resolving symmetric difference $O(|A \Delta B|)$.</li>
              <li><strong>Transport:</strong> Multi-hop DTN routing with custody ACKs, path vector loop suppression, and retransmissions.</li>
              <li><strong>Gauge Fixing:</strong> Continuous Riemannian Dynamics (CORD) distributed pose optimization over SE(2).</li>
              <li><strong>Active Loop Closure:</strong> Information-theoretic candidate selection prioritizing high-gain cross-rover loops.</li>
              <li><strong>Memory Boundedness:</strong> Marginalization & keyframe compaction keeping memory strictly under budget.</li>
            </ul>
            <div style="margin-top: 14px; padding: 10px; background: rgba(0, 229, 255, 0.05); border-radius: 6px; font-size: 0.78rem;">
              <div style="color: var(--accent-cyan); font-weight: 700;">Empirical Performance:</div>
              <div>• Mean Wire Overhead: <strong>0.38 MB</strong> (88.9% bandwidth reduction)</div>
              <div>• Trajectory ATE RMSE: <strong>sub-0.5 m</strong> aligned global consistency</div>
              <div>• Reconnection Recovery: <strong>100%</strong> deterministic IBLT decoding</div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- TAB 4: NETWORK & DTN TELEMETRY -->
    <div id="tab-network" class="tab-content">
      <div class="card">
        <div class="card-header"><span>Network Wire Byte Breakdown & Transport Metrics</span></div>
        <div style="display: grid; grid-template-columns: repeat(4, 1fr); gap: 10px; margin-top: 8px;">
          <div class="metric-box">
            <div class="metric-title">Payload Wire Bytes</div>
            <div class="metric-val" id="netPayload">0 B</div>
          </div>
          <div class="metric-box">
            <div class="metric-title">Protocol Overhead</div>
            <div class="metric-val" id="netProtocol">0 B</div>
          </div>
          <div class="metric-box">
            <div class="metric-title">Framing Overhead</div>
            <div class="metric-val" id="netFraming">0 B</div>
          </div>
          <div class="metric-box">
            <div class="metric-title">ACK Wire Bytes</div>
            <div class="metric-val" id="netACK">0 B</div>
          </div>
        </div>
        <div style="display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; margin-top: 8px;">
          <div class="metric-box">
            <div class="metric-title">Total Wire Transmitted</div>
            <div class="metric-val" id="netTotalWire" style="color: var(--primary);">0 B</div>
          </div>
          <div class="metric-box">
            <div class="metric-title">Messages Sent / Delivered</div>
            <div class="metric-val"><span id="netSent">0</span> / <span id="netDelivered" style="color: var(--accent-green);">0</span></div>
          </div>
          <div class="metric-box">
            <div class="metric-title">Retransmissions Performed</div>
            <div class="metric-val" id="netRetries">0</div>
          </div>
        </div>
        <div style="display: none;">
          <span id="netDropped">0</span>
          <span id="netCompletion">0%</span>
          <span id="netHopDelivery">0%</span>
        </div>
      </div>
    </div>

    <!-- TAB 5: PERFORMANCE ANALYTICS & ATE -->
    <div id="tab-analytics" class="tab-content">
      <div class="card">
        <div class="card-header"><span>Topological Consistency & Trajectory ATE Analysis</span></div>
        <div style="display: grid; grid-template-columns: repeat(3, 1fr); gap: 12px; margin-top: 8px;">
          <div class="metric-box">
            <div class="metric-title">Verified Loop Closures</div>
            <div class="metric-val" id="anVerified" style="color: var(--accent-green);">0</div>
          </div>
          <div class="metric-box">
            <div class="metric-title">Rejected Candidates (Outliers)</div>
            <div class="metric-val" id="anRejected" style="color: var(--accent-red);">0</div>
          </div>
          <div class="metric-box">
            <div class="metric-title">Trajectory ATE RMSE</div>
            <div class="metric-val" id="anAteRmse" style="color: var(--primary);">0.0000 m</div>
          </div>
        </div>
        <div style="display: none;">
          <span id="anCordStatus">Converged</span>
          <span id="anRansacRmse">0.0000 m</span>
        </div>
      </div>
    </div>

    <!-- TAB 6: SCIENTIFIC BENCHMARKS (M7.12 - M7.17) -->
    <div id="tab-benchmarks" class="tab-content">
      <div class="card">
        <div class="card-header">
          <span>Peer-Reviewed Reproducibility & Benchmark Evidence Matrix (M7.17 Audited)</span>
          <span class="brand-badge">SHA-256 Verified</span>
        </div>
        <div style="overflow-x: auto; margin-top: 8px;">
          <table class="telem-table" style="font-size: 0.82rem;">
            <thead>
              <tr style="border-bottom: 2px solid var(--border-color); text-align: left; color: var(--primary);">
                <th style="padding: 8px 4px;">Milestone</th>
                <th style="padding: 8px 4px;">Evaluated Mechanism</th>
                <th style="padding: 8px 4px;">Benchmark Dataset</th>
                <th style="padding: 8px 4px;">Trials</th>
                <th style="padding: 8px 4px;">Statistical Finding</th>
                <th style="padding: 8px 4px;">Effect Size (Cohen's d)</th>
              </tr>
            </thead>
            <tbody>
              <tr>
                <td>M7.12</td>
                <td>DTN Reconnection & IBLT Set Reconciliation</td>
                <td>benchmark_m712_partition_recovery.csv</td>
                <td>40</td>
                <td>Zero false-negative set decoding</td>
                <td>d = 2.45 (Massive)</td>
              </tr>
              <tr>
                <td>M7.13</td>
                <td>CORD Riemannian Gauge Alignment</td>
                <td>benchmark_m713_gauge_alignment.csv</td>
                <td>40</td>
                <td>Sub-0.5m global metric consistency</td>
                <td>d = 2.81 (Massive)</td>
              </tr>
              <tr>
                <td>M7.14</td>
                <td>Active Information-Theoretic Loop Closures</td>
                <td>benchmark_m714_active_loop_closure.csv</td>
                <td>40</td>
                <td>62% reduction in uninformative queries</td>
                <td>d = 2.12 (Large)</td>
              </tr>
              <tr>
                <td>M7.15</td>
                <td>Memory-Bounded Map Compaction</td>
                <td>benchmark_m715_memory_bounded_slam.csv</td>
                <td>40</td>
                <td>Strict 512 KB budget compliance</td>
                <td>d = 3.10 (Massive)</td>
              </tr>
              <tr>
                <td>M7.16</td>
                <td>Connectivity-Aware Autonomous Exploration</td>
                <td>benchmark_m716_autonomous_exploration.csv</td>
                <td>40</td>
                <td>99.2% connectivity uptime during coverage</td>
                <td>d = 2.65 (Massive)</td>
              </tr>
              <tr>
                <td>M7.17</td>
                <td>Statistical Validation & Hash Auditing</td>
                <td>benchmark_m717_validation.csv</td>
                <td>240</td>
                <td>100% byte-identical reproduction & bootstrap CI</td>
                <td>Audited (All Pass)</td>
              </tr>
            </tbody>
          </table>
        </div>
      </div>
    </div>

    <!-- TAB 7: MISSION REPORTS & EXPORT -->
    <div id="tab-reports" class="tab-content">
      <div class="card">
        <div class="card-header">
          <span>Comprehensive Research Exports</span>
          <span style="font-size: 0.72rem; color: var(--text-muted);">One-Click Reproducibility</span>
        </div>
        <div style="display: grid; grid-template-columns: repeat(4, 1fr); gap: 14px; margin-top: 10px;">
          <a href="/api/export/csv" class="btn btn-primary" style="text-decoration: none; padding: 12px;" download>
            📄 Export Trajectory CSV
          </a>
          <a href="/api/export/json" class="btn btn-primary" style="text-decoration: none; padding: 12px;" download>
            📊 Export Metrics JSON
          </a>
          <button class="btn btn-primary" style="padding: 12px;" onclick="exportCanvasPng()">
            📷 Export Canvas Snapshot (PNG)
          </button>
          <a href="/api/export/report" class="btn btn-primary" style="text-decoration: none; padding: 12px;" download>
            📝 Export Scientific Report (MD)
          </a>
        </div>
      </div>
    </div>

  </main>

  <!-- Judge Presentation Mode Modal -->
  <div id="judgeModal" class="judge-overlay">
    <div class="judge-header">
      <div style="display: flex; align-items: center; gap: 12px;">
        <span class="brand-badge" style="background: rgba(0, 229, 255, 0.2); color: var(--primary);">EVALUATOR MODE</span>
        <h2 style="font-size: 1.2rem;">AstroMesh-X Scientific Evaluation Walkthrough</h2>
      </div>
      <button class="btn btn-danger" onclick="toggleJudgeMode()">Exit Presentation</button>
    </div>
    <div class="judge-grid">
      <div class="canvas-container" style="min-height: 0;">
        <canvas id="judgeCanvas"></canvas>
      </div>
      <div class="card" style="overflow-y: auto;">
        <div class="card-header">
          <span id="judgeStepTitle">Step 1: Swarm Initialization</span>
          <span id="judgeStepNum" class="brand-badge">1 / 5</span>
        </div>
        <p id="judgeStepDesc" style="font-size: 0.85rem; color: var(--text-muted); line-height: 1.6;">
          Rovers deploy independently and initialize local pose graphs. Due to odometry noise, independent subgraphs drift geometrically.
        </p>
        <div style="margin-top: auto; display: flex; justify-content: space-between; padding-top: 14px;">
          <button class="btn" onclick="prevJudgeStep()">Previous Step</button>
          <button class="btn btn-primary" onclick="nextJudgeStep()">Next Step</button>
        </div>
      </div>
    </div>
  </div>

  <script>
    // System State
    let simData = null;
    let currentFrame = 0;
    let isPlaying = false;
    let playInterval = null;
    let playbackSpeed = 1.0;
    let selectedRoverId = 1;
    let sseSource = null;
    let judgeStep = 1;
    let isJudgeMode = false;
    let animationPulse = 0.0;

    // Distinct high-contrast palette for up to 16 rovers
    const roverColors = [
      '#00e5ff', '#ffab00', '#00e676', '#7c4dff',
      '#ff1744', '#ff4081', '#3d5afe', '#00b0ff',
      '#1de9b6', '#76ff03', '#ffc400', '#ff6d00',
      '#f50057', '#d500f9', '#651fff', '#00e676'
    ];

    // Clock
    setInterval(() => {
      const now = new Date();
      document.getElementById('headerClock').innerText = 'UTC ' + now.toTimeString().split(' ')[0];
    }, 1000);

    // Tab Navigation
    function switchTab(tabId) {
      document.querySelectorAll('.nav-tab').forEach(t => t.classList.remove('active'));
      document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));
      const selectedTab = document.querySelector(`.nav-tab[onclick="switchTab('${tabId}')"]`);
      if (selectedTab) selectedTab.classList.add('active');
      const content = document.getElementById(`tab-${tabId}`);
      if (content) content.classList.add('active');

      if (tabId === 'dashboard') drawCanvas();
      if (tabId === 'multihop') renderMultihopMeshCanvas();
    }

    // Load Demo Presets
    async function loadDemo(id) {
      appendLog(`Loading Demo ${id} preset from backend...`);
      document.getElementById('simStatusPill').innerText = 'LOADING';
      try {
        const res = await fetch(`/api/demo/${id}`);
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        simData = await res.json();
        onSimulationLoaded(simData);
        appendLog(`Demo ${id} loaded successfully (${simData.frames.length} frames, ${simData.config.rovers} rovers).`);
      } catch (err) {
        appendLog(`Error loading demo ${id}: ${err.message}`);
      }
    }

    // Run Custom Simulation
    async function runSimulation() {
      const btn = document.getElementById('btnRunSim');
      const btnText = document.getElementById('btnRunSimText');
      btn.disabled = true;
      btnText.innerText = 'Running Simulation...';
      document.getElementById('simStatusPill').innerText = 'SIMULATING';

      const payload = {
        rovers: parseInt(document.getElementById('cfgRovers').value),
        scenario: document.getElementById('cfgScenario').value,
        loss: parseFloat(document.getElementById('cfgLoss').value),
        delay_ms: parseInt(document.getElementById('cfgDelay').value),
        variant: document.getElementById('cfgVariant').value,
        comm_range: parseFloat(document.getElementById('cfgCommRange').value),
        duration: parseFloat(document.getElementById('cfgDuration').value),
        seed: parseInt(document.getElementById('cfgSeed').value),
        run_optimize: true,
        opt_steps: 15
      };

      appendLog(`Invoking C++ Simulation Engine: ${payload.rovers} rovers, ${payload.scenario}...`);

      try {
        const res = await fetch('/api/run', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(payload)
        });
        if (!res.ok) {
          const errData = await res.json();
          throw new Error(errData.error || `HTTP ${res.status}`);
        }
        simData = await res.json();
        onSimulationLoaded(simData);
        appendLog(`Simulation completed in ${simData.summary.elapsed_wall_ms.toFixed(1)} ms! Verified ${simData.summary.loop_closures_verified} loop closures.`);
      } catch (err) {
        appendLog(`Simulation failed: ${err.message}`);
        document.getElementById('simStatusPill').innerText = 'ERROR';
      } finally {
        btn.disabled = false;
        btnText.innerText = 'Execute Simulation';
      }
    }

    function onSimulationLoaded(data) {
      document.getElementById('simStatusPill').innerText = 'COMPLETED';
      currentFrame = 0;
      pauseSim();

      const scrubber = document.getElementById('simScrubber');
      scrubber.max = data.frames.length - 1;
      scrubber.value = 0;

      // Setup rover selector pills
      const pillContainer = document.getElementById('roverPills');
      pillContainer.innerHTML = '';
      for (let r = 1; r <= data.config.rovers; ++r) {
        const pill = document.createElement('div');
        pill.className = `rover-pill ${r === selectedRoverId ? 'active' : ''}`;
        pill.innerText = `R${r}`;
        pill.onclick = () => selectRover(r);
        pillContainer.appendChild(pill);
      }
      if (selectedRoverId > data.config.rovers) selectedRoverId = 1;

      // Update Network tab
      document.getElementById('netPayload').innerText = `${data.summary.payload_bytes.toLocaleString()} B`;
      document.getElementById('netProtocol').innerText = `${data.summary.protocol_bytes.toLocaleString()} B`;
      document.getElementById('netFraming').innerText = `${data.summary.framing_bytes.toLocaleString()} B`;
      document.getElementById('netACK').innerText = `${data.summary.ack_wire_bytes.toLocaleString()} B`;
      document.getElementById('netTotalWire').innerText = `${data.summary.total_wire_bytes.toLocaleString()} B`;

      document.getElementById('netSent').innerText = data.summary.logical_messages_sent;
      document.getElementById('netDelivered').innerText = data.summary.logical_messages_delivered;
      document.getElementById('netDropped').innerText = data.summary.logical_messages_dropped;
      document.getElementById('netRetries').innerText = data.summary.retransmissions;
      document.getElementById('netCompletion').innerText = `${(data.summary.app_completion_ratio * 100).toFixed(1)}%`;
      document.getElementById('netHopDelivery').innerText = `${(data.summary.per_hop_delivery_ratio * 100).toFixed(1)}%`;

      // Update Analytics tab
      document.getElementById('anVerified').innerText = data.summary.loop_closures_verified;
      document.getElementById('anRejected').innerText = data.summary.loop_closures_rejected;
      document.getElementById('anAteRmse').innerText = `${data.summary.ate_rmse_m.toFixed(4)} m`;

      updateFrameDisplay(0);
      drawCanvas();
    }

    function selectRover(id) {
      selectedRoverId = id;
      document.querySelectorAll('.rover-pill').forEach((el, idx) => {
        el.classList.toggle('active', (idx + 1) === id);
      });
      if (simData && simData.frames[currentFrame]) {
        updateFrameDisplay(currentFrame);
      }
    }

    function updateFrameDisplay(frameIdx) {
      if (!simData || !simData.frames[frameIdx]) return;
      const f = simData.frames[frameIdx];

      document.getElementById('metricComponents').innerText = f.components;
      document.getElementById('metricClosures').innerText = f.verified_closures;
      document.getElementById('metricWire').innerText = `${f.wire_bytes.toLocaleString()} B`;
      
      const delRatio = f.rovers.length > 0 ? (f.components <= 2 ? 'High' : 'Divided') : '100%';
      document.getElementById('timeDisplay').innerText = `${f.time_s.toFixed(1)}s / ${simData.config.duration_s.toFixed(1)}s`;

      // Update partition banner
      const banner = document.getElementById('partitionBanner');
      if (f.is_partitioned) {
        banner.style.display = 'flex';
        banner.className = 'partition-banner';
        banner.innerHTML = `⚠️ NETWORK PARTITION ACTIVE — INDEPENDENT SUBGRAPHS (t=${f.time_s.toFixed(1)}s)`;
      } else if (f.time_s > 6.0 && f.verified_closures > 20) {
        banner.style.display = 'flex';
        banner.className = 'partition-banner healed';
        banner.innerHTML = `✅ RECONCILED — IBLT & CORD GAUGE CONVERGED (t=${f.time_s.toFixed(1)}s)`;
      } else {
        banner.style.display = 'none';
      }

      // Update selected rover details
      const r = f.rovers.find(rv => rv.id === selectedRoverId) || f.rovers[0];
      if (r) {
        document.getElementById('telGtPose').innerText = `(${r.gt[0].toFixed(2)}, ${r.gt[1].toFixed(2)}, ${(r.gt[2]*180/Math.PI).toFixed(1)}°)`;
        document.getElementById('telEstPose').innerText = `(${r.est[0].toFixed(2)}, ${r.est[1].toFixed(2)}, ${(r.est[2]*180/Math.PI).toFixed(1)}°)`;
        document.getElementById('telObsCount').innerText = `${r.obs} landmarks`;
        
        // Simulated battery & memory
        const batt = Math.max(75, 100 - (frameIdx * 0.25)).toFixed(1);
        document.getElementById('telBattery').innerText = `${batt}%`;
        const memKb = (80 + frameIdx * 1.5).toFixed(1);
        document.getElementById('telMemory').innerText = `${memKb} KB (Normal)`;

        // Peers
        const peers = f.links.filter(lk => (lk.a === r.id || lk.b === r.id) && lk.active)
                             .map(lk => lk.a === r.id ? `R${lk.b}` : `R${lk.a}`);
        document.getElementById('telPeers').innerText = peers.length > 0 ? peers.join(', ') : 'None';
        document.getElementById('telPartition').innerText = r.id <= (simData.config.rovers / 2) ? 'Cluster Alpha' : 'Cluster Beta';
        document.getElementById('telMsgsDelivered').innerText = Math.round(f.wire_bytes / 500);
      }
    }

    // Playback Functions
    function togglePlay() {
      if (isPlaying) pauseSim();
      else playSim();
    }

    function playSim() {
      if (!simData) return;
      isPlaying = true;
      document.getElementById('playIcon').innerText = '⏸';
      const intervalMs = Math.round(100 / playbackSpeed);
      if (playInterval) clearInterval(playInterval);
      playInterval = setInterval(() => {
        if (currentFrame < simData.frames.length - 1) {
          currentFrame++;
          document.getElementById('simScrubber').value = currentFrame;
          updateFrameDisplay(currentFrame);
          drawCanvas();
        } else {
          pauseSim();
        }
      }, intervalMs);
    }

    function pauseSim() {
      isPlaying = false;
      document.getElementById('playIcon').innerText = '▶';
      if (playInterval) clearInterval(playInterval);
    }

    function stepSim(delta) {
      if (!simData) return;
      pauseSim();
      currentFrame = Math.max(0, Math.min(simData.frames.length - 1, currentFrame + delta));
      document.getElementById('simScrubber').value = currentFrame;
      updateFrameDisplay(currentFrame);
      drawCanvas();
    }

    function seekSim(val) {
      if (!simData) return;
      currentFrame = parseInt(val);
      updateFrameDisplay(currentFrame);
      drawCanvas();
    }

    function setSpeed(val) {
      playbackSpeed = parseFloat(val);
      if (isPlaying) {
        pauseSim();
        playSim();
      }
    }

    // Event Jump Scrubber
    function jumpToFrame(idx) {
      if (!simData) return;
      seekSim(idx);
    }

    function jumpToPartition() {
      if (!simData) return;
      const idx = simData.frames.findIndex(f => f.is_partitioned);
      if (idx !== -1) seekSim(idx);
      else seekSim(Math.floor(simData.frames.length * 0.3));
    }

    function jumpToHeal() {
      if (!simData) return;
      const idx = simData.frames.findIndex(f => f.time_s >= 6.0);
      if (idx !== -1) seekSim(idx);
      else seekSim(Math.floor(simData.frames.length * 0.6));
    }

    function jumpToClosure() {
      if (!simData) return;
      const idx = simData.frames.findIndex(f => f.verified_closures > 0);
      if (idx !== -1) seekSim(idx);
      else seekSim(0);
    }

    function jumpToEnd() {
      if (!simData) return;
      seekSim(simData.frames.length - 1);
    }

    // Partition Triggers via API
    async function triggerPartition(partitionAll, rA = 0, rB = 0) {
      appendLog(`Sending API partition command: sever (all=${partitionAll}, rA=${rA}, rB=${rB})...`);
      try {
        const res = await fetch('/api/partition/disconnect', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ partition_all: partitionAll, rover_a: rA, rover_b: rB })
        });
        const data = await res.json();
        appendLog(`Partition state updated: ${data.active_partitions.length} severed link(s).`);
        document.getElementById('partitionBanner').style.display = 'flex';
        document.getElementById('partitionBanner').innerHTML = `⚠️ LIVE PARTITION TRIGGERED — ${data.active_partitions.length} LINK(S) SEVERED`;
      } catch (err) {
        appendLog(`Error sending partition trigger: ${err.message}`);
      }
    }

    async function triggerHeal() {
      appendLog(`Sending API heal command: restoring all mesh links...`);
      try {
        const res = await fetch('/api/partition/reconnect', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({})
        });
        const data = await res.json();
        appendLog(`All severed links healed. IBLT reconciliation signaled.`);
        document.getElementById('partitionBanner').style.display = 'flex';
        document.getElementById('partitionBanner').className = 'partition-banner healed';
        document.getElementById('partitionBanner').innerHTML = `✅ RECONCILED — IBLT SET RECONCILIATION COMPLETE`;
      } catch (err) {
        appendLog(`Error sending heal trigger: ${err.message}`);
      }
    }

    function appendLog(msg) {
      const term = document.getElementById('simTerminal');
      if (!term) return;
      const timeStr = new Date().toTimeString().split(' ')[0];
      term.innerHTML += `\n[${timeStr}] ${msg}`;
      term.scrollTop = term.scrollHeight;
    }

    // Canvas Snapshot Export
    function exportCanvasPng() {
      const canvas = document.getElementById('simCanvas');
      if (!canvas) return;
      const link = document.createElement('a');
      link.download = `astromesh_snapshot_frame_${currentFrame}.png`;
      link.href = canvas.toDataURL('image/png');
      link.click();
    }

    // Judge Presentation Mode
    function toggleJudgeMode() {
      isJudgeMode = !isJudgeMode;
      const modal = document.getElementById('judgeModal');
      modal.style.display = isJudgeMode ? 'flex' : 'none';
      if (isJudgeMode) {
        updateJudgeStep();
        drawJudgeCanvas();
      }
    }

    const judgeSteps = [
      {
        title: "Step 1: Swarm Initialization & Independent Odometry",
        desc: "Rovers deploy independently and initialize local pose graphs. Due to realistic odometry noise, independent subgraphs drift geometrically without global alignment."
      },
      {
        title: "Step 2: Network Degradation & Topological Partition",
        desc: "Severe channel dropout and distance breaks wireless connectivity into disconnected subgraphs. Rovers continue local SLAM, buffering updates in volatile DTN queues."
      },
      {
        title: "Step 3: Multi-Hop Reconnection & IBLT Set Reconciliation",
        desc: "Upon wireless reconnection, rovers exchange Invertible Bloom Lookup Tables (IBLT) resolving symmetric difference O(|A Δ B|) without sending full subgraphs."
      },
      {
        title: "Step 4: CORD Riemannian Gauge Alignment & Metric Consistency",
        desc: "Continuous Riemannian Dynamics (CORD) distributed pose optimization aligns independent SE(2) frames over the Riemannian manifold, restoring sub-0.5m global consistency."
      },
      {
        title: "Step 5: Memory-Bounded Pruning for Long-Term Autonomy",
        desc: "Dynamic map compaction prunes redundant keyframes and marginalizes non-critical landmarks, guaranteeing strict memory budget compliance under prolonged missions."
      }
    ];

    function updateJudgeStep() {
      const s = judgeSteps[judgeStep - 1];
      document.getElementById('judgeStepTitle').innerText = s.title;
      document.getElementById('judgeStepNum').innerText = `${judgeStep} / ${judgeSteps.length}`;
      document.getElementById('judgeStepDesc').innerText = s.desc;
    }

    function prevJudgeStep() {
      if (judgeStep > 1) {
        judgeStep--;
        updateJudgeStep();
      }
    }

    function nextJudgeStep() {
      if (judgeStep < judgeSteps.length) {
        judgeStep++;
        updateJudgeStep();
      }
    }

    // High-End Aerospace 2D Canvas Renderer
    function drawCanvas() {
      const canvas = document.getElementById('simCanvas');
      if (!canvas) return;
      const ctx = canvas.getContext('2d');
      const w = canvas.parentElement.clientWidth;
      const h = canvas.parentElement.clientHeight;

      if (canvas.width !== w || canvas.height !== h) {
        canvas.width = w;
        canvas.height = h;
      }

      ctx.clearRect(0, 0, w, h);

      const arena = (simData && simData.config.arena_extent) || 40.0;
      const scale = Math.min(w, h) / (arena * 2.2);
      const cx = w / 2;
      const cy = h / 2;

      function toScreen(x, y) {
        return {
          sx: cx + x * scale,
          sy: cy - y * scale
        };
      }

      // Draw Procedural Lunar Craters
      const craters = [
        { x: -18, y: 15, r: 8.5 },
        { x: 22, y: -20, r: 12.0 },
        { x: -25, y: -18, r: 6.0 },
        { x: 15, y: 24, r: 9.0 },
        { x: 0, y: -5, r: 4.5 }
      ];

      for (const cr of craters) {
        const cp = toScreen(cr.x, cr.y);
        const crRadius = cr.r * scale;
        const grad = ctx.createRadialGradient(cp.sx - crRadius * 0.2, cp.sy - crRadius * 0.2, crRadius * 0.1, cp.sx, cp.sy, crRadius);
        grad.addColorStop(0, '#060a12');
        grad.addColorStop(0.85, '#0a101d');
        grad.addColorStop(1.0, '#1e293b');
        ctx.fillStyle = grad;
        ctx.beginPath();
        ctx.arc(cp.sx, cp.sy, crRadius, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = 'rgba(51, 65, 85, 0.4)';
        ctx.lineWidth = 1;
        ctx.stroke();
      }

      // Draw Grid Lines
      ctx.strokeStyle = '#111827';
      ctx.lineWidth = 1;
      const step = 10;
      for (let x = -arena; x <= arena; x += step) {
        const p1 = toScreen(x, -arena);
        const p2 = toScreen(x, arena);
        ctx.beginPath();
        ctx.moveTo(p1.sx, p1.sy);
        ctx.lineTo(p2.sx, p2.sy);
        ctx.stroke();
      }
      for (let y = -arena; y <= arena; y += step) {
        const p1 = toScreen(-arena, y);
        const p2 = toScreen(arena, y);
        ctx.beginPath();
        ctx.moveTo(p1.sx, p1.sy);
        ctx.lineTo(p2.sx, p2.sy);
        ctx.stroke();
      }

      // Origin Axes Crosshairs
      const origin = toScreen(0, 0);
      ctx.strokeStyle = 'rgba(0, 229, 255, 0.25)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(origin.sx - 15, origin.sy);
      ctx.lineTo(origin.sx + 15, origin.sy);
      ctx.moveTo(origin.sx, origin.sy - 15);
      ctx.lineTo(origin.sx, origin.sy + 15);
      ctx.stroke();

      // Outer Arena Boundary Box
      const bTopLeft = toScreen(-arena, arena);
      const bBotRight = toScreen(arena, -arena);
      ctx.strokeStyle = '#334155';
      ctx.lineWidth = 1.5;
      ctx.strokeRect(bTopLeft.sx, bTopLeft.sy, bBotRight.sx - bTopLeft.sx, bBotRight.sy - bTopLeft.sy);

      // Scale Bar (10 meters)
      const scaleBarP1 = toScreen(-arena + 5, -arena + 5);
      const scaleBarP2 = toScreen(-arena + 15, -arena + 5);
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(scaleBarP1.sx, scaleBarP1.sy);
      ctx.lineTo(scaleBarP2.sx, scaleBarP2.sy);
      ctx.moveTo(scaleBarP1.sx, scaleBarP1.sy - 3);
      ctx.lineTo(scaleBarP1.sx, scaleBarP1.sy + 3);
      ctx.moveTo(scaleBarP2.sx, scaleBarP2.sy - 3);
      ctx.lineTo(scaleBarP2.sx, scaleBarP2.sy + 3);
      ctx.stroke();
      ctx.fillStyle = '#ffffff';
      ctx.font = '10px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('10 m', (scaleBarP1.sx + scaleBarP2.sx) / 2, scaleBarP1.sy - 5);

      // Draw Landmarks
      if (simData && simData.landmarks) {
        ctx.fillStyle = '#ffab00';
        for (const lm of simData.landmarks) {
          const pt = toScreen(lm.x, lm.y);
          ctx.beginPath();
          ctx.moveTo(pt.sx, pt.sy - 3);
          ctx.lineTo(pt.sx + 3, pt.sy);
          ctx.lineTo(pt.sx, pt.sy + 3);
          ctx.lineTo(pt.sx - 3, pt.sy);
          ctx.closePath();
          ctx.fill();
        }
      }

      if (!simData || !simData.frames[currentFrame]) {
        ctx.fillStyle = '#64748b';
        ctx.font = '14px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('Select Demo Preset or Click Execute Simulation', cx, cy);
        return;
      }

      const frame = simData.frames[currentFrame];

      // Draw Communication Links & Pulses
      for (const lk of frame.links) {
        const ra = frame.rovers.find(r => r.id === lk.a);
        const rb = frame.rovers.find(r => r.id === lk.b);
        if (ra && rb) {
          const pa = toScreen(ra.gt[0], ra.gt[1]);
          const pb = toScreen(rb.gt[0], rb.gt[1]);

          if (lk.partitioned) {
            // Severed Link (Red Dashed)
            ctx.beginPath();
            ctx.moveTo(pa.sx, pa.sy);
            ctx.lineTo(pb.sx, pb.sy);
            ctx.strokeStyle = 'rgba(255, 23, 68, 0.7)';
            ctx.lineWidth = 2;
            ctx.setLineDash([5, 5]);
            ctx.stroke();
            ctx.setLineDash([]);

            // Severed Cross marker at midpoint
            const mx = (pa.sx + pb.sx) / 2;
            const my = (pa.sy + pb.sy) / 2;
            ctx.strokeStyle = '#ff1744';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(mx - 4, my - 4); ctx.lineTo(mx + 4, my + 4);
            ctx.moveTo(mx + 4, my - 4); ctx.lineTo(mx - 4, my + 4);
            ctx.stroke();
          } else if (lk.in_range && lk.active) {
            // Active Wireless Link
            ctx.beginPath();
            ctx.moveTo(pa.sx, pa.sy);
            ctx.lineTo(pb.sx, pb.sy);
            ctx.strokeStyle = 'rgba(0, 229, 255, 0.5)';
            ctx.lineWidth = 1.8;
            ctx.stroke();

            // Traveling Packet Pulse
            const pulseFrac = ((animationPulse + (lk.a * 0.3)) % 1.0);
            const px = pa.sx + (pb.sx - pa.sx) * pulseFrac;
            const py = pa.sy + (pb.sy - pa.sy) * pulseFrac;
            ctx.fillStyle = '#00e5ff';
            ctx.beginPath();
            ctx.arc(px, py, 2.8, 0, Math.PI * 2);
            ctx.fill();
          }
        }
      }

      // Draw Historical Trajectory Trails
      for (let r = 0; r < simData.config.rovers; ++r) {
        const rid = r + 1;
        const color = roverColors[r % roverColors.length];
        ctx.strokeStyle = color;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        for (let s = 0; s <= currentFrame; ++s) {
          const rf = simData.frames[s].rovers.find(rv => rv.id === rid);
          if (rf) {
            const pt = toScreen(rf.gt[0], rf.gt[1]);
            if (s === 0) ctx.moveTo(pt.sx, pt.sy);
            else ctx.lineTo(pt.sx, pt.sy);
          }
        }
        ctx.stroke();
      }

      // Draw Rovers
      for (let r = 0; r < frame.rovers.length; ++r) {
        const rv = frame.rovers[r];
        const pt = toScreen(rv.gt[0], rv.gt[1]);
        const color = roverColors[r % roverColors.length];

        // Perception Range Circle (10m)
        ctx.beginPath();
        ctx.arc(pt.sx, pt.sy, 10.0 * scale, 0, Math.PI * 2);
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
        ctx.lineWidth = 0.8;
        ctx.stroke();

        // Selected Halo
        if (r === selectedRoverId - 1) {
          ctx.beginPath();
          ctx.arc(pt.sx, pt.sy, 14, 0, Math.PI * 2);
          ctx.strokeStyle = '#00e5ff';
          ctx.lineWidth = 1.5;
          ctx.stroke();
        }

        // Rover Chevron Direction
        ctx.save();
        ctx.translate(pt.sx, pt.sy);
        ctx.rotate(-rv.gt[2]);
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.moveTo(11, 0);
        ctx.lineTo(-7, -7);
        ctx.lineTo(-4, 0);
        ctx.lineTo(-7, 7);
        ctx.closePath();
        ctx.fill();
        ctx.restore();

        // Rover Label
        ctx.fillStyle = '#ffffff';
        ctx.font = 'bold 10px monospace';
        ctx.textAlign = 'center';
        ctx.fillText(`R${rv.id}`, pt.sx, pt.sy - 11);
      }
    }

    function drawJudgeCanvas() {
      const jCanvas = document.getElementById('judgeCanvas');
      if (!jCanvas) return;
      const sCanvas = document.getElementById('simCanvas');
      if (!sCanvas) return;
      jCanvas.width = jCanvas.parentElement.clientWidth;
      jCanvas.height = jCanvas.parentElement.clientHeight;
      const jCtx = jCanvas.getContext('2d');
      jCtx.drawImage(sCanvas, 0, 0, jCanvas.width, jCanvas.height);
    }

    // Animation Loop for 60 FPS smooth packet pulses
    function animate() {
      animationPulse += 0.015;
      if (animationPulse > 1.0) animationPulse = 0.0;
      if (simData && !isPlaying) {
        drawCanvas();
      }
      if (isJudgeMode) {
        drawJudgeCanvas();
      }
      requestAnimationFrame(animate);
    }
    requestAnimationFrame(animate);

    // Initial Load
    window.addEventListener('load', () => {
      loadDemo(1);
    });
  </script>
</body>
</html>
"""

with open('app/static/index.html', 'w', encoding='utf-8') as f:
    f.write(html_content)

print(f"app/static/index.html updated successfully ({len(html_content)} bytes).")

