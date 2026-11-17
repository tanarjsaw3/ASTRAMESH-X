# AstroMesh-X — Milestone 7.18 Complete Scientific Reproducibility Package

**Document Version:** 1.0.0  
**Verification Date:** 2026-11-15  
**Originating Milestone:** Milestone 7.18 Research Manuscript Package  
**Target Environment:** Windows 10/11, MSYS2 UCRT64 GCC 16.2.0 (`x86_64-w64-mingw32`), CMake 4.4.4, Ninja 1.13.2, Python 3.10+

---

## 1. Toolchain & Operating System Requirements

AstroMesh-X is designed for deterministic, software-only reproducibility without external hardware, ROS 2, or Gazebo dependencies.

### 1.1 Verified Platform Configuration
- **Operating System:** Windows 10 / Windows 11 (64-bit)
- **Compiler:** GCC 16.2.0 (`x86_64-w64-mingw32`, MSYS2 UCRT64 runtime)
- **Build System:** Ninja 1.13.2 with CMake 4.4.4
- **Python Runtime:** Python 3.10+ (Standard Library only, `csv`, `hashlib`, `math`, `random`)
- **Networking Stack:** Real OS loopback UDP sockets (`127.0.0.1`, ports 29999, 8765)

### 1.2 Environment Setup (PowerShell)
Execute the following session command to configure your build shell:
```powershell
# Set compiler and toolchain paths in process-local PATH
$env:Path = "C:\msys64\ucrt64\bin;C:\msys64\usr\bin;" + $env:Path

# Verify toolchain availability
gcc --version     # Expected: gcc.exe (Rev..., Built by MSYS2 project) 16.2.0
ninja --version   # Expected: 1.13.2
cmake --version   # Expected: 4.4.4 or compatible
python --version  # Expected: Python 3.10+
```

---

## 2. Compilation and Build Configuration

All C++ targets are configured via CMake and compiled using Ninja in Release mode:
```powershell
# Configure clean build directory
cmake -B build-ucrt64-clean -G Ninja -DCMAKE_BUILD_TYPE=Release

# Compile full project (libraries, executables, benchmarks, and test targets)
cmd /c "set PATH=C:\msys64\ucrt64\bin;%PATH% && ninja -C build-ucrt64-clean"
```

---

## 3. Automated Verification Suites

### 3.1 Complete CTest Regression (66 Targets)
Execute all 66 unit, integration, and property tests:
```powershell
$env:Path = "C:\msys64\ucrt64\bin;C:\msys64\usr\bin;" + $env:Path
ctest --test-dir build-ucrt64-clean --output-on-failure
```
*Expected Result:* **100% tests passed (66 of 66 passed in $< 10.0$ seconds).**

### 3.2 Acceptance Test Suite (13 System Verification Checks)
Verify high-level mission control and simulation invariants:
```powershell
python scratch/test_acceptance.py
```
*Expected Result:* **13 of 13 acceptance checks PASS (100% accepted).**

### 3.3 Non-Destructive Dataset Schema & SHA-256 Validation
Verify that all canonical CSV datasets exist, conform to schemas, and possess exact SHA-256 hashes:
```powershell
python scratch/validate_reproducibility_package.py
```
*Expected Result:* **OVERALL RESULT: [PASS] All canonical benchmark datasets are 100% verified.**

### 3.4 Unified Statistical Validation Harness
Generate bootstrap confidence intervals, Cohen's $d$, and Wilcoxon signed-rank tests:
```powershell
python scratch/reproducibility_harness.py
```
*Outputs Generated:*
- `benchmark_m717_validation.csv`
- `benchmark_m717_statistical_summary.csv`
- `benchmark_m717_evidence_matrix.csv`

---

## 4. Canonical Dataset Inventory & SHA-256 Hashes

All historical milestone datasets are immutable and preserved byte-identically in the repository root:

| Dataset Filename | Size (Bytes) | Canonical SHA-256 Checksum | Focus Domain & Milestone |
| :--- | :---: | :---: | :--- |
| `benchmark_m712_partition_recovery.csv` | 5,978 B | `9f1e99be692b744232350783b6b9c14653fada3c9dabb8b8ffe99e1a217ab662` | Causal Version Vectors & IBLT Reconciliation (M7.12) |
| `benchmark_m714_active_loop_closure.csv` | 5,175 B | `cda3e4a1e9af44242ec8b5f12885a57fd6678324773a85103800072f31cbed21` | Information-Aware Active Loop Rendezvous (M7.14) |
| `benchmark_m715_memory_bounded_slam.csv` | 6,048 B | `ea3186182d4b51d850340a58734c3221b0696bb5fbc1716ae0faf7774732b566` | Conservative Degree-2 Compaction & Memory (M7.15) |
| `benchmark_m716_autonomous_exploration.csv` | 4,707 B | `8dbbe76fc74523921fa893ddaa2969d88fc32e7f22ebe7dfeb1d911c6b581da2` | Topological Sector Frontiers & Connectivity (M7.16) |
| `benchmark_distributed_trials.csv` | 7,556 B | `e79a83baeb6ff2847283b65575fa26b445bec3d8bc18a4347b8644e3ee6c3cda` | 81-Trial UDP Distributed Reconciliation (Sprint M5) |
| `benchmark_distributed_trajectory_validated.csv` | 15,132 B | `1c8f3405f3a7d8a95fed4bfe12856db50c67b27ccc5ea229efae1014e1939305` | 81-Trial Distributed CORD Trajectory Optimization (M7.10) |
| `benchmark_multihop_trials_reconciled.csv` | 7,406 B | `597891b07cf68b192b096d2b52ac109f75999d47adf49b750782eeb7ab2461cd` | 45-Trial Distance-Vector Multi-Hop SLAM Routing (M7.11) |
| `build-ucrt64-clean/benchmark_m713_gauge_alignment.csv` | 4,706 B | `6345302ca7df76878b66804a37213bb01807d2c3dfbcf388916d7a4242646d84` | 45-Trial Decentralized Gauge Fixing & Cycles (M7.13) |

---

## 5. Random Seed Policy & Determinism

1. **Synthetic Swarm Worlds:** All benchmark executables instantiate random number generators using explicit 64-bit seeds:
   ```cpp
   uint64_t seed = 20261100ULL + trial_id * 31ULL;
   std::mt19937_64 rng(seed);
   ```
2. **Statistical Resampling:** The bootstrap confidence interval generator in `scratch/reproducibility_harness.py` initializes:
   ```python
   BOOTSTRAP_SEED = 20261114
   rng = random.Random(BOOTSTRAP_SEED)
   ```
3. **Determinism Guarantee:** Re-running benchmark executables and statistical harnesses on any standard x86_64 platform reproduces exact metric outputs and byte-identical results.

---

## 6. Known Platform Dependencies & Troubleshooting

1. **Windows Console Encoding:** On Windows PowerShell, UTF-8 output may require setting `$OutputEncoding = [System.Text.Encoding]::UTF8`.
2. **Process Working Set RSS API:** Memory measurement queries `GetProcessMemoryInfo` from `psapi.h` (Windows kernel). On Linux/macOS, equivalent measurement requires reading `/proc/self/statm` (resident set size) or calling `getrusage(RUSAGE_SELF, &usage)`.
3. **UDP Port Conflicts:** Real socket integration tests use local UDP ports `29999` and `8765`. If another process is bound to these ports, terminate conflicting processes via `Get-Process` or `taskkill`.
4. **Ninja Compiler Missing from Global PATH:** Always ensure `$env:Path = "C:\msys64\ucrt64\bin;C:\msys64\usr\bin;" + $env:Path` is prepended in the active shell session.

---

## 7. Limitations of Reproducibility

1. **Hardware-in-the-Loop (HIL) Absence:** AstroMesh-X is currently evaluated as a 100% software-only C++17 demonstrator. No physical sensors (cameras, IMUs, lidars) or embedded flight processors (e.g., RAD750, Snapdragon Flight) are tested.
2. **Real Regolith Perceptual Aliasing:** Synthetic benchmarks model random non-matching features with uniformly distributed coordinates, resulting in empirical zero false positives ($FP = 0$). Real unstructured planetary terrain with repetitive visual textures may generate rigid imposters requiring candidate prior gating.
3. **Relay Buffer Volatility:** Relay transit queues are volatile heap structures (`std::deque<MultihopTransitFrame>`). Process crashes discard intermediate relay packets, relying on origin retransmission rather than flash storage.

