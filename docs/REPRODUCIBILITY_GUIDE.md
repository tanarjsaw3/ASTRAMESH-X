# AstroMesh-X — Scientific Reproducibility & Benchmark Execution Guide

**Document Version:** 1.0.0  
**Verification Date:** 2026-11-14  
**Target Environment:** Windows 10/11, MSYS2 UCRT64 GCC 16.2.0, CMake 4.4.4, Ninja 1.13.2, Python 3.10+

---

## 1. Environment Setup

### 1.1 Toolchain Prerequisites
Verify your environment in PowerShell:
```powershell
$env:Path = "C:\msys64\ucrt64\bin;C:\msys64\usr\bin;" + $env:Path
gcc --version     # Expected: GCC 16.2.0 (x86_64-w64-mingw32)
ninja --version   # Expected: 1.13.2 or compatible
cmake --version   # Expected: 4.4.4 or compatible
python --version  # Expected: Python 3.10+
```

### 1.2 Build Configuration
From the repository root (`ASTRAMESH-X`):
```powershell
# Configure with Ninja in release mode (preserving assertions via -UNDEBUG)
cmake -B build-ucrt64-clean -G Ninja -DCMAKE_BUILD_TYPE=Release
```

---

## 2. Running the Full Verification & Regression Test Suites

### 2.1 CTest Regression (All 65 Test Targets)
Execute the complete CTest suite:
```powershell
$env:Path = "C:\msys64\ucrt64\bin;C:\msys64\usr\bin;" + $env:Path
ctest --test-dir build-ucrt64-clean --output-on-failure
```
*Expected Result:* 100% tests passed (65/65 passed in $< 10$ seconds).

### 2.2 Acceptance Test Suite (13 System Verification Checks)
Execute the Python acceptance verification harness:
```powershell
python scratch/test_acceptance.py
```
*Expected Result:* 13 of 13 acceptance checks PASS (100% accepted).

---

## 3. Running the Unified Reproducibility Harness

Execute the automated reproducibility script:
```powershell
python scratch/reproducibility_harness.py
```

*Generated Outputs:*
- `benchmark_m717_validation.csv`: Empirical performance overview and baseline comparisons.
- `benchmark_m717_statistical_summary.csv`: Sample sizes, means, 95% bootstrap confidence intervals, Cohen's $d$, and Wilcoxon signed-rank $p$-values.
- `benchmark_m717_evidence_matrix.csv`: Categorization of all claims into Measured, Derived, Synthetic, Theoretical, Qualified, and Unverified.

---

## 4. Re-Executing Milestone Benchmark Campaigns

All benchmark executables are built via Ninja and write their datasets deterministically.

### 4.1 Milestone 7.12: Partition Recovery Benchmark (45 Trials)
```powershell
ninja -C build-ucrt64-clean benchmark_m712_partition_recovery
.\build-ucrt64-clean\bin\benchmark_m712_partition_recovery.exe
```
*Output File:* `benchmark_m712_partition_recovery.csv`

### 4.2 Milestone 7.13: Decentralized Gauge Fixing Benchmark (45 Trials)
```powershell
ninja -C build-ucrt64-clean benchmark_m713_gauge_alignment
.\build-ucrt64-clean\bin\benchmark_m713_gauge_alignment.exe
```
*Output File:* `benchmark_m713_gauge_alignment.csv`

### 4.3 Milestone 7.14: Active Loop Closure Prioritization (45 Trials)
```powershell
ninja -C build-ucrt64-clean benchmark_m714_active_loop_closure
.\build-ucrt64-clean\bin\benchmark_m714_active_loop_closure.exe
```
*Output File:* `benchmark_m714_active_loop_closure.csv`

### 4.4 Milestone 7.15: Memory-Bounded SLAM Benchmark (45 Trials)
```powershell
ninja -C build-ucrt64-clean benchmark_m715_memory_bounded_slam
.\build-ucrt64-clean\bin\benchmark_m715_memory_bounded_slam.exe
```
*Output File:* `benchmark_m715_memory_bounded_slam.csv`

### 4.5 Milestone 7.16: Autonomous Multi-Rover Exploration (45 Trials)
```powershell
ninja -C build-ucrt64-clean benchmark_m716_autonomous_exploration
.\build-ucrt64-clean\bin\benchmark_m716_autonomous_exploration.exe
```
*Output File:* `benchmark_m716_autonomous_exploration.csv`

---

## 5. Audit Invariants & Integrity Checklist

When reviewing or replicating experimental data:
- [x] **Single Global Alignment:** Ensure ATE RMSE uses a single global rigid $\mathrm{SE}(2)$ transformation across the combined multi-rover map.
- [x] **Total Wire Accounting:** Total wire bytes must equal data wire bytes plus ACK wire bytes.
- [x] **No Hardcoded Values:** Datasets must be produced exclusively by running compiled benchmark executables.
- [x] **No Hardware Extrapolation:** Do not extrapolate software simulation findings to physical flight readiness without hardware testing.

