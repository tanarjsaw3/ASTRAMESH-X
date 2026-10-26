# AstroMesh-X — Agent Operational Guidelines

This document outlines mandatory guidelines for AI agents and automated contributors assisting on the AstroMesh-X codebase.

---

## 1. Core Operating Principles

1. **Step-by-Step Guidance:** 
   - Work collaboratively with the user.
   - Explain goals in clear, beginner-friendly terms.
   - Propose a plan before introducing significant code changes.
   - Implement only the current assigned milestone.
   - Require explicit user approval before advancing to subsequent major milestones.

2. **Non-Destructive Git Policy:**
   - Never run destructive Git commands (`git reset --hard`, `git clean -fd`, `git push --force`, etc.).
   - Never delete uncommitted user modifications.
   - Never commit or push without showing proposed diffs/status and obtaining explicit user confirmation.
   - Never publish or hardcode credentials, tokens, or sensitive user data.

3. **Verifiable Execution:**
   - Only record build, test, or execution results in `docs/BUILD_LOG.md` if the command was physically run and its exact output was observed.
   - Never assume or hallucinate successful test results.

---

## 2. Scientific & Engineering Rules

1. **Set Reconciliation Distinction:**
   - An Invertible Bloom Lookup Table (IBLT) reconciles set *differences* (symmetric differences), subject to decoding capacity. Do not describe it as directly computing set intersections.
2. **Targets vs. Results:**
   - Targets (e.g., 1 KB message payload, >99% precision/recall, sub-0.5 m ATE, sub-30 s synchronization) are research goals to be tested, never reported as existing achievements until documented by reproducible benchmark logs.
3. **Prior Art & Novelty:**
   - CORD (Continuous Riemannian Dynamics) and IBLT are existing published methodologies. Implement CORD faithfully based on Tian et al. Do not replace it with simplified heuristics while labeling it CORD.
4. **Fair Baseline Maintenance:**
   - Preserve AstroMesh v1 (Counting Bloom Filter discovery, Voronoi partitioning, baseline DSU merge) to ensure rigorous and fair comparative benchmarking against AstroMesh-X.
5. **No Speculative Code:**
   - Do not generate fake algorithms, empty placeholder classes, or stub APIs simply to give the appearance of progress. Write test-driven, verifiable code incrementally.

---

## 3. Persistent Memory Protocols

Whenever modifying the project, maintain the following files in `docs/`:
- **`docs/PROJECT_STATE.md`**: Update after every milestone or substantial change. Must state current milestone, verified components, unverified components, known errors, environment details, git state, and immediate next action.
- **`docs/BUILD_LOG.md`**: Append executed shell commands, flags, timestamps, and real output/exit codes.
- **`docs/DECISIONS.md`**: Log any architectural or technical choice (ADR format) with context, options considered, decision rationale, and trade-offs.
- **`docs/ROADMAP.md`**: Reference phases and milestones. Check off completion criteria only when verified by tests.
- **`docs/TEST_PLAN.md`**: Define test specifications before writing implementation code.
- **`docs/EXPERIMENT_LOG.md`**: Record experimental hypotheses, test matrices, simulator parameters, and empirical metrics.
- **`docs/REFERENCES.md`**: Record academic citations, flagging any unverified bibliographic details.
