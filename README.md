# PointCloud Processing Toolkit

A lightweight, testable C++ toolkit for point cloud I/O, filtering, spatial search, feature estimation, and rigid registration.

> **Current status:** repository initialization. The implementation has not started yet. Features marked as planned below are not available on the current branch.

## Why This Project

Point cloud pipelines often need a small set of dependable geometric operations without the footprint of a large framework. This project implements that focused core with explicit algorithms, stable interfaces, and reproducible validation:

- point cloud data structures and PLY file I/O;
- statistics, filtering, and rigid transformations;
- spatial indexing and nearest-neighbor search;
- normal estimation and point cloud registration;
- command-line composition for repeatable processing;
- correctness tests, benchmarks, and real-data experiments.

The goal is not to replace PCL or Open3D. The project deliberately keeps a narrow scope so that each algorithm, data-flow decision, and performance result can be inspected and reproduced.

## Planned Capabilities

| Capability | Status | Target milestone |
|---|---|---|
| Repository structure and development roadmap | Complete | v0.0.1 |
| C++17/CMake project and core point cloud types | Planned | v0.1.0 |
| ASCII PLY reader and writer | Planned | v0.2.0 |
| Point cloud statistics and rigid transformations | Planned | v0.3.0 |
| Voxel-grid downsampling | Planned | v0.4.0 |
| Brute-force KNN/radius search and radius outlier removal | Planned | v0.5.0 |
| Three-dimensional KD-tree | Planned | v0.6.0 |
| PCA-based normal estimation | Planned | v0.7.0 |
| Point-to-point ICP registration | Planned | v0.8.0 |
| Unified command-line interface | Planned | v0.9.0 |
| Benchmarks, CI, and real-data validation | Planned | v0.10.0 |
| Documented and reproducible release | Planned | v1.0.0 |

Statuses are updated only after implementation, tests, documentation, and verification are complete.

## Intended Technical Stack

- C++17
- CMake and CMake Presets
- Eigen
- CTest with a lightweight unit-test framework
- GitHub Actions

Dependencies and exact supported compiler versions will be documented when the build system is introduced in v0.1.0.

## Design Principles

1. Keep the scope small enough to finish and understand.
2. Implement important algorithms rather than wrapping existing point cloud APIs.
3. Separate point cloud storage, I/O, algorithms, and the CLI.
4. Add deterministic tests before calling a feature complete.
5. Compare optimized implementations with simple correctness baselines.
6. Record hardware, compiler settings, dataset characteristics, and methodology for performance claims.
7. Document failure cases and limitations, not only successful examples.
8. Keep `main` buildable and testable after development begins.

## Planned Architecture

```text
PointCloud_Processing_Toolkit/
├── app/                    # Command-line executable
├── cmake/                  # Reusable CMake modules
├── data/                   # Small licensed samples and data documentation
├── docs/                   # Design, algorithm, benchmark, and experiment notes
├── include/pct/            # Public C++ headers
├── src/                    # Library implementation
├── tests/                  # Unit and integration tests
├── tools/                  # Benchmark and evaluation utilities
├── CMakeLists.txt
├── CMakePresets.json
└── README.md
```

Directories will be added when their first real content is implemented. Empty architecture placeholders are intentionally not committed.

## Build and Usage

There is no buildable source code in the repository initialization milestone. Build instructions will be added with the v0.1.0 project skeleton.

Once v0.1.0 is complete, the expected workflow will follow this form:

```powershell
cmake --preset windows-debug
cmake --build --preset windows-debug
ctest --preset windows-debug --output-on-failure
```

These commands are illustrative until the corresponding presets are committed and tested.

## Development Roadmap

Development follows the milestones in the capability table above. A milestone is complete only when its implementation, automated tests, user-facing documentation, and reproducible examples are all available on the main branch.

## Planned Validation Strategy

The project will use four levels of validation:

1. **Analytical unit tests:** tiny point clouds with hand-computed expected results.
2. **Baseline comparison:** KD-tree queries compared with brute-force search, and algorithm outputs sampled against independent reference implementations.
3. **Synthetic experiments:** known transforms, controlled noise, outliers, density, and overlap.
4. **Real-data experiments:** licensed public point clouds processed with documented commands and parameters.

All future benchmark claims must identify the build type, compiler, hardware, input scale, query count, and aggregation method.

## Non-Goals

- Replacing PCL or Open3D
- Building a full SfM, MVS, SLAM, or neural-rendering system
- Creating a GUI
- Supporting many file formats before the core algorithms are validated
- Claiming high performance without reproducible benchmarks
- Accumulating a large number of shallow filters

## Documentation

Source code, public APIs, commit messages, and repository documentation use English. Algorithm notes will state assumptions, numerical tolerances, known failure cases, and validation methodology.

## License

This project is licensed under the [MIT License](LICENSE).
