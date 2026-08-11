# PointCloud Processing Toolkit

A lightweight, testable C++ toolkit for point cloud I/O, filtering, spatial search, feature estimation, and rigid registration.

> **Current status:** v0.3.0 geometry utilities. The toolkit can compute point-cloud statistics and apply validated rigid transformations while preserving optional RGB attributes.

## Why This Project

Point cloud pipelines often need a small set of dependable geometric operations without the footprint of a large framework. This project implements that focused core with explicit algorithms, stable interfaces, and reproducible validation:

- point cloud data structures and PLY file I/O;
- statistics, filtering, and rigid transformations;
- spatial indexing and nearest-neighbor search;
- normal estimation and point cloud registration;
- command-line composition for repeatable processing;
- correctness tests, benchmarks, and real-data experiments.

The goal is not to replace PCL or Open3D. The project deliberately keeps a narrow scope so that each algorithm, data-flow decision, and performance result can be inspected and reproduced.

## Capabilities

| Capability                                               | Status   | Target milestone |
| -------------------------------------------------------- | -------- | ---------------- |
| Repository structure and development roadmap             | Complete | v0.0.1           |
| C++17/CMake project and core point cloud types           | Complete | v0.1.0           |
| ASCII PLY reader and writer                              | Complete | v0.2.0           |
| Point cloud statistics and rigid transformations         | Complete | v0.3.0           |
| Voxel-grid downsampling                                  | Planned  | v0.4.0           |
| Brute-force KNN/radius search and radius outlier removal | Planned  | v0.5.0           |
| Three-dimensional KD-tree                                | Planned  | v0.6.0           |
| PCA-based normal estimation                              | Planned  | v0.7.0           |
| Point-to-point ICP registration                          | Planned  | v0.8.0           |
| Unified command-line interface                           | Planned  | v0.9.0           |
| Benchmarks, CI, and real-data validation                 | Planned  | v0.10.0          |
| Documented and reproducible release                      | Planned  | v1.0.0           |

Statuses are updated only after implementation, tests, documentation, and verification are complete.

## Requirements

- CMake 3.25 or newer
- Git, required by CMake only when dependencies are first fetched
- A C++17 compiler
- Network access during the first configure step

The current Windows presets use the Visual Studio 2022 x64 generator. Eigen 3.4.1 and GoogleTest 1.17.0 are fetched automatically and pinned to release tags.

## Build and Test

Configure, build, and test the Debug preset from the repository root:

```powershell
cmake --preset windows-msvc-debug
cmake --build --preset windows-msvc-debug
ctest --preset windows-msvc-debug --output-on-failure
```

For Release:

```powershell
cmake --preset windows-msvc-release
cmake --build --preset windows-msvc-release
ctest --preset windows-msvc-release --output-on-failure
```

The first configure downloads pinned Eigen and GoogleTest sources into the ignored `build/` directory. Subsequent builds reuse the local dependency cache.

The v0.3.0 milestone has been verified with:

- Visual Studio Community 2022;
- MSVC 19.41.34120;
- Windows SDK 10.0.22621.0;
- CMake 4.3.2;
- 44 passing tests in both Debug and Release configurations.

## Command-Line Usage

The CLI exposes help, version, PLY inspection and statistics, ASCII PLY rewrite, and rigid-transform operations:

```powershell
& '.\build\windows-msvc-debug\Debug\pointcloud_tool.exe'
& '.\build\windows-msvc-debug\Debug\pointcloud_tool.exe' --help
& '.\build\windows-msvc-debug\Debug\pointcloud_tool.exe' --version
& '.\build\windows-msvc-debug\Debug\pointcloud_tool.exe' info input.ply
& '.\build\windows-msvc-debug\Debug\pointcloud_tool.exe' stats input.ply
& '.\build\windows-msvc-debug\Debug\pointcloud_tool.exe' convert input.ply output.ply
& '.\build\windows-msvc-debug\Debug\pointcloud_tool.exe' transform input.ply output.ply --translate 1 2 3
& '.\build\windows-msvc-debug\Debug\pointcloud_tool.exe' transform input.ply output.ply --rotate-z 90
& '.\build\windows-msvc-debug\Debug\pointcloud_tool.exe' transform input.ply output.ply --matrix 1 0 0 1 0 1 0 2 0 0 1 3 0 0 0 1
```

The library transformation API accepts angles in radians. The CLI accepts rotation angles in degrees and matrix values in row-major order. Running without arguments displays help and exits successfully. PLY, numeric, or geometry failures return exit code 1. Unknown commands, missing arguments, or extra arguments return exit code 2.

Example output from `info`:

```text
points: 2
has_color: true
```

Example output from `stats` for two points at `(1, 2, 3)` and `(-1, -2, -3)`:

```text
points: 2
centroid: 0 0 0
bbox_min: -1 -2 -3
bbox_max: 1 2 3
```

## Current Structure

```text
PointCloud_Processing_Toolkit/
├── app/
│   └── main.cpp
├── include/pct/
│   ├── core/
│   │   ├── point.hpp
│   │   └── point_cloud.hpp
│   ├── geometry/
│   │   ├── statistics.hpp
│   │   └── transform.hpp
│   └── io/
│       └── ply_io.hpp
├── src/
│   ├── core/
│   │   └── point_cloud.cpp
│   ├── geometry/
│   │   ├── statistics.cpp
│   │   └── transform.cpp
│   └── io/
│       └── ply_io.cpp
├── tests/
│   ├── fixtures/
│   │   ├── ascii_xyz.ply
│   │   ├── ascii_xyz_rgb.ply
│   │   ├── ascii_reordered_unknown.ply
│   │   └── malformed_*.ply
│   ├── unit/
│   │   ├── test_point_cloud.cpp
│   │   ├── test_ply_io.cpp
│   │   ├── test_statistics.cpp
│   │   └── test_transform.cpp
│   └── CMakeLists.txt
├── CMakeLists.txt
├── CMakePresets.json
└── README.md
```

Directories for filters, search, features, registration, benchmarks, and data will be introduced when their first real implementation is added. Empty architecture placeholders are intentionally not committed.

## Core API

The `pct::Point` type currently stores:

- a three-dimensional `Eigen::Vector3f` position;
- an optional 8-bit RGB color.

The `pct::PointCloud` type owns an aligned point container and provides:

- `empty()` and `size()`;
- checked access through `at()`;
- unchecked access through `operator[]`;
- copy and move insertion through `pushBack()`;
- `clear()` and direct container access.

File I/O and processing algorithms are intentionally kept outside `PointCloud` so that storage, serialization, and algorithms remain separate responsibilities.

The `pct::io` API currently provides:

- `readPly()` for validated ASCII PLY 1.0 input;
- `writePlyAscii()` for XYZ or XYZRGB output;
- `PlyError` for file access, format, schema, and numeric validation failures.

The `pct::geometry` API currently provides:

- single-pass point count, centroid, and axis-aligned bounding-box statistics;
- explicit undefined centroid and bounding-box results for empty point clouds;
- rigid-transform construction from translation and X/Y/Z axis rotations;
- validation of finite homogeneous matrices, rotation orthogonality, and determinant;
- point-cloud transformation with optional RGB attributes preserved.

## Validation

The v0.3.0 test suite covers:

- CLI startup without arguments;
- CLI help and version commands;
- default point state;
- optional RGB color storage;
- empty point clouds;
- copy/move insertion and access;
- checked out-of-range access;
- point cloud clearing.
- ASCII XYZ and XYZRGB input;
- reordered and unknown scalar vertex properties;
- XYZ and XYZRGB read/write/read round trips;
- missing files, binary format, missing coordinates, and point-count mismatches;
- non-finite coordinates and incomplete RGB schemas;
- partially colored point clouds and prevention of partial output files;
- the CLI `info` command against a real fixture.
- empty, single-point, and analytical multi-point statistics;
- centroid and axis-aligned bounding-box results;
- identity, translation, and X/Y/Z axis rotations;
- transform composition order and inverse recovery;
- preservation of optional RGB attributes during transformation;
- rejection of non-finite coordinates, non-finite matrices, scaling, and invalid homogeneous rows;
- the CLI `stats` and `transform` commands with translation, rotation, and row-major matrix input.

Future algorithms will use four levels of validation:

1. **Analytical unit tests:** tiny point clouds with hand-computed expected results.
2. **Baseline comparison:** optimized methods compared with simple correctness baselines.
3. **Synthetic experiments:** known transforms, controlled noise, outliers, density, and overlap.
4. **Real-data experiments:** licensed public point clouds processed with documented commands and parameters.

All benchmark claims will identify the build type, compiler, hardware, input scale, query count, and aggregation method.

## PLY Support and Limitations

The v0.2.0 milestone intentionally supports a narrow, explicit PLY subset:

- ASCII PLY format 1.0;
- required `x`, `y`, and `z` vertex properties;
- optional `red`, `green`, and `blue` properties;
- arbitrary ordering of supported vertex properties;
- ignored unknown scalar vertex properties;
- comments and `obj_info` header entries.

Current limitations:

- binary little-endian and big-endian PLY are rejected;
- face, edge, and other non-vertex data are not loaded or preserved;
- vertex list properties are not supported;
- unknown vertex attributes are not preserved when rewriting a file;
- coordinates are stored as single-precision floats;
- all points must either have RGB color or have no color when writing;
- this milestone does not target optimized loading of very large ASCII files.
- geometry coordinates and transformation matrices use single-precision floats;
- transformations are restricted to finite rigid matrices; scaling, shear, reflection, and projective transforms are rejected;
- axis-aligned bounding boxes depend on the current coordinate frame and must be recomputed after rotation;
- the geometry functions process point clouds in memory and do not yet provide parallel or streaming execution.

## Design Principles

1. Keep the scope small enough to finish and understand.
2. Implement important algorithms rather than wrapping existing point cloud APIs.
3. Separate point cloud storage, I/O, algorithms, and the CLI.
4. Add deterministic tests before calling a feature complete.
5. Compare optimized implementations with simple correctness baselines.
6. Record hardware, compiler settings, dataset characteristics, and methodology for performance claims.
7. Document failure cases and limitations, not only successful examples.
8. Keep the default branch buildable and testable.

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
