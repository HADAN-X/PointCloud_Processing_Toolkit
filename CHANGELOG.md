# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html) for implementation milestones.

## [Unreleased]

No changes yet.

## [0.7.0] - 2026-08-17

### Added

- KNN- and radius-based PCA normal-estimation APIs backed by the three-dimensional KD-tree.
- Per-point unit normal, curvature, actual neighbor count, and explicit valid, insufficient-neighbor, or degenerate-neighborhood status.
- Double-precision local centroid and covariance accumulation with Eigen self-adjoint eigendecomposition.
- Rank-aware rejection of collinear and coincident neighborhoods without non-finite output.
- Deterministic dominant-component normal orientation and optional orientation toward a finite viewpoint.
- Ten analytical, degenerate, spherical, noisy, and invalid-input tests.
- Fixed-seed Release experiment for normal accuracy, validity, curvature, and complete-pass runtime across four noise levels.

### Validation

- MSVC Debug build and 104/104 tests.
- MSVC Release build and 104/104 tests.
- Exact planar normals and near-zero curvature passed for KNN and radius neighborhoods, including boundary points.
- Synthetic sphere normals achieved an absolute radial dot product above 0.97.
- Insufficient, collinear, and coincident neighborhoods returned explicit finite invalid results.
- All 10,000 points remained valid at every recorded noise level.

### Experiment

- The fixed `100 x 100` plane used `k=20`, seed `20260817`, one warm-up, and five Release repetitions per noise level.
- Mean angular error increased from 0 degrees without noise to 0.576767, 3.017098, and 6.346142 degrees at Z-noise sigmas 0.001, 0.005, and 0.010.
- P95 angular error reached 12.360669 degrees and mean curvature reached 0.052496 at sigma 0.010.
- Reported total medians include KD-tree construction and all 10,000 normal estimates; timing variation is not interpreted as a noise-dependent speed trend.

### Limitations

- Normal quality depends on neighborhood scale, sampling density, noise, boundaries, and mixed-surface neighborhoods.
- PCA determines a local normal axis but does not provide global orientation propagation.
- Each public estimation call rebuilds the KD-tree and executes on one CPU thread.
- Normal and curvature results are not yet serialized as PLY vertex properties.

## [0.6.0] - 2026-08-17

### Added

- Median-balanced three-dimensional KD-tree with deterministic exact KNN and radius queries.
- Cyclic X/Y/Z splitting, recursive near/far traversal, and plane-distance pruning.
- Node-count, maximum-depth, and construction-time statistics.
- Randomized and degenerate-input comparisons against the brute-force baseline.
- Reproducible Release benchmark for construction, KNN, and radius performance at 10,000, 100,000, and 1,000,000 points.
- Public benchmark methodology, reproduction commands, results, interpretation, and limitations.

### Changed

- Moved the shared neighbor result and ordering into a search-independent header used by both implementations.

### Validation

- MSVC Debug build and 94/94 tests.
- MSVC Release build and 94/94 tests.
- Three hundred randomized KNN/radius query pairs matched brute force item by item.
- Empty, single-point, sorted, duplicate, collinear, coplanar, boundary, and excluded-index cases passed.
- Five-run Release medians recorded with 1,000 KNN and radius queries per scale.

### Performance

- At 1,000,000 uniformly distributed points, median construction took 611.915 ms.
- For 1,000 queries at that scale, KD-tree KNN took 10.296 ms versus 9,984.119 ms for brute force; radius search took 18.377 ms versus 9,540.028 ms.
- Results describe the complete validated APIs and the documented synthetic workload, not universal speedup guarantees.

### Limitations

- The KD-tree is a static non-owning index; its source cloud must outlive it and remain unchanged.
- Worst-case query time remains linear, and large K, large radii, or unfavorable distributions can weaken pruning.
- Cyclic splitting does not adapt to local point variance.

## [0.5.0] - 2026-08-13

### Added

- Deterministic brute-force KNN and radius search with squared-distance results and optional point-index exclusion.
- Radius outlier removal with explicit self-exclusion and preservation of point order and optional RGB attributes.
- `radius-filter --radius --min-neighbors` command-line operation with removal, retention, parameter, and processing-time reporting.
- Analytical coverage for line, regular-grid, duplicate, empty, boundary, isolated-point, and invalid-input cases.
- Independent full-sort baseline comparison for KNN correctness.

### Validation

- MSVC Debug build and 84/84 tests.
- MSVC Release build and 84/84 tests.
- End-to-end filtering retained five clustered colored points and removed one isolated point with `radius=1.5` and `min_neighbors=2`.
- Parameter-sensitivity checks demonstrated complete removal with `radius=0.5, min_neighbors=1` and with `radius=100, min_neighbors=6` on the six-point fixture.

### Limitations

- Brute-force queries scan every point and are intended as a correctness baseline for the forthcoming spatial index.
- Ordered radius results add result-sorting cost, and repeated brute-force queries make radius outlier removal unsuitable for large point clouds.
- Filter parameters depend on coordinate scale and sampling density and can remove valid sparse structures when poorly chosen.

## [0.4.0] - 2026-08-12

### Added

- Deterministic hash-based voxel-grid downsampling with signed 64-bit voxel indices.
- Position-centroid and RGB-mean aggregation with explicit attribute validation.
- `voxel --leaf-size` command-line operation with point-count, retention, and processing-time metrics.
- Reproducible synthetic benchmark with runtime and peak-resident-memory reporting.
- Analytical coverage for negative coordinates, voxel boundaries, input order, and index overflow.
- Before-and-after visualization with documented point counts and leaf size.

### Validation

- MSVC Debug build and 60/60 tests.
- MSVC Release build and 60/60 tests.
- Five-run Release medians recorded at 10,000, 100,000, and 1,000,000 synthetic points for leaf sizes 0.5 and 5.0.

### Limitations

- Voxel output is sorted for determinism, adding `O(V log V)` work after expected `O(N)` hash aggregation.
- The voxel grid is anchored at the coordinate origin and currently runs in memory on one thread.
- Peak-resident-memory results include the complete benchmark process.

## [0.3.0] - 2026-08-11

### Added

- Single-pass point count, centroid, and axis-aligned bounding-box statistics.
- Explicit undefined statistics for empty point clouds and finite-coordinate validation.
- Validated rigid transformations with translation and X/Y/Z axis rotations.
- `stats` and `transform` command-line operations, including row-major matrix input.
- Analytical tests for statistics, transform composition, inverse recovery, and color preservation.

### Validation

- MSVC Debug build and 44/44 tests.
- MSVC Release build and 44/44 tests.

### Limitations

- Transformations are restricted to finite rigid matrices; scaling, shear, reflection, and projective transforms are rejected.
- Geometry coordinates and transformation matrices use single-precision floats.

## [0.2.0] - 2026-08-11

### Added

- ASCII PLY 1.0 reader and writer for XYZ and optional RGB point clouds.
- Property-order-independent vertex parsing with ignored unknown scalar attributes.
- `info` and `convert` command-line operations.
- Valid, malformed, and round-trip PLY fixtures and automated tests.

### Validation

- MSVC Debug build and 23/23 tests.
- MSVC Release build and 23/23 tests.

### Limitations

- Binary PLY and non-vertex data are not supported in this milestone.

## [0.1.0] - 2026-08-05

### Added

- C++17 and CMake project skeleton with Visual Studio 2022 x64 Debug and Release presets.
- Eigen 3.4.1 dependency fetched through CMake `FetchContent`.
- `Point`, optional `ColorRgb8`, and aligned `PointCloud` core data structures.
- Checked and unchecked point access, copy/move insertion, size queries, and clearing.
- Initial command-line help and version commands with explicit success and error exit codes.
- GoogleTest 1.17.0 and CTest integration with nine automated tests.

### Changed

- Running the CLI without arguments now displays help and exits successfully.

### Verified

- MSVC 19.41 Debug build and all nine tests.
- MSVC 19.41 Release build and all nine tests.

## [0.0.1] - 2026-08-05

### Added

- Initial repository documentation and technical scope.
- Milestone-based feature status and development roadmap.
- Repository ignore rules and cross-platform line-ending policy.
- MIT license.

[Unreleased]: https://github.com/HADAN-X/PointCloud_Processing_Toolkit/compare/v0.7.0-normals...HEAD
[0.7.0]: https://github.com/HADAN-X/PointCloud_Processing_Toolkit/releases/tag/v0.7.0-normals
[0.6.0]: https://github.com/HADAN-X/PointCloud_Processing_Toolkit/releases/tag/v0.6.0-kdtree
[0.5.0]: https://github.com/HADAN-X/PointCloud_Processing_Toolkit/releases/tag/v0.5.0-neighborhood
[0.4.0]: https://github.com/HADAN-X/PointCloud_Processing_Toolkit/releases/tag/v0.4.0-voxel-grid
[0.3.0]: https://github.com/HADAN-X/PointCloud_Processing_Toolkit/releases/tag/v0.3.0-geometry
[0.2.0]: https://github.com/HADAN-X/PointCloud_Processing_Toolkit/releases/tag/v0.2.0-ply-ascii
[0.1.0]: https://github.com/HADAN-X/PointCloud_Processing_Toolkit/releases/tag/v0.1.0-skeleton
[0.0.1]: https://github.com/HADAN-X/PointCloud_Processing_Toolkit/commit/20dfc4fdcab73b0e4fc372bd920af92e3e900553
