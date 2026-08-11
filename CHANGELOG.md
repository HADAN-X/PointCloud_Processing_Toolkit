# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html) for implementation milestones.

## [Unreleased]

No changes yet.

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

[Unreleased]: https://github.com/HADAN-X/PointCloud_Processing_Toolkit/compare/v0.3.0-geometry...HEAD
[0.3.0]: https://github.com/HADAN-X/PointCloud_Processing_Toolkit/releases/tag/v0.3.0-geometry
[0.2.0]: https://github.com/HADAN-X/PointCloud_Processing_Toolkit/releases/tag/v0.2.0-ply-ascii
[0.1.0]: https://github.com/HADAN-X/PointCloud_Processing_Toolkit/releases/tag/v0.1.0-skeleton
[0.0.1]: https://github.com/HADAN-X/PointCloud_Processing_Toolkit/commit/20dfc4fdcab73b0e4fc372bd920af92e3e900553
