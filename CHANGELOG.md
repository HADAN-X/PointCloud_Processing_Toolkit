# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html) for implementation milestones.

## [Unreleased]

No changes yet.

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

[Unreleased]: https://github.com/HADAN-X/PointCloud_Processing_Toolkit/compare/v0.1.0-skeleton...HEAD
[0.1.0]: https://github.com/HADAN-X/PointCloud_Processing_Toolkit/releases/tag/v0.1.0-skeleton
[0.0.1]: https://github.com/HADAN-X/PointCloud_Processing_Toolkit/commit/20dfc4fdcab73b0e4fc372bd920af92e3e900553
