# Point-to-Point ICP Experiments

This document evaluates the toolkit's point-to-point ICP implementation on
fixed synthetic data with known transforms. The objective is to make accuracy,
convergence, robustness, and runtime observable under controlled changes. It
is not a comparison with external libraries or a claim about real-sensor
performance.

## Environment

- CPU: Intel Core i7-9750H
- RAM: 15.9 GiB
- Operating system: Windows 10.0.26200
- Compiler: MSVC 19.41.34120
- CMake: 4.3.2
- Eigen: 3.4.1
- Build type: Release

## Method

The experiment generates source points uniformly in `[-2, 2]^3` and applies a
known five-degree rotation plus translation `(0.08, -0.04, 0.03)` to create
the target. Each scenario uses deterministic seeds derived from `20260817`.

Registration uses:

- target-side KD-tree nearest-neighbor correspondences;
- one-way, non-unique source-to-target matching;
- point-to-point SVD rigid updates with reflection correction;
- maximum correspondence distance `0.35`;
- maximum 80 iterations;
- translation and rotation update thresholds of `1e-7`;
- RMSE-change threshold `1e-8`;
- double-precision optimization and transform accumulation;
- 2,000 source inliers and five independent repetitions per scenario.

The reported time covers the complete `alignPointToPoint()` call, including
target KD-tree construction, correspondence queries, SVD updates, and
diagnostic-history creation. Values are arithmetic means over the five runs.
The benchmark does not warm up or isolate memory allocation, and process-level
overhead is not reported.

Accuracy is measured from the error transform between the estimated and known
source-to-target transforms:

- rotation error is the angle of the relative rotation in degrees;
- translation error is the Euclidean norm of the relative translation;
- RMSE is the final residual over the correspondences accepted in the last
  completed iteration.

`success` counts runs whose termination is either transform-delta convergence
or RMSE-change convergence. It does not assert that the global transform is
correct.

## Scenarios

| Scenario | Controlled change |
| --- | --- |
| `exact_identity_init` | An inverse perturbation makes the starting pose approximately identity for the known small transform. |
| `noise_2mm` | Independent Gaussian noise with sigma `0.002` is added to every target coordinate. |
| `outliers_20pct` | Source receives 20% additional points in `[8, 12]^3`, well outside the distance gate. |
| `overlap_70pct` | Target retains only the first 70% of transformed source inliers. |
| `initial_error_5deg` | The initial pose has a five-degree rotational and `0.05` translational perturbation. |
| `initial_error_15deg` | The initial pose has a 15-degree rotational and `0.15` translational perturbation. |
| `perfect_initial` | The known transform is supplied as the initial pose. |

The coordinate unit is synthetic. The `noise_2mm` scenario name assumes the
coordinates are interpreted as metres; it does not represent measurements
from a particular sensor.

## Results

Command:

```powershell
& '.\build\windows-msvc-release\Release\pointcloud_icp_benchmark.exe' 2000 5
```

| Scenario | Success | Mean time (ms) | Mean iterations | Mean final correspondences | Mean final RMSE | Mean rotation error (deg) | Mean translation error |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `exact_identity_init` | 5/5 | 5.000860 | 5.0 | 2,000.0 | 0.000000 | 0.000000 | 0.000000 |
| `noise_2mm` | 5/5 | 4.683300 | 4.6 | 2,000.0 | 0.003475 | 0.000000 | 0.000072 |
| `outliers_20pct` | 5/5 | 38.489780 | 4.6 | 2,000.0 | 0.000000 | 0.000000 | 0.000000 |
| `overlap_70pct` | 5/5 | 8.338740 | 8.6 | 1,980.6 | 0.113836 | 0.163280 | 0.003259 |
| `initial_error_5deg` | 5/5 | 4.059140 | 4.0 | 2,000.0 | 0.000000 | 0.000000 | 0.000000 |
| `initial_error_15deg` | 5/5 | 19.694440 | 15.0 | 2,000.0 | 0.000000 | 0.000000 | 0.000000 |
| `perfect_initial` | 5/5 | 1.583140 | 1.0 | 2,000.0 | 0.000000 | 0.000000 | 0.000000 |

All 114 automated tests also passed in both MSVC Debug and Release builds.
The ten ICP-specific tests cover known transforms, initial-pose direction,
positive-determinant output, iteration history, invalid input, insufficient
correspondences, collinear geometry, and iteration-limit termination.

## Interpretation

The exact full-overlap cases recover the known transform to the displayed
precision. With target noise sigma `0.002`, final RMSE rises to `0.003475`,
close to the expected three-dimensional noise magnitude, while translation
error remains `0.000072`.

The 20% distant source outliers are rejected by the correspondence-distance
gate: the final accepted count remains 2,000 rather than 2,400, and pose error
remains zero to the displayed precision. Runtime nevertheless increases from
about 5.0 ms to 38.5 ms because every outlier still requires a KD-tree query;
distance filtering improves estimation robustness but does not remove query
cost.

At 70% target overlap, numerical convergence remains 5/5, but final RMSE,
rotation error, and translation error all increase. The final correspondence
count exceeds the 1,400 true overlapping pairs because non-overlap source
points can still find target neighbors within the distance gate. This is a
concrete example of why a convergence flag and low iteration delta are not
proof of globally correct registration.

Increasing the initial rotation error from five to 15 degrees increases the
mean iteration count from 4 to 15 and the mean runtime from about 4.1 ms to
19.7 ms. A perfect initial transform stops after one iteration. These results
are consistent with ICP's local nature and its dependence on initialization.

## Reproduction

From the repository root:

```powershell
cmake --preset windows-msvc-debug
cmake --build --preset windows-msvc-debug --parallel
ctest --preset windows-msvc-debug --output-on-failure

cmake --preset windows-msvc-release
cmake --build --preset windows-msvc-release --parallel
ctest --preset windows-msvc-release --output-on-failure

& '.\build\windows-msvc-release\Release\pointcloud_icp_benchmark.exe' 2000 5
```

The executable also accepts smaller values for a quick smoke run:

```powershell
& '.\build\windows-msvc-release\Release\pointcloud_icp_benchmark.exe' 200 1
```

Runtime will vary with CPU, compiler version, background load, and power
settings. Fixed seeds make the geometric inputs and accuracy metrics
reproducible; they do not make wall-clock time identical.

## Limitations

- The data is synthetic, uniformly random, and does not model real sensor
  noise, surfaces, occlusion, motion distortion, or repeated structures.
- Point-to-point ICP is locally convergent and requires an adequate initial
  source-to-target pose.
- Correspondences are one-way nearest neighbors and are neither reciprocal
  nor unique.
- The implementation does not use a robust loss, trimmed correspondence set,
  RANSAC initialization, or multiscale processing.
- The correspondence-distance threshold is scale-dependent and must be tuned
  for the input units, noise, overlap, and initial uncertainty.
- A numerical convergence state does not certify the globally correct pose.
- The implementation is single-threaded and builds a new target KD-tree for
  every registration call.
- The timing sample contains only five repetitions per scenario and is not a
  general performance characterization.

