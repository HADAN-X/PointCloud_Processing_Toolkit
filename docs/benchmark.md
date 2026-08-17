# KD-Tree Neighbor Search Benchmark

This benchmark compares the toolkit's public brute-force and KD-tree KNN and
radius-query APIs. It shows the effect of building a reusable spatial index
for repeated queries; it is not a comparison with external libraries.

## Environment

- CPU: Intel Core i7-9750H
- RAM: 15.9 GiB
- Operating system: Windows 10.0.26200
- Compiler: MSVC 19.41.34120
- CMake: 4.3.2
- Build type: Release

## Methodology

- Distribution: uniform random points in `[-100, 100]^3`
- Point seed: `20260813`
- Query seed: `20260814`
- Queries: uniform random points in the same cube
- KNN parameter: `k=16`
- Radius parameter: `radius=5.0`
- Query count: 1,000 per operation and scale
- Repetitions: 5 in one process
- Aggregation: median wall-clock time
- Warm-up: 10 KNN and 10 radius queries for each implementation
- Correctness gate: warm-up KD-tree results exactly match brute force
- Timing: construction, KNN, and radius queries measured separately
- Result handling: allocation and deterministic result ordering included
- Optimization guard: checksum consumes returned indices and result sizes

Both implementations use the same cloud and query sequence. Brute-force
figures measure the current validated public API, including per-query finite
coordinate validation and candidate allocation. KD-tree cloud validation is
performed once during construction. The speedup therefore describes the
complete toolkit APIs, not a validation-free inner-loop comparison.

## Results

| Points | Nodes | Max depth | KD build median (ms) | Brute KNN median (ms) | KD KNN median (ms) | KNN speedup | Brute radius median (ms) | KD radius median (ms) | Radius speedup |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 10,000 | 10,000 | 14 | 3.100 | 76.345 | 4.953 | 15.414× | 68.232 | 0.624 | 109.434× |
| 100,000 | 100,000 | 17 | 41.086 | 1,108.530 | 5.915 | 187.397× | 1,064.478 | 2.505 | 424.924× |
| 1,000,000 | 1,000,000 | 20 | 611.915 | 9,984.119 | 10.296 | 969.727× | 9,540.028 | 18.377 | 519.143× |

Node count equals input size at every scale. Maximum depth grows from 14 to
20, consistent with median-balanced construction. Brute-force time grows
approximately with input size, while the KD-tree prunes most subtrees for
this uniform three-dimensional distribution and these selective parameters.

Construction is a separate one-time cost. At one million points it took a
median of 611.915 ms; 1,000 KNN queries then took 10.296 ms and 1,000 radius
queries took 18.377 ms. Workloads with few queries must account for build
cost, while repeated-query pipelines can amortize it.

## Reproduction

```powershell
cmake --preset windows-msvc-release
cmake --build --preset windows-msvc-release
ctest --preset windows-msvc-release --output-on-failure

& '.\build\windows-msvc-release\Release\pointcloud_neighbor_benchmark.exe' 10000 1000 16 5 5
& '.\build\windows-msvc-release\Release\pointcloud_neighbor_benchmark.exe' 100000 1000 16 5 5
& '.\build\windows-msvc-release\Release\pointcloud_neighbor_benchmark.exe' 1000000 1000 16 5 5
```

## Limitations

- Results describe one CPU, compiler, synthetic distribution, K, and radius;
  they do not guarantee the same speedup elsewhere.
- Very large K, a radius covering much of the cloud, unfavorable
  distributions, and higher-dimensional data can weaken pruning.
- KD-tree queries retain worst-case linear time.
- Fixed benchmark order can produce different cache state; repetitions and
  medians reduce but do not eliminate this effect.
- The KD-tree is static and non-owning. The source cloud must outlive it, and
  coordinates and point ordering must not change after construction.
- Peak memory and external libraries are not measured.
