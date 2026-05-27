# FLiP.jl - Forest Lidar Processing

[![CI](https://github.com/xiangtaoxu/FLiP.jl/workflows/CI/badge.svg)](https://github.com/xiangtaoxu/FLiP.jl/actions)
[![Coverage](https://codecov.io/gh/xiangtaoxu/FLiP.jl/branch/main/graph/badge.svg)](https://codecov.io/gh/xiangtaoxu/FLiP.jl)

A high-performance Julia package for processing 3D point cloud data from LiDAR and other sensors.

## Features

- **Format Support**: Read and write LAS, LAZ, and E57 files with auto-dispatch via `read_pc`/`write_pc`
- **Subsampling**: Minimum-distance subsampling with spatial grid hashing
- **Noise Filtering**: Statistical outlier removal and voxel connected-component filtering
- **Ground Segmentation**: Voxel CC pre-filter + grid z-min + upward conic filter, with above-ground height (AGH) computed via IDW interpolation
- **Tree Segmentation**: Non-Branching Segments (NBS) extraction and assembly into per-tree clusters, with orphan rescue for ground-disconnected branches
- **QSM (Quantitative Structural Modeling)**: per-branch slicing, 2D periodic surface smoothing for cross-section fitting, frustum geometry → DBH / volume / surface area per tree
- **Graph Algorithms**: Radius graphs, connected components, quotient graphs, and shortest-path slicing
- **Mesh Operations**: Delaunay triangulation and cloud-to-mesh distance computation
- **Transformations**: Translation, rotation, scaling, arbitrary affine transforms, and bounding box crop
- **Pipeline**: TOML-configurable end-to-end processing via `run_pipeline`

## Installation

```julia
using Pkg
Pkg.add("FLiP")
```

Or for development:

```julia
using Pkg
Pkg.develop(url="https://github.com/xiangtaoxu/FLiP.jl")
```

## Quick Start

Filter primitives return `Vector{Int}` indices over an N×3 coordinate matrix —
index back into the `PointCloud` to materialize a filtered subset:

```julia
using FLiP

# Read a point cloud (auto-dispatch by extension)
pc = read_pc("input.laz")

# Subsample by minimum point spacing (5 cm)
indices = distance_subsample(coordinates(pc), 0.05)
pc_sub = pc[indices]

# Remove statistical outliers (returns inlier indices)
indices = statistical_filter(coordinates(pc_sub), 6, 1.0)
pc_clean = pc_sub[indices]

# Apply a transformation and save
pc_out = translate(pc_clean, 100.0, 200.0, 0.0)
write_pc("output.laz", pc_out)

# — or — run the full end-to-end pipeline from a TOML config:
run_pipeline("flip_config.toml")
```

## Core Functionality

### I/O Operations

```julia
# LAS/LAZ format
pc = read_las("file.las")
pc = read_laz("file.laz")
write_las("output.las", pc)
write_laz("output.laz", pc)

# E57 format
pc = read_e57("scan.e57")
write_e57("output.e57", pc)

# Auto-dispatch by file extension
pc = read_pc("file.laz")
write_pc("output.e57", pc)

# Read metadata without loading point data
meta = read_pc_metadata("file.laz")   # or read_las_metadata / read_e57_metadata
```

### Subsampling

```julia
# Minimum distance subsampling — returns Vector{Int} indices over the coord matrix
indices = distance_subsample(coordinates(pc), 0.03)
pc_sub = pc[indices]
```

### Filtering

All filter primitives take an N×3 coordinate matrix and return inlier indices
as `Vector{Int}`; index back into the `PointCloud` to materialize a subset.

```julia
coords = coordinates(pc)

# Statistical outlier removal (k=6, n_sigma=1.0)
indices = statistical_filter(coords, 6, 1.0)
pc_clean = pc[indices]

# Voxel connected-component filter — drop isolated voxel clusters
indices = voxel_connected_component_filter(coords, 0.1; min_cc_size=10)
pc_clean = pc[indices]

# Composing the ground filter chain (grid z-min → upward conic)
seed_idx     = grid_zmin_filter(coords, 1.0)
ground_local = upward_conic_filter(coords[seed_idx, :], 45.0)
ground_idx   = seed_idx[ground_local]
nonground_idx = sort(setdiff(1:npoints(pc), ground_idx))
```

### Transformations

```julia
# Translation
pc_translated = translate(pc, 10.0, 20.0, 5.0)

# Rotation (axis-angle or symbol shorthand)
pc_rotated = rotate(pc, [0, 0, 1], π/4)
pc_rotated = rotate(pc, :z, π/4)

# Scaling
pc_scaled = scale(pc, 2.0)              # uniform
pc_scaled = scale(pc, 2.0, 2.0, 1.0)   # non-uniform

# Arbitrary affine transformation
using CoordinateTransformations
tfm = Translation(10, 20, 30) ∘ LinearMap(RotZ(π/4))
pc_transformed = transform(pc, tfm)

# Bounding box crop
pc_cropped = bounding_box_crop(pc, [0, 0, 0], [10, 10, 10])
```

### Ground Segmentation

```julia
# Ground segmentation + above-ground height (AGH) interpolation
result = ground_segmentation(pc)
# result.ground_points       — ground point cloud
# result.aboveground_height  — per-point AGH (Vector{Float64})
# result.agh_cloud           — input cloud with :AGH attribute added
# result.ground_area         — area of ground polygon (m²)
# result.agh_computed        — Bool: whether AGH was actually computed
```

Lower-level helpers `segment_ground` and `calculate_aboveground_height` are
also exported for custom pipelines.

### Tree Segmentation

```julia
# Requires :AGH attribute (from ground_segmentation)
tree_result = tree_segmentation(result.agh_cloud)
# tree_result.pc_output       — input cloud annotated with :tree_id,
#                               :tree_nbs_id, :nbs_id, :node_id, :AGH
# tree_result.filtered_cloud  — near-ground / above-AGH-threshold subset used internally
# tree_result.skeleton_cloud  — proto-node skeleton point cloud
# tree_result.n_components    — number of connected components
# tree_result.neighbor_radius — radius used for the radius-neighbor graph

# Create a per-NBS skeleton point cloud
skeleton_pc = create_skeleton_cloud(tree_result.pc_output)
```

### QSM (Quantitative Structural Modeling)

```julia
# Fits per-branch cross-sections and aggregates to per-tree biometrics.
# Requires a tree-segmented point cloud (output of tree_segmentation).
qsm_result = qsm(
    tree_result   = tree_result,
    output_dir    = "out/",
    output_prefix = "demo_",
)
# qsm_result.pc_output         — input cloud annotated with :qsm_node_id
# qsm_result.qsm_surface_cloud — generated surface points with :tree_nbs_id, :rho
# qsm_result.node_csv_path     — per-node biometrics (DBH, cross-section area, volume)
# qsm_result.tree_csv_path     — per-tree aggregates (height, DBH, volume, surface area)
```

### Pipeline

`run_pipeline(config_path)` loads a TOML configuration and runs every enabled
stage in execution order (preprocess → ground segmentation → AGH → tree
segmentation → QSM), writing intermediate clouds and CSV outputs to
`pipeline.output_dir`. With no argument, it uses the package-default
`flip_config.toml` at the repo root.

```julia
run_pipeline("my_config.toml")
```

The config file mirrors the `FLiPConfig` struct one-to-one. A minimal example:

```toml
[pipeline]
input_path = "input.laz"
output_dir = "out/"
enable_qsm = true

[qsm]
min_node_size  = 5
rho_percentile = 0.85
```

See [`flip_config.toml`](flip_config.toml) for the full annotated template.

## Data Structure

The `PointCloud{T}` type stores 3D coordinates and optional attributes:

```julia
# Create from coordinates
coords = rand(Float64, 1000, 3)  # N×3 matrix
pc = PointCloud(coords)

# Add attributes
pc_with_attrs = PointCloud(coords,
    Dict(:intensity => rand(1000),
         :label => rand(1:5, 1000)))

# Access properties
n = npoints(pc)
coords = coordinates(pc)
bbox = bounds(pc)
centroid = center(pc)

# Indexing
subset = pc[1:10]  # Get first 10 points
subset = pc[indices]  # Index with integer vector
```

## Performance

FLiP.jl is designed for high performance on large point clouds:

- Efficient spatial indexing using NearestNeighbors.jl (KD-trees)
- Pre-allocated workspace structs for repeated graph operations
- Type-stable implementations throughout
- Index-based operations to minimize memory allocations

## Dependencies

- [NearestNeighbors.jl](https://github.com/KristofferC/NearestNeighbors.jl) - Spatial queries (KD-trees)
- [Graphs.jl](https://github.com/JuliaGraphs/Graphs.jl) - Graph algorithms
- [DelaunayTriangulation.jl](https://github.com/DanielVandH/DelaunayTriangulation.jl) - Mesh generation
- [MultivariateStats.jl](https://github.com/JuliaStats/MultivariateStats.jl) - PCA for linearity analysis
- [CoordinateTransformations.jl](https://github.com/JuliaGeometry/CoordinateTransformations.jl) / [Rotations.jl](https://github.com/JuliaGeometry/Rotations.jl) - Geometric transformations
- [StaticArrays.jl](https://github.com/JuliaArrays/StaticArrays.jl) - Efficient fixed-size arrays
- [PythonCall.jl](https://github.com/JuliaPy/PythonCall.jl) + CondaPkg — auto-managed Python env for LAS/LAZ I/O (`laspy` + `lazrs`) and E57 I/O (`pye57`)

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Citation

If you use FLiP.jl in your research, please cite:

```bibtex
@software{flip_jl,
  author = {Xu, Xiangtao},
  title = {FLiP.jl: Forest Lidar Processing in Julia},
  year = {2026},
  url = {https://github.com/xiangtaoxu/FLiP.jl}
}
```

## Related Projects

- [ForestLidarPackage](https://github.com/xiangtaoxu/ForestLidarPackage) - Python package for forest point cloud processing
- [CloudCompare](https://www.cloudcompare.org/) - 3D point cloud processing software
