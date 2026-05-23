"""
    preprocess(; cfg::FLiPConfig=_CFG) -> PointCloud

Discover, clean, and merge the input point clouds.

Workflow:
1. Discover input files via [`find_input_files`](@ref) — either a single
   path or every file in a directory matching `{input_prefix}*.{input_format}`.
2. For each file: read it, optionally distance-subsample, optionally
   statistical-filter, and write a per-scan output. Single-file runs
   produce `{prefix}preprocess.{fmt}`; multi-file runs produce
   `{prefix}preprocess_S{i}.{fmt}` per scan. For E57 inputs the subsample
   is applied to the raw coordinate matrix *before* building the
   `PointCloud`, to avoid materializing the full-size cloud in memory.
3. Merge every per-scan `PointCloud` into a single one
   (`merge_pointclouds` — vcat coords, intersect attribute keys).
4. Return the merged cloud. The on-disk artifacts written in step 2 are
   what downstream stages (and the resume path) consume.
"""
function preprocess(; cfg::FLiPConfig=_CFG)
    input_files = find_input_files(; cfg=cfg)
    output_dir = cfg.pipeline_output_dir
    output_prefix = cfg.pipeline_output_prefix
    output_fmt = lowercase(cfg.pipeline_output_format)

    mkpath(output_dir)

    n_files = length(input_files)
    T = coord_type(cfg)
    all_coords = Vector{Matrix{T}}(undef, n_files)
    all_attrs  = Vector{Dict{Symbol,Vector}}(undef, n_files)

    for (i, fpath) in enumerate(input_files)
        println("[preprocess] Reading file $i/$n_files: $fpath")
        ext = lowercase(splitext(fpath)[2])

        if ext == ".e57" && cfg.pipeline_enable_preprocess && cfg.pipeline_enable_subsample
            # For large E57 files: subsample on raw coords before building PointCloud
            # to avoid peak memory from full-size LAS construction
            coords, attrs = _read_e57_to_raw(fpath; precision=T)
            n_raw = size(coords, 1)
            println("[preprocess]   raw points: $n_raw, subsampling at $(cfg.pipeline_subsample_res)m...")
            keep = distance_subsample(coords, cfg.pipeline_subsample_res)
            coords = coords[keep, :]
            for (k, v) in attrs
                attrs[k] = v[keep]
            end
            println("[preprocess]   after subsample: $(size(coords, 1)) points")
            pc = PointCloud(coords, attrs)

            if cfg.preprocess_enable_statistical_filter
                pc = pc[statistical_filter(coordinates(pc),
                                           cfg.statistical_filter_k_neighbors,
                                           cfg.statistical_filter_n_sigma)]
            end
        else
            pc = read_pc(fpath)
            if cfg.pipeline_enable_preprocess
                pc = _preprocess_single(pc; cfg=cfg)
            end
        end

        # Write with _S{i} suffix for multi-file, no suffix for single file
        suffix = n_files > 1 ? "_S$(i)" : ""
        out_path = joinpath(output_dir, "$(output_prefix)preprocess$(suffix).$(output_fmt)")
        write_pc(out_path, pc)
        println("[preprocess] Wrote: $out_path  ($(npoints(pc)) points)")

        # Extract coords/attrs and release the PointCloud object
        all_coords[i] = coordinates(pc)
        all_attrs[i]  = _all_attributes(pc)
    end

    merged = merge_pointclouds(all_coords, all_attrs)
    n_files > 1 && println("[preprocess] Merged $n_files scans → $(npoints(merged)) points")
    return merged
end

# ── Per-scan helper ───────────────────────────────────────────────

"""
    _preprocess_single(pc::PointCloud; cfg::FLiPConfig=_CFG) -> PointCloud

Apply the per-scan preprocessing steps to a single in-memory cloud:
optional distance subsample, then optional statistical filter.
"""
function _preprocess_single(pc::PointCloud; cfg::FLiPConfig=_CFG)
    if cfg.pipeline_enable_subsample
        pc = pc[distance_subsample(coordinates(pc), cfg.pipeline_subsample_res)]
    end

    if cfg.preprocess_enable_statistical_filter
        pc = pc[statistical_filter(coordinates(pc),
                                   cfg.statistical_filter_k_neighbors,
                                   cfg.statistical_filter_n_sigma)]
    end

    return pc
end
