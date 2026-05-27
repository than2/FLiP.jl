"""
Logging helpers for the FLiP pipeline.

Provides three primitives used across all stages:
- `_LOG_PREFIX` — uniform `[FLiP]` prefix for all stage messages.
- `_with_stage_timing(name, f)` — wraps a stage body with start / end
  announcements and wall-clock elapsed timing.
- `_log_stage_skipped(name, reason)` — one-liner for disabled stages.
- `ProgressReporter` + `report!` — thread-safe percentage-throttled progress
  reporter that emits at 5% boundaries. Single-CAS gate; safe under
  `Threads.@threads`.
- `_fmt_elapsed(dt)` — adaptive seconds → minutes formatting.

All log calls go through Julia's standard `@info`; helpers do not bypass the
active logger. Helpers are internals — no exports.
"""

# Prefix used by every stage log line — keeps `grep "[FLiP]"` useful.
const _LOG_PREFIX = "[FLiP]"

# ── Elapsed-time formatting ───────────────────────────────────────────────

"""
    _fmt_elapsed(dt::Real) -> String

Adaptive elapsed-time formatter: sub-second → `"0.34s"`, seconds →
`"1.2s"`, above 1 minute → `"1m 24s"`.
"""
function _fmt_elapsed(dt::Real)
    dt < 1.0  && return string(round(dt, digits=2), "s")
    dt < 60.0 && return string(round(dt, digits=1), "s")
    m, s = divrem(dt, 60.0)
    return string(Int(m), "m ", Int(round(s)), "s")
end

# ── Stage timing wrappers ─────────────────────────────────────────────────

"""
    _with_stage_timing(f, stage_name) -> result-of-f

Emit a `>> <stage> starting` line, run `f()`, then emit a `<< <stage>
done (elapsed)` line. Returns whatever `f` returned. Start/end markers are
always visible — they are not gated by `enable_debug_info`.

Function-first signature so the standard do-block syntax works:
```julia
_with_stage_timing("preprocess") do
    preprocess(; cfg=cfg)
end
```
"""
function _with_stage_timing(f::Function, stage_name::AbstractString)
    @info "$_LOG_PREFIX >> $stage_name starting"
    t0 = time()
    result = f()
    @info "$_LOG_PREFIX << $stage_name done ($(_fmt_elapsed(time() - t0)))"
    return result
end

"""
    _log_stage_skipped(stage_name, reason="disabled by config")

One-liner for stages that are present in the pipeline but did not run.
"""
function _log_stage_skipped(stage_name::AbstractString,
                            reason::AbstractString="disabled by config")
    @info "$_LOG_PREFIX -- $stage_name skipped ($reason)"
end

# ── Thread-safe progress reporter ─────────────────────────────────────────

"""
    ProgressReporter(label, total)

Throttled progress reporter that emits `@info` only when a 5% boundary is
crossed on `n_done / total`. Thread-safe: the percentage gate is held in a
`Threads.Atomic{Int}` and the CAS in `report!` guarantees that at most one
thread emits per boundary.

Callers tracking `n_done` from inside a `Threads.@threads` loop should use a
shared `Threads.Atomic{Int}` for the count (atomic_add! per iteration) and
pass the snapshot value as `n_done`.
"""
mutable struct ProgressReporter
    label::String
    total::Int
    last_pct::Threads.Atomic{Int}
    t_start::Float64
end

ProgressReporter(label::AbstractString, total::Integer) =
    ProgressReporter(String(label), Int(total), Threads.Atomic{Int}(-5), time())

"""
    report!(p::ProgressReporter, n_done; extra="")

Emit a throttled progress line if `n_done / p.total` crossed the next 5%
boundary. No-op when `p.total == 0`. Optional `extra` is appended to the
message in parentheses.
"""
function report!(p::ProgressReporter, n_done::Integer; extra::AbstractString="")
    p.total > 0 || return
    pct = clamp(round(Int, 100.0 * n_done / p.total), 0, 100)
    while true
        last = p.last_pct[]
        pct < last + 5 && return
        new_last = pct - (pct % 5)
        # CAS: only one thread wins the boundary and prints.
        Threads.atomic_cas!(p.last_pct, last, new_last) === last || continue
        elapsed = _fmt_elapsed(time() - p.t_start)
        msg = isempty(extra) ?
              "$_LOG_PREFIX   $(p.label): $(new_last)% ($n_done/$(p.total), $elapsed)" :
              "$_LOG_PREFIX   $(p.label): $(new_last)% ($n_done/$(p.total), $elapsed, $extra)"
        @info msg
        return
    end
end

# ── Bounded parallel-for ──────────────────────────────────────────────────

"""
    parallel_for(f, n::Integer, n_thread::Integer)

Run `f(i)` for `i in 1:n`, splitting work across at most `n_thread` concurrent
tasks via `@sync` / `Threads.@spawn`. Static contiguous chunking — index `i` is
processed by exactly one task. Falls back to a plain serial loop when
`n_thread <= 1`, `n <= 1`, or `Threads.nthreads() == 1`.

`n_thread` should be the resolved thread budget from `effective_nthreads(cfg)`.
The function does not consult any global config; callers pass the count
explicitly.

`f` is specialized via the `where {F}` type parameter to avoid closure boxing.
"""
function parallel_for(f::F, n::Integer, n_thread::Integer) where {F}
    n <= 0 && return nothing
    nt = min(Int(n_thread), Int(n))
    if nt <= 1 || Threads.nthreads() == 1
        @inbounds for i in 1:n
            f(i)
        end
        return nothing
    end
    chunk_size = cld(Int(n), nt)
    @sync for c in 1:nt
        lo = (c - 1) * chunk_size + 1
        hi = min(c * chunk_size, Int(n))
        lo > Int(n) && break
        Threads.@spawn @inbounds for i in lo:hi
            f(i)
        end
    end
    return nothing
end
