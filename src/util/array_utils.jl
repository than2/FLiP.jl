"""
Generic array utilities used by point-cloud filtering.

Functions:
- `_uf_find!(parent, i)`       — union-find root lookup with path halving
- `_uf_union!(parent, rnk, a, b)` — union-find union by rank
"""

function _uf_find!(parent::Vector{Int}, i::Int)
    @inbounds while parent[i] != i
        parent[i] = parent[parent[i]]  # path halving
        i = parent[i]
    end
    return i
end

function _uf_union!(parent::Vector{Int}, rnk::Vector{Int}, a::Int, b::Int)
    ra = _uf_find!(parent, a)
    rb = _uf_find!(parent, b)
    ra == rb && return
    if rnk[ra] < rnk[rb]; ra, rb = rb, ra; end
    parent[rb] = ra
    rnk[ra] += (rnk[ra] == rnk[rb])
end
