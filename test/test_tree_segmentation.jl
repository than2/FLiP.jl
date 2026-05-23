@testset "tree_segmentation internals" begin
    @testset "_seed_trees_from_nearground!" begin
        # Two NBS: one with all points near ground, one above the ceiling
        nbs_points = Dict(1 => [1, 2, 3], 2 => [4, 5])
        agh        = [0.1, 0.2, 0.15, 3.0, 3.5]
        tree_id    = zeros(Int32, 5)

        seeded = FLiP._seed_trees_from_nearground!(tree_id, nbs_points, agh, 0.5)

        @test seeded.nbs_tree[1] == Int32(1)
        @test !haskey(seeded.nbs_tree, 2)
        @test 1 in seeded.assigned_nbs
        @test !(2 in seeded.assigned_nbs)
        @test tree_id[1:3] == Int32[1, 1, 1]
        @test all(==(Int32(0)), tree_id[4:5])
        @test seeded.next_tree_id == Int32(2)

        # All NBS near-ground — both seeded, tree ids 1 and 2
        tree_id2 = zeros(Int32, 5)
        seeded2  = FLiP._seed_trees_from_nearground!(tree_id2, nbs_points,
                                                     [0.1, 0.2, 0.15, 0.2, 0.3], 0.5)
        @test seeded2.next_tree_id == Int32(3)
        @test length(seeded2.assigned_nbs) == 2
        @test all(>(Int32(0)), tree_id2)

        # No NBS near-ground — nothing seeded
        tree_id3 = zeros(Int32, 5)
        seeded3  = FLiP._seed_trees_from_nearground!(tree_id3, nbs_points,
                                                     [5.0, 5.0, 5.0, 5.0, 5.0], 0.5)
        @test seeded3.next_tree_id == Int32(1)
        @test isempty(seeded3.assigned_nbs)
        @test all(==(Int32(0)), tree_id3)
    end
end
