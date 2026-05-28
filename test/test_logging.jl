@testset "Logging helpers" begin

    @testset "_fmt_elapsed — adaptive units" begin
        @test FLiP._fmt_elapsed(0.34)  == "0.34s"
        @test FLiP._fmt_elapsed(1.234) == "1.2s"
        @test FLiP._fmt_elapsed(59.5)  == "59.5s"
        @test FLiP._fmt_elapsed(84.0)  == "1m 24s"
        @test FLiP._fmt_elapsed(3600.0) == "60m 0s"
    end

    @testset "ProgressReporter — single threaded" begin
        p = FLiP.ProgressReporter("unit-test", 100)
        @test p.last_pct[] == -5
        for i in 1:100
            FLiP.report!(p, i)
        end
        # After processing all 100, the last reported boundary should be 100%.
        @test p.last_pct[] == 100
    end

    @testset "ProgressReporter — zero total is a no-op" begin
        p = FLiP.ProgressReporter("empty", 0)
        FLiP.report!(p, 0)   # should not throw
        FLiP.report!(p, 5)   # should not throw
        @test p.last_pct[] == -5
    end

    @testset "ProgressReporter — thread-safe under @threads" begin
        # Multiple threads racing on report! with a shared counter; the CAS
        # gate guarantees at most one print per 5% boundary and the atomic
        # ends at exactly 100%.
        p = FLiP.ProgressReporter("threaded", 1000)
        done = Threads.Atomic{Int}(0)
        Threads.@threads for i in 1:1000
            n = Threads.atomic_add!(done, 1) + 1
            FLiP.report!(p, n)
        end
        @test p.last_pct[] == 100
        @test done[] == 1000
    end

    @testset "_LOG_PREFIX constant defined" begin
        @test FLiP._LOG_PREFIX == "[FLiP]"
    end

    @testset "parallel_for — dynamic scheduling" begin
        # Empty / degenerate ranges.
        let hits = Threads.Atomic{Int}(0)
            FLiP.parallel_for(0, 4) do _; Threads.atomic_add!(hits, 1); end
            @test hits[] == 0
        end
        let hits = Threads.Atomic{Int}(0)
            FLiP.parallel_for(1, 4) do _; Threads.atomic_add!(hits, 1); end
            @test hits[] == 1
        end

        # Every index 1:n is visited exactly once, across thread budgets and for
        # n both smaller than and much larger than the budget. Each worker writes
        # its own disjoint slot, so no atomics are needed for correctness.
        for nt in (1, 3), n in (2, 1000)
            visited = zeros(Int, n)
            FLiP.parallel_for(n, nt) do i
                visited[i] += 1
            end
            @test all(==(1), visited)
        end

        # Skewed per-index cost (early indices sleep longer) must not change the
        # result: dynamic scheduling reorders execution but the per-slot output
        # matches the serial computation.
        let n = 200
            out = Vector{Int}(undef, n)
            FLiP.parallel_for(n, 3) do i
                i <= 5 && sleep(0.005)   # heavy items concentrated at the front
                out[i] = i * i
            end
            @test out == [i * i for i in 1:n]
        end
    end
end
