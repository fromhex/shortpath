
#include "benchmark/benchmark.h"
#include "bidirectional_dijkstra.h"
#include "dijkstra.h"

static void BM_DIJKSTRA(benchmark::State& state) {
    Dijkstra<int> d(9);
    d.add_edge(0, 1, 1);
    d.add_edge(0, 5, 4);
    d.add_edge(0, 4, 8);
    d.add_edge(1, 2, 1);
    d.add_edge(2, 3, 1);
    d.add_edge(3, 7, 1);
    d.add_edge(7, 8, 2);
    d.add_edge(5, 8, 1);
    d.add_edge(4, 6, 2);
    d.add_edge(6, 8, 6);

    for (auto _ : state) {
        auto short_paths = d.shortest_dist(0, 8);
    }
}

static void BM_BIDIJKSTRA(benchmark::State& state) {
    BiDirDijkstra<int> d(9);
    d.add_edge(0, 1, 1);
    d.add_edge(0, 5, 4);
    d.add_edge(0, 4, 8);
    d.add_edge(1, 2, 1);
    d.add_edge(2, 3, 1);
    d.add_edge(3, 7, 1);
    d.add_edge(7, 8, 2);
    d.add_edge(5, 8, 1);
    d.add_edge(4, 6, 2);
    d.add_edge(6, 8, 6);

    for (auto _ : state) {
        auto short_paths = d.shortest_dist(0, 8);
    }
}
BENCHMARK(BM_BIDIJKSTRA);
BENCHMARK(BM_DIJKSTRA);
BENCHMARK_MAIN();