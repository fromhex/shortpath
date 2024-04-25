#define DOCTEST_CONFIG_IMPLEMENT
#define ANKERL_NANOBENCH_IMPLEMENT
#include <iomanip>
#include <iostream>

#include "dijkstra/bidirectional_dijkstra.h"
#include "dijkstra/dijkstra.h"
#include "dijkstra/utils.h"

void get_result() {
    // spdlog::set_level(spdlog::level::debug);
    {
        Dijkstra<int> d(6);
        d.add_edge(0, 1, 1);
        d.add_edge(1, 2, 2);
        d.add_edge(2, 3, 2);
        d.add_edge(3, 4, 1);
        d.add_edge(1, 3, 3);
        using E = std::pair<int, int>;

        auto short_paths = d.dijkstra(0);
        print_shortest_path(short_paths);
    }

    std::cout << "==========================================" << std::endl;

    {
        Dijkstra<int> d(7);
        d.add_bidir_edge(0, 1, 2);
        d.add_bidir_edge(0, 2, 6);
        d.add_bidir_edge(1, 3, 5);
        d.add_bidir_edge(2, 3, 8);
        d.add_bidir_edge(3, 4, 10);
        d.add_bidir_edge(3, 5, 15);
        d.add_bidir_edge(4, 5, 3);
        d.add_bidir_edge(4, 6, 2);
        d.add_bidir_edge(5, 6, 6);
        {
            auto short_paths = d.dijkstra(0);
            print_shortest_path(short_paths);
        }
        std::cout << "------------------------------------------" << std::endl;
        {
            auto short_paths = d.dijkstra(2);
            print_shortest_path(short_paths);
        }
    }
    std::cout << "==========================================" << std::endl;

    {
        BiDirDijkstra<int> d(7);
        d.add_edge(0, 1, 2);
        d.add_edge(0, 2, 6);
        d.add_edge(1, 3, 5);
        d.add_edge(2, 3, 8);
        d.add_edge(3, 4, 10);
        d.add_edge(3, 5, 15);
        d.add_edge(4, 5, 3);
        d.add_edge(4, 6, 2);
        d.add_edge(5, 6, 6);
        auto short_paths = d.dijkstra(0);
        print_shortest_path(short_paths);
    }
    std::cout << "==========================================" << std::endl;

    {
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
        auto short_paths = d.dijkstra(0);
        print_shortest_path(short_paths);
    }
}

int main(int argc, char **argv) {
    doctest::Context context;
    context.applyCommandLine(argc, argv);
    int res = context.run();  // run doctest
    // important - query flags (and --exit) rely on the user doing this
    if (context.shouldExit()) {
        // propagate the result of the tests
        return res;
    }

    get_result();
}