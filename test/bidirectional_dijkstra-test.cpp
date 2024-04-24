
#include "bidirectional_dijkstra.h"

#include "gtest/gtest.h"

TEST(BiDijkstraTest, test1) {
    BiDirDijkstra<int> d(8);
    d.add_edge(0, 1, 2);
    d.add_edge(0, 2, 6);
    d.add_edge(1, 3, 5);
    d.add_edge(2, 3, 8);
    d.add_edge(3, 4, 10);
    d.add_edge(3, 5, 15);
    d.add_edge(4, 5, 3);
    d.add_edge(4, 6, 2);
    d.add_edge(5, 6, 6);
    {
        // 测试没有边的情况
        auto ans = d.shortest_path(0, 7);
        EXPECT_EQ(ans.first, -1);
    }
    {
        // 测试越界情况
        auto ans = d.shortest_path(0, 8);
        EXPECT_EQ(ans.first, -1);
    }
    {
        // 测试测试起点终点相同情况
        auto ans = d.shortest_path(5, 5);
        EXPECT_EQ(ans.first, 0);
        EXPECT_EQ(ans.second.size(), 1);
    }

    {
        std::vector<std::pair<int, std::vector<std::pair<int, int>>>> except_paths = {
            {0, {{0, 0}}},
            {2, {{0, 0}, {2, 1}}},
            {6, {{0, 0}, {6, 2}}},
            {7, {{0, 0}, {2, 1}, {5, 3}}},
            {17, {{0, 0}, {2, 1}, {5, 3}, {10, 4}}},
            {20, {{0, 0}, {2, 1}, {5, 3}, {10, 4}, {3, 5}}},
            {19, {{0, 0}, {2, 1}, {5, 3}, {10, 4}, {2, 6}}},
            {-1, {}},
        };
        auto short_paths = d.dijkstra(0);
        EXPECT_EQ(short_paths, except_paths);
    }
}

TEST(BiDijkstraTest, test2) {
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
    {
        std::vector<std::pair<int, std::vector<std::pair<int, int>>>> except_paths = {
            {0, {{0, 0}}},
            {1, {{0, 0}, {1, 1}}},
            {2, {{0, 0}, {1, 1}, {1, 2}}},
            {3, {{0, 0}, {1, 1}, {1, 2}, {1, 3}}},
            {8, {{0, 0}, {8, 4}}},
            {4, {{0, 0}, {4, 5}}},
            {10, {{0, 0}, {8, 4}, {2, 6}}},
            {4, {{0, 0}, {1, 1}, {1, 2}, {1, 3}, {1, 7}}},
            {5, {{0, 0}, {4, 5}, {1, 8}}},
        };

        auto short_paths = d.dijkstra(0);
        EXPECT_EQ(short_paths, except_paths);
    }
}