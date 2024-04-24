
#include "dijkstra.h"

#include "gtest/gtest.h"

TEST(DijkstraTest, test1) {
    // 有向图
    Dijkstra<int> d(6);
    d.add_edge(0, 1, 1);
    d.add_edge(1, 2, 2);
    d.add_edge(2, 3, 2);
    d.add_edge(3, 4, 1);
    d.add_edge(1, 3, 3);

    {
        // 测试起点终点之间有边的情况
        auto ans = d.shortest_path(0, 4);
        EXPECT_EQ(ans.first, 5);
    }
    {
        // 测试没有边的情况
        auto ans = d.shortest_path(0, 5);
        EXPECT_EQ(ans.first, -1);
        EXPECT_EQ(ans.second.size(), 0);
    }

    {
        // 测试越界情况
        auto ans = d.shortest_path(0, 6);
        EXPECT_EQ(ans.first, -1);
        EXPECT_EQ(ans.second.size(), 0);
    }
    {
        // 测试起点终点相同情况
        auto ans = d.shortest_path(0, 0);
        EXPECT_EQ(ans.first, 0);
        EXPECT_EQ(ans.second.size(), 1);
    }

    {
        std::vector<std::pair<int, std::vector<std::pair<int, int>>>> except_paths = {
            {0, {{0, 0}}},
            {1, {{0, 0}, {1, 1}}},
            {3, {{0, 0}, {1, 1}, {2, 2}}},
            {4, {{0, 0}, {1, 1}, {3, 3}}},
            {5, {{0, 0}, {1, 1}, {3, 3}, {1, 4}}},
            {-1, {}},
        };

        auto short_paths = d.dijkstra(0);
        EXPECT_EQ(short_paths, except_paths);
    }
}

TEST(DIjkstraTest, test2) {
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
        std::vector<std::pair<int, std::vector<std::pair<int, int>>>> except_paths = {
            {0, {{0, 0}}},
            {2, {{0, 0}, {2, 1}}},
            {6, {{0, 0}, {6, 2}}},
            {7, {{0, 0}, {2, 1}, {5, 3}}},
            {17, {{0, 0}, {2, 1}, {5, 3}, {10, 4}}},
            {20, {{0, 0}, {2, 1}, {5, 3}, {10, 4}, {3, 5}}},
            {19, {{0, 0}, {2, 1}, {5, 3}, {10, 4}, {2, 6}}},
        };
        auto short_paths = d.dijkstra(0);
        EXPECT_EQ(short_paths, except_paths);
    }
    {
        std::vector<std::pair<int, std::vector<std::pair<int, int>>>> except_paths = {
            {6, {{0, 2}, {6, 0}}},
            {8, {{0, 2}, {6, 0}, {2, 1}}},
            {0, {{0, 2}}},
            {8, {{0, 2}, {8, 3}}},
            {18, {{0, 2}, {8, 3}, {10, 4}}},
            {21, {{0, 2}, {8, 3}, {10, 4}, {3, 5}}},
            {20, {{0, 2}, {8, 3}, {10, 4}, {2, 6}}},
        };

        auto short_paths = d.dijkstra(2);
        EXPECT_EQ(short_paths, except_paths);
    }
}