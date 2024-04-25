#ifndef PATH_BIDIJKSTRA_H
#define PATH_BIDIJKSTRA_H
#include <doctest/doctest.h>
#include <nanobench.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <queue>
#include <set>
#include <vector>

template <typename T>
struct BiDirDijkstra {
    using E = std::pair<T, int>;
    using M = std::pair<T, std::vector<E>>;
    const int INF = -1;

    int n;
    std::vector<std::vector<E>> g, gr;  // 有权有向图，利用gr作反向图
    BiDirDijkstra(int N) : n(N), g(N), gr(n) {}

    bool check(int u) { return u >= 0 && u < n; }

    void add_edge(int u, int v, T cost) {
        if (!check(u) || !check(v)) return;
        g[u].emplace_back(cost, v);
        gr[v].emplace_back(cost, u);
    }
    std::vector<M> dijkstra(int s) {
        if (!check(s)) return {};
        std::vector<M> ans(n, {T(INF), {}});

        for (int i = 0; i < n; i++) {
            ans[i] = shortest_path(s, i);
        }
        return ans;
    }

    M shortest_path(int s, int t) {
        if (!check(s) || !check(t)) return {T(INF), {}};                             // 检查越界
        if (s == t) return {T(0), {{0, s}}};                                         // 起点和终点相同
        std::vector<std::priority_queue<E, std::vector<E>, std::greater<E>>> pq(2);  // 正向和反向优先队列
        std::vector<std::vector<T>> dist(2, std::vector<T>(n, T(INF)));  // 记录正向和反向最短路径长度
        std::vector<std::vector<E>> prev(2, std::vector<E>(n, {T(INF), INF}));  // 记录cost和前驱节点

        dist[0][s] = 0, dist[1][t] = 0;  // 起点和终点初始化为 0
        prev[0][s] = {0, INF}, prev[1][t] = {0, INF};
        pq[0].emplace(0, s);
        pq[1].emplace(0, t);  // 起点和终点入队

        bool ok = false;
        T estimate = T(INF);
        T tops = T(INF), topt = T(INF);
        // 记录最佳路径最后扩展的边
        int last_from = s, last_to = t, last_cost = 0;

        while (!pq[0].empty() && !pq[1].empty()) {
            {
                auto [cur_dist, cur_node] = pq[0].top();
                pq[0].pop();

                spdlog::debug("forward origin: ({}, {})", cur_dist, cur_node);

                if (dist[0][cur_node] < cur_dist) continue;

                tops = cur_dist;
                for (const auto& [cost, next_node] : g[cur_node]) {
                    // 松弛操作
                    if (dist[0][next_node] == T(INF) || cur_dist + cost < dist[0][next_node]) {
                        dist[0][next_node] = cur_dist + cost;
                        prev[0][next_node] = {cost, cur_node};  // 记录前驱节点

                        // 计算uv_cost，避免在dist[1][next_node]为INF时进行不必要的加法操作
                        T uv_cost =
                            (dist[1][next_node] == T(INF)) ? T(INF) : dist[0][cur_node] + dist[1][next_node] + cost;
                        // 更新estimate
                        if (uv_cost != T(INF)) {
                            if (estimate == T(INF) || uv_cost < estimate) {
                                estimate = uv_cost;
                                // 记录更新estimate时扩展的边
                                last_from = cur_node, last_to = next_node, last_cost = cost;
                                spdlog::debug("estimate: {}, update by: ({}, {})", estimate, cur_node, next_node);
                            }
                        }

                        pq[0].emplace(dist[0][next_node], next_node);
                        spdlog::debug("forward add: ({}, {})", dist[0][next_node], next_node);
                    }
                }
            }

            {
                auto [cur_dist, cur_node] = pq[1].top();
                pq[1].pop();
                spdlog::debug("backward origin: ({}, {})", cur_dist, cur_node);

                if (dist[1][cur_node] < cur_dist) continue;
                topt = cur_dist;
                for (const auto& [cost, next_node] : gr[cur_node]) {
                    if (dist[1][next_node] == T(INF) || cur_dist + cost < dist[1][next_node]) {
                        dist[1][next_node] = cur_dist + cost;
                        prev[1][next_node] = {cost, cur_node};

                        // 计算uv_cost，避免在dist[0][to]为INF时进行不必要的加法操作
                        T uv_cost =
                            (dist[0][next_node] == T(INF)) ? T(INF) : dist[1][cur_node] + dist[0][next_node] + cost;
                        // 更新estimate
                        if (uv_cost != T(INF)) {
                            if (estimate == T(INF) || uv_cost < estimate) {
                                estimate = uv_cost;
                                // 记录更新estimate时扩展的边
                                last_from = next_node, last_to = cur_node, last_cost = cost;
                                spdlog::debug("estimate: {}, update by: ({}, {})", estimate, cur_node, next_node);
                            }
                        }
                        pq[1].emplace(dist[1][next_node], next_node);
                        spdlog::debug("backward add: ({}, {})", dist[1][next_node], next_node);
                    }
                }
            }

            if (estimate != T(INF) && tops + topt >= estimate) break;
        }

        if (estimate == T(INF)) return {estimate, {}};  // 未找到路径

        std::vector<E> path;
        // 前向搜索回溯路径
        auto cur = last_from;
        while (cur != INF) {
            path.emplace_back(prev[0][cur].first, cur);
            cur = prev[0][cur].second;
        }
        std::reverse(path.begin(), path.end());

        // // 添加last_cost, last_to的边和权重
        path.emplace_back(last_cost, last_to);

        // 后向搜索路径
        auto rev_cur = last_to;
        while (prev[1][rev_cur].second != INF) {  // 下一个节点不是INF
            path.emplace_back(prev[1][rev_cur].first, prev[1][rev_cur].second);
            rev_cur = prev[1][rev_cur].second;
        }

        return {estimate, path};
    }

    T shortest_dist(int s, int t) {
        if (!check(s) || !check(t)) return T(INF);                                   // 检查越界
        if (s == t) return T(0);                                                     // 起点和终点相同
        std::vector<std::priority_queue<E, std::vector<E>, std::greater<E>>> pq(2);  // 正向和反向优先队列
        std::vector<std::vector<T>> dist(2, std::vector<T>(n, T(INF)));  // 记录正向和反向最短路径长度

        dist[0][s] = 0, dist[1][t] = 0;  // 起点和终点初始化为 0
        pq[0].emplace(0, s);
        pq[1].emplace(0, t);  // 起点和终点入队

        bool ok = false;
        T estimate = T(INF);
        T tops = T(INF), topt = T(INF);

        while (!pq[0].empty() && !pq[1].empty()) {
            {
                auto [cur_dist, cur_node] = pq[0].top();
                pq[0].pop();

                if (dist[0][cur_node] < cur_dist) continue;

                tops = cur_dist;
                for (const auto& [cost, next_node] : g[cur_node]) {
                    // 松弛操作
                    if (dist[0][next_node] == T(INF) || cur_dist + cost < dist[0][next_node]) {
                        dist[0][next_node] = cur_dist + cost;

                        // 计算uv_cost，避免在dist[1][next_node]为INF时进行不必要的加法操作
                        T uv_cost =
                            (dist[1][next_node] == T(INF)) ? T(INF) : dist[0][cur_node] + dist[1][next_node] + cost;
                        // 更新estimate
                        if (uv_cost != T(INF)) {
                            if (estimate == T(INF) || uv_cost < estimate) {
                                estimate = uv_cost;
                            }
                        }

                        pq[0].emplace(dist[0][next_node], next_node);
                    }
                }
            }

            {
                auto [cur_dist, cur_node] = pq[1].top();
                pq[1].pop();

                if (dist[1][cur_node] < cur_dist) continue;
                topt = cur_dist;
                for (const auto& [cost, next_node] : gr[cur_node]) {
                    if (dist[1][next_node] == T(INF) || cur_dist + cost < dist[1][next_node]) {
                        dist[1][next_node] = cur_dist + cost;

                        // 计算uv_cost，避免在dist[0][to]为INF时进行不必要的加法操作
                        T uv_cost =
                            (dist[0][next_node] == T(INF)) ? T(INF) : dist[1][cur_node] + dist[0][next_node] + cost;
                        // 更新estimate
                        if (uv_cost != T(INF)) {
                            if (estimate == T(INF) || uv_cost < estimate) {
                                estimate = uv_cost;
                            }
                        }
                        pq[1].emplace(dist[1][next_node], next_node);
                    }
                }
            }

            if (estimate != T(INF) && tops + topt >= estimate) break;
        }

        return estimate;
    }
};

TEST_CASE("BiDijkstraTest1") {
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

    SUBCASE("起点终点之间有边") {
        auto ans = d.shortest_path(0, 5);
        CHECK(ans.first == 20);
    }
    SUBCASE("起点终点之间没有边") {
        auto ans = d.shortest_path(0, 7);
        CHECK(ans.first == -1);
    }
    SUBCASE("起点终点越界") {
        auto ans = d.shortest_path(0, 9);
        CHECK(ans.first == -1);
    }
    SUBCASE("起点终点相同") {
        auto ans = d.shortest_path(0, 0);
        CHECK(ans.first == 0);
    }
    SUBCASE("所有结果") {
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
        CHECK(short_paths == except_paths);
    }
}

TEST_CASE("DijkstraTest2") {
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

    SUBCASE("Test 所有路径") {
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
        CHECK(short_paths == except_paths);
    }
}

TEST_CASE("DijkstraTest3") {
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

    ankerl::nanobench::Bench().run("BiDijkstra", [&] { d.shortest_dist(0, 8); });
}

#endif
