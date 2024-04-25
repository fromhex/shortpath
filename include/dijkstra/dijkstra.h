#ifndef PATH_DIJKSTRA_H
#define PATH_DIJKSTRA_H
#include <doctest/doctest.h>
#include <nanobench.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <queue>
#include <set>
#include <vector>

template <typename T>
struct Dijkstra {
    const int INF = -1;
    using E = std::pair<T, int>;             // 权重, 节点
    using P = std::pair<T, std::vector<E>>;  // 最短路径长度，具体路径

    int n;                           // 节点数
    std::vector<std::vector<E>> g;   // 邻接表
    Dijkstra(int N) : n(N), g(N) {}  // 初始化

    bool check(int u) { return u >= 0 && u < n; }

    /**
     * @brief 在图中添加一条从节点 u 到节点 v 的边，边的权值为 cost。
     *
     * @param u 起始节点
     * @param v 目标节点
     * @param cost 边的权值
     */
    void add_edge(int u, int v, T cost) {
        if (!check(u) || !check(v)) return;
        g[u].emplace_back(cost, v);
    }

    /**
     * @brief 在图中添加一条从节点 u 到节点 v 的边，并且添加一条从节点 v 到节点
     * u 的边，边的权重为给定的 cost。
     *
     * @param u 节点 u
     * @param v 节点 v
     * @param cost 边的权重
     */
    void add_bidir_edge(int u, int v, T cost) {
        if (!check(u) || !check(v)) return;
        add_edge(u, v, cost);
        add_edge(v, u, cost);
    }

    /**
     * @brief 使用 Dijkstra 算法计算从起点 s
     * 到所有其他顶点的最短路径。如果某个顶点不可达，则返回 INF。
     *
     * @param s 起点
     *
     * @return 返回一个 vector，其中每个元素表示从起点 s
     * 到对应顶点的最短路径长度
     */
    std::vector<P> dijkstra(int s) {  // unreachable : INF
        if (!check(s)) return {};
        std::vector<T> d(n, T(INF));
        std::priority_queue<E, std::vector<E>, std::greater<E>> pq;
        std::vector<E> prev(n, {T(INF), INF});  // 记录cost和前驱节点

        d[s] = 0;
        prev[s] = {0, INF};
        pq.emplace(0, s);

        while (!pq.empty()) {
            auto [cost, from] = pq.top();
            pq.pop();

            if (d[from] < cost) continue;  // 不会重复处理那些已知有更短路径的顶点

            for (const auto &[c, to] : g[from]) {  // 遍历 u 的所有邻接点
                if (d[to] == T(INF) || cost + c < d[to]) {
                    d[to] = cost + c;
                    prev[to] = {c, from};  // 更新代价和前驱节点
                    pq.emplace(d[to], to);
                }
            }
        }

        std::vector<P> short_paths(n);
        for (int i = 0; i < n; ++i) {
            short_paths[i] = {d[i], {}};
            if (d[i] == T(INF)) continue;  // 不可达
            auto cur = i;
            while (cur != INF) {
                short_paths[i].second.emplace_back(prev[cur].first, cur);  // 记录路径
                cur = prev[cur].second;
            }
            std::reverse(short_paths[i].second.begin(), short_paths[i].second.end());
        }
        return short_paths;
    }

    /**
     * @brief 使用 Dijkstra 算法计算从起点 s 到终点 t 的最短路径及其代价。
     *
     * @param s 起点
     * @param t 终点
     *
     * @return 返回包含最短路径代价和路径的 std::pair 对象
     */
    P shortest_path(int s, int t) {
        if (!check(s) || !check(t)) return {T(INF), {}};  // 检查越界
        if (s == t) return {T(0), {{0, s}}};              // 起点和终点相同

        std::vector<T> d(n, T(INF));
        std::vector<E> prev(n, {T(INF), INF});  // 记录cost和前驱节点

        std::priority_queue<E, std::vector<E>, std::greater<E>> pq;  // 最小堆

        d[s] = 0;
        prev[s] = {0, INF};
        pq.emplace(0, s);

        while (!pq.empty()) {
            auto [cost, from] = pq.top();
            pq.pop();

            if (d[from] < cost) continue;  // 不会重复处理那些已知有更短路径的顶点

            if (from == t) break;                  // 终点已找到，提前退出
            for (const auto &[c, to] : g[from]) {  // 遍历 u 的所有邻接点
                if (d[to] == T(INF) || cost + c < d[to]) {
                    d[to] = cost + c;
                    prev[to] = {c, from};  // 更新代价和前驱节点
                    pq.emplace(d[to], to);
                }
            }
        }

        T total_cost = d[t];
        if (total_cost == T(INF)) return {total_cost, {}};  // 未找到路径

        std::vector<E> short_path;
        auto cur = t;
        while (cur != INF) {                                // 回溯路径，直到没有前驱节点
            short_path.emplace_back(prev[cur].first, cur);  // 记录路径
            cur = prev[cur].second;
        }

        std::reverse(short_path.begin(), short_path.end());

        return {total_cost, short_path};
    }

    T shortest_dist(int s, int t) {
        if (!check(s) || !check(t)) return T(INF);  // 检查越界
        if (s == t) return T(0);                    // 起点和终点相同

        std::vector<T> d(n, T(INF));
        std::priority_queue<E, std::vector<E>, std::greater<E>> pq;  // 最小堆

        d[s] = 0;
        pq.emplace(0, s);

        while (!pq.empty()) {
            auto [cost, from] = pq.top();
            pq.pop();

            if (d[from] < cost) continue;  // 不会重复处理那些已知有更短路径的顶点

            if (from == t) break;                  // 终点已找到，提前退出
            for (const auto &[c, to] : g[from]) {  // 遍历 u 的所有邻接点
                if (d[to] == T(INF) || cost + c < d[to]) {
                    d[to] = cost + c;
                    pq.emplace(d[to], to);
                }
            }
        }

        return d[t];
    }
};

TEST_CASE("DijkstraTest1") {
    // 有向图
    auto dijkstra = std::make_unique<Dijkstra<int>>(6);
    dijkstra->add_edge(0, 1, 1);
    dijkstra->add_edge(1, 2, 2);
    dijkstra->add_edge(2, 3, 2);
    dijkstra->add_edge(3, 4, 1);
    dijkstra->add_edge(1, 3, 3);

    SUBCASE("起点终点之间有边") {
        auto ans = dijkstra->shortest_path(0, 4);
        CHECK(ans.first == 5);
    }
    SUBCASE("起点终点之间没有边") {
        auto ans = dijkstra->shortest_path(0, 5);
        CHECK(ans.first == -1);
    }
    SUBCASE("起点终点越界") {
        auto ans = dijkstra->shortest_path(0, 6);
        CHECK(ans.first == -1);
    }
    SUBCASE("起点终点相同") {
        auto ans = dijkstra->shortest_path(0, 0);
        CHECK(ans.first == 0);
    }
    SUBCASE("所有结果") {
        std::vector<std::pair<int, std::vector<std::pair<int, int>>>> except_paths = {
            {0, {{0, 0}}},
            {1, {{0, 0}, {1, 1}}},
            {3, {{0, 0}, {1, 1}, {2, 2}}},
            {4, {{0, 0}, {1, 1}, {3, 3}}},
            {5, {{0, 0}, {1, 1}, {3, 3}, {1, 4}}},
            {-1, {}},
        };

        auto short_paths = dijkstra->dijkstra(0);
        CHECK(short_paths == except_paths);
    }
}

TEST_CASE("DijkstraTest2") {
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

    SUBCASE("Test 起点1") {
        auto ans = d.shortest_path(0, 6);
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
        CHECK(short_paths == except_paths);
    }

    SUBCASE("Test 起点2") {
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
        CHECK(short_paths == except_paths);
    }
}

TEST_CASE("DijkstraTest3") {
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

    ankerl::nanobench::Bench().run("Dijkstra", [&] { d.shortest_dist(0, 8); });
}

#endif
