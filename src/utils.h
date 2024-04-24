#ifndef UTILS_H
#define UTILS_H
#include <format>
#include <sstream>

void print_shortest_path(const auto& short_paths) {
    for (int i = 0; i < short_paths.size(); ++i) {
        std::ostringstream path;
        std::ostringstream cos;
        bool first_element = true;

        for (const auto& e : short_paths[i].second) {
            if (first_element) {
                path << e.second;
                cos << e.first;
                first_element = false;
                continue;
            }
            path << "->" << e.second;
            cos << ", " << e.first;
        }

        std::cout << std::format("node: {0: <5} dist: {1: <5} path: {2: <20} cost: {3: <20}", i,
                                 short_paths[i].first, path.str(), cos.str())
                  << std::endl;

        // std::cout << "node: " << std::left << std::setw(5) << i << "dis: " << std::left
        //           << std::setw(5) << short_paths[i].first << "path: " << std::left <<
        //           std::setw(20)
        //           << path.str() << "cos: " << std::left << std::setw(20) << cos.str() <<
        //           std::endl;
    }
}

#endif