#ifndef DSTAR_LITE_HPP
#define DSTAR_LITE_HPP

#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <utility>
#include <chrono>
#include <iostream>
#include <thread>

// ANSI escape code for green color
#define ANSI_COLOR_GREEN "\033[32m"
// ANSI escape code to reset color
#define ANSI_COLOR_RESET "\033[0m"
struct Cell {
    int x, y;
    float g, rhs, cost;
    bool is_obstacle;

    // Default constructor
    Cell() : x(0), y(0), g(std::numeric_limits<float>::infinity()), 
             rhs(std::numeric_limits<float>::infinity()), cost(1.0), is_obstacle(false) {}

    // Parameterized constructor
    Cell(int x, int y)
        : x(x), y(y), g(std::numeric_limits<float>::infinity()), 
          rhs(std::numeric_limits<float>::infinity()), cost(1.0), is_obstacle(false) {}

    bool operator>(const Cell& other) const {
        return g > other.g;
    }
};

class DStarLitePlanner {
public:
    DStarLitePlanner(int width, int height);
    void set_obstacle(int x, int y);
    void remove_obstacle(int x, int y);
    bool plan(int start_x, int start_y, int goal_x, int goal_y);
    std::vector<std::pair<int, int>> get_path() const;

private:
    int width_, height_;
    std::vector<Cell> grid_;
    int start_x_, start_y_, goal_x_, goal_y_;
    struct PQElement {
        int x, y;
        float key1, key2;
        float g;

        PQElement(int x, int y, float key1, float key2, float g)
            : x(x), y(y), key1(key1), key2(key2), g(g) {}

        bool operator>(const PQElement& other) const {
            return key1 > other.key1 || (key1 == other.key1 && key2 > other.key2);
        }
    };

    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> open_list_;
    std::vector<std::pair<float, float>> key_;
    std::vector<std::pair<int, int>> path_;


    void initialize();
    void compute_shortest_path();
    void update_vertex(int x, int y);
    float heuristic(int x, int y) const;
    bool in_bounds(int x, int y) const;
    void reconstruct_path();
    bool is_adjacent_to_obstacle(int x, int y) const;
};

#endif // DSTAR_LITE_HPP
