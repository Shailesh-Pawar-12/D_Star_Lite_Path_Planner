#include "dstar_lite/dstar_lite.hpp"
#include <cmath>
#include <limits>
#include <iostream>
#include <vector>
#include <queue>

DStarLitePlanner::DStarLitePlanner(int width, int height)
    : width_(width), height_(height), start_x_(-1), start_y_(-1), goal_x_(-1), goal_y_(-1) {
    grid_.resize(width_ * height_);
    for (int i = 0; i < width_; ++i) {
        for (int j = 0; j < height_; ++j) {
            grid_[i + j * width_] = Cell(i, j);
        }
    }
}

void DStarLitePlanner::set_obstacle(int x, int y) {
    if (in_bounds(x, y)) {
        grid_[x + y * width_].is_obstacle = true;
    }
}

void DStarLitePlanner::remove_obstacle(int x, int y) {
    if (in_bounds(x, y)) {
        grid_[x + y * width_].is_obstacle = false;
    }
}

bool DStarLitePlanner::plan(int start_x, int start_y, int goal_x, int goal_y) {
    // Check if start and goal locations are within bounds
    if (!in_bounds(start_x, start_y) || !in_bounds(goal_x, goal_y)) {
        std::cerr << "Error: Start or goal location is out of bounds." << std::endl;
        return false;
    }

    // Check if start and goal locations are not obstacles
    if (grid_[start_x + start_y * width_].is_obstacle || grid_[goal_x + goal_y * width_].is_obstacle) {
        std::cerr << "Error: Start or goal location is on an obstacle." << std::endl;
        return false;
    }

    // Set the start and goal locations
    start_x_ = start_x;
    start_y_ = start_y;
    goal_x_ = goal_x;
    goal_y_ = goal_y;

    // Initialize and compute path
    initialize();
    compute_shortest_path();
    reconstruct_path();
    return true;
}

std::vector<std::pair<int, int>> DStarLitePlanner::get_path() const {
    return path_;
}

void DStarLitePlanner::initialize() {
    for (auto& cell : grid_) {
        cell.g = std::numeric_limits<float>::infinity();
        cell.rhs = std::numeric_limits<float>::infinity();
    }
    grid_[goal_x_ + goal_y_ * width_].rhs = 0.0;

    open_list_.emplace(goal_x_, goal_y_, heuristic(goal_x_, goal_y_), 0.0, 0.0);

    key_.resize(width_ * height_, std::make_pair(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()));
}

void DStarLitePlanner::compute_shortest_path() {
    while (!open_list_.empty()) {
        auto current = open_list_.top();
        open_list_.pop();

        int x = current.x;
        int y = current.y;

        if (grid_[x + y * width_].g > grid_[x + y * width_].rhs) {
            grid_[x + y * width_].g = grid_[x + y * width_].rhs;
            update_vertex(x, y);
        } else {
            grid_[x + y * width_].g = std::numeric_limits<float>::infinity();
        }
    }
}

bool DStarLitePlanner::is_adjacent_to_obstacle(int x, int y) const {
    std::vector<std::pair<int, int>> adjacent_cells = {
        {x+1, y}, {x-1, y}, {x, y+1}, {x, y-1},      
        {x+1, y+1}, {x+1, y-1}, {x-1, y+1}, {x-1, y-1} 
    };

    for (const auto& cell : adjacent_cells) {
        int nx = cell.first;
        int ny = cell.second;
        if (in_bounds(nx, ny) && grid_[nx + ny * width_].is_obstacle) {
            return true;
        }
    }
    return false;
}

void DStarLitePlanner::update_vertex(int x, int y) {
    if (x == start_x_ && y == start_y_) {
        return;
    }

    // Update neighbors 
    std::vector<std::pair<int, int>> neighbors = {
        {x+1, y}, {x-1, y}, {x, y+1}, {x, y-1},      
        {x+1, y+1}, {x+1, y-1}, {x-1, y+1}, {x-1, y-1} 
    };

    for (const auto& neighbor : neighbors) {
        int nx = neighbor.first;
        int ny = neighbor.second;

        if (in_bounds(nx, ny) && !grid_[nx + ny * width_].is_obstacle && !is_adjacent_to_obstacle(nx, ny)) {
            // For diagonal moves, ensure that adjacent cells are also free
            if (x != nx && y != ny) { // Diagonal move
                if (grid_[x + ny * width_].is_obstacle || grid_[nx + y * width_].is_obstacle) {
                    continue; // Skip this diagonal move if adjacent cells are obstacles
                }
            }
            float cost = (x != nx && y != ny) ? std::sqrt(2.0f) : 1.0f; 
            float new_rhs = grid_[x + y * width_].g + cost;
            if (new_rhs < grid_[nx + ny * width_].rhs) {
                grid_[nx + ny * width_].rhs = new_rhs;
                open_list_.emplace(nx, ny, new_rhs + heuristic(nx, ny), new_rhs, new_rhs);
            }
        }
    }
}

float DStarLitePlanner::heuristic(int x, int y) const {
    return std::sqrt(std::pow(x - goal_x_, 2) + std::pow(y - goal_y_, 2));
}

bool DStarLitePlanner::in_bounds(int x, int y) const {
    return x >= 0 && x < width_ && y >= 0 && y < height_;
}

void DStarLitePlanner::reconstruct_path() {
    path_.clear();
    int x = start_x_, y = start_y_;

    while (x != goal_x_ || y != goal_y_) {
        path_.emplace_back(x, y);

        std::vector<std::pair<int, int>> neighbors = {
            {x+1, y}, {x-1, y}, {x, y+1}, {x, y-1},      
            {x+1, y+1}, {x+1, y-1}, {x-1, y+1}, {x-1, y-1} 
        };

        float min_cost = std::numeric_limits<float>::infinity();
        std::pair<int, int> next_step = {x, y};

        for (const auto& neighbor : neighbors) {
            int nx = neighbor.first;
            int ny = neighbor.second;

            if (in_bounds(nx, ny) && !grid_[nx + ny * width_].is_obstacle && !is_adjacent_to_obstacle(nx, ny)) {
                // For diagonal steps, ensure that adjacent cells are not obstacles
                if (x != nx && y != ny) { // Diagonal move
                    if (grid_[x + ny * width_].is_obstacle || grid_[nx + y * width_].is_obstacle) {
                        continue; // Skip this diagonal move if adjacent cells are obstacles
                    }
                }

                float cost = grid_[nx + ny * width_].g;
                if (cost < min_cost) {
                    min_cost = cost;
                    next_step = neighbor;
                }
            }
        }

        if (next_step == std::make_pair(x, y)) {
            // No valid moves found, stop path reconstruction
            break;
        }

        x = next_step.first;
        y = next_step.second;
    }
    path_.emplace_back(goal_x_, goal_y_);
}
