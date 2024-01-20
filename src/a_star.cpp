#include "../include/path_generator/a_star.hpp"

#include <algorithm>

using namespace std::placeholders;

bool AStar::Vec2i::operator==(const Vec2i& coordinates_) { return (x == coordinates_.x && y == coordinates_.y); }

bool AStar::Vec2i::operator!=(const Vec2i& coordinates_) { return !(x == coordinates_.x && y == coordinates_.y); }

AStar::Vec2i operator+(const AStar::Vec2i& left_, const AStar::Vec2i& right_) {
    return {left_.x + right_.x, left_.y + right_.y};
}

AStar::Node::Node(Vec2i coordinates_, Node* parent_) {
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

AStar::uint AStar::Node::getScore() { return G + H; }

AStar::Generator::Generator() {
    setDiagonalMovement(false);          // 禁止对角线移动
    setHeuristic(&Heuristic::manhattan); // 使用曼哈顿启发式函数
    // 这些向量表示方向：上、右、下、左以及四个对角线方向
    direction = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}, {-1, -1}, {1, 1}, {-1, 1}, {1, -1}}; // std::vector<Vec2i> direction
}

void AStar::Generator::setWorldSize(Vec2i worldSize_) { worldSize = worldSize_; }

void AStar::Generator::setDiagonalMovement(bool enable_) { directions = (enable_ ? 8 : 4); }

// 允许用户向 AStar::Generator 设置自定义的启发式估价函数，以便在执行A*寻路算法时根据实际需求调整路径搜索策略
void AStar::Generator::setHeuristic(HeuristicFunction heuristic_) { heuristic = std::bind(heuristic_, _1, _2); }

// 在全局障碍物列表 walls 中添加新的碰撞坐标点，以确保寻路算法不会尝试通过这些不可通行的位置
void AStar::Generator::addCollision(Vec2i coordinates_) {
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x || coordinates_.y < 0 || coordinates_.y >= worldSize.y)
        return;

    // 检测这个坐标是否已经是一个已知的碰撞点
    if (!detectCollision(coordinates_)) walls.push_back(coordinates_); // std::vector<Vec2i>walls
}

// 添加一个方形区域内的所有坐标点作为障碍物
void AStar::Generator::addCollision(Vec2i coordinates_, int size) {
    if (size == 0) {
        addCollision(coordinates_);
        return;
    }

    for (int i = 0; i < 4; i++) {
        // 计算出以输入坐标为中心的新坐标点 new_coord
        // 循环遍历四个基本方向
        Vec2i new_coord = coordinates_ + direction[i];
        addCollision(new_coord, size - 1);
    }
}

void AStar::Generator::removeCollision(Vec2i coordinates_) {
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

void AStar::Generator::clearCollisions() { walls.clear(); }

AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_) {
    Node* current = nullptr;
    NodeSet openSet, closedSet; // std::vector<Node*>
    openSet.reserve(100);
    closedSet.reserve(100);
    openSet.push_back(new Node(source_));

    // timespec 结构体被用于测量函数执行的时间
    struct timespec start, end;
    // 获取当前时间戳
    // CLOCK_MONOTONIC 表示从某个固定时间点（通常系统启动时）开始的单调递增的时间，适合于测量时间间隔和计算持续时间
    clock_gettime(CLOCK_MONOTONIC, &start);

    while (!openSet.empty()) {
        auto current_it = openSet.begin();
        current = *current_it;

        // 从开放集中选取具有最小F值的节点
        for (auto it = openSet.begin(); it != openSet.end(); it++) {
            auto node = *it;
            if (node->getScore() <= current->getScore()) {
                current = node;  // 当前最佳节点
                current_it = it; // 位置的迭代器，指向该节点在开放集中的位置
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &end);
        double time_result =
            end.tv_sec - start.tv_sec + (end.tv_nsec - start.tv_nsec) * 1e-9; // 1e-9 是将纳秒转换为秒的系数

        if (time_result > 1.0) { // time failure limit
            printf("time: %lf\n", time_result);
            break;
        }

        if (current->coordinates == target_) {
            break;
        }

        closedSet.push_back(current);
        openSet.erase(current_it);

        // directions为4：水平移动，8：对角线移动
        for (uint i = 0; i < directions; ++i) {
            Vec2i newCoordinates(current->coordinates + direction[i]);
            // 查找newCoordinates是否在closedSet中
            if (detectCollision(newCoordinates) || findNodeOnList(closedSet, newCoordinates)) {
                continue;
            }

            // i 小于4时，通常意味着沿直线（水平或垂直）移动，将10添加到 current->G 上作为额外的移动代价
            // i 大于等于4时，则可能表示对角线移动，代价更高为14
            uint totalCost = current->G + ((i < 4) ? 10 : 14);

            Node* successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr) {
                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);
                openSet.push_back(successor);
            } else if (totalCost < successor->G) {
                successor->parent = current;
                successor->G = totalCost;
            }
        }
    }

    CoordinateList path;

    if (current->coordinates == target_) {
        while (current != nullptr) {
            path.push_back(current->coordinates);
            current = current->parent;
        }
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);

    return path;
}

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec2i coordinates_) {
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_) {
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

bool AStar::Generator::detectCollision(Vec2i coordinates_) {
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x || coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
        std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
        return true;
    }

    return false;
}

AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_) {
    return {abs(source_.x - target_.x), abs(source_.y - target_.y)};
}

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_) {
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y));
}

AStar::uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_) {
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_) {
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}
