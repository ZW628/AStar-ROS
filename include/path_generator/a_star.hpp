/*
    Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
    Following tool is licensed under the terms and conditions of the ISC license.
    For more information visit https://opensource.org/licenses/ISC.
*/
#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <math.h>

#include <functional>
#include <set>
#include <vector>

namespace AStar {
// 坐标
struct Vec2i {
    int x, y;
    // 重载操作符==和!=
    bool operator==(const Vec2i& coordinates_);
    bool operator!=(const Vec2i& coordinates_);
};

using uint = unsigned int;
using HeuristicFunction = std::function<uint(Vec2i, Vec2i)>;
using CoordinateList = std::vector<Vec2i>;

// 节点
struct Node {
    uint G, H;
    Vec2i coordinates;
    Node* parent;

    // 构造函数
    Node(Vec2i coord_, Node* parent_ = nullptr);
    // 成员函数
    uint getScore();
};

using NodeSet = std::vector<Node*>;

// 用于生成或查找某种路径
class Generator {
    bool detectCollision(Vec2i coordinates_);
    Node* findNodeOnList(NodeSet& nodes_, Vec2i coordinates_);
    void releaseNodes(NodeSet& nodes_);

public:
    Generator();
    void setWorldSize(Vec2i worldSize_);
    void setDiagonalMovement(bool enable_);
    void setHeuristic(HeuristicFunction heuristic_);
    CoordinateList findPath(Vec2i source_, Vec2i target_);
    void addCollision(Vec2i coordinates_);
    void addCollision(Vec2i coordinates_, int size);
    void removeCollision(Vec2i coordinates_);
    void clearCollisions();

private:
    HeuristicFunction heuristic;
    CoordinateList direction, walls;
    Vec2i worldSize;
    uint directions;
};

// 计算不同的启发式距离
class Heuristic {
    static Vec2i getDelta(Vec2i source_, Vec2i target_);

public:
    static uint manhattan(Vec2i source_, Vec2i target_);
    static uint euclidean(Vec2i source_, Vec2i target_);
    static uint octagonal(Vec2i source_, Vec2i target_);
};

} // namespace AStar

#endif // ASTAR_HPP
