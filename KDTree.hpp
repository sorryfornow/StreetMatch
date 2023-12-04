//
// Created by Siqing Zhang on 4/12/2023.
//

#ifndef STREETMATCH_KDTREE_HPP
#define STREETMATCH_KDTREE_HPP

#include <cmath>
#include <memory>
#include <algorithm>
#include <limits>

struct Point {
    double latitude;
    double longitude;
    int node_id;

    Point(double lat = 0, double lon = 0, int id = 0) : latitude(lat), longitude(lon), node_id(id) {}

    double distanceSquared(const Point& other) const {
        double lat_diff = latitude - other.latitude;
        double lon_diff = longitude - other.longitude;
        return lat_diff * lat_diff + lon_diff * lon_diff;
    }
};

class KDTreeNode {
public:
    Point point;
    std::unique_ptr<KDTreeNode> left;
    std::unique_ptr<KDTreeNode> right;

    KDTreeNode(Point p) : point(p), left(nullptr), right(nullptr) {}
};

class KDTree {
public:
    KDTree() : root(nullptr) {}

    void insert(Point point) {
        insert(root, point, 0);
    }

    Point nearestNeighbor(Point queryPoint) {
        Point nearest;
        double nearestDist = std::numeric_limits<double>::max();
        nearestNeighbor(root, queryPoint, 0, nearest, nearestDist);
        return nearest;
    }

private:
    std::unique_ptr<KDTreeNode> root;

    void insert(std::unique_ptr<KDTreeNode>& node, Point point, unsigned depth) {
        if (!node) {
            node = std::make_unique<KDTreeNode>(point);
            return;
        }

        unsigned axis = depth % 2;
        if ((axis == 0 && point.latitude < node->point.latitude) || (axis == 1 && point.longitude < node->point.longitude)) {
            insert(node->left, point, depth + 1);
        } else {
            insert(node->right, point, depth + 1);
        }
    }

    void nearestNeighbor(std::unique_ptr<KDTreeNode>& node, Point queryPoint, unsigned depth, Point& nearest, double& nearestDist) {
        if (!node) return;

        double dist = node->point.distanceSquared(queryPoint);
        if (dist < nearestDist) {
            nearestDist = dist;
            nearest = node->point;
        }

        unsigned axis = depth % 2;
        std::unique_ptr<KDTreeNode>& first = (axis == 0 && queryPoint.latitude < node->point.latitude) || (axis == 1 && queryPoint.longitude < node->point.longitude) ? node->left : node->right;
        std::unique_ptr<KDTreeNode>& second = first == node->left ? node->right : node->left;

        nearestNeighbor(first, queryPoint, depth + 1, nearest, nearestDist);

        double axisDist = axis == 0 ? queryPoint.latitude - node->point.latitude : queryPoint.longitude - node->point.longitude;
        if (axisDist * axisDist < nearestDist) {
            nearestNeighbor(second, queryPoint, depth + 1, nearest, nearestDist);
        }
    }
};


#endif //STREETMATCH_KDTREE_HPP
