//
// Created by Siqing Zhang on 26/11/2023.
//
#ifndef STREETMATCH_MAPNODE_HPP
#define STREETMATCH_MAPNODE_HPP

#include <iostream>
#include <string>
#include <regex>
#include <sstream>
#include <fstream>
#include <vector>
#include <chrono>
#include <optional>
#include <limits>
#include <memory>
#include <cmath>
#include <compare>
#include <algorithm>
#include <iomanip>

namespace StreetMatch {
    class MapNode {
    private:
        // osmium::object_id_type id;
        long id;
        double latitude;
        double longitude;
        int street_count;

    public:
        // Delete default constructor
        MapNode() = delete;

//        // Delete copy constructor and copy assignment operator to make MapNode uncopyable
//        MapNode(const MapNode &other) = delete;
//        MapNode &operator=(const MapNode &other) = delete;
        MapNode(const MapNode &other) = default;
        MapNode &operator=(const MapNode &other) = default;

        // Default move constructor and move assignment operator
        MapNode(MapNode &&other) = default;
        MapNode &operator=(MapNode &&other) = default;

        // Default destructor
        ~MapNode() = default;

        // Parameterized constructor
        explicit MapNode(long id, double lat, double lon, int trip_id)
                : id(id), latitude(lat), longitude(lon), street_count(trip_id) {}

        // Getters
        [[nodiscard]] long getId() const { return this->id; }
        [[nodiscard]] double getLon() const { return this->longitude; }
        [[nodiscard]] double getLat() const { return this->latitude; }
        [[nodiscard]] int getStreetCount() const { return this->street_count; }

        [[nodiscard]] double distanceSquared(const MapNode& other) const {
            // L2 euclidean distance
            double lat_diff = latitude - other.latitude;
            double lon_diff = longitude - other.longitude;
            return lat_diff * lat_diff + lon_diff * lon_diff;
        }

        // Stream output operator
        friend std::ostream &operator<<(std::ostream &os, const MapNode &node) {
            os << "<node id=\"" << node.id << "\">\n"
               << "  <data key=\"d4\">" << node.latitude << "</data>\n"
               << "  <data key=\"d5\">" << node.longitude << "</data>\n"
               << "  <data key=\"d6\">" << node.street_count << "</data>\n"
               << "</node>\n";
            return os;
        }

        auto operator<=>(const MapNode&) const = default;
    };


    class KDTreeNode {
    public:
        MapNode point;
        std::unique_ptr<KDTreeNode> left;
        std::unique_ptr<KDTreeNode> right;

        explicit KDTreeNode(MapNode p) : point(p), left(nullptr), right(nullptr) {}
        // Ensure the class is not default constructible
        KDTreeNode() = delete;
    };

    class KDTree {
    public:
        KDTree() : root(nullptr) {}

        void insert(MapNode point) {
            insert(root, point, 0);
        }

        MapNode nearestNeighbor(MapNode queryPoint) {
            std::optional<MapNode> nearest;
            double nearestDist = std::numeric_limits<double>::max();
            findNearestNeighbor(root, queryPoint, 0, nearest, nearestDist);
            if (nearest.has_value()) {
                return nearest.value();
            } else {
                throw std::runtime_error("No nearest neighbor found");
            }
        }

    private:
        std::unique_ptr<KDTreeNode> root;

        void insert(std::unique_ptr<KDTreeNode>& node, MapNode point, unsigned depth) {
            if (!node) {
                node = std::make_unique<KDTreeNode>(point);
                return;
            }

            unsigned axis = depth % 2;
            if ((axis == 0 && point.getLat() < node->point.getLat()) || (axis == 1 && point.getLon() < node->point.getLon())) {
                insert(node->left, point, depth + 1);
            } else {
                insert(node->right, point, depth + 1);
            }
        }

        void findNearestNeighbor(std::unique_ptr<KDTreeNode>& node, MapNode queryPoint, unsigned depth, std::optional<MapNode>& nearest, double& nearestDist) {
            if (!node) return;

            double dist = node->point.distanceSquared(queryPoint);
//            std::cout << "dist: " << dist << std::endl;
//            std::cout << "nearestDist: " << nearestDist << std::endl;
            if (dist < nearestDist) {
                nearestDist = dist;
                nearest = node->point;
//                std::cout << "nearest: " << nearest.value().getId() << std::endl;
            }

            unsigned axis = depth % 2; // 0 for latitude, 1 for longitude
            std::unique_ptr<KDTreeNode>& first = (axis == 0 && queryPoint.getLat() < node->point.getLat()) || (axis == 1 && queryPoint.getLon() < node->point.getLon()) ? node->left : node->right;
            std::unique_ptr<KDTreeNode>& second = first == node->left ? node->right : node->left;

            // Search down the tree
            findNearestNeighbor(first, queryPoint, depth + 1, nearest, nearestDist);

            // Check if we need to search the other branch
            double axisDist = axis == 0 ? queryPoint.getLat() - node->point.getLat() : queryPoint.getLon() - node->point.getLon();
            if (axisDist * axisDist < nearestDist) {
                findNearestNeighbor(second, queryPoint, depth + 1, nearest, nearestDist);
            }
        }

    };

} // StreetMatch

#endif //STREETMATCH_MAPNODE_HPP


