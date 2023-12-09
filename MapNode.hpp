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
#include <stack>
#include <utility>

namespace StreetMatch {

    // 荷兰境内的纬度平均值约为 52°
    static constexpr double avgLatRad = 52.0 * (M_PI / 180.0);
    static constexpr double avgLonRad = 5.0 * (M_PI / 180.0);
    // 每度纬度和经度在荷兰地区大约对应的米数
    static constexpr double metersPerLat = 111132.92; // 纬度每度大约对应的米数
    // static constexpr double metersPerLon = 111412.84 * std::cos(avgLatRad); // 经度每度大约对应的米数
    static constexpr double metersPerLon = 111412.84 * 0.61566; // 使用 52° 的余弦近似值
    // 每度纬度和经度在荷兰地区大约对应的km
    static constexpr double kmPerLat = 111.13292; // 纬度每度大约对应的km
    static constexpr double kmPerLon = 111.41284 * 0.61566; // 使用 52° 的余弦近似值
    // 每度纬度和经度在荷兰地区大约对应的millmeters
    static constexpr double millmetersPerLat = .11113292;
    static constexpr double millmetersPerLon = .11141284 * 0.61566; // 使用 52° 的余弦近似值
    // 每度纬度和经度在荷兰地区大约对应的relative millmeters
    static constexpr double relativeMillmetersPerLat = 11.113292;
    static constexpr double relativeMillmetersPerLon = 11.141284 * 0.61566; // 使用 52° 的余弦近似值

    [[nodiscard]] static double toRadians(double degree) {
        return degree * (M_PI / 180.0);
    }

    class MapNode {
    private:
        // osmium::object_id_type id;
        long id;
        std::string latitude;
        std::string longitude;
        double latRad, lonRad;
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
//        explicit MapNode(long id, double lat, double lon, int trip_id)
//                : id(id), latitude(std::to_string(lat)), longitude(std::to_string(lon)), street_count(trip_id) {}
        explicit MapNode(long id, std::string lat, std::string lon, int trip_id)
                : id(id), latitude(std::move(lat)), longitude(std::move(lon)), street_count(trip_id) {
//            latRad = toRadians(std::stod(this->latitude));
//            lonRad = toRadians(std::stod(this->longitude));
            latRad = std::stod(this->latitude);
            lonRad = std::stod(this->longitude);
        }

        // Getters
        [[nodiscard]] long getId() const { return this->id; }
        [[nodiscard]] double getLon() const { return std::stod(this->longitude); }
        [[nodiscard]] double getLat() const { return std::stod(this->latitude); }
        [[nodiscard]] int getStreetCount() const { return this->street_count; }
        [[nodiscard]] std::string getLatString() const { return this->latitude; }
        [[nodiscard]] std::string getLonString() const { return this->longitude; }
        [[nodiscard]] double getLatRad() const { return this->latRad; }
        [[nodiscard]] double getLonRad() const { return this->lonRad; }

        [[nodiscard]] double distanceSquared(const MapNode& other) const {
            // L2 euclidean
            double lat_diff = std::stod(latitude) - std::stod(other.latitude);
            double lon_diff = std::stod(longitude) - std::stod(other.longitude);
            return lat_diff * lat_diff + lon_diff * lon_diff;
        }

        [[nodiscard]] double approximateDistanceNL(const MapNode& other) const {

            double dLatMeters = (other.latRad - latRad) * StreetMatch::relativeMillmetersPerLat;
            double dLonMeters = (other.lonRad - lonRad) * StreetMatch::relativeMillmetersPerLon;

            return dLatMeters * dLatMeters + dLonMeters * dLonMeters;
        }

        [[nodiscard]] double haversineDistance(const MapNode& other) const {
            // Haversine
            static constexpr double R = 6371.0;  // Earth's radius in kilometers

            double dLat = other.latRad - latRad;
            double dLon = other.lonRad - lonRad;

            double sin_dLat_half = std::sin(dLat / 2);
            double sin_dLon_half = std::sin(dLon / 2);

            double a = sin_dLat_half * sin_dLat_half +
                       std::cos(latRad) * std::cos(other.latRad) *
                       sin_dLon_half * sin_dLon_half;

            return R * 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
        }

        [[nodiscard]] double relativeHaversineDistance(const MapNode& other) const {
            double dLat = other.latRad - latRad;
            double dLon = other.lonRad - lonRad;

            double sin_dLat_half = std::sin(dLat / 2);
            double sin_dLon_half = std::sin(dLon / 2);

            // 'a' value from Haversine formula
            return sin_dLat_half * sin_dLat_half +
                   std::cos(latRad) * std::cos(other.latRad) *
                   sin_dLon_half * sin_dLon_half;
        }

        [[nodiscard]] double approximateDistance(const MapNode& other) const {
            double dLat = other.latRad - latRad;
            double dLon = other.lonRad - lonRad;
            double latMid = (latRad + other.latRad) / 2;

            return dLat * dLat + std::cos(latMid) * std::cos(latMid) * dLon * dLon;
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

        explicit KDTreeNode(MapNode p) : point(std::move(p)), left(nullptr), right(nullptr) {}
        // Ensure the class is not default constructible
        KDTreeNode() = delete;
    };

    class KDTree {
    public:
        KDTree() : root(nullptr) {}

        void insert(const MapNode & point) {
            insertOne(root, point, 0);
        }

        MapNode nearestNeighbor(const MapNode & queryPoint) {
            std::optional<MapNode> nearest;
            double nearestDist = std::numeric_limits<double>::max();
//            findNearestNeighbor(root, queryPoint, nearest, nearestDist);
            findNearestNeighbor(root, queryPoint, 0, nearest, nearestDist);
            if (nearest.has_value()) {
                return nearest.value();
            } else {
                throw std::runtime_error("No nearest neighbor found");
            }
        }

    private:
        std::unique_ptr<KDTreeNode> root;

        void insertOne(std::unique_ptr<KDTreeNode>& node, const MapNode & point, unsigned depth) {
            if (!node) {
                node = std::make_unique<KDTreeNode>(point);
                return;
            }

            unsigned axis = depth % 2;
            if ((axis == 0 && point.getLat() < node->point.getLat()) || (axis == 1 && point.getLon() < node->point.getLon())) {
                insertOne(node->left, point, depth + 1);
            } else {
                insertOne(node->right, point, depth + 1);
            }
        }

//        static void findNearestNeighbor(std::unique_ptr<KDTreeNode>& root, const MapNode& queryPoint, std::optional<MapNode>& nearest, double& nearestDist) {
//            if (!root) return;
//
//            std::stack<std::pair<std::unique_ptr<KDTreeNode>&, unsigned>> stack;
//            stack.emplace(root, 0);
//
//            while (!stack.empty()) {
//                auto& [node, depth] = stack.top();
//                stack.pop();
//
//                if (!node) continue;
//
//                // Same logic as before
//                unsigned axis = depth % 2; // 0 for latitude, 1 for longitude
//                double axisDist = (axis == 0) ? std::abs(queryPoint.getLatRad() - node->point.getLatRad())
//                                              : std::abs(queryPoint.getLonRad() - node->point.getLonRad());
//
//                if (axisDist < nearestDist) {
//                    // double dist = node->point.distanceSquared(queryPoint);
//                    // double dist = node->point.relativeHaversineDistance(queryPoint);
//                    double dist = node->point.approximateDistanceNL(queryPoint);
//                    if (dist < nearestDist) {
//                        nearestDist = dist;
//                        nearest = node->point;
//                    }
//                }
//
//                bool goLeft = (axis == 0) ? (queryPoint.getLatRad() < node->point.getLatRad())
//                                          : (queryPoint.getLonRad() < node->point.getLonRad());
//
//                if (goLeft) {
//                    if (node->right) stack.emplace(node->right, depth + 1);
//                    if (node->left) stack.emplace(node->left, depth + 1);
//                } else {
//                    if (node->left) stack.emplace(node->left, depth + 1);
//                    if (node->right) stack.emplace(node->right, depth + 1);
//                }
//            }
//        }
//        void findNearestNeighbor(std::unique_ptr<KDTreeNode>& node, const MapNode & queryPoint, unsigned depth, std::optional<MapNode>& nearest, double& nearestDist) {
//            if (!node) return;
//
//            // double dist = node->point.haversineDistance(queryPoint);
//            // double dist = node->point.distanceSquared(queryPoint);
//            // double dist = node->point.relativeHaversineDistance(queryPoint);
//            double dist = node->point.approximateDistanceNL(queryPoint);
//            if (dist < nearestDist) {
//                nearestDist = dist;
//                nearest = node->point;
//            }
//
//            unsigned axis = depth % 2; // 0 for latitude, 1 for longitude
//            bool goLeft = (axis == 0 && queryPoint.getLatRad() < node->point.getLatRad()) || (axis == 1 && queryPoint.getLonRad() < node->point.getLonRad());
//            std::unique_ptr<KDTreeNode>& first = goLeft ? node->left : node->right;
//            std::unique_ptr<KDTreeNode>& second = first == node->left ? node->right : node->left;
//
//            // Search down the tree
//            findNearestNeighbor(first, queryPoint, depth + 1, nearest, nearestDist);
//
//            // Check if we need to search the other branch
//            double axisDist = (axis == 0 ? queryPoint.getLatRad() - node->point.getLatRad() : queryPoint.getLonRad() - node->point.getLonRad());
//            if (axisDist * axisDist < nearestDist) {
//                findNearestNeighbor(second, queryPoint, depth + 1, nearest, nearestDist);
//            }
//        }
        void findNearestNeighbor(std::unique_ptr<KDTreeNode>& node, const MapNode & queryPoint, unsigned depth, std::optional<MapNode>& nearest, double& nearestDist) {
            if (!node) return;

            double dist = node->point.approximateDistanceNL(queryPoint);
            if (dist < nearestDist) {
                nearestDist = dist;
                nearest = node->point;
            }

            unsigned axis = depth % 2;
            std::unique_ptr<KDTreeNode>& first = (axis == 0 && queryPoint.getLat() < node->point.getLat()) || (axis == 1 && queryPoint.getLon() < node->point.getLon()) ? node->left : node->right;
            std::unique_ptr<KDTreeNode>& second = first == node->left ? node->right : node->left;

            findNearestNeighbor(first, queryPoint, depth + 1, nearest, nearestDist);

            double axisDist = axis == 0 ? queryPoint.getLat() - node->point.getLat() : queryPoint.getLon() - node->point.getLon();
            if (axisDist * axisDist < nearestDist) {
                findNearestNeighbor(second, queryPoint, depth + 1, nearest, nearestDist);
            }
        }

    };

} // StreetMatch

#endif //STREETMATCH_MAPNODE_HPP


