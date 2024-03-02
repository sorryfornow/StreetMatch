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
#include <stdexcept>
#include <format>

namespace StreetMatch {

    // 荷兰境内的纬度平均值约为 52°
    static constexpr double avgLatRad = 52.0 * (M_PI / 180.0);
    static constexpr double avgLonRad = 5.0 * (M_PI / 180.0);
    // 每度纬度和经度在荷兰地区大约对应的米数
    static constexpr double metersPerLat = 111132.92; // 纬度每度大约对应的米数
    // static constexpr double metersPerLon = 111412.84 * std::cos(avgLatRad); // 经度每度大约对应的米数
    static constexpr double metersPerLon = 111412.84 * 0.61566; // 使用 52。06° 的余弦近似值 0.61474370776
    // 每度纬度和经度在荷兰地区大约对应的km
    static constexpr double kmPerLat = 111.13292; // 纬度每度大约对应的km
    static constexpr double kmPerLon = 111.41284 * 0.61566; // 使用 52° 的余弦近似值
    // 每度纬度和经度在荷兰地区大约对应的millmeters
    static constexpr double millmetersPerLat = .11113292;
    static constexpr double millmetersPerLon = .11141284 * 0.61566; // 使用 52° 的余弦近似值

    // 每度纬度和经度在荷兰地区大约对应的relative millmeters
    // 52.04675216210598 为 平均纬度的余弦值对应2020_10_v2
    static constexpr double relativeMillmetersPerLat = 11.113292;
//    static constexpr double relativeMillmetersPerLon = 11.141284 * 0.61566; // 使用 52° 的余弦近似值
    static constexpr double relativeMillmetersPerLon = 11.141284 * 0.6150182700828235; // 使用 52.04675216210598° 的余弦近似值

    [[nodiscard]] static double toRadians(double degree) {
        return degree * (M_PI / 180.0);
    }

    class MapNode {
    private:
        // pointID;tripID;xGPS;yGPS;UnixTime;heading;speed;accuracy
        std::size_t pointID;
        std::size_t tripID;
        std::string latitude;
        std::string longitude;
        std::size_t UnixTime;
        int heading;
        double speed;
        double accuracy;
        double latRad, lonRad;


    public:
        MapNode() = delete;

        MapNode(std::size_t id, std::size_t tripID, std::string lat, std::string lon, std::size_t UnixTime, int heading, double speed, double accuracy)
                : pointID(id), tripID(tripID), latitude(std::move(lat)), longitude(std::move(lon)),
                  UnixTime(UnixTime), heading(heading), speed(speed), accuracy(accuracy) {
            latRad = toRadians(std::stod(this->latitude));
            lonRad = toRadians(std::stod(this->longitude));
        }

        MapNode(const std::string& line) {
            std::istringstream ss(line);
//            std::cout << "line: " << line << std::endl;
            std::string token;
            std::getline(ss, token, ';');
            this->pointID = std::stoul(token);
            std::getline(ss, token, ';');
            this->tripID = std::stoul(token);
            std::getline(ss, token, ';');
            this->latitude = token;
            std::getline(ss, token, ';');
            this->longitude = token;
            std::getline(ss, token, ';');
            this->UnixTime = std::stoul(token);
            std::getline(ss, token, ';');
            // if NaN, set heading to -1
            if (token == "NaN") [[unlikely]] this->heading = -1;
            else this->heading = std::stoi(token);
            std::getline(ss, token, ';');
            this->speed = std::stod(token);
            std::getline(ss, token, ';');
            this->accuracy = std::stod(token);
            latRad = toRadians(std::stod(this->latitude));
            lonRad = toRadians(std::stod(this->longitude));
        }

        // Getters
        [[nodiscard]] std::size_t getID() const { return this->pointID; }
        [[nodiscard]] std::size_t getTripID() const { return this->tripID; }
        [[nodiscard]] double getLat() const { return std::stod(this->latitude); }
        [[nodiscard]] double getLon() const { return std::stod(this->longitude); }
        [[nodiscard]] std::string getLatString() const { return this->latitude; }
        [[nodiscard]] std::string getLonString() const { return this->longitude; }
        [[nodiscard]] std::size_t getUnixTime() const { return this->UnixTime; }
        [[nodiscard]] int getHeading() const { return this->heading; }
        [[nodiscard]] double getSpeed() const { return this->speed; }
        [[nodiscard]] double getAccuracy() const { return this->accuracy; }
        [[nodiscard]] double getLatRad() const { return this->latRad; }
        [[nodiscard]] double getLonRad() const { return this->lonRad; }

        // Distance calculation methods remain the same, using latRad and lonRad for calculations

        [[nodiscard]] double approximateDistance(const MapNode& other) const {
            double dLat = other.latRad - latRad;
            double dLon = other.lonRad - lonRad;
            double latMid = (latRad + other.latRad) / 2;

            return dLat * dLat + std::cos(latMid) * std::cos(latMid) * dLon * dLon;
        }

        [[nodiscard]] double approximateDistanceNL(const MapNode& other) const {
            // sqrt dLatMeters * dLatMeters + dLonMeters * dLonMeters then time it 10 to get kilometers
            // thus if distance is 100 meters(0.1km), the result is 0.0001
            double dLatMeters = (other.latRad - latRad) * StreetMatch::relativeMillmetersPerLat;
            double dLonMeters = (other.lonRad - lonRad) * StreetMatch::relativeMillmetersPerLon;

            return dLatMeters * dLatMeters + dLonMeters * dLonMeters;
        }

        // haversine distance
        [[nodiscard]] double haversineDistance(const MapNode& other) const {
            double dLat = other.latRad - latRad;
            double dLon = other.lonRad - lonRad;
            double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
                       std::cos(latRad) * std::cos(other.latRad) *
                       std::sin(dLon / 2) * std::sin(dLon / 2);
            double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
            return 6371 * c; // 6371 is the radius of the Earth in kilometers
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
            findNearestNeighbor(root, queryPoint, 0, nearest, nearestDist);
            if (nearest.has_value()) {
                return nearest.value();
            } else {
                throw std::runtime_error("No nearest neighbor found");
            }
        }

        std::vector<MapNode> nearestNeighbors(const MapNode & queryPoint, int k) {
            // k stands for the number of nearest neighbors to find
            std::vector<std::pair<std::optional<MapNode>, double>> nearests( k, std::make_pair(std::nullopt, std::numeric_limits<double>::max()) );
            findNearestNeighbors(root, queryPoint, 0, nearests);

            std::vector<MapNode> nearest;
            std::transform(nearests.begin(), nearests.end(), std::back_inserter(nearest), [](const auto& n) {
                if (n.first.has_value()) {
                    return n.first.value();
                } else {
                    throw std::runtime_error("No matching nearest neighbor found");
                }
            });
            return nearest;
        }

//        std::vector<std::pair<MapNode, double>>  nearestNeighborsWithinRadius(const MapNode & queryPoint, double radius) {
//            // radius stands for the radius in kilometer within which to find the nearest neighbors
//            // radius : 0.10 -> 0.10 * 0.10 / 100 = 0.0001
//            double radiusReshaped = radius * radius / 100;
//            std::vector<std::pair<std::optional<MapNode>, double>> nearests;
//            findNearestNeighborsWithinR(root, queryPoint, 0, nearests, radiusReshaped);
//            std::vector<std::pair<MapNode, double>> nearest;
//            std::transform(nearests.begin(), nearests.end(), std::back_inserter(nearest), [](const auto& n) {
//                if (n.first.has_value()) {
//                    return std::make_pair(n.first.value(), std::sqrt(n.second) * 10);
//                } else {
//                    throw std::runtime_error("No matching nearest neighbor found");
//                }
//            });
//            return nearest;
//        }

    private:
        std::unique_ptr<KDTreeNode> root;

        void insertOne(std::unique_ptr<KDTreeNode>& node, const MapNode & point, unsigned depth) {
            if (!node) {
                node = std::make_unique<KDTreeNode>(point);
                return;
            } else {
                unsigned axis = depth % 2;
                if ((axis == 0 && point.getLat() < node->point.getLat()) || (axis == 1 && point.getLon() < node->point.getLon())) {
                    insertOne(node->left, point, depth + 1);
                } else {
                    insertOne(node->right, point, depth + 1);
                }
            }
        }

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

        void findNearestNeighbors(std::unique_ptr<KDTreeNode>& node, const MapNode & queryPoint, unsigned depth, std::vector<std::pair<std::optional<MapNode>, double>>& nearests) {
            if (!node) return;

            double dist = node->point.approximateDistanceNL(queryPoint);
            for (auto& n : nearests) {
                if (dist < n.second) {
                    n.second = dist;
                    n.first = node->point;
                    break;
                }
            }

            unsigned axis = depth % 2;
            std::unique_ptr<KDTreeNode>& first = (axis == 0 && queryPoint.getLat() < node->point.getLat()) || (axis == 1 && queryPoint.getLon() < node->point.getLon()) ? node->left : node->right;
            std::unique_ptr<KDTreeNode>& second = first == node->left ? node->right : node->left;

            findNearestNeighbors(first, queryPoint, depth + 1, nearests);

            double axisDist = axis == 0 ? queryPoint.getLat() - node->point.getLat() : queryPoint.getLon() - node->point.getLon();
            if (axisDist * axisDist < nearests.back().second) {
                findNearestNeighbors(second, queryPoint, depth + 1, nearests);
            }
        }

//        // return all points within radius
//        void findNearestNeighborsWithinR(std::unique_ptr<KDTreeNode>& node, const MapNode & queryPoint, unsigned depth, std::vector<std::pair<std::optional<MapNode>, double>>& nearests, double radius) {
//            if (!node) return;
//
//            double dist = node->point.approximateDistanceNL(queryPoint);
//            if (dist < radius) {
//                nearests.emplace_back(std::make_pair(node->point, dist));
//            }
//
//            unsigned axis = depth % 2;
//            std::unique_ptr<KDTreeNode>& first = (axis == 0 && queryPoint.getLat() < node->point.getLat()) || (axis == 1 && queryPoint.getLon() < node->point.getLon()) ? node->left : node->right;
//            std::unique_ptr<KDTreeNode>& second = first == node->left ? node->right : node->left;
//
//            findNearestNeighborsWithinR(first, queryPoint, depth + 1, nearests, radius);
//
//            double axisDist = axis == 0 ? queryPoint.getLat() - node->point.getLat() : queryPoint.getLon() - node->point.getLon();
//            if (axisDist * axisDist < radius) {
//                findNearestNeighborsWithinR(second, queryPoint, depth + 1, nearests, radius);
//            }
//        }

    };

    class Trip {
    private:
        std::vector<StreetMatch::MapNode> tripNodes{};
    public:
        explicit Trip() = default;

        [[nodiscard]] int addNodes (StreetMatch::MapNode newNode) {
            if (!this->tripNodes.empty()) {
                if (this->tripNodes.front().getTripID() == newNode.getTripID()) {
                    this->tripNodes.emplace_back(newNode);
                    return 1;
                }
            } else {
                this->tripNodes.emplace_back(newNode);
                return 1;
            }
            return 0;
        }

        [[nodiscard]] std::size_t getTripID() const {
            if (!this->tripNodes.empty()) return this->tripNodes.front().getTripID();
            return 0;
        }
        [[nodiscard]] std::size_t getTripSize() const { return this->tripNodes.size(); }
        [[nodiscard]] const std::vector<StreetMatch::MapNode>& getAllNodes() const { return this->tripNodes; }
        [[nodiscard]] const StreetMatch::MapNode& getNode(std::size_t nodeID) const {
//            // binary search
//            std::size_t low = 0;
//            std::size_t high = this->tripNodes.size() - 1;
//            while (low <= high) {
//                std::size_t mid = (low + high) / 2;
//                if (this->tripNodes.at(mid).getID() < nodeID) {
//                    low = mid + 1;
//                } else if (this->tripNodes.at(mid).getID() > nodeID) {
//                    high = mid - 1;
//                } else {
//                    return this->tripNodes.at(mid);
//                }
//            }
//            // throw std::runtime_error(std::format("Node {} not found in trip {}", nodeID, this->tripNodes.front().getTripID()));
//            throw std::runtime_error("Node " + std::to_string(nodeID) + " not found in trip " + std::to_string(this->tripNodes.front().getTripID()));
            return this->tripNodes.at(nodeID-this->tripNodes.front().getID()); // continuously increasing nodeID
        }
    };

    class TripList {
    private:
        std::vector<StreetMatch::Trip> Trips;
//        std::vector<std::size_t> offsets;
        std::vector<std::size_t> nodeBelongings;
    public:
        explicit TripList() = delete;
//        explicit TripList(std::vector<StreetMatch::Trip>&& trips, std::vector<std::size_t>&& offsets) : Trips(std::move(trips)), offsets(std::move(offsets)) {}
        explicit TripList(std::vector<StreetMatch::Trip>&& trips, std::vector<std::size_t> belongings) : Trips(std::move(trips)), nodeBelongings(std::move(belongings)){}
        [[nodiscard]] std::size_t getTripListSize() const { return this->Trips.size(); }
//        [[nodiscard]] std::size_t getOffset(std::size_t index) const { return this->offsets.at(index); }
        [[nodiscard]] const std::vector<StreetMatch::Trip>& getAllTrips() const { return this->Trips; }
        [[nodiscard]] const StreetMatch::Trip& getTrip(std::size_t tripID) const {
            // tripID starts from 1 but vector index starts from 0
            if (tripID > this->Trips.size()) [[unlikely]] throw std::runtime_error("Trip with ID " + std::to_string(tripID) + " not found");
            if (this->Trips.at(tripID - 1).getTripID() == tripID) [[likely]] return this->Trips.at(tripID - 1);
            else throw std::runtime_error("Trip with ID " + std::to_string(tripID) + " not found, but found " + std::to_string(this->Trips.at(tripID - 1).getTripID()) + " instead");
        }
        [[nodiscard]] std::size_t getNodeBelonging(std::size_t nodeID) const {
            return this->nodeBelongings.at(nodeID);
        }
        [[nodiscard]] const StreetMatch::MapNode& getNode(std::size_t nodeID, std::size_t tripID) const {
            return this->getTrip(tripID).getNode(nodeID);
        }
        [[nodiscard]] const StreetMatch::MapNode& getNode(std::size_t nodeID) const {
            return this->getTrip(this->nodeBelongings.at(nodeID)).getNode(nodeID);
        }
    };

    struct MapNodeNbr {
        std::size_t nodeID;
        std::size_t tripID;
        std::vector<std::pair<MapNode, double>> neighbors;
    };

} // StreetMatch

#endif //STREETMATCH_MAPNODE_HPP


