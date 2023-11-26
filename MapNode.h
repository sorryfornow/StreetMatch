//
// Created by Siqing Zhang on 26/11/2023.
//

#ifndef STREETMATCH_MAPNODE_H
#define STREETMATCH_MAPNODE_H

#include "osmium.h"

namespace StreetMatch {

    class MapNode {
    private:
        osmium::object_id_type id;
        double lon;
        double lat;
    public:
        MapNode() = delete;

        MapNode(const MapNode &other) = default;

        MapNode &operator=(const MapNode &other) = default;

        MapNode(MapNode &&other) = default;

        MapNode &operator=(MapNode &&other) = default;

        ~MapNode() = default;

        explicit MapNode(const osmium::Node &node) : id(node.id()) {
            this->lon = node.location().lon();
            this->lat = node.location().lat();
        }

        explicit MapNode(const osmium::object_id_type &id, const double &lon, const double &lat)
        : id(id), lon(lon), lat(lat) {}

        [[nodiscard]] osmium::object_id_type getId() const {
            return this->id;
        }

        [[nodiscard]] double getLon() const {
            return this->lon;
        }

        [[nodiscard]] double getLat() const {
            return this->lat;
        }
    };

} // StreetMatch

#endif //STREETMATCH_MAPNODE_H
