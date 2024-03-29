//
// Created by Siqing Zhang on 26/11/2023.
//
#ifndef STREETMATCH_OSMIUM_H
#define STREETMATCH_OSMIUM_H

#endif //STREETMATCH_OSMIUM_H

#include <osmium/osm.hpp>
#include <osmium/io/reader.hpp>
#include <osmium/io/writer.hpp>
#include <osmium/handler.hpp>
#include <osmium/index/map/all.hpp>
#include <osmium/visitor.hpp>
#include <osmium/geom/geojson.hpp>
#include <osmium/geom/wkb.hpp>
#include <osmium/geom/wkt.hpp>
#include <osmium/geom/geojson.hpp>
#include <osmium/geom/mercator_projection.hpp>
#include <osmium/geom/coordinates.hpp>
#include <osmium/geom/factory.hpp>
#include <osmium/geom/relations.hpp>
#include <osmium/io/any_input.hpp>
#include <osmium/io/any_output.hpp>
#include <osmium/io/any_compression.hpp>

#include <string>
#include <regex>
#include <sstream>
#include <fstream>

//#include "/opt/homebrew/include/tinyxml2.h"
//#include <tinyxml2.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/graphml.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>