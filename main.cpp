// Created by Siqing Zhang on 26/11/2023.

#include "MapNode.hpp"
//#include "osmium.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

/**
 * Parse the map data from OSM file to a txt file first.
 * The txt file will be used to build the KDTree.
 * The txt file will be in the format of:
 * node_id, latitude, longitude, street_count
**/


static constexpr std::string_view mapPath = "../Data/maps/NL2.osm";

void fileRead(){
    const auto startTime = std::chrono::high_resolution_clock::now();
    static std::ifstream file(mapPath);
    if (!file.is_open()) {
        std::cerr << "error open\n" << std::endl;
    }

    std::ofstream outputFile("../Data/maps/output_file.txt");
    // if not exist, create a new file

    std::string line;
    std::regex nodeStartPattern("<node id=\"([^\"]+)\">");
    std::regex nodeEndPattern("</node>");
    std::regex dataPattern("<data key=\"d([456])\">([^<]+)</data>");
    std::smatch matches;

    std::string nodeId, latitude, longitude, tripId;
    bool inNode = false;

    while (std::getline(file, line)) {
        if (std::regex_search(line, matches, nodeStartPattern)) {
            // start a new node
            nodeId = matches[1];
            latitude = longitude = tripId = ""; // reset value
            inNode = true;
        } else if (inNode && std::regex_search(line, matches, dataPattern)) {
            // parsing node data
            switch (std::stoi(matches[1])) {
                case 4: latitude = matches[2]; break;
                case 5: longitude = matches[2]; break;
                case 6: tripId = matches[2]; break;
            }
        } else if (std::regex_search(line, nodeEndPattern)) {
            // end current node and restore data
//             std::cout << "id = " << nodeId << std::endl;
//             std::cout << "latitude = " << latitude << std::endl;
//             std::cout << "longitude = " << longitude << std::endl;
//             std::cout << "trip_id = " << tripId << std::endl << std::endl;
            // open A New File To Write The Data
            outputFile << nodeId << "," << latitude << "," << longitude << "," << tripId << std::endl;
            inNode = false;
        }
    }

    file.close();
    const auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = endTime - startTime; // time diff

    std::cout << "Runing time: " << elapsed.count()/1000 << " seconds." << std::endl;
}

void boostParsing() {
    boost::property_tree::ptree pt;
    try {
        // loading XML
        boost::property_tree::read_xml(std::string(mapPath), pt);

        // check if root is graph node
        if (pt.get_child_optional("graph")) {
            // traverse all nodes
            for (const auto& node : pt.get_child("graph")) {
                if (node.first == "node") { // check if node
                    auto id = node.second.get<std::string>("<xmlattr>.id");
                    auto latitude = node.second.get<std::string>("data[key='d4']");
                    auto longitude = node.second.get<std::string>("data[key='d5']");
                    auto tripId = node.second.get<std::string>("data[key='d6']");

                    std::cout << "id = " << id << std::endl;
                    std::cout << "latitude = " << latitude << std::endl;
                    std::cout << "longitude = " << longitude << std::endl;
                    std::cout << "trip_id = " << tripId << std::endl << std::endl;
                }
            }
        } else {
            std::cerr << "Invalid XML structure: root element is not 'graph'." << std::endl;
        }
    } catch (const boost::property_tree::xml_parser::xml_parser_error& e) {
        std::cerr << "XML Parsing Error: " << e.what() << std::endl;
    }

}

//void parsingMap() {
//    osmium::io::File mapFile(mapPath.data());
//    osmium::io::Reader mapReader(mapFile, osmium::osm_entity_bits::node);
//
//    StreetMatch::NodeHandler handler;
//    osmium::apply(mapReader, handler);
//
//    mapReader.close();
//}



int main() {
    try {
//        fileRead();

    } catch (const std::exception& e) {
        std::cerr << "Exception caught in main: " << e.what() << std::endl;
    }


    return 0;
}
