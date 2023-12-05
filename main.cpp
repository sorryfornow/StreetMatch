// Created by Siqing Zhang on 26/11/2023.

#include "MapNode.hpp"
//#include "osmium.h"
//#include <boost/property_tree/ptree.hpp>
//#include <boost/property_tree/xml_parser.hpp>

/**
 * Parse the map data from OSM file to a txt file first.
 * The txt file will be used to build the KDTree.
 * The txt file will be in the format of:
 * node_id, latitude, longitude, street_count
**/

static constexpr std::string_view mapPath = "../Data/maps/NL2.osm";
//static constexpr std::string_view queryPath = "../Data/query/DataFile_2020_10_01_clean.csv";
static constexpr std::string_view queryPath = "../Data/query/query_test.csv";
static constexpr std::string_view outputPath = "../Data/maps/output_file.txt";
static constexpr std::string_view resultPath = "../Data/result/result_test.csv";

void fileRead(){
    const auto startTime = std::chrono::high_resolution_clock::now();
    static std::ifstream file(mapPath);
    if (!file.is_open()) {
        std::cerr << "error open\n" << std::endl;
    }

    std::ofstream outputFile(outputPath.data());
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

    std::cout << "Running time: " << elapsed.count()/1000 << " seconds." << std::endl;
}

//void boostParsing() {
//    boost::property_tree::ptree pt;
//    try {
//        // loading XML
//        boost::property_tree::read_xml(std::string(mapPath), pt);
//
//        // check if root is graph node
//        if (pt.get_child_optional("graph")) {
//            // traverse all nodes
//            for (const auto& node : pt.get_child("graph")) {
//                if (node.first == "node") { // check if node
//                    auto id = node.second.get<std::string>("<xmlattr>.id");
//                    auto latitude = node.second.get<std::string>("data[key='d4']");
//                    auto longitude = node.second.get<std::string>("data[key='d5']");
//                    auto tripId = node.second.get<std::string>("data[key='d6']");
//
//                    std::cout << "id = " << id << std::endl;
//                    std::cout << "latitude = " << latitude << std::endl;
//                    std::cout << "longitude = " << longitude << std::endl;
//                    std::cout << "trip_id = " << tripId << std::endl << std::endl;
//                }
//            }
//        } else {
//            std::cerr << "Invalid XML structure: root element is not 'graph'." << std::endl;
//        }
//    } catch (const boost::property_tree::xml_parser::xml_parser_error& e) {
//        std::cerr << "XML Parsing Error: " << e.what() << std::endl;
//    }
//
//}

//void parsingMap() {
//    osmium::io::File mapFile(mapPath.data());
//    osmium::io::Reader mapReader(mapFile, osmium::osm_entity_bits::node);
//
//    StreetMatch::NodeHandler handler;
//    osmium::apply(mapReader, handler);
//
//    mapReader.close();
//}

std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

bool endsWith(std::string_view str, std::string_view suffix) {
    return str.size() >= suffix.size() &&
           str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

void matchQuery() {
    const auto startTime = std::chrono::high_resolution_clock::now();
    // Load map data from a file
    std::vector<StreetMatch::MapNode> mapData;
    std::ifstream file(outputPath.data());
    std::string line;

    while (std::getline(file, line)) {
        auto tokens = split(line, ','); // comma as delimiter
        if (tokens.size() >= 4) [[likely]]{
            try {
                long id = std::stol(tokens[0]);
                double lat = std::stod(tokens[1]);
                double lon = std::stod(tokens[2]);
                int street_count = std::stoi(tokens[3]);
                mapData.emplace_back(id, lat, lon, street_count);
            } catch (const std::invalid_argument& e) {
                std::cout << tokens[0] << std::endl;
                std::cerr << "Error parsing map data: " << e.what() << std::endl;
            }
        }
    }

    // Construct KD-Tree with map data
    StreetMatch::KDTree tree;
    for (const auto& node : mapData) {
        tree.insert(node);
    }
    file.close();

    // Load query data from a file
    std::vector<StreetMatch::MapNode> queryData;
    std::ifstream qfile(queryPath.data());
    line.clear();
    std::size_t queryCount = 0;
    while (std::getline(qfile, line)) {
        auto tokens = split(line, ';'); // Semicolon as delimiter
        if (tokens.size() >= 3) {
            double lat = std::stod(tokens[0]);
            double lon = std::stod(tokens[1]);
            int sCount = std::stoi(tokens[2]);
            queryData.emplace_back(0, lat, lon, sCount);
            queryCount++;
        }
    }

    std::ofstream resultFile(resultPath.data());  // if not exist, create a new file
    // if resultPath is .csv file, add header
    if (endsWith(resultPath, ".csv")) [[likely]]
        resultFile << "latitude,longitude,trip_id,nearest_node,nearest_node_latitude,nearest_node_longitude" << std::endl;

    // For each query point, find the nearest neighbor in the map data
    for (const auto& queryPoint : queryData) {
        StreetMatch::MapNode nearest = tree.nearestNeighbor(queryPoint);
//        std::cout << queryPoint.getLat() << ',' << queryPoint.getLon() << ',' << queryPoint.getStreetCount()
//                  << ',' << nearest.getId() << ',' << nearest.getLat() << ',' << nearest.getLon()
//                  << std::endl;
//        resultFile << std::fixed << std::setprecision(12) << queryPoint.getLat() << ',' << queryPoint.getLon() << ',' << queryPoint.getStreetCount()
//                   << ',' << nearest.getId() << ',' << nearest.getLat() << ',' << nearest.getLon()
//                   << std::endl;
        resultFile << std::to_string(queryPoint.getLat()) << ',' << std::to_string(queryPoint.getLon()) << ',' << std::to_string(queryPoint.getStreetCount())
                   << ',' << nearest.getId() << ',' << std::to_string(nearest.getLat()) << ',' << std::to_string(nearest.getLon())
                   << std::endl;
//            std::cout << "Nearest to (" << queryPoint.getLat() << ", " << queryPoint.getLon() << ") is Node ID "
//                      << nearest.getId() << std::endl;
    }
    qfile.close();
    const auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = endTime - startTime; // time diff

    std::cout << "Running time: " << elapsed.count()/1000 << " seconds." << std::endl;
}

[[maybe_unused]]void getLength(){
    // count the length of the mapFile
    std::ifstream file(outputPath.data());
    std::string line;
    int count = 0;
    while (std::getline(file, line)) {
        count++;
    }
    std::cout << "The length of the mapFile is: " << count << std::endl;
}

int main() {
    try {
//         fileRead();
         matchQuery();
//        getLength();
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in main: " << e.what() << std::endl;
    }

    return 0;
}
