// Created by Siqing Zhang on 26/11/2023.

#include "MapNode.hpp"
#include "Paths.h"
//#include "osmium.h"
//#include <boost/property_tree/ptree.hpp>
//#include <boost/property_tree/xml_parser.hpp>

/**
 * Parse the map data from OSM file to a txt file first.
 * The txt file will be used to build the KDTree.
 * The txt file will be in the format of:
 * node_id, latitude, longitude, street_count
**/


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

    /** KDTree initialization **/
    // Load map data from a file
    std::vector<StreetMatch::MapNode> mapData;
    std::ifstream file(outputPath.data());
    std::string line;

    while (std::getline(file, line)) {
        auto tokens = split(line, ','); // comma as delimiter
//        if (tokens.size() >= 4) [[likely]] {
//            try {
//                long id = std::stol(tokens[0]);
//                std::string lat = tokens[1];
//                std::string lon = tokens[2];
//                int street_count = std::stoi(tokens[3]);
//                mapData.emplace_back(id, lat, lon, street_count);
//            } catch (const std::invalid_argument& e) {
//                std::cout << tokens[0] << std::endl;
//                std::cerr << "Error parsing map data: " << e.what() << std::endl;
//            }
//        }
        if (tokens.size() >= 8) [[likely]] {
            try {
                long id = std::stol(tokens[0]);
                int tripID = std::stoi(tokens[3]);
                std::string lat = tokens[1];
                std::string lon = tokens[2];
                long UnixTime = std::stol(tokens[4]);
                int heading = std::stoi(tokens[5]);
                double speed = std::stod(tokens[6]);
                double accuracy = std::stod(tokens[7]);
                mapData.emplace_back(id, tripID, lat, lon, UnixTime, heading, speed, accuracy);
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

    /** KDTree now built **/

    // Load query data from a file
    std::vector<StreetMatch::MapNode> queryData;
    std::ifstream qfile(queryPath.data());
    line.clear();

    while (std::getline(qfile, line)) {
        auto tokens = split(line, ';'); // Semicolon as delimiter
//        if (tokens.size() >= 3) {
//            std::string lat = tokens[0];
//            std::string lon = tokens[1];
//            int sCount = std::stoi(tokens[2]);
//            queryData.emplace_back(0, lat, lon, sCount);
//
//        }
        if (tokens.size() >= 3) [[likely]] {
            std::string lat = tokens[0];
            std::string lon = tokens[1];
            int tripID = std::stoi(tokens[2]);
            // Defaulting UnixTime, heading, speed, and accuracy to 0, 0, 0.0, and 0.0 respectively
            long UnixTime = 0;
            int heading = 0;
            double speed = 0.0;
            double accuracy = 0.0;

            queryData.emplace_back(0, tripID, lat, lon, UnixTime, heading, speed, accuracy);
        }
    }

    std::cout<< "The size of the queryData is: " << queryData.size() << std::endl;
    const auto startTime = std::chrono::high_resolution_clock::now();
    std::ofstream resultFile(resultPath.data());  // if not exist, create a new file
    // if resultPath is .csv file, add header
    if (endsWith(resultPath, ".csv")) [[likely]]
        resultFile << "latitude,longitude,trip_id,nearest_node,nearest_node_latitude,nearest_node_longitude" << std::endl;

    // For each query point, find the nearest neighbor in the map data
    [[maybe_unused]] std::size_t queryCount = 0;
    for (const auto& queryPoint : queryData) {
        StreetMatch::MapNode nearest = tree.nearestNeighbor(queryPoint);

//        resultFile << std::fixed << std::setprecision(12) << queryPoint.getLat() << ',' << queryPoint.getLon() << ',' << queryPoint.getStreetCount()
//                   << ',' << nearest.getId() << ',' << nearest.getLat() << ',' << nearest.getLon()
//                   << std::endl;
//        resultFile << queryPoint.getLatString() << ',' << queryPoint.getLonString() << ',' << queryPoint.getStreetCount()
//                   << ',' << nearest.getID() << ',' << nearest.getLatString() << ',' << nearest.getLonString()
//                   << std::endl;
        resultFile << queryPoint.getLatString() << ',' << queryPoint.getLonString() << ',' << queryPoint.getTripID()
                   << ',' << nearest.getID() << ',' << nearest.getTripID() << ',' << nearest.getLatString() << ',' << nearest.getLonString()
//                   << nearest.getUnixTime() << ',' << nearest.getHeading() << ',' << nearest.getSpeed() << ',' << nearest.getAccuracy()
                   << std::endl;
//            std::cout << "Nearest to (" << queryPoint.getLat() << ", " << queryPoint.getLon() << ") is Node ID "
//                      << nearest.getId() << std::endl;
//        queryCount++;
//        if (queryCount % 1000 == 0){
//            const auto curTime = std::chrono::high_resolution_clock::now();
//            std::chrono::duration<double, std::milli> elapsed = curTime - startTime; // time diff
//            std::cout << "Processed " << queryCount << " queries. Time elapsed: " << elapsed.count()/1000 << " seconds." << std::endl;
//        }

    }
    qfile.close();
    const auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = endTime - startTime; // time diff

    std::cout << "Running time: " << elapsed.count()/1000 << " seconds." << std::endl;
}

[[maybe_unused]]void getLength() {
    // count the length of the mapFile
    std::ifstream file(outputPath.data());
    std::string line;
    std::size_t count = 0;
    while (std::getline(file, line)) {
        count++;
    }
    std::cout << "The length of the mapFile is: " << count << std::endl;
}


/**
 * Test for zuid_holland dataset as DataFile_2020_10_v2.csv in Data_New folder
 * The file contains the following columns:
 * tripID;xGPS;yGPS;UnixTime;heading;speed;accuracy
 **/

void zuid_holland_test(const std::string candidateNUM_str, const std::string searchRadius_str, const std::string_view zuid_holland_path) {
    // file loading
    const std::size_t candidateNUM = std::stoi(candidateNUM_str);
    const double searchRadius = std::stod(searchRadius_str);
    std::ifstream file(zuid_holland_path.data());
    std::string line;
    std::getline(file,line);

    if (line != "pointID;tripID;xGPS;yGPS;UnixTime;heading;speed;accuracy")[[unlikely]] throw std::runtime_error("error input file format or header!");
    /** IMPORTANT: require tripID starts from 1, nodeID starts from 1
     * and all tripID and node ID are naturally sorted in ascending ordering and continuous.
     * That if trip A with id a, trip B with id b with a<b,
     * then all nodeIDs in trip B bigger than any node in trip A.
     * **/

    StreetMatch::KDTree tree;
    auto allTrips = std::make_unique<std::vector<StreetMatch::Trip>>();
//    auto offsetTrip = std::make_unique<std::vector<std::size_t>>(); // containing the first PPoint of a trip
    auto belongings = std::make_unique<std::vector<std::size_t>>(); // containing the tripID of each point
    belongings->emplace_back(0); // dummy node

    std::size_t line_cnt = 0;
    std::size_t trip_cnt = 0;

    while(std::getline(file, line)) {
        StreetMatch::MapNode new_point(line);
        tree.insert(new_point);
        belongings->emplace_back(new_point.getTripID());

        if (new_point.getTripID() != trip_cnt) {
            trip_cnt = new_point.getTripID();
            allTrips->emplace_back(StreetMatch::Trip());
//            offsetTrip->emplace_back(new_point.getID());

            if (!allTrips->back().addNodes(new_point)) [[unlikely]] {
                std::cerr << "Error adding new point (new trip): " << new_point.getID() << std::endl;
            }
        } else {
            if (!allTrips->back().addNodes(new_point)) [[unlikely]] {
                std::cerr << "Error adding new point (existed trip): " << new_point.getID() << std::endl;
            }
        }

        line_cnt++;

    }

    std::cout << "The size of the allTrips is: " << allTrips->size() << " or trip_cnt: " << trip_cnt << std::endl;
    std::cout << "The size of all nodes is: " << line_cnt << std::endl;

    auto tripList = StreetMatch::TripList(std::move(*allTrips), std::move(*belongings));

//    // nbr list for each node in the map nodeId -> list of nbr(trip_id, node_id, distance)
//    auto nbrList = std::make_unique<std::vector<std::vector<std::tuple<std::size_t,std::size_t, double>>>>(line_cnt+1);
//    // scan tripList to find the nearest neighbors for each node
//    for (auto& trip: tripList.getAllTrips()) {
//        for (auto& node: trip.getAllNodes()) {
//            auto nearest = tree.nearestNeighborsWithinRadius(node, searchRadius);
//            for (auto& nbr: nearest) {
//                if (nbr.first.getID() != node.getID()) {
//                    nbrList->at(node.getID()).emplace_back(std::make_tuple(nbr.first.getTripID(), nbr.first.getID(), nbr.second));
//                }
//            }
//        }
//    }

    // write the nbrList to a file
    // output as tripID, nodeID, size of nbr, list of nearestNbrs [(tripID, nodeID, distance)]
    std::ofstream nbrFile("../Data/Data_New/nbrList-" + std::to_string(candidateNUM) +"-r"+ searchRadius_str + ".txt");
    nbrFile << "tripID;nodeID;nbrSize;nearestNbrLst[(tripID, nodeID, distance)]" << std::endl;
    for (const auto& trip : tripList.getAllTrips()) {
//        std::cout << "Processing trip: " << trip.getTripID() << std::endl;
        for (const auto& node : trip.getAllNodes()) {
            auto nearest = tree.nearestNeighbors(node, candidateNUM+1); // +1 to exclude itself
            std::ostringstream lineStream; // 创建一个ostringstream对象用于构建输出行
            std::size_t cnt_size = 0;   // 计算size（排除自己）
            for (const auto& nbr : nearest) {
                double dist = node.haversineDistance(nbr);
                if (nbr.getID() != node.getID() && dist < searchRadius ) { // 排除自己作为最近邻居的情况 和 超过阈值的情况
                    lineStream << nbr.getTripID() << "," << nbr.getID() << "," << dist << ";";
                    ++cnt_size;
                }
            }
            nbrFile << trip.getTripID() << ";" << node.getID() << ";" << cnt_size << ";";
            // cout warning
            if (cnt_size == candidateNUM) std::cout << "Warning: " << trip.getTripID() << " " << node.getID() << " has " << cnt_size << " neighbors;" << std::endl;
            if (!cnt_size) std::cout << "Warning: " << trip.getTripID() << " " << node.getID() << " has no neighbors;" << std::endl;

            if (cnt_size) [[likely]] nbrFile << lineStream.str() << std::endl; // 将构建的字符串写入文件
//            std::cout << "Processed node: " << node.getID() << " with " << cnt_size << " neighbors;" << std::endl;
        }
    }

    file.close();
}



// 读入nbrFile 然后计算实际距离

// data shape:
//pointID,tripID,xGPS,yGPS,UnixTime,heading,speed,accuracy
//1,1;51.981735;5.903015;1601596727;8;0;11.4
//2,1;51.983225;5.9037533;1601596742;59;36.2;9.5
//3,1;51.9834167;5.9094583;1601596772;77;42.7;9.5
//4,2;52.3547133;4.7849183;1601596684;56;0;7.7
//5,2;52.3554967;4.78842;1601596713;67;29.6;4.6
//6,2;52.355635;4.7886333;1601596719;3;17.2;6.1
//7,2;52.3558083;4.78871;1601596722;61;24.9;4.6
//8,3;51.8464617;5.862505;1601596137;353;0;13.6
//9,3;51.8463333;5.8624967;1601596167;197;2.4;2.1
//10,3;51.846255;5.8624833;1601596197;159;1.2;9.5
//11,3;51.846245;5.86243;1601596227;159;0;3.3
//12,3;51.8463183;5.862365;1601596258;351;2.4;11.4

// step1
// 对于一段trip中的每个 point 而言，通过kd-tree找到最近的 points 点的集合（阈值使用绝对距离）
// 收集这些点的trip id，然后对这些trip id进行统计
// point_id, latitude, longitude, trip_id -> point_id, trip_id, list of nearest [(point_id,trip_id)]
// 对于该trip中 某一段连续的nodes，如果这些nodes对应的临近trip id中有一些trip id的数量超过一定的阈值，那么这些trip id可以被分为一组
// 截断trip形成route可以用heading，speed，或者是slope来判断
// 或者通过路口 （即包含相对更多个trip id的point） 来判断

// step2
// 在这个组里 不仅包含这些trip id（和每个trip中的特定 存在于这条道路的点集合） 还包含这些trip id的点集合 将其看作一段道路route，
// 即 每个道路中包含走了这条道路的一些trip id，以及这些trip id所代表的trip中，经过这条道路的所有点的集合
// 然后 现在 每个trip就变成了 route1，route2，route3，...，routeN 这样的一些route
// （先用坐标点匹配到route，再压缩route，比如 1，1，2，2，2，3，3压缩为1，2，3）

// step3
// 同时 建立新的graph，该graph中每个node表示一条道路，每个node的属性包括这条道路上的所有点的集合，以及这些点所代表的trip id，
// 通过trip中相邻的route来联通不同的node，即不同的道路，这样就可以找到相邻的道路
// 在graph中每个node的属性中，还可以包括这条道路的长度，以及这条道路的中心点的坐标，这样就可以通过这个graph来找到最近的道路
// 类似于 pagerank，在选择最近的道路的时候，可以通过选择trip id的数量来选择最优的道路，即选择trip id数量最多的道路，
// 考虑到单行道的存在，这种下一个node的挑选是单向的（在寻找route的时候可以是也最好是双向的）

// 查询：对于给定的输入query坐标，找到最近的node point 然后找到这个node point代表的道路，然后用dijkstra算法找到最近的路径
// （考虑pagerank优化）

// step1，2可以用spark或者mapreduce来实现，step3可以用c++实现

void nbrAnalysis(const std::string_view nbrPath){
    // get nbr size and radius from the file name
    std::string_view candidateNUM_str = nbrPath.substr(nbrPath.find_first_of('-')+1, nbrPath.find_last_of('-')-nbrPath.find_first_of('r'));
    std::string_view searchRadius_str = nbrPath.substr(nbrPath.find_last_of('r')+1, nbrPath.find_last_of('.')-nbrPath.find_last_of('r'));
    const std::size_t candidateNUM = std::stoi(candidateNUM_str.data());
    const double searchRadius = std::stod(searchRadius_str.data());
    std::cout << "candidateNUM: " << candidateNUM << " searchRadius: " << searchRadius << std::endl;

}


int main() {
    try {
//         fileRead();
//         matchQuery();
//        getLength();
        // para: filePath, outputPath, searchRadius, number of nbr candidates, number of threads
        static const std::string candidateNUM_str = "100";  // limit the number of nearest neighbors
        static const std::string searchRadiusKM_str = "0.01";   // 10 meters if 0.01
        static constexpr std::string_view zuid_holland_path = "../Data/Data_New/DataFile_2020_10_v2_with_index.txt";
//        zuid_holland_test(candidateNUM_str, searchRadiusKM_str, zuid_holland_path);

        static const std::string_view nbrPath = "../Data/Data_New/nbrList-100-r0.01.txt";
        nbrAnalysis(nbrPath);
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in main: " << e.what() << std::endl;
    }

    return 0;
}
