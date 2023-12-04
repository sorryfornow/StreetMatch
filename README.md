# StreetMatch
Matching bicycle track coordinate points on the Netherlands map 

## Setup 
You need Cpp20 to run the program. <br>
You can check your Cpp version by running
```
g++ --version
```
If you are using Mac OS, you can install Cpp20 by using Homebrew
```
brew install gcc
```
See https://www.educative.io/edpresso/how-to-install-gcc-compiler-on-mac-osx for more information about installing gcc in Mac OS

The steps below are
ONLY NECESSARY IF parsing OSM data by libosmium
1. Clone the repository
    ```
    git clone https://github.com/sorryfornow/StreetMatch.git
    ```
2. Install the requirements <br> 
If you are using Mac OS, you need to install libosmium first by using Homebrew
    ```
    brew install libosmium
    brew info libosmium
    ```
    find path like:
    ```
    /opt/homebrew/Cellar/libosmium/2.20.0/include
    ```
    include the path in CMakeLists.txt
    ```
    include_directories(/opt/homebrew/Cellar/libosmium/2.20.0/include)
    ```
    See https://osmcode.org/libosmium/ for more information about installing in the different operating systems
3. Prepare the data <br> 
You can download the data from https://download.geofabrik.de/ 
<br> For example, you can
    ```
    cd StreetMatch
    mkdir Data
    cd Data
    wget https://download.geofabrik.de/europe/netherlands-latest.osm.pbf
    ```
    or you can use the data in your own directory
    assuming the data is in the directory /Users/username/data/netherlands-latest.osm.pbf

    ```
    cd StreetMatch
    mkdir Data
    cd Data
    ln -s /Users/username/data/netherlands-latest.osm.pbf netherlands-latest.osm.pbf
    ```
   
4. Run the program
    ```
    cd StreetMatch
    mkdir build
    cd build
    cmake ..
    make
    ./StreetMatch
    ```

## Usage
The directory structure is as follows:
```
StreetMatch
├── CMakeLists.txt
├── Data
│   └── maps
│       └── NL2.osm
│   └── query
│       └── DataFile_2020_10_01_clean.csv
│   └── result
│       └── result.txt
├── main.cpp
├── MapNode.hpp
├── README.md
```
The program will read the data from Data/maps/NL2.osm and Data/query/DataFile_2020_10_01_clean.csv and write the result to Data/result/result.txt
<br> You can change the input and output file path in main.cpp
<br> 
You can use command line to run the program
```
./StreetMatch -m <map file path> -q <query file path> -o <output file path>
```
For example
```
./StreetMatch -m ../Data/maps/NL2.osm -q ../Data/query/DataFile_2020_10_01_clean.csv -o ../Data/result/result.txt
```
## Result
The result will be written to Data/result/result.txt with the following format:
```
query_latitute, query_longitude, query_streetCount, matched_nodeId, matched_latitute, matched_longitude, matched_streetCount
```
and the distance from the query point to the nearest matched point can also be yielded if needed
