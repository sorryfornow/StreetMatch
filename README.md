# StreetMatch
Matching bicycle track coordinate points on the Netherlands map 

## Setup
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