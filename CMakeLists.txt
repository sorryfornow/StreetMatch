cmake_minimum_required(VERSION 3.23)
project(StreetMatch)

set(CMAKE_CXX_STANDARD 23)
#/opt/homebrew/Cellar/libosmium/2.20.0/include

# Add the include directory for libosmium
include_directories(/opt/homebrew/Cellar/libosmium/2.20.0/include)

# Add your executable
add_executable(StreetMatch main.cpp osmium.h MapNode.h)

# If libosmium has any dependencies that need to be linked, add them here
# target_link_libraries(StreetMatch <library_name>)
