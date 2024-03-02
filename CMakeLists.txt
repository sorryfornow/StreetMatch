cmake_minimum_required(VERSION 3.23)
project(StreetMatch)

set(CMAKE_CXX_STANDARD 23)

# Set the path where CMake can find the FindTinyXML2.cmake file
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

# Find TinyXML2 using your custom module
#find_package(TinyXML2 REQUIRED)

# Add the include directory for libosmium and protozero
#include_directories(/Users/siqingzhang/protozero/include)
#include_directories(/opt/homebrew/Cellar/libosmium/2.20.0/include)
#include_directories(/opt/homebrew/Cellar/tinyxml2/9.0.0/include)


# Add your executable
add_executable(StreetMatch main.cpp osmium.h MapNode.hpp
        Paths.h
        Paths.h)

# Link with zlib, Bzip2, and Expat
#find_package(ZLIB REQUIRED)
#find_package(BZip2 REQUIRED)
#find_package(EXPAT REQUIRED)
#target_link_libraries(StreetMatch ZLIB::ZLIB BZip2::BZip2 EXPAT::EXPAT)

# Find Boost and add Boost include directories
find_package(Boost 1.83.0 REQUIRED COMPONENTS graph)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(StreetMatch ${Boost_LIBRARIES})

## Include directories and link against TinyXML2
#include_directories(${TinyXML2_INCLUDE_DIRS})
## print out dir
#message(STATUS "TinyXML2_INCLUDE_DIRS: ${TinyXML2_INCLUDE_DIRS}")
#target_link_libraries(StreetMatch ${TinyXML2_LIBRARIES})
