CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
set (CMAKE_CXX_STANDARD 11)
PROJECT( octoccupancy )

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

ADD_EXECUTABLE(octoccupancy octoccupancy.cc)
