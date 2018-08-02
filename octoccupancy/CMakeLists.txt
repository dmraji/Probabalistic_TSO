CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
set (CMAKE_CXX_STANDARD 11)

# setting a variable equal to the executable name
set(name octoccupancy)

PROJECT( octoccupancy )

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

# get the implementation files
file(GLOB sources ${PROJECT_SOURCE_DIR}/src/*.cc)

# get the header files
file(GLOB headers ${PROJECT_SOURCE_DIR}/incl/*.hh)
include_directories(${PROJECT_SOURCE_DIR}/incl)

# create the executable
add_executable(${name} ${name}.cc ${sources} ${headers})

# linking for external libs
link_directories(/usr/lib/x86_64-linux-gnu/hdf5/serial)
include_directories(/usr/include/hdf5/serial)
target_link_libraries(${name} hdf5 hdf5_cpp)
