cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
set (CMAKE_CXX_STANDARD 11)

# setting a variable equal to the executable name
set(name rc)

# name the project
project(rc)

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
include_directories(/usr/include/sparsepp)
target_link_libraries(${name} hdf5 hdf5_cpp)
