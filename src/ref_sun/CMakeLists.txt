cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 11)
# set the project name
project(
  Sun_MPC
  VERSION 1.0
  LANGUAGES CXX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D __USE_LONG_INTEGERS__")
find_package(Eigen3 REQUIRED)
link_directories("/usr/local/lib")
include_directories("include")
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories("/usr/local/include/qpOASES")
file(GLOB_RECURSE SOURCES "src/*.cpp")

# add the executable
add_executable(testTrajectoryMPC ${SOURCES})
target_link_libraries(testTrajectoryMPC libqpOASES.so) 
# target_link_libraries (testTrajectoryMPC Eigen3::Eigen)
