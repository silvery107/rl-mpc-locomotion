cmake_minimum_required(VERSION 3.4...3.18)
set(CMAKE_CXX_STANDARD 11)
project(C_SolverMPC)


# Find library and headers
# find_package(osqp REQUIRED)
find_package(OsqpEigen)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D __USE_LONG_INTEGERS__")
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
find_package(pybind11 CONFIG)
pybind11_add_module(C_SolverMPC SolverMPC.cpp)

target_link_libraries(C_SolverMPC PRIVATE OsqpEigen::OsqpEigen)
# Link the OSQP shared library
# target_link_libraries(C_SolverMPC PRIVATE osqp::osqp)
# or...
# Link the OSQP static library
# target_link_libraries(C_SolverMPC PRIVATE osqp::osqpstatic)



