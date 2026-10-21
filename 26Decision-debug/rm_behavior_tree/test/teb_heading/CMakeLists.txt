cmake_minimum_required(VERSION 3.16)
project(teb_heading_regression)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(teb_local_planner REQUIRED)
find_package(Eigen3 REQUIRED)
find_path(SUITESPARSE_HEADERS cs.h PATH_SUFFIXES suitesparse REQUIRED)
find_path(G2O_HEADERS g2o/core/sparse_optimizer.h
  PATHS ${teb_local_planner_INCLUDE_DIRS} PATH_SUFFIXES g2o REQUIRED)
find_library(G2O_CORE g2o_core
  PATHS "${G2O_HEADERS}/../lib/${CMAKE_LIBRARY_ARCHITECTURE}"
        "${G2O_HEADERS}/../../lib/${CMAKE_LIBRARY_ARCHITECTURE}" REQUIRED)
find_library(G2O_STUFF g2o_stuff
  PATHS "${G2O_HEADERS}/../lib/${CMAKE_LIBRARY_ARCHITECTURE}"
        "${G2O_HEADERS}/../../lib/${CMAKE_LIBRARY_ARCHITECTURE}" REQUIRED)
ament_add_gtest(test_teb_heading test_teb_heading.cpp)
ament_target_dependencies(test_teb_heading teb_local_planner)
target_include_directories(test_teb_heading PRIVATE ${G2O_HEADERS} ${SUITESPARSE_HEADERS})
target_link_libraries(test_teb_heading Eigen3::Eigen ${G2O_CORE} ${G2O_STUFF})
