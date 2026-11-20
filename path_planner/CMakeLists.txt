cmake_minimum_required(VERSION 3.10)
project(PathPlanner)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBAR REQUIRED zbar)

include_directories(${OpenCV_INCLUDE_DIRS} ${ZBAR_INCLUDE_DIRS})

add_executable(path_planner main.cpp)

target_link_libraries(path_planner ${OpenCV_LIBS} ${ZBAR_LIBRARIES})