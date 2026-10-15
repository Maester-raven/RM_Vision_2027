cmake_minimum_required(VERSION 3.16)

project(test_omni_angle_solver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

add_executable(test_angle_solver
    test_angle_solver.cpp
    ${PROJECT_ROOT}/tasks/omniperception/angle_solver.cpp
)

target_include_directories(test_angle_solver PRIVATE
    ${PROJECT_ROOT}
)

target_link_libraries(test_angle_solver PRIVATE
    ${OpenCV_LIBS}
    Eigen3::Eigen
)