# build with catkin_make -DCATKIN_ENABLE_TESTING=1 -DCMAKE_BUILD_TYPE=Debug
# run with rosrun panther_driver VelocityManagerTest


set(This VelocityManagerTest)
project(${This})
cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-std=c++17)
include(Dart)

find_package(GTest REQUIRED)

set(Sources
    VelocityManagerTest.cxx ../src/VelocityManager.cxx
)

add_executable(${This} ${Sources})
target_link_libraries(${This} PUBLIC
    gtest_main
    #Project
    ${GTEST_LIBRARIES} 
    pthread
)

add_test(
    NAME ${This}
    COMMAND ${This}
)
