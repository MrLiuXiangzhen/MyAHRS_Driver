cmake_minimum_required(VERSION 3.22)
project(MyAHRS_Driver_LiuXiangzhen)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
#set(CMAKE_CXX_STANDARD 14)

include_directories(myahrs_plus.hpp)

add_executable(MyAHRS_Driver_LiuXiangzhen
        myahrs.cpp)
target_link_libraries(MyAHRS_Driver_LiuXiangzhen -lpthread)