cmake_minimum_required(VERSION 3.20)

project(state_machine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Boost REQUIRED COMPONENTS system)
find_package(nlohmann_json REQUIRED)

add_executable(state-machine main.cpp)
target_link_libraries(state-machine PRIVATE Boost::system nlohmann_json::nlohmann_json)
