project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11 -g)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/jmt.cpp src/jmt.h src/utils.h src/decision_making.h src/decision_making.cpp src/coordinates.h src/coordinates.cpp src/neighbor.cpp src/neighbor.h src/trajectory.cpp src/trajectory.h src/graph.h src/utils.cpp src/get_cost.cpp)

find_package(PythonLibs REQUIRED)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

    include_directories(/usr/local/include)
    include_directories(/usr/local/opt/openssl/include)

    link_directories(/usr/local/lib)
    link_directories(/usr/local/opt/openssl/lib)
    link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/include/python2.7)
link_directories(/usr/include/python2.7)


add_executable(test_program ${sources} src/test.cpp)

add_executable(path_planning ${sources} src/main.cpp)


target_link_libraries(path_planning z ssl uv uWS python2.7)
