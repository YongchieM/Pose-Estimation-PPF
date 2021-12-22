cmake_minimum_required(VERSION 2.9)
project(myppf)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")

include_directories("/usr/local/include/eigen3/")

find_package(PCL 1.8 REQUIRED)
find_package(OpenCV 3.4 REQUIRED)
find_package(Boost COMPONENTS thread system REQUIRED)

include_directories(${Boost_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(THIRD_PARTY_LIBS
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES})
    
set(SOURCES
    src/test.cpp
    src/pose_3d.cpp
    src/ppf_helpers.cpp
    src/ppf_match.cpp)
    
add_executable(test ${SOURCES})
target_link_libraries(test PRIVATE ${THIRD_PARTY_LIBS} ${PROJECT_SOURCE_DIR}/include)
install(TARGETS test RUNTIME DESTINATION bin)
