cmake_minimum_required(VERSION 2.8)

project(airTraffic)

find_package(OpenSceneGraph REQUIRED osgDB osgUtil osgViewer osgTerrain osgGA)
add_executable(airTraffic
    src/directRender.cpp
    src/waypoint.cpp src/waypoint.h
    )
target_link_libraries(airTraffic ${OPENSCENEGRAPH_LIBRARIES})
