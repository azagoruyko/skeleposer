cmake_minimum_required(VERSION 3.5)

project (skeleposer) 
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

find_package(Maya REQUIRED)

set(source
    source/main.cpp 
    source/skeleposer.cpp
    source/skeleposer.h
    source/blendMatrix.cpp
    source/blendMatrix.h
    source/stickyMatrix.cpp
    source/stickyMatrix.h
    source/utils.hpp)

add_library(skeleposer SHARED ${source})

MAYA_PLUGIN( skeleposer )
