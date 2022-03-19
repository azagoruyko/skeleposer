cmake_minimum_required(VERSION 3.0)

if(CMAKE_CONFIGURATION_TYPES)
   set(CMAKE_CONFIGURATION_TYPES Debug Release RelWithDebInfo)
   set(CMAKE_CONFIGURATION_TYPES "${CMAKE_CONFIGURATION_TYPES}" CACHE STRING
     "Reset the configurations to Debug and Release"
     FORCE)
endif()

if (NOT CMAKE_BUILD_TYPE) 
    set (CMAKE_BUILD_TYPE "Release") 
endif()

project (skeleposer) 
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

# Packages
if (MSVC)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
endif ()

find_package(Maya REQUIRED)

ADD_DEFINITIONS(-D__TBB_MAKE_EXCEPTION_PTR_PRESENT -D__TBB_NO_IMPLICIT_LINKAGE)

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

target_include_directories(skeleposer PRIVATE ${MAYA_INCLUDE_DIR})

target_link_libraries(skeleposer ${MAYA_LIBRARIES})

set_target_properties( skeleposer PROPERTIES OUTPUT_NAME skeleposer CLEAN_DIRECT_OUTPUT 1 SUFFIX ".mll" )

MAYA_SET_PLUGIN_PROPERTIES( skeleposer )

install(TARGETS skeleposer RUNTIME DESTINATION .)

