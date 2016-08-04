cmake_minimum_required(VERSION 3.5.1)

project(CarDemo)

# Configured for Linux
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake;/usr/local/lib/OGRE/cmake/;${CMAKE_MODULE_PATH}")

if (CMAKE_BUILD_TYPE STREQUAL "")
	# CMake defaults to leaving CMAKE_BUILD_TYPE empty. This screws up
	# differentiation between debug and release builds.
	set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build, options are: None CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif ()

# Note that CXX flags were explicitly overwritten to allow for
# optimization-free debugging
set(CMAKE_CXX_FLAGS_DEBUG  "${CMAKE_CXX_FLAGS_DEBUG} -O0")

find_package(OGRE REQUIRED)
find_package(OIS REQUIRED)
find_package(BULLET REQUIRED)

# If on, compile for unit testing. The executable file will solely execute these tests and will not run the simulation.
option(build_gtests "Build and run gtests" OFF)
if (build_gtests)
	find_package(GTest REQUIRED)
	include_directories(${GTEST_INCLUDE_DIRS})
	add_definitions(-DCOMPILE_UNIT_TESTS)
	list(APPEND EXTLIBS ${GTEST_BOTH_LIBRARIES})
endif()

set(BOOST_COMPONENTS system thread filesystem wave)
find_package(Boost COMPONENTS ${BOOST_COMPONENTS} REQUIRED QUIET)

include_directories(
	${OGRE_INCLUDE_DIRS}
	${OIS_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${BULLET_INCLUDE_DIRS}
)

link_directories(
	${Boost_LIBRARY_DIRS}
)

# EXTLIBS contains the name of all external libraries
list(APPEND EXTLIBS ${OIS_LIBRARIES})
list(APPEND EXTLIBS ${OGRE_LIBRARIES})
list(APPEND EXTLIBS ${OGRE_Terrain_LIBRARIES})
list(APPEND EXTLIBS ${OGRE_Plugin_OctreeSceneManager_LIBRARIES})
list(APPEND EXTLIBS ${Boost_LIBRARIES})
list(APPEND EXTLIBS ${BULLET_LIBRARIES})

# The following libraries are included in this repository
add_subdirectory(shiny)
list(APPEND EXTLIBS "shiny")
list(APPEND EXTLIBS "shiny.OgrePlatform")

add_subdirectory(btOgre)
list(APPEND EXTLIBS "btOgre")

add_subdirectory(tinyxml)
list(APPEND EXTLIBS "tinyxml")

# Header files
file(GLOB HDRS
	*.hpp
	util/*.hpp
	vdrift/*.hpp
	terrain/*.hpp
	ai/*.hpp
	ai/predictSteering/*.hpp
	road/*.hpp
)

# Source files
file(GLOB SRCS
	*.cpp
	util/*.cpp
	vdrift/*.cpp
	terrain/*.cpp
	ai/*.cpp
	ai/predictSteering/*.cpp
	road/*.cpp
)

add_executable(CarDemo ${HDRS} ${SRCS})
target_link_libraries(CarDemo ${EXTLIBS})