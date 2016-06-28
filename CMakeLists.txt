cmake_minimum_required(VERSION 3.5.1)

project(CarDemo)

# Assumption of Linux/Unix
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake;/usr/local/lib/OGRE/cmake/;${CMAKE_MODULE_PATH}")

if (CMAKE_BUILD_TYPE STREQUAL "")
	# CMake defaults to leaving CMAKE_BUILD_TYPE empty. This screws up
	# differentiation between debug and release builds.
	set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build, options are: None CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif ()

set(CMAKE_DEBUG_POSTFIX "_d")
# Note that CXX flags were explicitly overwritten to allow for
# optimization-free debugging
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O0 -g -DNDEBUG")

set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/dist")

# Update as needed
find_package(OGRE REQUIRED)
find_package(OIS REQUIRED)
find_package(BULLET REQUIRED)

option(build_gtests "Build and run gtests" ON)
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

list(APPEND EXTLIBS ${Boost_LIBRARIES})
list(APPEND EXTLIBS ${BULLET_LIBRARIES})

# External libraries
add_subdirectory(shiny)
list(APPEND EXTLIBS "shiny")
list(APPEND EXTLIBS "shiny.OgrePlatform")

# Header files
file(GLOB HDRS
	*.hpp
	util/*.hpp
	vdrift/*.hpp
	terrain/*.hpp
)

# Source files
file(GLOB SRCS
	*.cpp
	util/*.cpp
	vdrift/*.cpp
	terrain/*.cpp
)

add_executable(CarDemo ${HDRS} ${SRCS})
set_target_properties(CarDemo PROPERTIES DEBUG_POSTFIX _d)

# Update as needed
target_link_libraries(CarDemo
	${EXTLIBS}
	${OIS_LIBRARIES}
	${OGRE_LIBRARIES}
	${OGRE_Terrain_LIBRARIES}
)

# Make binary directories
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/dist/bin)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/dist/data)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/dist/config)

# Execute in the binary directory
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/dist/bin")

install(TARGETS CarDemo
	RUNTIME DESTINATION bin
	CONFIGURATIONS ALL
)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/data
	DESTINATION ./
	CONFIGURATIONS Release RelWithDebInfo Debug
)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/config
	DESTINATION ./
	CONFIGURATIONS Release RelWithDebInfo Debug
)
