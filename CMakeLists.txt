# The latest version as of 06/10/2016
cmake_minimum_required(VERSION 3.5.1)

project(CarDemo)

# Assumption of Linux/Unix
set(CMAKE_MODULE_PATH "/usr/local/lib/OGRE/cmake/;${CMAKE_MODULE_PATH}")

if (CMAKE_BUILD_TYPE STREQUAL "")
	# CMake defaults to leaving CMAKE_BUILD_TYPE empty. This screws up
	# differentiation between debug and release builds.
	set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build, options are: None CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif ()

set(CMAKE_DEBUG_POSTFIX "_d")

# As of now, not using any special "dist" directory
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR})

# Update as needed
find_package(OGRE REQUIRED)

# Source files
set(SRCS
	./main.cpp
)

# Update as needed
include_directories(
	${OGRE_INCLUDE_DIRS}
)

add_executable(CarDemo ${SRCS})
set_target_properties(CarDemo PROPERTIES DEBUG_POSTFIX _d)

# Update as needed
target_link_libraries(CarDemo
	${OGRE_LIBRARIES}
)

# Output executables in a subdirectory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
