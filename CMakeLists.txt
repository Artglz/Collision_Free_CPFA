cmake_minimum_required(VERSION 2.8.12)
project(CPFA-ARGoS)

# For verbose output
set(CMAKE_VERBOSE_MAKEFILE ON)

#CMAKE_BUILD_TYPE(Release)
#ARGOS_BUILD_NATTIVE(ON)

# Set the compilation type
if(CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel ..." FORCE)
endif(CMAKE_BUILD_TYPE STREQUAL "")

# Deactivate RPATH for MacOSX.
if (APPLE)
set(CMAKE_MACOSX_RPATH 0)
endif()

# Set the path where CMake will find additional scripts.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
set(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find the ARGoS package.
find_package(ARGoS REQUIRED)
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIR})
link_libraries(${ARGOS_LDFLAGS})


# Find Python
#find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
#include_directories(${Python3_INCLUDE_DIRS})

# Explicitly set the Python library path
#set(Python3_LIBRARY "/opt/homebrew/Cellar/python@3.10/3.10.16/Frameworks/Python.framework/Versions/3.10/lib/libpython3.10.dylib")

# Ensure the linker finds it
#link_directories("/opt/homebrew/Cellar/python@3.10/3.10.16/Frameworks/Python.framework/Versions/3.10/lib")

# Find Python
find_package(Python3 3.10 EXACT COMPONENTS Interpreter Development REQUIRED)

# Set manually installed Python paths
set(Python3_EXECUTABLE "/Library/Frameworks/Python.framework/Versions/3.10/bin/python3.10")
set(Python3_INCLUDE_DIR "/Library/Frameworks/Python.framework/Versions/3.10/include/python3.10")
set(Python3_LIBRARY "/Library/Frameworks/Python.framework/Versions/3.10/lib/libpython3.10.dylib")

# Include directories and link paths
include_directories(${Python3_INCLUDE_DIR})
link_directories(${Python3_LIBRARY})

# Check for required libraries to compile code depending on Qt and OpenGL.
# include(ARGoSCheckQTOpenGL)

# Find the ARGoS package
# find_package(PkgConfig)
# pkg_check_modules(ARGOS REQUIRED argos3_simulator)
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)

# Find the GSL package.
# find_package(GSL REQUIRED)

# Find the LUA package.
#find_package(Lua REQUIRED)

# Set ARGoS include directory.
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${GSL_INCLUDE_DIR} ${LUA_INCLUDE_DIR} source)

link_directories(${ARGOS_LIBRARY_DIRS} build)

# Descend into the source code directory and build the CPFA libraries. This is all that's needed to run experiments.
add_subdirectory(source)

# Only build the evolver program if selected.
if (BUILD_EVOLVER)



# Find the boost filesystem package.
find_package( Boost REQUIRED COMPONENTS system filesystem )
include_directories( ${Boost_INCLUDE_DIRS} )

# Specify the evolver program as a compile target.
add_executable(cpfa_evolver source/evolver.cpp)

# Specify the libraries needed to compile the evolver.
target_link_libraries(cpfa_evolver CPFA_controller
				   CPFA_loop_functions
				   GA-MPI   
				   ${MPI_LIBRARIES}
		   		   ${Boost_LIBRARIES}
				   argos3core_simulator)
else()
endif()
