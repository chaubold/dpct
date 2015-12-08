cmake_minimum_required(VERSION 2.8.6)
project(dpct)

# ------------------------------------------
# user defined configuration
set(WITH_PYTHON "false" CACHE BOOL "Build with python wrapper.")
set(WITH_TESTS "false" CACHE BOOL "Build tests.")
set(WITH_LOG "false" CACHE BOOL "Enable logs.")

if(WITH_LOG)
	add_definitions(-DDEBUG_LOG)
endif()

# ------------------------------------------
# find dependencies
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_extensions/)
find_package( Lemon REQUIRED )
include_directories(${LEMON_INCLUDE_DIR})

# ------------------------------------------
# global configuration
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)

# check for C++ 11 support:
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(SEND_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# C++ STL debug symbols
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set(WITH_DEBUG_STL "False" CACHE BOOL "Build with C++ stl debug symbols?")
  if(WITH_DEBUG_STL)
    add_definitions(-D_GLIBCXX_DEBUG)
  endif()
endif()

# ------------------------------------------
# build library
file(GLOB_RECURSE SOURCES src/*.cpp)
file(GLOB_RECURSE HEADERS include/*.h)
add_library(dpct SHARED ${SOURCES} ${HEADERS})
target_link_libraries(dpct ${LEMON_LIBRARIES})

# installation
install(TARGETS dpct 
		ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/" 
		DESTINATION include/dpct 
		PATTERN ".h" 
		PATTERN ".git" EXCLUDE)

# ------------------------------------------
# add subdirectories
if(WITH_TESTS)
	enable_testing()
	add_subdirectory(test)
endif()

add_subdirectory(bin)

if(WITH_PYTHON)
	add_subdirectory(python)
endif()

