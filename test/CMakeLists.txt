cmake_minimum_required(VERSION 2.8)
message( "\nConfiguring tests:" )
find_package(Boost REQUIRED COMPONENTS unit_test_framework)

include_directories(${Boost_INCLUDE_DIRS})
add_definitions(-DBOOST_TEST_DYN_LINK)

# autodiscover test sources and add tests
file(GLOB TEST_SRCS *.cpp)
foreach(test_src ${TEST_SRCS})
	get_filename_component(test_name ${test_src} NAME_WE)
	add_executable( ${test_name} ${test_src} )
	target_link_libraries( ${test_name} dpct ${Boost_LIBRARIES})
	add_test( ${test_name} ${test_name})
endforeach(test_src)