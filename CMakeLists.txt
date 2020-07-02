cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_BUILD_TYPE Release)
project(pcl_utils)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
file(GLOB SOURCES "src/*.cpp")

add_executable (pcl_utils_test ${SOURCES})
target_link_libraries (pcl_utils_test ${PCL_LIBRARIES})

#Generate the shared library from the sources
add_library(pcl_utils SHARED ${SOURCES})
 
set_target_properties(pcl_utils PROPERTIES PUBLIC_HEADER include/pcl_utils.hpp)

#Set the location for library installation -- i.e., /usr/lib in this case
# not really necessary in this example. Use "sudo make install" to apply
install(TARGETS pcl_utils
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})