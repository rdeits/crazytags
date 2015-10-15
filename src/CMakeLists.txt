cmake_minimum_required(VERSION 2.8)

include(setup_c++11.cmake)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_INSTALL_PREFIX}/share/cmake)
include(pods)
include(lcmtypes)
lcmtypes_build()

find_package(OpenCV REQUIRED)
add_executable(image-capture image_capture.cpp)
target_link_libraries(image-capture ${OpenCV_LIBS})
install(TARGETS image-capture DESTINATION bin)

pods_find_pkg_config(eigen3 3.2)
if (eigen3_FOUND)
   pods_use_pkg_config_includes( eigen3 )
else()
   # then use the traditional approach to finding Eigen3
   find_package( Eigen3 3.2 )
   if (EIGEN3_FOUND)
      set(eigen3_FOUND 1)
      include_directories( ${EIGEN3_INCLUDE_DIR} )
   else()
      message(FATAL_ERROR "Could not find eigen, which is a required depedency")
   endif()
endif()

add_executable(publish-tags publish_tags.cpp)
pods_use_pkg_config_packages(publish-tags apriltags lcm)
target_link_libraries(publish-tags ${OpenCV_LIBS} pthread)
install(TARGETS publish-tags DESTINATION bin)
