cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_INSTALL_PREFIX}/share/cmake)

############################### C++ 11 Boilerplate #######################################
if (CMAKE_COMPILER_IS_GNUCC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wreturn-type -Wuninitialized -Wunused-variable") # -Wunused-parameter")

  execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
  if (NOT (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7))
    message(FATAL_ERROR "requires gcc version >= 4.7")  # to support the c++0x flag below
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  endif()
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wreturn-type -Wuninitialized -Wunused-variable") # -Wunused-parameter")

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  if (APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
  endif()
elseif (MSVC)
   if (NOT ${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER 18.00.40628)
     # version number decoder ring at https://en.wikipedia.org/wiki/Visual_C%2B%2B 
     # at least one user hit a compiler crash with VC++ 12.0, which was resolved by installing the latest service packs.  I don't know that 40629 is required, but know that 00 is not sufficient.
     message(FATAL_ERROR "requires MS VC++ 12.0 update 5 or greater (Visual Studio >= 2013).  download for free at http://visualstudio.com")
   endif()

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4996") # disable sprintf security warning

endif()
############################### C++ 11 Boilerplate #######################################

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
