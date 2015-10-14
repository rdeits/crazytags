cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_INSTALL_PREFIX}/share/cmake)
include(pods)

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
  if (APPLE)  # this was a step towards getting things to work with
  #   clang on mac, but ultimately we didn't get there...   (but I would
  #   be worried about sharing pointers between objects compiled against
  #   different c++ libs, so removing it)
  #    # http://stackoverflow.com/questions/13445742/apple-and-shared-ptr
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
  endif()
elseif (MSVC)
   if (NOT ${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER 18.00.40628)
     # version number decoder ring at https://en.wikipedia.org/wiki/Visual_C%2B%2B 
     # at least one user hit a compiler crash with VC++ 12.0, which was resolved by installing the latest service packs.  I don't know that 40629 is required, but know that 00 is not sufficient.
     message(FATAL_ERROR "requires MS VC++ 12.0 update 5 or greater (Visual Studio >= 2013).  download for free at http://visualstudio.com")
   endif()

#  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /arch:SSE2 /openmp")
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:SSE2 /openmp")

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4251")
  # disable warning C4251: e.g.'RigidBody::linkname' : class 'std::basic_string<_Elem,_Traits,_Ax>' needs to have dll-interface to be used by clients of class 'RigidBody'
  # followed by template linking errors.  After reading, e.g.:
  #   https://connect.microsoft.com/VisualStudio/feedback/details/696593/vc-10-vs-2010-basic-string-exports
  # I think we're not being sufficiently careful with our library interfaces (on other platforms as well) - Russ

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4503")
  # disable C4503: 'identifier' : decorated name length exceeded, name was truncated
  # these occur due to AutoDiffScalar inputs to DrakeJoint methods, which results in very long type names
  # From https://msdn.microsoft.com/en-us/library/074af4b6.aspx:
  # It is possible to ship an application that generates C4503, but if you get link time errors on a truncated symbol,
  # it will be more difficult to determine the type of the symbol in the error. Debugging will also be more difficult;
  # the debugger will also have difficultly mapping symbol name to type name. The correctness of the program, however,
  # is unaffected by the truncated name.

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4522")
  # disable C4522: 'class' : multiple assignment operators specified
  # The class has multiple assignment operators of a single type. This warning is informational;
  # the constructors are callable in your program.

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4996") # disable sprintf security warning

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj") # after receiving RigidBodyManipulator.cpp : fatal error C1128: number of sections exceeded object file format limit: compile with /bigobj
endif()
############################### C++ 11 Boilerplate #######################################

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
pods_use_pkg_config_packages(publish-tags apriltags lcm lcmtypes_bot2-core)
target_link_libraries(publish-tags ${OpenCV_LIBS} pthread)
install(TARGETS publish-tags DESTINATION bin)
