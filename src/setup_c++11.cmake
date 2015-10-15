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