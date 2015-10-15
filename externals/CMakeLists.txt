cmake_minimum_required(VERSION 2.8)

project(CrazyTagsExternals)

# PODs out-of-source build logic
if (CMAKE_INSTALL_PREFIX STREQUAL "/usr/local" OR CMAKE_INSTALL_PREFIX STREQUAL "C:/Program Files/${PROJECT_NAME}")
    find_file(_build_dir build PATHS ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/.. ${PROJECT_SOURCE_DIR}/../.. ${PROJECT_SOURCE_DIR}/../../.. ${PROJECT_SOURCE_DIR}/../../../..)
    if (_build_dir)
        set(CMAKE_INSTALL_PREFIX "${_build_dir}")
    else()
        execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_SOURCE_DIR}/../build)
        set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/../build)
    endif()
endif()
message(STATUS CMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX})

include(ExternalProject)

find_program(MAKE_EXECUTABLE make)
if (NOT MAKE_EXECUTABLE)
	message(FATAL_ERROR "couldn't find gnu make")
endif()
if (${CMAKE_GENERATOR} STREQUAL "Unix Makefiles")
	set(PODS_MAKE_COMMAND "$(MAKE)")   # so we can pass through commandline arguments.
else()
	set(PODS_MAKE_COMMAND ${MAKE_EXECUTABLE})
endif()

ExternalProject_Add(pods-cmake
    DOWNLOAD_DIR ${CMAKE_INSTALL_PREFIX}/share/cmake
    SOURCE_DIR ${CMAKE_INSTALL_PREFIX}/share/cmake
    GIT_REPOSITORY https://github.com/RobotLocomotion/cmake.git
    GIT_TAG 94a181859aca6512861084077066e1080678b819
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    )

ExternalProject_Add(lcm-pod
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lcm-pod
    DOWNLOAD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lcm-pod
    GIT_REPOSITORY https://github.com/RobotLocomotion/lcm-pod.git
    GIT_TAG ddf981b125e600b7aaada8daeab3e3f2ba2d2c55
    BUILD_IN_SOURCE 1
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ${PODS_MAKE_COMMAND} BUILD_PREFIX=${CMAKE_INSTALL_PREFIX} BUILD_TYPE=${CMAKE_BUILD_TYPE}
    UPDATE_COMMAND ""
    INSTALL_COMMAND ""
    )

ExternalProject_Add(apriltags-pod
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/apriltags-pod
    DOWNLOAD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/apriltags-pod
    GIT_REPOSITORY https://github.com/psiorx/apriltags-pod.git
    GIT_TAG ed2972f01e00d9b5a4afa2612b018374d889641f
    BUILD_IN_SOURCE 1
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ${PODS_MAKE_COMMAND} BUILD_PREFIX=${CMAKE_INSTALL_PREFIX} BUILD_TYPE=${CMAKE_BUILD_TYPE}
    INSTALL_COMMAND ""
    )