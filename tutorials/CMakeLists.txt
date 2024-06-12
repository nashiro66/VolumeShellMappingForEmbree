## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

include(CMakeDependentOption)

CONFIGURE_FILE("${CMAKE_CURRENT_SOURCE_DIR}/README.md.in" "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
INSTALL(FILES "${CMAKE_CURRENT_SOURCE_DIR}/README.md" DESTINATION "src" COMPONENT examples)

ADD_SUBDIRECTORY(common/image)
CMAKE_DEPENDENT_OPTION(EMBREE_TUTORIALS_GLFW "Enables GLFW usage in tutorials. When disabled tutorial can only render to disk." ON "EMBREE_TUTORIALS" OFF)
CMAKE_DEPENDENT_OPTION(EMBREE_BUILD_GLFW_FROM_SOURCE "Builds GLFW from source." OFF "EMBREE_TUTORIALS_GLFW" OFF)

IF (EMBREE_TUTORIALS)

IF (EMBREE_TUTORIALS_GLFW)

  IF (NOT EMBREE_BUILD_GLFW_FROM_SOURCE)
    FIND_PACKAGE(glfw3 QUIET)
  ENDIF()

  IF(NOT glfw3_FOUND)
    CMAKE_MINIMUM_REQUIRED(VERSION 3.14)
    MESSAGE("-- glfw3 installation not found. download and build from source")

    INCLUDE(FetchContent)
    SET(FETCHCONTENT_QUIET ON)

    set(GLFW_BUILD_DOCS OFF CACHE INTERNAL "")
    set(GLFW_BUILD_EXAMPLES OFF CACHE INTERNAL "")
    set(GLFW_BUILD_TESTS OFF CACHE INTERNAL "")

    # The patch command first resets the glfw repository to a clean state. That
    # is because CMake wants to apply the patch again when CMake needs to
    # reconfigure, which fails.
    FetchContent_Declare(
      glfw
      GIT_REPOSITORY https://github.com/glfw/glfw
      GIT_TAG 3.3.8
      PATCH_COMMAND git reset --hard && git apply -v ${CMAKE_CURRENT_SOURCE_DIR}/common/glfw/glfw.patch
    )

    FetchContent_GetProperties(glfw)
    if(NOT glfw_POPULATED)
      FetchContent_Populate(glfw)
      # We want to build glfw and link statically, so no need to
      # install it as part of the Embree install targets.
      add_subdirectory(${glfw_SOURCE_DIR} ${glfw_BINARY_DIR} EXCLUDE_FROM_ALL)
    endif()

    MARK_AS_ADVANCED(FETCHCONTENT_BASE_DIR)
    MARK_AS_ADVANCED(FETCHCONTENT_FULLY_DISCONNECTED)
    MARK_AS_ADVANCED(FETCHCONTENT_QUIET)
    MARK_AS_ADVANCED(FETCHCONTENT_SOURCE_DIR_GLFW)
    MARK_AS_ADVANCED(FETCHCONTENT_UPDATES_DISCONNECTED)
    MARK_AS_ADVANCED(FETCHCONTENT_UPDATES_DISCONNECTED_GLFW)
  ELSE()
    MESSAGE("-- glfw3 installation found.")
  ENDIF()

  FIND_PACKAGE(OpenGL)
  IF (NOT OPENGL_FOUND)
    MESSAGE(FATAL_ERROR "OpenGL not found! Please either install OpenGL or disable EMBREE_TUTORIALS.")
  ENDIF()

  ADD_DEFINITIONS(-DUSE_GLFW)

  # all libraries we have to link against when using a window/gui
  SET(GUI_LIBRARIES imgui glfw ${OPENGL_LIBRARIES})
ENDIF()

ADD_SUBDIRECTORY(common)

ADD_SUBDIRECTORY(pathtracer)

ENDIF()
