# Copyright (c) 2013 ISIR CNRS
# Authors: Sovannara Hak

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)

SET(PROJECT_NAME roboptim-core-plugin-quadprog)
SET(PROJECT_DESCRIPTION "RobOptim core Quadprog++ plug-in")
SET(PROJECT_URL "http://github.com/sovannara-hak/roboptim-core-plugin-quadprog")

SET(CXX_DISABLE_WERROR 1)

SET(PKG_CONFIG_ADDITIONAL_VARIABLES plugindir ${PKG_CONFIG_ADDITIONAL_VARIABLES})
SETUP_PROJECT()

SET(${PROJECT_NAME}_HEADERS
    ${CMAKE_SOURCE_DIR}/include/roboptim/core/plugin/quadprog/quadprog.hh
)

SEARCH_FOR_BOOST()

ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.2.0")
ADD_REQUIRED_DEPENDENCY("roboptim-core >= 0.5")
ADD_REQUIRED_DEPENDENCY("quadprog")

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

HEADER_INSTALL("${${PROJECT_NAME}_HEADERS}")

SETUP_PROJECT_FINALIZE()
