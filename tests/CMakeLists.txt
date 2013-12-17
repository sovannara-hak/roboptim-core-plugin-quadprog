# Copyright (c) 2013 ISIR CNRS
# Authors: Sovannara Hak

ADD_EXECUTABLE(testquadprog 
    quadprog.cpp
)

ADD_EXECUTABLE(testquadprogPlugin
    quadprogPlugin.cpp
)

# Set plugin path of roboptim-core 
SET(PLUGIN_PATH "${CMAKE_BINARY_DIR}/src")

SET_TARGET_PROPERTIES(
    testquadprogPlugin
    PROPERTIES COMPILE_FLAGS
    "-DPLUGIN_PATH='\"${PLUGIN_PATH}\"'"
)

PKG_CONFIG_USE_DEPENDENCY(testquadprog quadprog)
PKG_CONFIG_USE_DEPENDENCY(testquadprog eigen3)
PKG_CONFIG_USE_DEPENDENCY(testquadprogPlugin quadprog)
PKG_CONFIG_USE_DEPENDENCY(testquadprogPlugin roboptim-core)
PKG_CONFIG_USE_DEPENDENCY(testquadprogPlugin eigen3)
