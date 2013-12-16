# Copyright (c) 2013 ISIR CNRS
# Authors: Sovannara Hak

ADD_EXECUTABLE(testquadprog 
    quadprog.cpp
)

ADD_EXECUTABLE(testquadprogPlugin
    quadprogPlugin.cpp
)

# Set plugin path of roboptim-core from the .pc file
EXECUTE_PROCESS(
    COMMAND "${PKG_CONFIG_EXECUTABLE}"
    "--variable=plugindir" "roboptim-core"
    OUTPUT_VARIABLE "PC_ROBOPTIM-CORE_PLUGINDIR"
    ERROR_QUIET
)
STRING(REPLACE "\n" "" "PC_ROBOPTIM-CORE_PLUGINDIR" "${PC_ROBOPTIM-CORE_PLUGINDIR}")

SET_TARGET_PROPERTIES(
    testquadprogPlugin
    PROPERTIES COMPILE_FLAGS
    "-DPLUGIN_PATH='\"${PC_ROBOPTIM-CORE_PLUGINDIR}\"'"
)

PKG_CONFIG_USE_DEPENDENCY(testquadprog quadprog)
PKG_CONFIG_USE_DEPENDENCY(testquadprog eigen3)
PKG_CONFIG_USE_DEPENDENCY(testquadprogPlugin quadprog)
PKG_CONFIG_USE_DEPENDENCY(testquadprogPlugin roboptim-core)
PKG_CONFIG_USE_DEPENDENCY(testquadprogPlugin eigen3)
