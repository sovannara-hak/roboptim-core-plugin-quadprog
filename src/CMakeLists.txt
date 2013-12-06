# Copyright (c) 2013 ISIR CNRS
# Authors: Sovannara Hak

ADD_LIBRARY(roboptim-core-plugin-quadprog 
    MODULE quadprog.cc
)

ADD_DEPENDENCIES(roboptim-core-plugin-quadprog roboptim-core)
#PKG_CONFIG_USE_DEPENDENCY(roboptim-core-plugin-quadprog liblog4cxx)
PKG_CONFIG_USE_DEPENDENCY(roboptim-core-plugin-quadprog quadprog)
PKG_CONFIG_USE_DEPENDENCY(roboptim-core-plugin-quadprog roboptim-core)

SET_TARGET_PROPERTIES(roboptim-core-plugin-quadprog
    PROPERTIES PREFIX "" SOVERSION 2.0.0
)

INSTALL(TARGETS roboptim-core-plugin-quadprog
    DESTINATION ${ROBOPTIM_CORE_PLUGINDIR}
)

