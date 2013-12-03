# Copyright (c) 2013 ISIR CNRS
# Authors: Sovannara Hak

ADD_LIBRARY(roboptim-core-plugin-quadprog 
    MODULE quadprog.cc ${${PROJECT_NAME}_HEADERS}
)

SET_TARGET_PROPERTIES(roboptim-core-plugin-quadprog
    PROPERTIES PREFIX "" SOVERSION 2.0.0
)

INSTALL(TARGETS roboptim-core-plugin-quadprog
    DESTINATION ${ROBOPTIM_CORE_PLUGINDIR}
)

PKG_CONFIG_USE_DEPENDENCY(roboptim-core-plugin-quadprog quadprog)
PKG_CONFIG_USE_DEPENDENCY(roboptim-core-plugin-quadprog roboptim-core)
