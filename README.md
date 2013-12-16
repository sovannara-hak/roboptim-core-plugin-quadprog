Roboptim-core-plugin-quadprog
=============================

This is a plugin for roboptim-core to add the Quadprog++ solver modified to use Eigen matrices.

https://github.com/roboptim/roboptim-core

https://github.com/serena-ivaldi/quadprog

Install:
--------

    git clone --recursive https://github.com/sovannara-hak/roboptim-core-plugin-quadprog.git
    cd roboptim-core-plugin-quadprog
    mkdir _build
    cd _build
    cmake -DCMAKE_INSTALL_PREFIX=prefix ..
    make 
    make install
