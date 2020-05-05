## Multi Agent Exploration and Pursuit
This repository contains code for multi-agent persistent coverage and target pursuit algorithms  
on a 4-connected grid map. The predactor has limited perception range and has no informa-  
tion about the world except its dimension, thus they perform persistent coverage planning  
and coordinated target pursuit once the prey is within their sight.

## Dependency 
Install [Opencv4](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html)

## Build
    mkdir build && cd ./build && cmake .. && make -j8

## Run with manual control
    ./build/bin/MEP

## Run with auto control
    ./build/bin/MEP auto

## Run test
    ./build/bin/run_unit_tests

  
