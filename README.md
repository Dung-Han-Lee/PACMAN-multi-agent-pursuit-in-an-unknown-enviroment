## Authors
This work was made possible by [Stanley Winata](https://github.com/stanleywinata) and Dung-Han Lee.

## Multi-Agent Pursuit in Unknown Environment 
This repository contains code for multi-agent persistent coverage and target pursuit algorithms  
on a 4-connected grid map. The predactor has limited perception range and has no informa-  
tion about the world except its dimension, thus they perform persistent coverage planning  
and coordinated target pursuit once the prey is within their sight.

[![Output sample](https://media.giphy.com/media/Rfk5DFsDIDOHGf7HlK/giphy.gif)](https://youtu.be/N6bs6yq623Y)

## Dependency 
Install [Opencv4](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html)

## Build
    mkdir build && cd ./build && cmake .. && make -j8

## Run with manual control
    ./build/bin/MEP

## Run with auto control
    ./build/bin/MEP auto

  
