#!/bin/sh

#temp: close All
#cd ~/ && sh closeAll.sh &&

#Launch from package (assuming gazebo and gazebo_fifa are in the same directory)
#cd models 
#export GAZEBO_MODEL_PATH=$PWD
export GAZEBO_MODEL_PATH=`pwd`/models
#cd ../build
export GAZEBO_PLUGIN_PATH=`pwd`/build

#valgrind ./gazebo/build/gazebo/gzserver gazebo_fifa/worlds/factoryWorldPlugin.world --verbose  &   #TO ANALISE MEMORY LEAKS 
./gazebo/build/gazebo/gzserver worlds/factoryWorldPlugin.world --verbose  &   #DISABLE TO DEBUG

./gazebo/build/gazebo/gui/gzclient --verbose  &

./build/teleopJoystickGz
#./gazebo_fifa/build/teleopKeyboardGz

