cmake_minimum_required(VERSION 3.0.2)
project(gazeboSoccer)

#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

#For joystick
#SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11") #para activar cpp11 y multihilos  -pthread o -lpthread 

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++17)


find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
#include_directories(include ${Boost_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS}) #headers in directory  /include
include_directories( ${Boost_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS})



#----------- gaGAZEBOFIFA V3.0 (with gz transport)

#Only gazebo transport 
add_executable(teleopJoystickGz src/apiJoystickGz.cc src/joystick.cc)
target_link_libraries(teleopJoystickGz ${GAZEBO_LIBRARIES})
add_executable(teleopKeyboardGz src/keyboard.cc)
target_link_libraries(teleopKeyboardGz ${GAZEBO_LIBRARIES})


#----------- gaGAZEBOFIFA V5.0 (without gz transport)
add_library(factoryWorldPlugin SHARED src/factoryWorldPlugin.cc src/FEntity.cpp src/FScene.cpp src/FPlayer.cpp src/FBall.cpp src/FStadium.cpp src/FTeam.cpp)
target_link_libraries(factoryWorldPlugin ${GAZEBO_LIBRARIES})

#state machine
#ADD_EXECUTABLE(mainPlayer src/mainPlayer.cpp src/PlayerStateA.cpp src/PlayerStateB.cpp  )    
#target_link_libraries(mainPlayer ${GAZEBO_LIBRARIES}  )    