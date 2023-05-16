cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_speech_recognition)

find_package(catkin REQUIRED)
catkin_package(DEPENDS)

catkin_python_setup()

install(PROGRAMS src/audioTreatment.py src/voiceControl.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

