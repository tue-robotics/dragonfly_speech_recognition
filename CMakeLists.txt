cmake_minimum_required(VERSION 2.8.3)
project(dragonfly_speech_recognition)

find_package(catkin REQUIRED COMPONENTS
  hmi_msgs
  rospy
)

catkin_package()
