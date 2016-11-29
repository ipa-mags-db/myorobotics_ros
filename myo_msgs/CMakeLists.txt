cmake_minimum_required(VERSION 2.8.3)
project(myo_msgs)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

# Add message and service files
add_message_files(
  FILES
  statusMessage.msg
  analogMessage.msg
  )

add_service_files(
  FILES
  SetReference.srv
  SetPID.srv
  SetClutch.srv
  )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs
  )

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
  )
