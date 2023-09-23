cmake_minimum_required(VERSION 2.8.3)
project(esp_now_ros)

find_package(catkin REQUIRED COMPONENTS message_generation catkin_virtualenv)

catkin_generate_virtualenv(
  PYTHON_INTERPRETER python3
  CHECK_VENV FALSE 
)

catkin_python_setup()

add_message_files(
  FILES
  Packet.msg
)

generate_messages()

catkin_package(
    CATKIN_DEPENDS message_runtime
)

file(GLOB ${PROJECT_NAME}_node_scripts ${PROJECT_SOURCE_DIR}/node_scripts/*)

catkin_install_python(
  PROGRAMS ${${PROJECT_NAME}_node_scripts}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
