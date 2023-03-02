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

catkin_install_python(
  PROGRAMS
    node_scripts/act_as_ambulance.py
    node_scripts/broadcast_test_packet.py
    node_scripts/emergency_caller.py
    node_scripts/esp_now_packet_printer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
