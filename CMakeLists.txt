cmake_minimum_required(VERSION 3.8)
project(dji_serial_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Find dependencies ──────────────────────────────────────────────────────────
find_package(ament_cmake        REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(geometry_msgs      REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── Generate custom messages ───────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotPose.msg"
  "msg/RefSysStatus.msg"
  "msg/CVTarget.msg"
  DEPENDENCIES std_msgs
)

# Make generated headers available before the executable target is defined.
ament_export_dependencies(rosidl_default_runtime)

# ── Bridge node executable ─────────────────────────────────────────────────────
add_executable(dji_serial_bridge_node
  src/dji_serial_bridge_node.cpp
  src/crc_dji.cpp
)

target_include_directories(dji_serial_bridge_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

ament_target_dependencies(dji_serial_bridge_node
  rclcpp
  std_msgs
  geometry_msgs
)

# Link the generated message type-support to the executable.
rosidl_target_interfaces(dji_serial_bridge_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# ── Install ────────────────────────────────────────────────────────────────────
install(TARGETS dji_serial_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/test_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
