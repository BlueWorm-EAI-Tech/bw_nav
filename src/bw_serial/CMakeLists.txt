cmake_minimum_required(VERSION 3.18)
project(bw_serial)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror -Wno-unused-variable)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(serial_driver REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bw_interface REQUIRED)
find_package(bw_utils REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# find dependencies
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CtrlCmd.msg"
  "msg/ForceFdb.msg"
  "msg/MantisCommandV2.msg"
  "msg/MantisFeedbackV2.msg"
  "srv/SetCtrlSrc.srv"
  DEPENDENCIES builtin_interfaces
)
# Make the generated interfaces available to other packages
ament_export_dependencies(rosidl_default_runtime)

# Component Library
add_library(mantis_comm_node_component SHARED
  src/mantis_comm_node.cpp
  src/pelvis_jog_controller.cpp
  src/waist_jog_controller.cpp
  src/v3_frame_codec.cpp
  src/v3_waist_safety.cpp
)
target_include_directories(mantis_comm_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mantis_comm_node_component
  rclcpp
  rclcpp_components
  serial_driver
  sensor_msgs
  std_msgs
  bw_interface
  bw_utils
  rcl_interfaces
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(mantis_comm_node_component
  "${cpp_typesupport_target}"
)

# Register component
rclcpp_components_register_nodes(mantis_comm_node_component
  "bw_serial::MantisCommNode"
)

add_executable(mantis_comm_node
  src/mantis_comm_node_main.cpp
)
target_link_libraries(mantis_comm_node
  mantis_comm_node_component
)

# Install
install(TARGETS mantis_comm_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS mantis_comm_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(v3_waist_safety_test
    test/v3_waist_safety_test.cpp
  )
  if(TARGET v3_waist_safety_test)
    target_include_directories(v3_waist_safety_test PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(v3_waist_safety_test
      mantis_comm_node_component
    )
  endif()

  ament_add_gtest(v3_frame_codec_test
    test/v3_frame_codec_test.cpp
  )
  if(TARGET v3_frame_codec_test)
    target_include_directories(v3_frame_codec_test PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(v3_frame_codec_test
      mantis_comm_node_component
    )
  endif()

  ament_add_gtest(pelvis_jog_controller_test
    test/pelvis_jog_controller_test.cpp
  )
  if(TARGET pelvis_jog_controller_test)
    target_include_directories(pelvis_jog_controller_test PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(pelvis_jog_controller_test
      mantis_comm_node_component
    )
  endif()

  ament_add_gtest(waist_jog_controller_test
    test/waist_jog_controller_test.cpp
  )
  if(TARGET waist_jog_controller_test)
    target_include_directories(waist_jog_controller_test PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(waist_jog_controller_test
      mantis_comm_node_component
    )
  endif()

  ament_add_pytest_test(v3_test_publisher
    test/test_v3_test_publisher.py
  )
  ament_add_pytest_test(v3_control_flag
    test/test_v3_control_flag.py
  )
  ament_add_pytest_test(v3_chassis_accel
    test/test_v3_chassis_accel.py
  )
  ament_add_pytest_test(v3_hardware_shape_auto_detection
    test/test_v3_hardware_shape_auto_detection.py
  )
endif()

install(PROGRAMS
  scripts/v3_test_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
