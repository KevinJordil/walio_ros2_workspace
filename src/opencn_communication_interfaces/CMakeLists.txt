cmake_minimum_required(VERSION 3.8)
project(opencn_communication_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CMPinBit.msg"
  "msg/CMPinFloat.msg"
  "msg/CMPinI32.msg"
  "msg/CMPinU32.msg"
  "msg/Pin.msg"
  "srv/Pins.srv"
  "msg/CMParamBit.msg"
  "msg/CMParamFloat.msg"
  "msg/CMParamI32.msg"
  "msg/CMParamU32.msg"
  "msg/Param.msg"
  "srv/Params.srv"
  "srv/Init.srv"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
