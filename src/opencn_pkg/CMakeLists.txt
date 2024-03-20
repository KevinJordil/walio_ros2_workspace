cmake_minimum_required(VERSION 3.8)
project(opencn_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(opencn_communication_interfaces REQUIRED)

# Add the following line to find capnproto if not already included
find_library(CAPNPROTO_RPC_LIBRARY NAMES capnp-rpc-0.7.0 PATHS /usr/local/lib)
if(NOT CAPNPROTO_RPC_LIBRARY)
    message(FATAL_ERROR "capnp-rpc-0.7.0 library not found")
endif()

# Ajouter les fichiers source à votre exécutable ou à une bibliothèque si nécessaire
add_executable(opencn_node 
  src/opencn_node.cpp
  src/transaction.cpp
  src/opencn_interface.capnp.c++
  ) 
ament_target_dependencies(opencn_node rclcpp std_msgs sensor_msgs opencn_communication_interfaces)

# Inclure le répertoire où se trouvent vos fichiers d'en-tête
target_include_directories(opencn_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Spécifier les fonctionnalités du langage pour votre exécutable
target_compile_features(opencn_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Ajouter la bibliothèque capnproto à la liste des bibliothèques liées
target_link_libraries(opencn_node capnp-rpc-0.7.0 capnp-0.7.0)

target_link_libraries(opencn_node kj-async-0.7.0 kj-0.7.0)

# Lier la bibliothèque .so à votre exécutable
target_link_libraries(opencn_node ${CMAKE_CURRENT_SOURCE_DIR}/lib/libcmctl-client.so)

install(TARGETS opencn_node
  DESTINATION lib/${PROJECT_NAME})

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
