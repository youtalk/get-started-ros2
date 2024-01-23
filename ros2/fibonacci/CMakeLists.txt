cmake_minimum_required(VERSION 3.5)

project(fibonacci)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(hello_world_msgs REQUIRED)

# ビルド設定の便利関数
function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp"
    "rclcpp_action"
    "hello_world_msgs")
  install(TARGETS ${target}
    DESTINATION lib)
endfunction()

# action_serverノードのビルド設定
custom_executable(action_server)
# action_clientノードのビルド設定
custom_executable(action_client)

ament_package()
