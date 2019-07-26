cmake_minimum_required(VERSION 3.5)
project(hello_world)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
if($ENV{ROS_DISTRO} STREQUAL dashing)
  find_package(rclcpp_components REQUIRED)
endif()
find_package(hello_world_msgs REQUIRED)

# ビルド設定の便利関数
function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp"
    "hello_world_msgs")
  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME})
endfunction()

# talkerノードのビルド設定
custom_executable(talker)
custom_executable(talker_with_service)
if($ENV{ROS_DISTRO} STREQUAL dashing)
  custom_executable(talker_with_service_param)
endif()
# listenerノードのビルド設定
custom_executable(listener)
# client_asyncノードのビルド設定
custom_executable(client_async)
# talker/listenerノードのプロセス間通信のビルド設定
custom_executable(talker_listener_composition)
if($ENV{ROS_DISTRO} STREQUAL dashing)
  # パラメータ取得・設定ノードのビルド設定
  custom_executable(set_and_get_parameters)
endif()

include_directories(include)

if($ENV{ROS_DISTRO} STREQUAL dashing)
  # 共有ライブラリの生成
  add_library(talker_component SHARED
    src/talker_component.cpp)
  target_compile_definitions(talker_component
    PRIVATE "COMPOSITION_BUILDING_DLL")
  ament_target_dependencies(talker_component
    "rclcpp"
    "rclcpp_components"
    "hello_world_msgs")
  # クラスローダーへの共有ライブラリの登録
  rclcpp_components_register_nodes(talker_component
    "hell_world::TalkerComponent")

  # 共有ライブラリの生成
  add_library(listener_component SHARED
    src/listener_component.cpp)
  target_compile_definitions(listener_component
    PRIVATE "COMPOSITION_BUILDING_DLL")
  ament_target_dependencies(listener_component
    "rclcpp"
    "rclcpp_components"
    "hello_world_msgs")
  # クラスローダーへの共有ライブラリの登録
  rclcpp_components_register_nodes(listener_component
    "hello_world::ListenerComponent")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(TEST_LAUNCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test)
endif()

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
