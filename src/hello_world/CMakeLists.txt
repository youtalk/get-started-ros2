cmake_minimum_required(VERSION 3.12)
project(hello_world)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
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
custom_executable(talker_with_service_param)
# listenerノードのビルド設定
custom_executable(listener)
# client_syncノードのビルド設定
custom_executable(client_sync)
# client_asyncノードのビルド設定
custom_executable(client_async)
# talker/listenerノードのプロセス間通信のビルド設定
custom_executable(talker_listener_composition)
# パラメータ取得・設定ノードのビルド設定
custom_executable(set_and_get_parameters)

include_directories(include)

# コンポーネント設定の便利関数
function(custom_component target class_name)
  # クラスローダーへの共有ライブラリの登録
  rclcpp_components_register_node(${target}
    PLUGIN ${class_name}
    EXECUTABLE ${target})
  ament_target_dependencies(${target}
    "rclcpp"
    "rclcpp_components"
    "hello_world_msgs")
endfunction()

# 共有ライブラリの生成
custom_component(talker_component "hello_world::TalkerComponent")
custom_component(listener_component "hello_world::ListenerComponent")

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
