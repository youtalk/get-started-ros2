cmake_minimum_required(VERSION 2.8.3)
project(hello_world)

# 依存パッケージの発見
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs
  message_generation message_runtime)

# SetMessageサービスの生成
add_service_files(FILES SetMessage.srv)
generate_messages(DEPENDENCIES std_msgs)

# ヘッダーファイルの読込先設定
include_directories(include ${catkin_INCLUDE_DIRS})

# ビルド設定の便利関数
function(custom_executable target)
  add_executable(${target} ${target}.cpp)
  target_link_libraries(${target} ${catkin_LIBRARIES})
  add_dependencies(${target} hello_world_gencpp)
  install(TARGETS ${target}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endfunction()

# talkerノードのビルド設定
custom_executable(talker)
custom_executable(talker_with_service)
custom_executable(talker_with_service_param)
# listenerノードのビルド設定
custom_executable(listener)

# catkinパッケージ宣言
catkin_package()
