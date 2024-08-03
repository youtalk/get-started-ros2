cmake_minimum_required(VERSION 3.12)
project(ros2_practice)

# C++17の設定
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# 依存パッケージ
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS highgui imgproc objdetect)

# face_detectionノードのビルド設定
add_executable(face_detection src/face_detection.cpp)
ament_target_dependencies(face_detection
  "cv_bridge"
  "image_transport"
  "rclcpp"
  "sensor_msgs"
)
target_link_libraries(face_detection
  opencv_highgui
  opencv_imgproc
  opencv_objdetect
)

install(TARGETS
  face_detection
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
