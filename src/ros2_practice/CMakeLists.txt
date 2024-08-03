cmake_minimum_required(VERSION 3.12)
project(ros2_practice)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS highgui imgproc objdetect)

add_executable(face_detection src/face_detection.cpp)
target_link_libraries(face_detection
  cv_bridge::cv_bridge
  image_transport::image_transport
  opencv_highgui
  opencv_imgproc
  opencv_objdetect
  rclcpp::rclcpp
  "${sensor_msgs_TARGETS}"
)
install(TARGETS
  face_detection
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
