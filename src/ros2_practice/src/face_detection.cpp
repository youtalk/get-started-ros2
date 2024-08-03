// Copyright 2024 Yutaka Kondo
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>

const char* kWindowName = "Result";
const char* kClassifierFileName = "haarcascade_frontalface_alt.xml";

class FaceDetectionComponent : public rclcpp::Node {
public:
  FaceDetectionComponent() : Node("face_detection") {
    cv::namedWindow(kWindowName);

    if (!classifier_.load(kClassifierFileName)) {
      RCLCPP_ERROR(this->get_logger(), "%s not found", kClassifierFileName);
      std::abort();
    }

    rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
    pub_ = image_transport::create_publisher(this, "face_detection_result", qos);
    sub_ = image_transport::create_subscription(this, "/camera/color/image_raw", std::bind(&FaceDetectionComponent::ImageCallback, this, std::placeholders::_1), "raw", qos);
  }

  ~FaceDetectionComponent() {
    cv::destroyWindow(kWindowName);
  }

private:
  void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    cv_bridge::CvImagePtr cv_image;
    try {
      cv_image = cv_bridge::toCvCopy(msg, msg->encoding);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());
      return;
    }

    cv::Mat gray;
    cv::cvtColor(cv_image->image, gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(gray, gray);
    std::vector<cv::Rect> faces;
    classifier_.detectMultiScale(gray, faces, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
    for (auto face: faces) {
      cv::rectangle(cv_image->image, face, cv::Scalar(255, 0, 0), 2);
    }

    cv::imshow(kWindowName, cv_image->image);
    cv::waitKey(3);
    pub_.publish(cv_image->toImageMsg());
  }

  cv::CascadeClassifier classifier_;
  image_transport::Publisher pub_;
  image_transport::Subscriber sub_;
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FaceDetectionComponent>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
