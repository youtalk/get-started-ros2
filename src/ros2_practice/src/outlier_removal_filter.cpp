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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class OutlierRemovalFilter : public rclcpp::Node {
public:
  OutlierRemovalFilter()
  : Node("outlier_removal_filter")
  {
    leaf_size_ = declare_parameter("leaf_size", 0.05);
    RCLCPP_INFO(this->get_logger(), "leaf_size: %f", leaf_size_);

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    // 外れ値除去結果のトピック送信
    pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
        "filter_result", qos);
    // RealSenseカメラの点群のトピック受信
    sub_ =
      create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/camera/depth/color/points", qos,
        std::bind(&OutlierRemovalFilter::PointCloud2Callback, this,
          std::placeholders::_1));
  }

private:
  void PointCloud2Callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);
    filter.filter(*cloud_filtered);

    sensor_msgs::msg::PointCloud2::SharedPtr msg_filtered(
      new sensor_msgs::msg::PointCloud2);
    // pcl::PointCloud型からsensor_msgs/PointCloud2型への変換
    pcl::toROSMsg(*cloud_filtered, *msg_filtered);
    msg_filtered->header = msg->header;
    pub_->publish(*msg_filtered);
  }

  double leaf_size_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OutlierRemovalFilter>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
