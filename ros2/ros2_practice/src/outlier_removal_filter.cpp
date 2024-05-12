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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

class OutlierRemovalFilterComponent : public rclcpp::Node {
public:
  OutlierRemovalFilterComponent() : Node("outlier_removal_filter") {
    mean_k_ = this->declare_parameter("mean_k", 50);
    stddev_mul_thresh_ = this->declare_parameter("stddev_mul_thresh", 0.1);

    rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filter_result", qos);
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points",
        qos,
        std::bind(&OutlierRemovalFilterComponent::PointCloud2Callback, this, std::placeholders::_1) \
    );
  }

private:
  void PointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> filter;
    filter.setInputCloud(cloud);
    filter.setMeanK(mean_k_);
    filter.setStddevMulThresh(stddev_mul_thresh_);
    filter.setNegative(false);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    filter.filter(*cloud_filtered);

    sensor_msgs::msg::PointCloud2::SharedPtr filtered_msg(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*cloud_filtered, *filtered_msg);
    filtered_msg->header = msg->header;
    pub_->publish(*filtered_msg);
  }

  int mean_k_;
  int stddev_mul_thresh_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};
