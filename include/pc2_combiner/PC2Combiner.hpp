#pragma once

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace pc2_combiner
{
class PC2Combiner : public rclcpp::Node
{
  public:
    /**
     * @brief Constructor of the main node class.
     */
    explicit PC2Combiner();

  private:
    /**
     * @brief Waits for the transform to be available and gets it.
     */
    void getTransform();

    /**
     * @brief Checks if the transform is ready.
     * 
     * @return true if the transform is available, otherwise false.
     */
    bool isTransformAvailable() const;

    void topic_callback(const sensor_msgs::msg::PointCloud2& msg) const;


    // Transform tools.
    tf2_ros::Buffer m_tf_buffer;
    tf2_ros::TransformListener m_tf_listener;

    // Stores the transform for velodyne_left.
    geometry_msgs::msg::TransformStamped m_tf_left;

    // Stores the transform for velodyne_right.
    geometry_msgs::msg::TransformStamped m_tf_right;
    
    // Stores the transform for velodyne_top.
    geometry_msgs::msg::TransformStamped m_tf_top;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};
}  // namespace pc2_combiner