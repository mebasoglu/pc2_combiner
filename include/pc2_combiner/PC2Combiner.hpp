#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "pc2_combiner/PointCloudType.hpp"

namespace pc2_combiner
{

// ApproximateTime policy type for message filter <PC2, PC2, PC2>.
using policy_t = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;

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

    /**
     * @brief Callback function for the message filter.
     *
     * @param t_left_msg The left lidar PointCloud2 message.
     * @param t_right_msg The right lidar PointCloud2 message.
     * @param t_top_msg The top lidar PointCloud2 message.
     */
    void cloudCallback(const sensor_msgs::msg::PointCloud2& t_left_msg,
                       const sensor_msgs::msg::PointCloud2& t_right_msg,
                       const sensor_msgs::msg::PointCloud2& t_top_msg);

    // Transform tools.
    tf2_ros::Buffer m_tf_buffer;
    tf2_ros::TransformListener m_tf_listener;

    // Stores the transform for velodyne_left.
    geometry_msgs::msg::TransformStamped m_tf_left;

    // Stores the transform for velodyne_right.
    geometry_msgs::msg::TransformStamped m_tf_right;

    // Stores the transform for velodyne_top.
    geometry_msgs::msg::TransformStamped m_tf_top;

    // Subscriber for velodyne_left.
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_left_sub;

    // Subscriber for velodyne_right.
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_right_sub;

    // Subscriber for velodyne_top.
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_top_sub;

    // Message filter synchronizer.
    message_filters::Synchronizer<policy_t> m_synchronizer;

    // Publisher for combined cloud.
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> m_publisher;
};
}  // namespace pc2_combiner