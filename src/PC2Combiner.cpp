#include "pc2_combiner/PC2Combiner.hpp"

namespace pc2_combiner
{
PC2Combiner::PC2Combiner()
  : Node{ "pc2_combiner" }, m_tf_buffer{ get_clock() }, m_tf_listener{ m_tf_buffer }
{
    RCLCPP_INFO(get_logger(), "Starting %s node.", get_name());

    getTransform();

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/sensing/lidar/left/pointcloud_raw", rclcpp::SensorDataQoS(),
        std::bind(&PC2Combiner::topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Initialized %s node.", get_name());
}

void PC2Combiner::getTransform()
{
    try
    {
        rclcpp::Rate wait_rate{ 1 };
        while (!isTransformAvailable() && rclcpp::ok())
        {
            // Wait for the transform to become available.
            RCLCPP_INFO(get_logger(), "Waiting for transform.");
            wait_rate.sleep();
        }
        // Store transforms.
        m_tf_left = m_tf_buffer.lookupTransform("base_link", "velodyne_left", tf2::TimePointZero);
        m_tf_right = m_tf_buffer.lookupTransform("base_link", "velodyne_right", tf2::TimePointZero);
        m_tf_top = m_tf_buffer.lookupTransform("base_link", "velodyne_top", tf2::TimePointZero);
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_INFO(get_logger(), "Error. %s", ex.what());
    }
}

bool PC2Combiner::isTransformAvailable() const
{
    const bool tf_left{ m_tf_buffer.canTransform("base_link", "velodyne_left",
                                                 tf2::TimePointZero) };

    const bool tf_right{ m_tf_buffer.canTransform("base_link", "velodyne_right",
                                                  tf2::TimePointZero) };

    const bool tf_top{ m_tf_buffer.canTransform("base_link", "velodyne_top", tf2::TimePointZero) };

    return tf_left && tf_right && tf_top;
}

void PC2Combiner::topic_callback(const sensor_msgs::msg::PointCloud2& msg) const
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.header.frame_id.c_str());
    RCLCPP_INFO(get_logger(), "%f", m_tf_left.transform.translation.z);
    RCLCPP_INFO(get_logger(), "%f", m_tf_right.transform.translation.z);
    RCLCPP_INFO(get_logger(), "%f", m_tf_top.transform.translation.z);
}
}  // namespace pc2_combiner