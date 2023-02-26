#include "pc2_combiner/PC2Combiner.hpp"

namespace pc2_combiner
{
PC2Combiner::PC2Combiner()
  : Node{ "pc2_combiner" }
  , m_tf_buffer{ get_clock() }
  , m_tf_listener{ m_tf_buffer }
  , m_left_sub{ this, "/sensing/lidar/left/pointcloud_raw", rmw_qos_profile_sensor_data }
  , m_right_sub{ this, "/sensing/lidar/right/pointcloud_raw", rmw_qos_profile_sensor_data }
  , m_top_sub{ this, "/sensing/lidar/top/pointcloud_raw", rmw_qos_profile_sensor_data }
  , m_synchronizer{ policy_t(5), m_left_sub, m_right_sub, m_top_sub }
{
    RCLCPP_INFO(get_logger(), "Starting %s node.", get_name());
    getTransform();
    m_synchronizer.registerCallback(&PC2Combiner::cloudCallback, this);
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

void PC2Combiner::cloudCallback(const sensor_msgs::msg::PointCloud2& t_left_msg,
                                const sensor_msgs::msg::PointCloud2& t_right_msg,
                                const sensor_msgs::msg::PointCloud2& t_top_msg)
{
    RCLCPP_INFO(get_logger(), "yes");
    RCLCPP_INFO(get_logger(), "%d", t_left_msg.header.stamp.sec);
    RCLCPP_INFO(get_logger(), "%d", t_right_msg.header.stamp.sec);
    RCLCPP_INFO(get_logger(), "%d", t_top_msg.header.stamp.sec);

    PointCloud mycloud{ t_left_msg };

    for (size_t i = 0; i < mycloud.getPointCount(); ++i)
    {
        const Point mypoint{ mycloud.getCurrentPoint() };
        RCLCPP_INFO(get_logger(), "Point: (%f,%f,%f,%f,%d)", mypoint.x, mypoint.y, mypoint.z,
                    mypoint.intensity, mypoint.ring);
        mycloud.nextPoint();
    }

    // Create empty PC2
    // sensor_msgs::msg::PointCloud2 combined_cloud;

    // Publish combined cloud.
}

}  // namespace pc2_combiner