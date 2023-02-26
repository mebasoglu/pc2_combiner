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

    // Wait and get transforms.
    getTransform();

    // Set the message filter callback.
    m_synchronizer.registerCallback(&PC2Combiner::cloudCallback, this);

    // Create publisher for combined PointCloud.
    m_publisher = create_publisher<sensor_msgs::msg::PointCloud2>("asd", rclcpp::SensorDataQoS());

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
    // Create empty PointCloud.
    sensor_msgs::msg::PointCloud2 combined_cloud_pc2;
    PointCloud combined_cloud{ combined_cloud_pc2 };

    PointCloudConst left_cloud{ t_left_msg };
    PointCloudConst right_cloud{ t_right_msg };
    PointCloudConst top_cloud{ t_top_msg };
    // RCLCPP_INFO(get_logger(), "Created clouds.");

    // Get the point counts to find max and sum of them.
    std::array<unsigned int, 3> point_counts;
    point_counts[0] = left_cloud.getPointCount();
    point_counts[1] = right_cloud.getPointCount();
    point_counts[2] = top_cloud.getPointCount();
    unsigned int max_count{ *std::max_element(point_counts.begin(), point_counts.end()) };
    unsigned int sum{ 0 };
    for (const auto& count : point_counts)
        sum += count;
    // RCLCPP_INFO(get_logger(), "Calculated max and sum.");

    // Set the size of the combined cloud to sum.
    combined_cloud.setFieldsAndResize(sum);
    // RCLCPP_INFO(get_logger(), "Configured combined cloud.");

    // Iterate through point clouds, transform points and append them to combined cloud.
    for (size_t i = 0; i < max_count; ++i)
    {
        if (left_cloud.getCurrentIndex() < left_cloud.getPointCount())
        {
            // Transform and append the point from the left cloud.
            Point left_point{ left_cloud.getCurrentPoint() };
            left_point.transform(m_tf_left);
            // RCLCPP_INFO(get_logger(), "Transformed left point.");
            combined_cloud.append(left_point);
            // RCLCPP_INFO(get_logger(), "Appended left point.");
        }

        if (right_cloud.getCurrentIndex() < right_cloud.getPointCount())
        {
            // Transform and append the point from the right cloud.
            Point right_point{ right_cloud.getCurrentPoint() };
            right_point.transform(m_tf_right);
            // RCLCPP_INFO(get_logger(), "Transformed right point.");
            combined_cloud.append(right_point);
            // RCLCPP_INFO(get_logger(), "Appended right point.");
        }

        if (top_cloud.getCurrentIndex() < top_cloud.getPointCount())
        {
            // Transform and append the point from the top cloud.
            Point top_point{ top_cloud.getCurrentPoint() };
            top_point.transform(m_tf_top);
            // RCLCPP_INFO(get_logger(), "Transformed top point.");
            combined_cloud.append(top_point);
            // RCLCPP_INFO(get_logger(), "Appended top point.");
        }

        left_cloud.nextPoint();
        right_cloud.nextPoint();
        top_cloud.nextPoint();
        // RCLCPP_INFO(get_logger(), "Went next.");
    }

    // Publish combined cloud.
    combined_cloud_pc2.header.stamp = now();
    m_publisher->publish(combined_cloud_pc2);
}

}  // namespace pc2_combiner