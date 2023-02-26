#pragma once

#include <iostream>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace pc2_combiner
{

struct Point
{
    float x;
    float y;
    float z;
    float intensity;
    unsigned short ring;
};

class PointCloudIterator
{
  public:
    PointCloudIterator(const sensor_msgs::msg::PointCloud2& t_cloud_left,
                       const sensor_msgs::msg::PointCloud2& t_cloud_right,
                       const sensor_msgs::msg::PointCloud2& t_cloud_top,
                       sensor_msgs::msg::PointCloud2& t_cloud_combined,
                       std::array<unsigned int, 3> t_point_counts)
      : m_cloud_left{ t_cloud_left }
      , m_cloud_right{ t_cloud_right }
      , m_cloud_top{ t_cloud_top }
      , m_cloud_combined{ t_cloud_combined }
      , m_cloud_left_x{ m_cloud_left, "x" }
      , m_cloud_left_y{ m_cloud_left, "y" }
      , m_cloud_left_z{ m_cloud_left, "z" }
      , m_cloud_left_intensity{ m_cloud_left, "intensity" }
      , m_cloud_left_ring{ m_cloud_left, "ring" }

      , m_cloud_right_x{ m_cloud_right, "x" }
      , m_cloud_right_y{ m_cloud_right, "y" }
      , m_cloud_right_z{ m_cloud_right, "z" }
      , m_cloud_right_intensity{ m_cloud_right, "intensity" }
      , m_cloud_right_ring{ m_cloud_right, "ring" }

      , m_cloud_top_x{ m_cloud_top, "x" }
      , m_cloud_top_y{ m_cloud_top, "y" }
      , m_cloud_top_z{ m_cloud_top, "z" }
      , m_cloud_top_intensity{ m_cloud_top, "intensity" }
      , m_cloud_top_ring{ m_cloud_top, "ring" }

      , m_cloud_combined_x{ m_cloud_combined, "x" }
      , m_cloud_combined_y{ m_cloud_combined, "y" }
      , m_cloud_combined_z{ m_cloud_combined, "z" }
      , m_cloud_combined_intensity{ m_cloud_combined, "intensity" }
      , m_cloud_combined_ring{ m_cloud_combined, "ring" }

      , m_current_index{ 0 }
      , m_point_counts{ t_point_counts }
    {
        m_point_count = *std::max_element(m_point_counts.begin(), m_point_counts.end());
    }

    void iterateClouds()
    {
        while (m_current_index < m_point_count)
        {
            // Transform points
            std::cout << "Left ring: " << *m_cloud_left_ring << '\n';

            // Append transformed points to combined cloud.

            // Go to next point.
            nextPoint();
        }
    }

  private:
    void nextPoint()
    {
        if (m_current_index < m_point_counts[0])
        {
            ++m_cloud_left_x;
            ++m_cloud_left_y;
            ++m_cloud_left_z;
            ++m_cloud_left_intensity;
            ++m_cloud_left_ring;
        }

        if (m_current_index < m_point_counts[1])
        {
            ++m_cloud_right_x;
            ++m_cloud_right_y;
            ++m_cloud_right_z;
            ++m_cloud_right_intensity;
            ++m_cloud_right_ring;
        }

        if (m_current_index < m_point_counts[2])
        {
            ++m_cloud_top_x;
            ++m_cloud_top_y;
            ++m_cloud_top_z;
            ++m_cloud_top_intensity;
            ++m_cloud_top_ring;
        }

        ++m_cloud_combined_x;
        ++m_cloud_combined_y;
        ++m_cloud_combined_z;
        ++m_cloud_combined_intensity;
        ++m_cloud_combined_ring;
    }

    const sensor_msgs::msg::PointCloud2& m_cloud_left;
    const sensor_msgs::msg::PointCloud2& m_cloud_right;
    const sensor_msgs::msg::PointCloud2& m_cloud_top;
    sensor_msgs::msg::PointCloud2& m_cloud_combined;

    sensor_msgs::PointCloud2ConstIterator<float> m_cloud_left_x;
    sensor_msgs::PointCloud2ConstIterator<float> m_cloud_left_y;
    sensor_msgs::PointCloud2ConstIterator<float> m_cloud_left_z;
    sensor_msgs::PointCloud2ConstIterator<float> m_cloud_left_intensity;
    sensor_msgs::PointCloud2ConstIterator<float> m_cloud_left_ring;

    sensor_msgs::PointCloud2ConstIterator<float> m_cloud_right_x;
    sensor_msgs::PointCloud2ConstIterator<float> m_cloud_right_y;
    sensor_msgs::PointCloud2ConstIterator<float> m_cloud_right_z;
    sensor_msgs::PointCloud2ConstIterator<float> m_cloud_right_intensity;
    sensor_msgs::PointCloud2ConstIterator<float> m_cloud_right_ring;

    sensor_msgs::PointCloud2ConstIterator<float> m_cloud_top_x;
    sensor_msgs::PointCloud2ConstIterator<float> m_cloud_top_y;
    sensor_msgs::PointCloud2ConstIterator<float> m_cloud_top_z;
    sensor_msgs::PointCloud2ConstIterator<float> m_cloud_top_intensity;
    sensor_msgs::PointCloud2ConstIterator<float> m_cloud_top_ring;

    sensor_msgs::PointCloud2Iterator<float> m_cloud_combined_x;
    sensor_msgs::PointCloud2Iterator<float> m_cloud_combined_y;
    sensor_msgs::PointCloud2Iterator<float> m_cloud_combined_z;
    sensor_msgs::PointCloud2Iterator<float> m_cloud_combined_intensity;
    sensor_msgs::PointCloud2Iterator<float> m_cloud_combined_ring;

    unsigned int m_point_count;
    size_t m_current_index;
    std::array<unsigned int, 3>& m_point_counts;
};

}  // namespace pc2_combiner
