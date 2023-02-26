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

/**
 * @brief This class takes the reference of an existing const PointCloud2 message and enables
 * iterating its points.
 *
 */
class PointCloud
{
  public:
    /**
     * @brief Constructs a new PointCloud object with the reference of an existing PointCloud2 msg.
     *
     * @param t_pointcloud2 The PointCloud2 message.
     */
    PointCloud(const sensor_msgs::msg::PointCloud2& t_pointcloud2)
      : m_pointcloud2{ t_pointcloud2 }
      , m_point_count{ m_pointcloud2.height * m_pointcloud2.width }
      , m_current_index{ 0 }
      , m_iter_x{ m_pointcloud2, "x" }
      , m_iter_y{ m_pointcloud2, "y" }
      , m_iter_z{ m_pointcloud2, "z" }
      , m_iter_intensity{ m_pointcloud2, "intensity" }
      , m_iter_ring{ m_pointcloud2, "ring" }
    {
    }

    /**
     * @brief Get the point count of the cloud.
     *
     * @return Point count.
     */
    unsigned int getPointCount() const
    {
        return m_point_count;
    }

    /**
     * @brief Get the current point of the cloud.
     *
     * @return Current point.
     */
    Point getCurrentPoint() const
    {
        const Point point{ *m_iter_x, *m_iter_y, *m_iter_z, *m_iter_intensity, *m_iter_ring };
        return point;
    }

    /**
     * @brief Increases iterators by 1.
     *
     */
    void nextPoint()
    {
        if (m_current_index < m_point_count)
        {
            ++m_iter_x;
            ++m_iter_y;
            ++m_iter_z;
            ++m_iter_intensity;
            ++m_iter_ring;
        }
    }

  private:
    // Reference of the existing PointCloud2.
    const sensor_msgs::msg::PointCloud2& m_pointcloud2;

    // Number of the points in the cloud.
    const unsigned int m_point_count;

    // Index of the current point.
    size_t m_current_index;

    // PointCloud2 iterators.
    sensor_msgs::PointCloud2ConstIterator<float> m_iter_x;
    sensor_msgs::PointCloud2ConstIterator<float> m_iter_y;
    sensor_msgs::PointCloud2ConstIterator<float> m_iter_z;
    sensor_msgs::PointCloud2ConstIterator<float> m_iter_intensity;
    sensor_msgs::PointCloud2ConstIterator<unsigned short> m_iter_ring;
};

}  // namespace pc2_combiner
