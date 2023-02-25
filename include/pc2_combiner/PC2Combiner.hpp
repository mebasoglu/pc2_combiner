#pragma once

#include "rclcpp/rclcpp.hpp"

namespace pc2_combiner
{
class PC2Combiner : public rclcpp::Node
{
  public:
    /*!
     * Constructor.
     */
    explicit PC2Combiner();
};
}  // namespace pc2_combiner