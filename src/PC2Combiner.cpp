#include "pc2_combiner/PC2Combiner.hpp"

namespace pc2_combiner
{
PC2Combiner::PC2Combiner() : Node{ "pc2_combiner" }
{
    RCLCPP_INFO(get_logger(), "Hello World!");
}
}  // namespace pc2_combiner