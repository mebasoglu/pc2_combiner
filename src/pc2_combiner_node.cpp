#include "rclcpp/rclcpp.hpp"
#include "pc2_combiner/PC2Combiner.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pc2_combiner::PC2Combiner>());
    rclcpp::shutdown();
    return 0;
}