#include "bw_serial/mantis_comm_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bw_serial::MantisCommNode>();
    // 多线程 executor：发送定时器(独立回调组)与订阅回调分线程运行，
    // 避免单线程下 500Hz 发送被其它回调阻塞造成串口发送抖动
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
