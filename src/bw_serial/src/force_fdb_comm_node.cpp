#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <serial_driver/serial_driver.hpp>
#include "bw_serial/msg/force_fdb.hpp"
#include "protocol.hpp"
#include "comm_utils.hpp"

namespace bw_serial
{
    class ForceFeedbackCommNode : public rclcpp::Node
    {
    public:
        ForceFeedbackCommNode() : Node("force_feedback_comm_node")
        {
            this->declare_parameter("port_name", "/dev/ttyUSB0");
            this->declare_parameter("baud_rate", 115200);
            this->declare_parameter("publish_frequency", 200);
            this->get_parameter("port_name", this->port_name_);
            this->get_parameter("baud_rate", this->baud_rate_);
            this->get_parameter("publish_frequency", this->publish_frequency_);

            this->left_controller_cmd_sub_ = this->create_subscription<bw_serial::msg::CtrlCmd>(
                "Teleop/controller_cmd_left", 10,
                std::bind(&ForceFeedbackCommNode::left_controller_cmd_sub_callback, this, std::placeholders::_1)
            );
            this->right_controller_cmd_sub_ = this->create_subscription<bw_serial::msg::CtrlCmd>(
                "Teleop/controller_cmd_right", 10,
                std::bind(&ForceFeedbackCommNode::right_controller_cmd_sub_callback, this, std::placeholders::_1)
            );
            this->left_force_fdb_pub_ = this->create_publisher<bw_serial::msg::ForceFdb>(
                "force_feedback_left", 10
            );
            this->right_force_fdb_pub_ = this->create_publisher<bw_serial::msg::ForceFdb>(
                "force_feedback_right", 10
            );

            // 初始化串口
            drivers::serial_driver::SerialPortConfig config(
                this->baud_rate_,
                drivers::serial_driver::FlowControl::NONE,
                drivers::serial_driver::Parity::NONE,
                drivers::serial_driver::StopBits::ONE
            );
            RCLCPP_INFO(this->get_logger(), "Serial port openning...");
            try
            {
                io_context_ = std::make_shared<drivers::common::IoContext>(1);
                // 初始化 serial_driver_
                serial_driver_ = std::make_shared<drivers::serial_driver::SerialDriver>(*io_context_);
                serial_driver_->init_port(this->port_name_, config);
                serial_driver_->port()->open();

                RCLCPP_INFO(this->get_logger(), "Serial port initialized successfully");
                RCLCPP_INFO(this->get_logger(), "Using device: %s", serial_driver_->port().get()->device_name().c_str());
                RCLCPP_INFO(this->get_logger(), "Baud_rate: %d", config.get_baud_rate());
            }
            catch (const std::exception &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port: %s", ex.what());
                return;
            }
            // 设置发送定时器
            transmit_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000 / this->publish_frequency_)),
                std::bind(&ForceFeedbackCommNode::transmit_timer_callback, this)
            );

            async_receive_message(); // 进入异步接收
        }

    private:
        void transmit_timer_callback()
        {
            /**
             * @brief 上位机 -> 下位机
             * @details 
             */
            try
            {
#if 1
                // 将结构体指针重新解释为字节指针，然后 std::vector 直接从原始结构体转换为字节数组
                transmit_data_buffer = std::vector<uint8_t>(
                    reinterpret_cast<const uint8_t*>(&this->controller_cmd_),
                    reinterpret_cast<const uint8_t*>(&this->controller_cmd_) + sizeof(ControllerCommand_t)
                );
#else
                auto serialized_buffer = serialize(this->controller_cmd_);
                transmit_data_buffer = std::vector<uint8_t>(serialized_buffer, serialized_buffer + sizeof(serialized_buffer));
#endif
                // serial_driver_->port()->send() 函数是真正的发送函数
                size_t bytes_transmit_size = serial_driver_->port()->send(transmit_data_buffer);
                std::stringstream ss;
                ss << std::hex << std::setfill('0');
                for (size_t i = 0; i < bytes_transmit_size; ++i)
                {
                    ss << " 0x" << std::setw(2) << static_cast<int>(transmit_data_buffer[i]);
                }
                RCLCPP_INFO(this->get_logger(), "Transmitted %ld bytes:%s", bytes_transmit_size, ss.str().c_str());
            }
            catch (const std::exception &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "Error Transmiting from serial port:%s", ex.what());
            }
        }
        
        // 异步接收串口数据
        void async_receive_message()
        {
            auto port = serial_driver_->port();

            // 设置接收回调函数
            port->async_receive(
                [this](const std::vector<uint8_t> &data, const size_t &size)
                {
                    if (size > 0)
                    {
                        // 处理接收到的数据
                        // RCLCPP_INFO(this->get_logger(), "Received %zu bytes from serial port", size);
                        this->receive_data_buffer.insert(
                            receive_data_buffer.end(), data.begin(), data.begin() + size
                        );
                        while (receive_data_buffer.size() >= FORCE_FEEDBACK_FRAME_SIZE) { // 为一帧长度
#if  1   // 打印十六进制原始数据
                            std::stringstream ss;
                            ss << std::hex << std::setfill('0');
                            for (size_t i = 0; i < receive_data_buffer.size(); ++i)
                            {
                                ss << " 0x" << std::setw(2) << static_cast<int>(receive_data_buffer[i]);
                            }
                            // RCLCPP_INFO(this->get_logger(), "Received %ld bytes:%s",  receive_data_buffer.size(), ss.str().c_str());
                            #endif
                            // 查找帧头
                            auto it = std::find(receive_data_buffer.begin(), receive_data_buffer.end(), FORCE_FEEDBACK_FRAME_HEADER);
                            if (it == receive_data_buffer.end() || std::distance(it, receive_data_buffer.end()) < static_cast<int>(FORCE_FEEDBACK_FRAME_SIZE)) {
                                // 没有帧头或剩余数据不足一帧，等待下次
                                break;
                            }
                            size_t pos = std::distance(receive_data_buffer.begin(), it);
                            // 检查帧尾
                            if (*(it + FORCE_FEEDBACK_FRAME_SIZE - 1) == FORCE_FEEDBACK_FRAME_TAIL) {
                                // 提取并处理一帧数据
                                std::vector<uint8_t> frame(it, it + FORCE_FEEDBACK_FRAME_TAIL);
                                this->force_fdb_ = deserialize<ForceFeedback_t>(frame.data());
                                this->logger_force_fdb(this->force_fdb_);
                                // 发布力反馈消息 
                                this->left_force_fdb_msg.status = this->force_fdb_.status;
                                this->left_force_fdb_msg.force_feedback = this->force_fdb_.left_force_feedback;
                                
                                this->right_force_fdb_msg.status = this->force_fdb_.status;
                                this->right_force_fdb_msg.force_feedback = this->force_fdb_.right_force_feedback;

                                this->left_force_fdb_pub_->publish(this->left_force_fdb_msg);
                                this->right_force_fdb_pub_->publish(this->right_force_fdb_msg);
                                // 移除已处理数据
                                receive_data_buffer.erase(receive_data_buffer.begin(), it + FORCE_FEEDBACK_FRAME_SIZE);
                            } else {
                                // 帧头后不是合法帧，丢弃该帧头，继续查找
                                RCLCPP_WARN(this->get_logger(), "Frame header found but no valid frame tail, discarding frame header.");
                                receive_data_buffer.erase(receive_data_buffer.begin(), it + 1);
                            }
                        }
                    }
                    // 继续监听新的数据
                    async_receive_message();
                }
            );
        }

        void left_controller_cmd_sub_callback(const bw_serial::msg::CtrlCmd::SharedPtr msg)
        {
            // RCLCPP_INFO(this->get_logger(), "Received controller command.");
            controller_cmd_.right_vel_x = msg->right_vel_x;
            controller_cmd_.right_vel_y = msg->right_vel_y;
            controller_cmd_.right_motor_vel = msg->right_motor_vel;
            auto rb1 = msg->right_button_1 ? 1 : 0;
            auto rb2 = msg->right_button_2 ? 1 : 0;
            auto rb3 = msg->right_button_3 ? 1 : 0;
            auto rb4 = msg->right_button_4 ? 1 : 0;
            controller_cmd_.right_button_val = rb1 | (rb2 << 1) | (rb3 << 2) | (rb4 << 3);
        }

        void right_controller_cmd_sub_callback(const bw_serial::msg::CtrlCmd::SharedPtr msg)
        {
            // RCLCPP_INFO(this->get_logger(), "Received controller command.");
            controller_cmd_.left_vel_x = msg->right_vel_x;
            controller_cmd_.left_vel_y = msg->right_vel_y;
            controller_cmd_.left_motor_vel = msg->right_motor_vel;
            auto rb1 = msg->right_button_1 ? 1 : 0;
            auto rb2 = msg->right_button_2 ? 1 : 0;
            auto rb3 = msg->right_button_3 ? 1 : 0;
            auto rb4 = msg->right_button_4 ? 1 : 0;
            controller_cmd_.left_button_val = rb1 | (rb2 << 1) | (rb3 << 2) | (rb4 << 3);
        }

        void logger_force_fdb(const ForceFeedback_t &force_fdb)
        {
            RCLCPP_INFO(this->get_logger(), "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
            RCLCPP_INFO(this->get_logger(), "+ Received Force Feedback Data:");
            RCLCPP_INFO(this->get_logger(), "+   Frame Header: 0x%02X", force_fdb.frame_header);
            RCLCPP_INFO(this->get_logger(), "+   Frame Length: %d", force_fdb.frame_length);
            RCLCPP_INFO(this->get_logger(), "+   Status: %d", force_fdb.status);
            RCLCPP_INFO(this->get_logger(), "+   Right Force Feedback: %.2f", force_fdb.right_force_feedback);
            RCLCPP_INFO(this->get_logger(), "+   Left Force Feedback: %.2f", force_fdb.left_force_feedback);
            RCLCPP_INFO(this->get_logger(), "+   Frame Tail: 0x%02X", force_fdb.frame_tail);
            RCLCPP_INFO(this->get_logger(), "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
            // Add more fields as needed
        }
        // communication configuration
        ControllerCommand_t controller_cmd_ = {
            .frame_header = CONTROLLER_COMMAND_FRAME_HEADER,
            .frame_tail = CONTROLLER_COMMAND_FRAME_TAIL,
        }; // 上位机发送的命令
        ForceFeedback_t force_fdb_; // 上位机发送的命令

        // usart configuration
        std::string port_name_; // 串口设备名
        int baud_rate_;         // 串口波特率
        int publish_frequency_; // 通信频率

        // ros configuration
        std::shared_ptr<drivers::serial_driver::SerialDriver> serial_driver_; // 串口驱动
        std::shared_ptr<drivers::common::IoContext> io_context_;              // IO上下文
        rclcpp::TimerBase::SharedPtr transmit_timer_;                         // 用于定时发布
        
        rclcpp::Subscription<bw_serial::msg::CtrlCmd>::SharedPtr left_controller_cmd_sub_;
        rclcpp::Subscription<bw_serial::msg::CtrlCmd>::SharedPtr right_controller_cmd_sub_;
        rclcpp::Publisher<bw_serial::msg::ForceFdb>::SharedPtr left_force_fdb_pub_;
        rclcpp::Publisher<bw_serial::msg::ForceFdb>::SharedPtr right_force_fdb_pub_;
        bw_serial::msg::ForceFdb left_force_fdb_msg; // 用于发布力反馈数据
        bw_serial::msg::ForceFdb right_force_fdb_msg; // 用于发布力反馈数据
        std::vector<uint8_t> transmit_data_buffer = std::vector<uint8_t>(1024); // 发送缓冲区
        std::vector<uint8_t> receive_data_buffer;  // 接收缓冲区
    };  // end of class ForceFeedbackCommNode

} // namespace bw_serial

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bw_serial::ForceFeedbackCommNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
