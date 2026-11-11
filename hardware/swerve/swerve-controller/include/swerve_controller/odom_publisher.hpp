#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>

class OdomPublisher : public rclcpp::Node{
    public:
        OdomPublisher() 
        {
            odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
                "~/odom", rclcpp::SystemDefaultsQoS());
            RCLCPP_INFO(this->get_logger(), "Odom publisher created.");
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000),
                [this]() {
                    nav_msgs::msg::Odometry odom;
                    odom_publisher_->publish(odom);
                    RCLCPP_INFO(this->get_logger(), "Odom published");
                });
            RCLCPP_INFO(this->get_logger(), "Odom timer created");    
        }
    private:
        void timer_callback(){
            auto message = nav_msgs::msg::Odometry();
            message.header.stamp = this->get_clock()->now();
            message.header.frame_id = "odom";
            message.child_frame_id = "base_link";

            message.pose.position.x = 0.0;
            message.pose.position.y = 0.0;
            message.pose.position.z = 0.0;
            message.pose.orientation.x = 0.0;
            message.pose.orientation.y = 0.0;
            message.pose.orientation.z = 0.0;
            message.pose.orientation.w = 1.0;

            message.twist.twist.linear_x = 1.0;
            message.twist.twist.angular_z = 0.0;



            odom_publisher_->publish(message);
        }


        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;


};

