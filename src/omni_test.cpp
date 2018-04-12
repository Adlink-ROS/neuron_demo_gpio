#include <stdio.h>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time_source.hpp"
#include "std_msgs/msg/string.hpp"

#define TOPIC_CMD "string_cmd"

using namespace std::chrono_literals;
using std::placeholders::_1;

class OmniTestNode : public rclcpp::Node
{
  private:
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
	rclcpp::TimerBase::SharedPtr timer_;
    
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // Print the received message
        printf("------------------------------------------------------------------\n");
        printf("=>>> receive from -- Topic <\"%s\">: \"%s\".\n", TOPIC_CMD, msg->data.c_str());
        printf("\n");
        return;
    }

    void timer_callback()
    {
        rclcpp::TimeSource ts(shared_from_this());
        rclcpp::Clock::SharedPtr clk2 = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
        ts.attachClock(clk2);
        rclcpp::Clock clk3;
        
        RCLCPP_INFO(this->get_logger(), "clk2 time: %zu", clk2->now().nanoseconds());
        // this will work!!
        RCLCPP_INFO(this->get_logger(), "clk3 time: %zu", RCUTILS_NS_TO_MS(clk3.now().nanoseconds()));
        RCLCPP_INFO(this->get_logger(), "this->now time: %zu", this->now().nanoseconds());
        RCLCPP_INFO(this->get_logger(), "CONTACT!!! time: %ld", std::chrono::system_clock::now());
        
        return;
    }


  public:
    rclcpp::Clock::SharedPtr clock_;
    //====== Constructor ======//
    explicit OmniTestNode() : Node("test_io")
    {
        //publisher_ = this->create_publisher<std_msgs::msg::String>(
        //      TOPIC_DATA, rmw_qos_profile_sensor_data);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
                TOPIC_CMD, std::bind(&OmniTestNode::topic_callback, this, _1),
                rmw_qos_profile_sensor_data);
                
        timer_ = this->create_wall_timer(1000ms, std::bind(&OmniTestNode::timer_callback, this));
        clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
        //ts_ = std::make_shared<rclcpp::TimeSource>(shared_from_this());
        //ts_ = rclcpp::TimeSource(this);
    }

    //====== Destructor ======//
    virtual ~OmniTestNode()
    {
        printf("Node shutting down, reset all GPIOs");
    }
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
	auto node = std::make_shared<OmniTestNode>();
     
    printf("spin");
    rclcpp::spin(node);
    printf("stop");
    node.reset();
    rclcpp::shutdown();

    return 0;
}
