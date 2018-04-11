#ifndef __NEURON_OMNI_IO_NODETEST__
#define __NEURON_OMNI_IO_NODETEST__

#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time_source.hpp"
#include "std_msgs/msg/string.hpp"
#include "neuronGpio.hpp"
#include <stdio.h>

/*  Topic Name Settings */
#define TOPIC_CMD "neuron_gpio_cmd"
#define TOPIC_DATA "neuron_gpio_data"

/* GPIO Settings */
#define GPIO_LED_R_PIN (0)	// red LED
#define GPIO_LED_O_PIN (2)	// orange LED
#define GPIO_LED_Y_PIN (4)	// yello LED
#define GPIO_LED_G_PIN (6)	// green LED
#define GPIO_SW_CONTACT_PIN (1)	// conatct 
#define GPIO_SW_ONOFF_PIN (3)	// on off LED


using namespace std::chrono_literals;
using std::placeholders::_1;

class NeuronOmniIoNodeTest : public rclcpp::Node
{
  private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
	rclcpp::TimerBase::SharedPtr timer_;

    //std::shared_ptr<NeuronGpio> gpio_led_r_, gpio_led_o_, gpio_led_y_, gpio_led_g_;
	//std::shared_ptr<NeuronGpio> gpio_sw_contact_, gpio_sw_onoff_;
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        msg;
        return;
    }

    void timer_callback()
    {

        RCLCPP_INFO(this->get_logger(), "CONTACT!!! time: %ld", this->now());
        RCLCPP_INFO(this->get_logger(), "CONTACT!!! time: %ld",
                                                std::chrono::system_clock::now());
        return;
    }

  public:
    //====== Constructor ======//
    explicit NeuronOmniIoNodeTest() : Node("neuron_gpio")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
                TOPIC_CMD, std::bind(&NeuronOmniIoNodeTest::topic_callback, this, _1),
                rmw_qos_profile_sensor_data);
        
        timer_ = this->create_wall_timer(100ms,
                                         std::bind(&NeuronOmniIoNodeTest::timer_callback, this));

    }
    //====== Destructor ======//
    virtual ~NeuronOmniIoNodeTest()
    {
        printf("Node shutting down, reset all GPIOs");
    }
};

#endif