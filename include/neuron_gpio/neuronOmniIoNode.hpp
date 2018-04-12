// Copyright 2017 ADLINK Technology, Inc.
// Developer: Alan Chen (alan.chen@adlinktech.com)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef __NEURON_OMNI_IO_NODE__
#define __NEURON_OMNI_IO_NODE__

#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time_source.hpp"
#include "std_msgs/msg/string.hpp"
#include "neuronGpio.hpp"

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


using std::placeholders::_1;

class NeuronOmniIoNode : public rclcpp::Node
{
  public:
    explicit NeuronOmniIoNode();
    virtual ~NeuronOmniIoNode();

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);
	void timer_callback();
	void set_led(const uint32_t (&state)[4]);
    
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
	rclcpp::TimerBase::SharedPtr timer_;

    
    std::shared_ptr<NeuronGpio> gpio_led_r_, gpio_led_o_, gpio_led_y_, gpio_led_g_;
	std::shared_ptr<NeuronGpio> gpio_sw_contact_, gpio_sw_onoff_;
    
    uint32_t rotate_i_;
	bool switch_on_;
	bool last_contact_;
    bool msg_recieved_;
};


#endif /* __NEURON_OMNI_IO_NODE__ */
