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
#include <stdio.h>
#include "neuronOmniIoNode.hpp"


/* * * * * * * * * * 
 * Private Methods *
 * * * * * * * * * */
void NeuronOmniIoNode::topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
    // Print the received message
    printf("------------------------------------------------------------------\n");
    printf("=>>> receive from -- Topic <\"%s\">: \"%s\".\n", TOPIC_CMD, msg->data.c_str());
    printf("\n");

    // Check the availability of the SEMA library 
    if (NeuronGpio::IsAvailable() == false)
    {
        printf("[ERROR] NeuronOmniIoNode - SEMA Lib not found.\n");
        return;
    }

    // Toggle LEDs Level
    uint32_t level[4] = {0};
	uint32_t contact_sw_level, onoff_sw_level;
	
	rotate_i_ = (rotate_i_+1)%4;
	level[rotate_i_] = EAPI_GPIO_HIGH;
  	if(!last_contact_)	{	set_led(level);		} 		// LED override check

    /* publish data
	// Send it out
    std::string stmp;
    msg->data = stmp;
    printf("<<<= send to ------- Topic <\"%s\">: \"%s\".\n", TOPIC_DATA, msg->data.c_str());*/
    //publisher_->publish(msg);
    
    return;
}

void NeuronOmniIoNode::timer_callback()
{
	// read contact switch
	uint32_t contact_sw_level, onoff_sw_level;
    gpio_sw_contact_->ReadLevel(contact_sw_level);
	gpio_sw_onoff_->ReadLevel(onoff_sw_level);

	uint32_t level[4] = {0};
    if(contact_sw_level == EAPI_GPIO_LOW)
	{
		if(!last_contact)		// rising edge
		{			
			RCLCPP_INFO(this->get_logger(), "CONTACT!!! time: %d", clock_::now());
		}
		// set all LEDs HIGH if contact
		for(int j = 0;j<4;j++)	{	level[j] = EAPI_GPIO_HIGH;	}
		last_contact_ = true;
	}else{
		last_contact = false;
	}
	set_led(level);	
	
	
	if (onoff_sw_level == EAPI_GPIO_LOW) switch_on_ = true;
	else switch_on_ = false;
	return;
}

void NeuronOmniIoNode::set_led(const uint32_t (&state)[4])
{
	gpio_led_r_->SetLevel(state[0]);
	gpio_led_o_->SetLevel(state[1]);
	gpio_led_y_->SetLevel(state[2]);
	gpio_led_g_->SetLevel(state[3]);
	return;
}


/* * * * * * * * * * 
 * Public Methods  *
 * * * * * * * * * */
 
//====== Constructor ======//
NeuronOmniIoNode::NeuronOmniIoNode() : Node("neuron_gpio")
{
    //publisher_ = this->create_publisher<std_msgs::msg::String>(
    //      TOPIC_DATA, rmw_qos_profile_sensor_data);

    subscription_ = this->create_subscription<std_msgs::msg::String>(
            TOPIC_CMD, std::bind(&NeuronOmniIoNode::topic_callback, this, _1),
            rmw_qos_profile_sensor_data);
			
	timer_ = this->create_wall_timer(10_ms, timer_callback);
	clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
        
    NeuronGpio::InitLib();
    
	if(NeuronGpio::IsAvailable())
	{
		gpio_led_r_ = std::make_shared<NeuronGpio>(GPIO_LED_R_PIN);
		gpio_led_o_ = std::make_shared<NeuronGpio>(GPIO_LED_O_PIN);
		gpio_led_y_ = std::make_shared<NeuronGpio>(GPIO_LED_Y_PIN);
		gpio_led_g_ = std::make_shared<NeuronGpio>(GPIO_LED_G_PIN);
		gpio_sw_contact_ = std::make_shared<NeuronGpio>(GPIO_SW_CONTACT_PIN);
		gpio_sw_onoff_ = std::make_shared<NeuronGpio>(GPIO_SW_ONOFF_PIN);
		
		gpio_led_r_->SetDir(EAPI_GPIO_OUTPUT);
		gpio_led_o_->SetDir(EAPI_GPIO_OUTPUT);
		gpio_led_y_->SetDir(EAPI_GPIO_OUTPUT);
		gpio_led_g_->SetDir(EAPI_GPIO_OUTPUT);
		gpio_sw_contact_->SetDir(EAPI_GPIO_INPUT);
		gpio_sw_onoff_->SetDir(EAPI_GPIO_INPUT);
		
	}
	
	rotate_i_ = 0;
	switch_on_ = false;
	last_contact_ = false;
}

//====== Destructor ======//
NeuronOmniIoNode::~NeuronOmniIoNode()
{
    uint32_t level[4] = {0};
    set_led(level);
    gpio_led_r_->SetDir(EAPI_GPIO_INPUT);
	gpio_led_o_->SetDir(EAPI_GPIO_INPUT);
	gpio_led_y_->SetDir(EAPI_GPIO_INPUT);
	gpio_led_g_->SetDir(EAPI_GPIO_INPUT);
	printf("Node shutting down, reset all GPIOs");
    NeuronGpio::UnInitLib();
}
