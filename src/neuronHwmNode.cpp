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
#include "neuronHwmNode.hpp"


/* * * * * * * * * * 
 * Private Methods *
 * * * * * * * * * */
 void NeuronHwmNode::topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
    // Print the received message
    printf("------------------------------------------------------------------\n");
    printf("=>>> receive from -- Topic <\"%s\">: \"%s\".\n", TOPIC_CMD, msg->data.c_str());
    printf("\n");

    // Check the availability of the SEMA library 
    if (NeuronHwm::IsAvailable() == false || hwm_ == NULL)
    {
        printf("[ERROR] neuronHwmNode - SEMA Lib not found.\n");
        return;
    }

    // Get next HWM item data message
    std::string itemMsg;
    if (hwm_->GetNextItemMessage(itemMsg) == false)
    {        
        printf("[ERROR] neuronHwmNode - GetNextItemMessage() Failed.\n");         
        return;
    }
    
    // Send it out
    msg->data = itemMsg;
    printf("<<<= send to ------- Topic <\"%s\">: \"%s\".\n", TOPIC_DATA, msg->data.c_str());
    publisher_->publish(msg);    
    return;
}


/* * * * * * * * * * 
 * Public Methods  *
 * * * * * * * * * */
NeuronHwmNode::NeuronHwmNode() : Node("neuron_hwm")
{
    publisher_ = this->create_publisher<std_msgs::msg::String>(
            TOPIC_DATA, rmw_qos_profile_sensor_data);

    subscription_ = this->create_subscription<std_msgs::msg::String>(
            TOPIC_CMD, std::bind(&NeuronHwmNode::topic_callback, this, _1),
            rmw_qos_profile_sensor_data);

        
    NeuronHwm::InitLib();
    
    hwm_ = NeuronHwm::IsAvailable()? std::make_shared<NeuronHwm>() : NULL;

}

NeuronHwmNode::~NeuronHwmNode()
{
    NeuronHwm::UnInitLib();
}