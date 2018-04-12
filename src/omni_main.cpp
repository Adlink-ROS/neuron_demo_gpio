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

#include "neuronOmniIoNode.hpp"
#include<signal.h>

void intHandler(int dummy)
{
    if(dummy){} // surpress compiler warning
    rclcpp::shutdown();
}

/*
 *  In this DEMO, the NeuronOmniIoNode subscribes the TOPIC_CMD topic.
 *  ANY kind of std_msgs data from TOPIC_CMD will trigger the node to
 *  toggle the GPIO PIN10 and publish the GPIO HIGH/LOW to TOPOC_DATA topic.
 * 
 *  The setting of TOPIC_CMD and TOPIC_DATA can be found in reuronHwmNode.hpp
*/
int main(int argc, char *argv[])
{
    signal(SIGINT, intHandler);
    rclcpp::init(argc, argv);
	auto node = std::make_shared<NeuronOmniIoNode>();

    rclcpp::spin(node);
    node.reset();	// calling destructor through shared_ptr
    rclcpp::shutdown();

    return 0;
}
