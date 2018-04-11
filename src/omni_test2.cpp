#include "neuronOmniIoNodeTest.hpp"
#include<signal.h>
using std::placeholders::_1;

void intHandler(int dummy)
{
    printf("stop");
    rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
    
    rclcpp::init(argc, argv);
	auto node = std::make_shared<NeuronOmniIoNodeTest>();
    printf("spin");
    signal(SIGINT, intHandler);
    rclcpp::spin(node);
    printf("stop");
    node.reset();	// calling destructor through shared_ptr
    rclcpp::shutdown();

    return 0;
}
