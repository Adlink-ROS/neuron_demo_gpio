# ROS2 demo node of GPIO with ADLINK-ROScube and Starter Kit.
This package is a demonstration of the integration of ROS2 and onboard GPIO control.
It will provide ADLINK omni-NeuronBot peripheral sensor and visual indicators of robot state.
The GPIO control is implemented with the ADLINK Smart Embedded Management Agent (SEMA) function, which comes with the Neuron AmITX-Motherboard.
The SEMA features various board monitoring, failsafe, as well as GPIO and I2C functionality.

**[Watch the demo video!](https://youtu.be/gpo7qE80okU)**

## Getting Started
These instructions will get you a copy of this demo and running on your local machine.

### Prerequisites  
1. Hardware and library  
You'll need the ADLINK SEMA library and a compatible motherboard to run this example. You can conatct **SOMEONE** for more information.
2. Download the source of this project to your ROS2 workspace  
    ```
    cd ros2_ws/src/ros2
    git clone https://github.com/Adlink-ROS/neuron_demo_gpio.git
    ```  
3. 4 LEDs, 2 switches(one tactile, one contact)
4. Your motherboard and GPIO connected  
**INSERT SOME SCHEMATICS HERE**  

### Installing
1. Install ADLINK SEMA
Please go to **EMPTY PROJECT** in which provides a script to help you download and install SEMA.

2. Setup soft link (by executing `setlink.sh` under `/lib` )  
Run the following command in the terminal command. If you get something like _`error: no such file as...`_, you'll need to make the setlink.sh executable by `chmod +x setlink.sh`.
    ```
    cd ${project_root}/lib
        e.g.: cd ros2_ws/src/ros2/neuron_demo_gpio/lib
    ./setlink.sh
    ```  
      

3. Compile the source code  
We'll need root access for library linking, that is done by the second step below; ROS2 uses _ament_ as its default build/installation tool.
    ```
    cd ros2_ws
    sudo -sE
    ament build
    ```  
  
## Running the demo
Each of the code block below is a individual terminal with ROS2 environment veriable configured.
* `ros2 run demo_nodes_cpp talker -t neuron_gpio_cmd`
* `root@neuron: ros2 run neuron_demo_gpio neuron_demo_gpio`
* `root@neuron: ros2 run neuron_demo_gpio neuron_omni_io`  

### NOTE
1. terminal can gain root access by first doing `sudo -sE`
2. alternatively, you can publish the triggering topic manually by `run ros2 topic pub /neuron_hwm_cmd std_msgs/String "data: 0"`

### Monitoring
You can checkout the topics by:  
`ros2 topic list`  
and  
`ros2 topic echo /TOPIC`

### What should happened?
Besides the information you see on each terminal, proper ROS2 topic is published. 
* **neuron_demo_gpio** node  
Subscribes to topic _neuron_gpio_cmd_ from _talker_  
Publishes topic _neuron_gpio_data_ of type _std_msgs::String_  
* **neuron_omni_io** node  
Subscribes to topic _neuron_gpio_cmd_ from _talker_  
Controls the LEDs and read from switches  
**INSERT NODE GRAPH HERE**


## Code explained
****TBD****
**ANY** kind of std_msgs data from **TOPIC_CMD** will trigger the node to
publish the next hardware monitor data to **TOPOC_DATA** topic.
**1.** The setting of **TOPIC_CMD** and **TOPIC_DATA** can be found in ***reuronHwmNode.hpp***

## Notice
1. This application **MUST** be running under **SUDO -E** since the SEMA driver need both root privilege and the user-exported variables.


## Version
0.1.0

## Authors
* **Alan Chen** - *Initial library* - (alan.chen@adlinktech.com)
* **Ewing Kang** - *Demo implementation* - (https://github.com/EwingKang)

## License
This project is licensed under the Apache License, Version 2.0

## Acknowledgments
