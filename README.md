## Description
In this **DEMO**, the ***NeuronHwmNode*** subscribes to the **TOPIC_CMD** topic.

**ANY** kind of std_msgs data from **TOPIC_CMD** will trigger the node to
publish the next hardware monitor data to **TOPOC_DATA** topic.

## Notice
**1.** The setting of **TOPIC_CMD** and **TOPIC_DATA** can be found in ***reuronHwmNode.hpp***

**2.** This application **MUST** be running under **SUDO -E** since the SEMA driver need both root privilege and the user-exported variables.

## Steps
**1. Install ADLINK SEMA**

Reference to [This Project](http://ros2.local:10080/Neuron/sema-3.5-installer/) which provides a script to help you download and install SEMA.

**2. Setup soft link**
```` bash
cd ${project_root}
cd ./lib
./setlink.sh
````
**3. Compile the source codes**

**4. Execute it!**

```` bash
cd ${project_root}
cd ./build/
sudo -E ./neuron_demo_hwm
````
**5. Create a publisher in ANOTHER terminal to trigger it to response**
````bash
ros2 topic pub /neuron_hwm_cmd std_msgs/String "data: 0"
````

** 6. The terminal running neuron_demo_hwm should display the HWM data like below. **
```` bash
data: 'Temp System: 32.000000 Celcius'
data: 'Volt Vcore: 0.725000 V'
data: 'Volt 3.3V: 3.332000 V'
data: 'Volt VBAT: 3.090000 V'
data: 'Volt 5V: 5.025000 V'
data: 'Volt 5VSB: 5.031000 V'
data: 'FanSpeed CPU: 0 RPM'
data: 'FanSpeed System: 1041 RPM'
data: 'Temp CPU: 32.100000 Celcius'
data: 'Temp System: 32.000000 Celcius'
data: 'Volt Vcore: 0.721000 V'
````