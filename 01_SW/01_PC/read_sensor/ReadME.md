**Linux ROS-based Application for Proximity and Tactile Sensor Reading.**

The aim of this guide is to provide a brief description of the steps needed to run the Linux ROS-based application for the Proximity and Tactile Sensor reading. 
The application is structured as follow:
- serial ROS package: support library for Serial Interface handling
- read_sensor ROS package: ROS package for Tactile/Proximity Sensor handling

To properly launch the 'read_sensor' ROS node, execute the following command:
1. Check if the current USER is in the 'dialout' group. Execute the 'groups' command in the terminal and verify that 'dialout' is listed.
2. If the current USER is not in the 'dialout' group, execute the following command in the terminal: 'sudo usermod -a -G dialout <user>', where <user> is the name of the current user. Reboot the system.
3. If not installed, install 'setserial' tool with the following command: sudo apt install setserial
4. If needed, compile the ros packages with 'catkin_make' command
5. Execute 'roscore'
6. Execute 'rosrun read_sensor read_sensor_node'

5b. Alternatively you can execute the launch file 'read_sensors.sh' in 'read_serial/launch'. Opening it, it is possibile to configure the launch file with the right Sensor Serial Ports.

WARNING: when you want to launch two or more ROS nodes to read the sensors data, first of all execute all the read_sensor_node you need and, only then, select the USB/Serial device and start the sensor data acquisition.
