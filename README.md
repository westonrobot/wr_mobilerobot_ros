# Weston Robot Platform ROS Support Packages

## Packages

* robot_bringup: launch and configuration files to start ROS nodes 
* robot_base: a ROS wrapper around **wrp_sdk** to monitor and control the robot
* robot_description: URDF model for the mobile base, a sample urdf (robot_description/sample/scout_v2_nav.xacro) is provided for customized robot with addtional sensors
* robot_msgs: scout related message definitions

## Supported hardware

* Scout V2.1

**Note**: please contact Weston Robot if you're not sure whether your hardware is compatible with the packages in this repository.

## Supported software environment

* Ubuntu 18.04: ROS melodic
* Ubuntu 20.04: ROS noetic

## Communication interface

Currently, only the CAN bus interface is supported. You can use the provided CAN-TO-USB adapter to set up the connection for robot control and state monitoring.

### Setup CAN-TO-USB adapter

1. Enable gs_usb kernel module
   
    ```
    $ sudo modprobe gs_usb
    ```

2. Plug CAN-TO-USB device and then bringup it with the bitrate of 1Mbit/s
   
   ```
   $ sudo ip link set can0 up type can bitrate 1000000
   ```

3. If no error occured during the previous steps, you should be able to see the can device now by using command
   
   ```
   $ ifconfig -a
   ```

4. Install and use can-utils to test the hardware
   
    ```
    $ sudo apt install can-utils
    ```

5. Testing command
   
    ```
    # receive data from can0
    $ candump can0

    # send data to can0
    $ cansend can0 001#1122334455667788
    ```

### Nvidia Jeston TX2/Xavier/XavierNX

Nvidia jetson boards have CAN controller(s) integrated in the main SOC. If you're using a dev kit, you need to add a CAN transceiver for proper CAN communication. Please refer to Nvidia documentation for information about how to enable the CAN bus interface on your board.

If the communication between your Linux computer and the robot is set up corretly, you should be able to see data received from the CAN bus interface using "candump".

## Build the ROS packages

1. Install dependent libraries

    ```
    $ echo "deb https://westonrobot.jfrog.io/artifactory/wrtoolbox-release bionic main" | sudo tee /etc/apt/sources.list.d/weston-robot.list
    $ curl -sSL 'https://westonrobot.jfrog.io/artifactory/api/security/keypair/wr-deb/public' | sudo apt-key add -
    $ sudo apt-get update
    $ sudo apt-get install wrp_sdk
    ```

2. Clone the packages into your catkin or colcon workspace and compile

    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/westonrobot/wr_mobilerobot_ros.git  
    $ cd ..
    $ catkin_make
    ```

3. Launch ROS nodes
 
* Start the base node for the real robot

    ```
    $ roslaunch robot_bringup scout_minimal.launch
    ```

    The [robot_bringup/scout_minimal.launch](scout_bringup/launch/scout_minimal.launch) has 4 parameters:

    - port_name: specifies the port used to communicate with the robot, default = "can0"
    - simulated_robot: indicates if launching with a simulation, default = "false"
    - model_xacro: specifies the target ".xacro" file for the publishing of tf frames, default = [scout_v2.xacro](scout_base/description/scout_v2.xacro)
    - odom_topic_name: sets the name of the topic which calculated odometry is published to, defaults = "odom"

## ROS topics

### Subscribed topics

- /cmd_vel: linear and angular speed command to mobile robot
- /scout_light_control: the front light control command to mobile robot

### Published topics

- /mobilebase_motion_status: feedback the desired and actual motion of the robot
- /mobilebase_light_status: feedback the current status of the front light
- /mobilebase_actuator_status: include motor and driver status info
- /mobilebase_system_status: feedback system status including error code, control mode and operational state
  
Please refer to **scout_msgs** package for more details about the messages.
