## Requirements

- Gazebo Ignition Fortress

- ROS2 HUMBLE

- Docker

- ros-gz pkg

## Changing Plugin path

In `launch_robot_5G` at line 4 you have to change the path to Gazebo Plugin to your own (sometimes it can be in ~/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins )
## Builiding Images ad building ROS2 pkgs
On a terminal in the main repo directory:

```
./build_images.sh
```
## Running simulation

1. On a terminal launch :

```
source launch_robot_5G.sh
```

Insert your password when required.

This command will launch the CN container, the Gazebo Simulation with the OAI gNB embedded (visualization + container) and  the robot-UE container and the RCC container. It will take 40 seconds to start everything, it will finish when Rviz opens.
The first time you launch the simulation, it might fail since the UE plugin will build the robot-UE plugin for a long time.

2. On Rviz click on "Nav2 goal" and then select a feasible location.

3. Whenever you want, look for the three vertical dot on the top right of Gazebo GUI, click it and in the search box of Gazebo search "UE_Power_Plugin" and "gNB_Power_Plugin". In the Gazebo GUI two buttons will pop up, that will switch on or off the UE and gNB of the robot.

4. Press the gNB button, that will switch both gNB and UE off.

5. The robot will continue its path (or it will stay still if he does not have one), you can try to give another goal but the communication won' t be enabled

6. Switch on the UE and gNB from the buttons

7. Give another Nav2 Goal or wait for the robot to finish its path

## Remove all the container and kill gazebo

In the main folder:

```

./kill_all.sh

```

## Modify gNB parameters

Inside `gazebo_launch/src/ign_turtlebot/worlds/world_only.sdf`change the parameters at line 106.

```xml


<include>

  

<uri>model://phine_gNB</uri>

  

<name>gNB1</name>

  

<pose>-6.8 1 7 0 0 0</pose>

  

<plugin name="phine_plugins::gNB_plugin" filename="gNB_plugin">

  

<link_name>phine_cell</link_name>

  

<net_name>phine-net</net_name>

  

<IP_GNB>192.168.70.160</IP_GNB>

  

<IP_AMF>192.168.70.132</IP_AMF>

  

<debug>false</debug>

  

<mobile_country_code>001</mobile_country_code>

  

<mobile_network_code>01</mobile_network_code>

  

</plugin>

  

</include>

```
## Modify UE parameters
Inside `gazebo_launch/src/ign_turtlebot/worlds/world_only.sdf`change the parameters at line 56.
```xml
<plugin name="phine_plugins::UE_plugin" filename="UE_plugin">

<robot_container_name>ue_turtlebot</robot_container_name>

<robot_id>1</robot_id>

<ip_robotUE>192.168.70.150</ip_robotUE>

<net_name>phine-net</net_name>

<ip_gnb>192.168.70.160</ip_gnb>

<debug>true</debug>

<subnet_5G>10.0.0.0/28</subnet_5G>

<imsi>001010000000101</imsi>

<key>fec86ba6eb707ed08905757b1bb44b8f</key>

<opc>C42449363BBAD02B66D16BC975D77CC1</opc>

<dnn>oai</dnn>

<nssai_sst>1</nssai_sst>

<nssai_sd>no</nssai_sd>

<robot_project_path>${PROJECT_PATH}/nav_stack</robot_project_path>

<robot_project_name>nav_stack</robot_project_name>

<execute_robot_launch_file>true</execute_robot_launch_file>

<robot_package_name>ign_turtlebot</robot_package_name>

<ros_gz_bridge_name>ros_gz_bridge.launch.py</ros_gz_bridge_name>

<robot_launch_file_name>nav_stack.launch.py</robot_launch_file_name>

<ros_discovery_server>192.168.70.159:11811</ros_discovery_server>

</plugin>
```

## Remove CN from launch file

In `launch_robot_5G.sh` comment:

```sh

cd oai_setup

docker compose up -d

cd ..

```

## Changing ROS DOMAIN (avoiding multi-user bugs)

In `launch_robot_5G.sh` modify with the ID that you want(each PC should have its own):

```

export ROS_DOMAIN_ID=ID

```

## Changing GZ partion(avoiding multi-user bugs)

In `launch_robot_5G.sh` modify with the partion that you want(each PC should have its own):

```

export IGN_PARTITION=partition_name

```

## Using the Button UE and gNB plugin

If you change the container name in the gNB and UE declarations, you should match the same container names for the two environment variable in `launch_robot_5G.sh`: `GNB_NAME_FOR_BUTTON` , `UE_NAME_FOR_BUTTON`.