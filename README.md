##  gNB Gazebo simulation 
This repository contains the model and the plugin, developed by phine.tech, to simulate the behaviour of one or more gNBs in Gazebo Ignition. The underlying 5G component are docker containers developed by Open Air Interface(OAI), that have been exploited by phine.tech to make accessible these advance networking modules to the robotic community. The aim is that a roboticist who is not an expert in 5G connectivity can easily simulate its robotic applications in a 5G end-to-end network without knowing the technical details on which the OAI Framework is built on.
The table of content of this README file is:
1. [Release Versions](#release-versions)
2. [OAI setup](#oai-setup)
3. [Phine.tech gNodeB](#phine.tech-gnodeb)
	- [Visualization](#visualization)
	- [gNB plugin](#gnb-plugin)
	- [World file declaration](#world-file-declaration)
4. [Upcoming features](#upcoming-features)
## Requirements
The project has been tested with:
- Ubuntu 22.04
- ROS2 humble
- Docker version 27.5.1, build 9f9e405
- docker-compose version 1.29.2
- gazebo ignition fortress 2.6.9
## OAI setup
Considering `path` the absolute path to the repository, the OAI files can be found in `path/oai_setup`. Inside this folder there are 3 docker-compose file:
- docker-compose.yml for launching the OAI Core Network containers
- docker-compose-gNB.yml for launching the OAI gNB container
- docker-compose-ue.yml for launching the OAI UE container
Other folders are present inside `oai-setup`, containing configuration files useful for these containers. It possible to verify that the OAI modules work, by following this pipeline:
 - In one terminal run the core network:
```
cd path/oai_setup
docker compose up -d

```
In the same terminal (wait 5-10 seconds):
```
docker logs oai-udr
```
If everything went correctly, in the logs you should see:
```
[2025-02-28 13:01:58.215] [udr_app] [debug]     Instance name: OAI-UDR
[2025-02-28 13:01:58.215] [udr_app] [debug]     Instance type: UDR
[2025-02-28 13:01:58.215] [udr_app] [debug]     Instance fqdn: 
[2025-02-28 13:01:58.215] [udr_app] [debug]     Status: REGISTERED
[2025-02-28 13:01:58.215] [udr_app] [debug]     HeartBeat timer: 50
[2025-02-28 13:01:58.215] [udr_app] [debug]     Priority: 1
[2025-02-28 13:01:58.215] [udr_app] [debug]     Capacity: 100
[2025-02-28 13:01:58.215] [udr_app] [debug]     IPv4 Addr:
[2025-02-28 13:01:58.215] [udr_app] [debug]         192.168.70.136
[2025-02-28 13:01:58.215] [udr_app] [debug] 	UDR Info
[2025-02-28 13:01:58.215] [udr_app] [debug] 		GroupId: oai-udr-testgroupid
[2025-02-28 13:01:58.215] [udr_app] [debug] 		 SupiRanges: Start - 208950000000131, End - , Pattern - ^imsi-20895[31-131]{6}$
[2025-02-28 13:01:58.215] [udr_app] [debug] 		 GpsiRanges: Start - 752740000, End - 752749999, Pattern - ^gpsi-75274[0-9]{4}$
[2025-02-28 13:01:58.215] [udr_app] [debug] 		 Data Set Id: 0210
[2025-02-28 13:01:58.215] [udr_app] [debug] 		 Data Set Id: 9876

```

- in the same terminal run the gNB container:
```bash
docker compose -f docker-compose-ran.yml up -d
```
In the same terminal (wait 5-10 seconds):
```bash
docker logs oai-amf
```
If everything goes well you should see in the logs a table with the list of the connected gNBs and their names (in this case one named gnb-rfsim):
```
  |------------------------------gNBs' Information-------------------------|
  
 |  Index  |     gNB Name    |   Status      |    PLMN     |     Global Id  |                                                            
 |    1    |     gnb-rfsim   |  Connected    |    001,01   |      0x0E00    |                                                                     
  -------------------------------------------------------------------------------
```
- in the same terminal run the UE container :
```bash
docker compose -f docker-compose-ue.yml up oai-nr-ue -d
```
In the same terminal(wait 5-10 seconds):
```bash
docker logs oai-nr-ue
```
in the logs you will see  if it is working:
```
I Interface oaitun_ue1 successfully configured, IPv4 10.0.0.2, IPv6 (null)
```
To stop and remove all the contianers:
```bash
docker compose -f docker-compose-gNB.yml down
docker compose -f docker-compose-ue.yml down
docker compose down -d

```
## Phine.tech gNodeB
### Visualization
The gazebo visualization of the gNB can be found in `path/model_folder/phine_gNB/model.sdf`. It should remind of a 5G cell, a white thin box shape. Gravity is deactivated for this model, to make the placement of the gNB for the user.
## gNB plugin
The source code of the gNB gazebo plugin can be found in `path/phine-gz-ros/src/phine-plugins/src`. The plugin is already present in  `path/phine-gz-ros/build/phine_plugins/libgNB-plugin.so`.Upon modification of the source code, to modify the plugin, it is necessary to build the whole ros2 package `phine-plugins` in the workspace `phine-gz-ros`:
```
cd path/phine-gz-ros
colcon build
```
It s possible that some warning messages pop out.
The plugin at load time:
- modifies the configuration files for the gNB container (`docker-compose-gNB.yml` and `gnb.sa.bandn78.fr1.106PRB.rfsim.conf`) regarding net name, gNB IPv4, AMF IPv4 
- launches the gNB container
- sends debug information if needed
The plugin at runtime:
- sends the pose of the gNB by publishing on a ROS2 topic called `pose_of_<name_of_gNB>`
### World file declaration
Having referenced in `env`:
- `IGN_GAZEBO_SYSTEM_PLUGIN_PATH=<path-to-gazebo-models>`
- `IGN_GAZEBO_RESOURCE_PATH=<path-to-Plugins>`
With the model folder containing the gNB model and the plugin folder containing the gNB plugin, it is possible to add to your `world-file.sdf` the gNB by:
```
<include>

	<uri>model://phine_gNB</uri>
	
	<name>gNB1</name>
	
	<pose>0 0 1 0 0 0</pose>
	
	<plugin name="phine_plugins::gNB_plugin" filename="gNB_plugin">
	
		<link_name>phine_cell</link_name>
		
		<net_name>phine-net</net_name>
		
		<IP_GNB>192.168.70.159</IP_GNB>
		
		<IP_AMF>192.168.70.132</IP_AMF>
		
		<debug>false</debug>
		
		<mobile_country_code>001</mobile_country_code>
		
		<mobile_network_code>01</mobile_network_code>
	
	</plugin>

</include>
```
The parameters that can be changed are:
- gNB name (t keep it gNB1, gNB2, ... otherwise the plugin won't work)
- pose
- link of the gNB model whose pose is streamed via ROS2(keep it like in the example if the model remains unchanged)
- net name 
- IPv4 of the gNB
- IPv4 of the amf, that has to be the same as the one in your amf (can be changed in `path/oai_setup/conf/conf.yaml`, if you do not change that keep it like in the example ) 
- plotting debug info (for developers)
- mobile_country_code, that has to be the same as the one in your amf (can be changed in `path/oai_setup/conf/conf.yaml`, if you do not change that keep it like in the example )
- mobile_network_code,that has to be the same as the one in your amf(can be changed in `path/oai_setup/conf/conf.yaml`, if you do not change that keep it like in the example )
## Modify gNB source code
The source code of the gNB is at `path/phine-plugins/src/phine_plugins/src`. You can modify it as you wish consulting [Coding Guidelines](https://github.com/phinetech/RoboSim5G/blob/develop/CODE_OF_CONDUCT.md) and [Contributing](https://github.com/phinetech/RoboSim5G/blob/develop/CONTRIBUTING.md),in case you would like to share your work. Once you have modified the code, delete `path/phine-plugins/log`,`path/phine-plugins/install`,`path/phine-plugins/build` and run in `path/phine-plugins`:
```
colcon build

```
The updated plugin will be in `path/phine-plugins/build`.
## Upcoming features
- a UE plugin that can be attached to robots to fully simulate a 5g end-to-end network
