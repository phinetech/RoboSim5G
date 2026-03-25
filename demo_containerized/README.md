## Requirements

- Docker version 27.5.1, build 9f9e405

- Docker Compose version v2.33.1

## Choosing a Core Network

RoboSim5G supports **three 5G Core Network implementations**. Each has its own launch and kill scripts:

| Core Network | Launch Script | Kill Script |
|--------------|--------------|-------------|
| **OAI** (OpenAirInterface) | `launch_robot_5G_docker.sh` | `kill_all.sh` |
| **free5GC** | `launch_robot_5G_docker_free5gc.sh` | `kill_all_free5gc.sh` |
| **Open5GS** | `launch_robot_5G_docker_open5gs.sh` | `kill_all_open5gs.sh` |

**Important:** When you choose a core network, you must also **configure the corresponding Gazebo world file** in the simulation container (see [Configuring Gazebo World for Your CN](#configuring-gazebo-world-for-your-cn) section below).

**Key Difference - free5GC N3 Interface:**
free5GC uses a **separate N3 network** (`demo-n3` bridge) for the gNB ↔ UPF GTP-U tunnel, while OAI and Open5GS use the main `phine-net` for all communication.

📖 **For detailed comparison and guidance on choosing:** See [doc/5G_CORE_NETWORKS.md](../doc/5G_CORE_NETWORKS.md)

## Changing Plugin path

In `launch_robot_5G` at line 4 you have to change the path to Gazebo Plugin to your own (sometimes it can be in /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins)
## Builiding Images ad building ROS2 pkgs
On a terminal in the main repo directory:

```
./build_images.sh
```
## Running simulation

1. On a terminal, launch the script for your chosen core network:

**For OAI:**
```bash
source launch_robot_5G_docker.sh
```

**For free5GC:**
```bash
source launch_robot_5G_docker_free5gc.sh
```

**For Open5GS:**
```bash
source launch_robot_5G_docker_open5gs.sh
```

Insert your password when required.

These commands will launch the CN container, the Gazebo Simulation with the gNB embedded (visualization + container), the robot-UE container, and the RCC container. It will take 40 seconds to start everything, finishing when Rviz opens.
The first time you launch the simulation, it might fail since the UE plugin will build the robot-UE plugin for a long time.

2. On Rviz click on "Nav2 goal" and then select a feasible location.

3. Whenever you want, look for the three vertical dot on the top right of Gazebo GUI, click it and in the search box of Gazebo search "UE_Power_Plugin" and "gNB_Power_Plugin". In the Gazebo GUI two buttons will pop up, that will switch on or off the UE and gNB of the robot.

4. Press the gNB button, that will switch both gNB and UE off.

5. The robot will continue its path (or it will stay still if he does not have one), you can try to give another goal but the communication won't be enabled

6. Switch on the UE and gNB from the buttons

7. Give another Nav2 Goal or wait for the robot to finish its path

## Remove all the container and kill gazebo

In the main folder, use the kill script for your chosen core network:

**For OAI:**
```bash
./kill_all.sh
```

**For free5GC:**
```bash
./kill_all_free5gc.sh
```

**For Open5GS:**
```bash
./kill_all_open5gs.sh
```

## Configuring Gazebo World for Your CN

**IMPORTANT:** The Gazebo world file in the simulation container must match your chosen core network. Each CN requires different configurations (IMSI, DNN, network addresses, etc.).

### Available World Files

Three world files are provided in `images/simulation/gazebo_launch/src/ign_turtlebot/worlds/`:

- `world_only.sdf` - **For OAI**
- `world_only_free5gc.sdf` - **For free5GC**
- `world_only_open5gs.sdf` - **For Open5GS**

### Configure the Launch File

Edit `images/simulation/gazebo_launch/src/ign_turtlebot/launch/gazebo_launch.launch.py` at **line 68**:

```python
# Change this line to match your chosen CN:
world = os.path.join(get_package_share_directory('ign_turtlebot'), 
                     "worlds", "world_only_free5gc.sdf")  # Change filename here
```

**Examples:**
- For OAI: `"world_only.sdf"`
- For free5GC: `"world_only_free5gc.sdf"`
- For Open5GS: `"world_only_open5gs.sdf"`

After changing the world file, rebuild the simulation container:
```bash
./build_images.sh
```

## Modify gNB parameters

Inside the world file for your chosen CN (see [Configuring Gazebo World for Your CN](#configuring-gazebo-world-for-your-cn)), change the parameters at line 106.

**For OAI:** `images/simulation/gazebo_launch/src/ign_turtlebot/worlds/world_only.sdf`
**For free5GC:** `images/simulation/gazebo_launch/src/ign_turtlebot/worlds/world_only_free5gc.sdf`
**For Open5GS:** `images/simulation/gazebo_launch/src/ign_turtlebot/worlds/world_only_open5gs.sdf`

```xml
<include>
  <uri>model://phine_gNB</uri>
  <name>gNB1</name>
  <pose>-6.8 1 7 0 0 0</pose>
  <plugin name="phine_plugins::gNB_plugin" filename="gNB_plugin">
    <cn_type>oai</cn_type>  <!-- Options: oai, free5gc, open5gs -->
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

**Note:** The `cn_type` parameter specifies which 5G Core Network the plugin should connect to.
## Modify UE parameters

Inside the world file for your chosen CN (see [Configuring Gazebo World for Your CN](#configuring-gazebo-world-for-your-cn)), change the parameters at line 56.

**For OAI:** `images/simulation/gazebo_launch/src/ign_turtlebot/worlds/world_only.sdf`
**For free5GC:** `images/simulation/gazebo_launch/src/ign_turtlebot/worlds/world_only_free5gc.sdf`
**For Open5GS:** `images/simulation/gazebo_launch/src/ign_turtlebot/worlds/world_only_open5gs.sdf`
```xml
<plugin name="phine_plugins::UE_plugin" filename="UE_plugin">
  <cn_type>oai</cn_type>  <!-- Options: oai, free5gc, open5gs -->
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

**Note:** The `cn_type` parameter specifies which 5G Core Network the plugin should connect to. Ensure it matches the CN you're using.

## Remove CN from launch file

If you want to manage the CN containers separately, you can comment out the CN launch section in your chosen launch script.

**Example for OAI** (`launch_robot_5G_docker.sh`):
```sh
# cd oai_setup
# docker compose up -d
# cd ..
```

**Example for free5GC** (`launch_robot_5G_docker_free5gc.sh`):
```sh
# cd free5gc_setup
# docker compose up -d
# cd ..
```

**Example for Open5GS** (`launch_robot_5G_docker_open5gs.sh`):
```sh
# cd open5gs_setup
# docker compose up -d
# cd ..
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

## NVIDIA GPU Acceleration (Optional)

To enable hardware acceleration for the Gazebo simulation, you need an NVIDIA GPU and the **NVIDIA Container Toolkit** installed on your host machine. This significantly improves simulation frame rates and reduces CPU load.

### 1. Install the Toolkit
Follow the official [NVIDIA Container Toolkit Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) to set up the repository and install the `nvidia-container-toolkit` package.

### 2. Configure the Launch Script
The simulation container is designed to switch between the standard Linux runtime and the NVIDIA runtime dynamically. Open `launch_robot_5G_docker.sh` and ensure the `GPU_RUNTIME` variable is set correctly:

```bash
# Set to "nvidia" to use your GPU, or "runc" for standard CPU execution
export GPU_RUNTIME="nvidia"
```
## Configuring Containerized Simulation

To develop and deploy custom robotic logic within the existing containerized environment, follow these steps to integrate your ROS 2 workspaces and launch scripts.
To add custom code, it should be consider that the images have installed:

- Ubuntu 22.04

- ROS2 humble

- gazebo ignition fortress 2.6.9


## 1. Integrate Your Workspaces

Place your ROS 2 workspaces (e.g., one containing your world with Phine plugins and another containing your robot logic) inside the `images/simulation/` directory on your host machine.

Then, update the `images/simulation/Dockerfile` (typically around lines 66–67) to copy these local directories into the simulation image during the build process:

    # Inside images/simulation/Dockerfile
    # Copy your custom workspaces into the image
    COPY ./world_workspace /app/world_workspace
    COPY ./robot_logic_workspace /app/robot_logic_workspace

---

## 2. Configure the UE Plugin

The UE Plugin in your `.sdf` world file acts as the bridge between the 5G simulation and your ROS 2 logic. You must modify the `<robot_package_name>` parameter to match the package name within your custom workspace and `<robot_project_name>` has to match the name of your robot workspace.

Locate:

    images/simulation/gazebo_launch/src/ign_turtlebot/worlds/world_only.sdf

Modify:

    <plugin name="phine_plugins::UE_plugin" filename="UE_plugin">
        ...
        <robot_project_name>our_custom_workspace<robot_project_name>
        <robot_package_name>your_custom_package_name</robot_package_name>
        <robot_launch_file_name>your_robot_logic.launch.py</robot_launch_file_name>
        ...
    </plugin>

---

## 3. Update the Launch Orchestration

To ensure the simulation container starts with your specific world and configurations, modify the `docker-compose-ue.yml` file located in:

    images/simulation/oai_setup/docker-compose-ue.yml

At approximately line 145, update the `entrypoint` or the `command` associated with the simulation service to call your custom launch file:

    # Inside images/simulation/oai_setup/docker-compose-ue.yml
    services:
      simulation:
        ...
        entrypoint:
          - /bin/bash
          - -c
          - |
              ...
              ros2 launch your_gazebo_package_name your_world_launch.launch.py

---

## Scripts in `launch-robot_5G_docker.sh` 

### `wait_for_core_network.sh`
This script ensures that the OAI core network is fully operational before proceeding with the simulation. It waits for the MySQL database container to become healthy and checks that all critical OAI services (`oai-nrf`, `oai-amf`, `oai-smf`, `oai-upf`) are running. The script is called automatically during the simulation launch process to avoid race conditions and ensure a reliable startup sequence.

### `create_physical_bridge.sh`
This script creates a custom Docker bridge network (`ros_gz_net`) with a specified subnet. It is used to enable communication between simulation containers (such as Gazebo, robot-UE, and network components) on a dedicated network, ensuring proper connectivity and isolation for the simulation environment. The script is executed as part of the launch process and typically does not require manual intervention.

## Important

After making these changes, remember to rebuild your images to apply the updates to the container layers:

    ./build_images.sh
