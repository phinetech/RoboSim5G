## Requirements

- Gazebo Ignition Fortress

- ROS2 HUMBLE

- Docker

- ros-gz pkg

- `mongodb-mongosh` (required only for Open5GS core network — install with `sudo apt install mongodb-mongosh`)

## Choosing a Core Network

RoboSim5G supports **three 5G Core Network implementations**: OAI, free5GC, and Open5GS. It also supports connecting to an **external** core network that is already running elsewhere.

A single `launch.sh` and `kill.sh` script is provided. You select the core network using the `CORE_NETWORK` environment variable (defaults to `oai`):

```bash
CORE_NETWORK=oai       ./launch.sh   # OpenAirInterface (default)
CORE_NETWORK=free5gc   ./launch.sh   # free5GC
CORE_NETWORK=open5gs   ./launch.sh   # Open5GS
CORE_NETWORK=external  ./launch.sh   # External / pre-existing core network
```

The core network containers are launched from the `core_network_setup/` folder at the root of the repository. Each CN has its own subfolder (`core_network_setup/oai/`, `core_network_setup/free5gc/`, `core_network_setup/open5gs/`) containing the docker-compose file and related configuration.

**Key Difference — free5GC N3 Interface:**
free5GC uses a **separate N3 network** (`demo-n3` bridge) for the gNB ↔ UPF GTP-U tunnel, while OAI and Open5GS use the main `phine-net` for all communication.

📖 **For detailed comparison and guidance on choosing:** See [doc/5G_CORE_NETWORKS.md](../doc/5G_CORE_NETWORKS.md)

### Open5GS: UE Subscriber Registration

When using Open5GS, the launch script automatically calls `core_network_setup/open5gs/scripts/add_ue_subscriber.sh` to register the UE in the Open5GS MongoDB database. This script uses the `open5gs-dbctl` tool (included in the repo) which requires `mongosh` to be installed on the host.

Install the prerequisite:
```bash
sudo apt install mongodb-mongosh
```

### External Core Network

When `CORE_NETWORK=external`, the script **does not start any local core network containers**. Instead it reads the routing parameters from `external.env` (in the `demo/` directory):

| Variable | Description |
|---|---|
| `UE_ROUTE_SUBNET` | Subnet routed through the UPF (added as a host route via `phine-net`) |
| `UE_ROUTE_GW` | IP of the UPF interface used as the PDU session gateway |
| `UE_CARRIER_FREQ` | Downlink carrier frequency in Hz — must match the external gNB |

Edit `external.env` to match your external core network before launching:

```bash
# Edit the values in external.env, then:
CORE_NETWORK=external ./launch.sh
```

You can also point to a custom env file:

```bash
EXTERNAL_ENV_FILE=/path/to/my-network.env CORE_NETWORK=external ./launch.sh
```

> **⚠️ Important:** When using an external core network, you must ensure that:
> - The UE subscriber (IMSI, keys) is already registered in the external core network.
> - The `phine-net` Docker network (`192.168.70.0/24`) is reachable from the external AMF and UPF.
> - The values in `world_only_external.sdf` (located in `gazebo_launch/src/ign_turtlebot/worlds/`) match your external core network. Edit that file directly to adjust gNB/UE plugin parameters (IPs, IMSI, keys, PLMN, etc.).

### Automatic World Selection

The Gazebo world file is **automatically selected** based on the `CORE_NETWORK` environment variable. The launch file (`gazebo_launch/src/ign_turtlebot/launch/gazebo_launch.launch.py`) reads the `CORE_NETWORK` variable and loads the corresponding world file (`world_only_oai.sdf`, `world_only_free5gc.sdf`, `world_only_open5gs.sdf`, or `world_only_external.sdf`). No manual editing of the launch file or rebuilding is required when switching between core networks.

### RAN Docker Compose Structure

The `oai/` folder in this demo contains the docker-compose files for the Radio Access Network components. The folder is named `oai/` because the gNB and UE are from **OAI v2026.w04** (`oaisoftwarealliance/oai-gnb:2026.w04`), used with all three core networks:

- `docker-compose-gNB.yml` — Base gNB compose file, used for **all** core networks
- `docker-compose-gNB.free5gc.yml` — Docker Compose **extension** for free5GC only (adds the N3 network interface to the gNB)
- `docker-compose-ue.yml` — UE, RCC, and DDS Discovery Server containers
- `conf/gNB_config.yaml` — gNB configuration (modified at runtime by the Gazebo gNB plugin)
- `conf/UE_config.yaml` — UE configuration (modified at runtime by the Gazebo UE plugin)

## Changing Plugin path

In `launch.sh` you can change the `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` variable to point to your Gazebo plugin path (sometimes it can be in /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins).

## Building Images and ROS2 pkgs
On a terminal in the demo directory:

```
./build_images.sh
```

This will build the `gazebo_launch` ROS2 workspace (colcon build), and the `dds_discovery_server` and `rcc` Docker images. The `ue_amr` Docker image is built automatically by the UE plugin at simulation runtime.

## Running simulation

1. On a terminal, launch the simulation with your chosen core network:

```bash
CORE_NETWORK=oai source launch.sh
```

(Replace `oai` with `free5gc` or `open5gs` as needed. If `CORE_NETWORK` is not set, it defaults to `oai`.)

Insert your password when required.

This command will launch the CN containers (from `core_network_setup/`), the Gazebo Simulation with the gNB embedded (visualization + container), the robot-UE container, and the RCC container. It will take 40 seconds to start everything, finishing when Rviz opens.
The first time you launch the simulation, it might fail since the UE plugin will build the robot-UE image for a long time.

2. On Rviz click on "Nav2 goal" and then select a feasible location.

3. Whenever you want, look for the three vertical dots on the top right of Gazebo GUI, click it and in the search box of Gazebo search "UE_Power_Plugin" and "gNB_Power_Plugin". In the Gazebo GUI two buttons will pop up, that will switch on or off the UE and gNB of the robot.

4. Press the gNB button, that will switch both gNB and UE off.

5. The robot will continue its path (or it will stay still if it does not have one), you can try to give another goal but the communication won't be enabled.

6. Switch on the UE and gNB from the buttons.

7. Give another Nav2 Goal or wait for the robot to finish its path.

## Remove all the containers and kill Gazebo

In the demo folder, run the kill script with the same `CORE_NETWORK` value:

```bash
CORE_NETWORK=oai ./kill.sh
```

## Modify gNB parameters

Inside the world file for your chosen CN, change the gNB parameters. The world files are located in `gazebo_launch/src/ign_turtlebot/worlds/`:

- `world_only_oai.sdf` — For OAI
- `world_only_free5gc.sdf` — For free5GC
- `world_only_open5gs.sdf` — For Open5GS

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

After modifying world files, rebuild the workspace:
```bash
cd gazebo_launch
colcon build
source install/setup.bash
```

## Modify UE parameters

Inside the world file for your chosen CN (same files as above), change the UE parameters:

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

If you want to manage the CN containers separately, you can comment out the CN launch section in `launch.sh`:

```sh
# cd "$CN_DIR"
# docker compose up -d
```

## Changing ROS DOMAIN (avoiding multi-user bugs)

Set the `ROS_DOMAIN_ID` environment variable before launching (each PC should have its own):

```bash
ROS_DOMAIN_ID=42 CORE_NETWORK=oai source launch.sh
```

## Changing GZ partition (avoiding multi-user bugs)

Set the `IGN_PARTITION` environment variable before launching (each PC should have its own):

```bash
IGN_PARTITION=my_partition CORE_NETWORK=oai source launch.sh
```

## Using the Button UE and gNB plugin

If you change the container name in the gNB and UE declarations, you should match the same container names for the two environment variables `GNB_NAME_FOR_BUTTON` and `UE_NAME_FOR_BUTTON` (set before launching or export them).

## Scripts in `launch.sh`

### `wait_for_core_network.sh`
Located in `core_network_setup/oai/`, this script ensures that the OAI core network is fully operational before proceeding with the simulation. It waits for the MySQL database container to become healthy and checks that all critical OAI services (`oai-nrf`, `oai-amf`, `oai-smf`, `oai-upf`) are running. The script is called automatically during the simulation launch process to avoid race conditions and ensure a reliable startup sequence.

### `ros_gz_net` bridge creation
The `launch.sh` script creates a custom Docker bridge network (`ros_gz_net`) with a specified subnet. It is used to enable communication between simulation containers (such as Gazebo, robot-UE, and network components) on a dedicated network, ensuring proper connectivity and isolation for the simulation environment. This is executed as part of the launch process and typically does not require manual intervention.
