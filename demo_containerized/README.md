## Requirements

- Docker version 27.5.1, build 9f9e405

- Docker Compose version v2.33.1

- `mongodb-mongosh` (required only for Open5GS core network — install with `sudo apt install mongodb-mongosh`)

## Choosing a Core Network

RoboSim5G supports **three 5G Core Network implementations**: OAI, free5GC, and Open5GS.

A single `launch.sh` and `kill.sh` script is provided. You select the core network using the `CORE_NETWORK` environment variable (defaults to `oai`):

```bash
CORE_NETWORK=oai       ./launch.sh   # OpenAirInterface (default)
CORE_NETWORK=free5gc   ./launch.sh   # free5GC
CORE_NETWORK=open5gs   ./launch.sh   # Open5GS
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

### Automatic World Selection

The Gazebo world file is **automatically selected** based on the `CORE_NETWORK` environment variable. The launch file (`gazebo_launch.launch.py`) inside the simulation container reads the `CORE_NETWORK` variable and loads the corresponding world file (`world_only_oai.sdf`, `world_only_free5gc.sdf`, or `world_only_open5gs.sdf`). No manual editing of the launch file or rebuilding of the simulation image is required when switching between core networks.

### RAN Docker Compose Structure

The `oai/` folder in this demo contains the docker-compose files for the Radio Access Network components. The folder is named `oai/` because the gNB and UE are from **OAI v2026.w04** (`oaisoftwarealliance/oai-gnb:2026.w04`), used with all three core networks:

- `docker-compose-gNB.yml` — Base gNB compose file, used for **all** core networks
- `docker-compose-gNB.free5gc.yml` — Docker Compose **extension** for free5GC only (adds the N3 network interface to the gNB)
- `docker-compose-ue.yml` — UE, RCC, DDS Discovery Server, and **simulation** containers
- `conf/gNB_config.yaml` — gNB configuration (modified at runtime by the Gazebo gNB plugin)
- `conf/UE_config.yaml` — UE configuration (modified at runtime by the Gazebo UE plugin)

## Configuration via Volumes and `DEMO_PROJECT_PATH`

In the containerized demo, configuration files are **not baked into the Docker images**. Instead, they are mounted as volumes from the host, allowing the Gazebo plugins running inside the simulation container to modify them at runtime.

### The `DEMO_PROJECT_PATH` Environment Variable

`DEMO_PROJECT_PATH` is set by `launch.sh` to the absolute path of the `demo_containerized/` directory on the host. It is critical for all volume mounts in the docker-compose files to resolve correctly. The docker-compose files use it to mount:

- **UE config**: `${DEMO_PROJECT_PATH}/oai/conf/UE_config.yaml` → `/opt/oai-nr-ue/etc/nr-ue.yaml` (inside the UE container)
- **gNB config**: `${DEMO_PROJECT_PATH}/oai/conf/gNB_config.yaml` → `/opt/oai-gnb/etc/gnb.yaml` (inside the gNB container)
- **OAI folder**: `${DEMO_PROJECT_PATH}/oai` → `/app/oai` (inside the simulation container)

### How Plugin Configuration Works Inside the Container

The simulation container has `PROJECT_PATH=/app` set internally. The Gazebo plugins (gNB_plugin and UE_plugin) read `PROJECT_PATH` and modify files under `$PROJECT_PATH/oai/` — i.e., `/app/oai/conf/gNB_config.yaml` and `/app/oai/docker-compose-gNB.yml`. Because `/app/oai` is a volume mount from the host (`${DEMO_PROJECT_PATH}/oai`), these modifications are reflected on the host filesystem.

### Docker Socket Volume

The simulation container mounts the host Docker socket (`/var/run/docker.sock:/var/run/docker.sock`). This allows the Gazebo plugins, which run inside the simulation container, to execute `docker compose` commands that actually run on the **host** Docker daemon. This is how the gNB and UE containers are launched by the plugins — the plugin calls `docker compose -f docker-compose-gNB.yml up -d` from inside the simulation container, but the container is created on the host.

**This means:**
- The gNB and UE containers are **sibling containers** (not nested), running directly on the host Docker daemon
- The plugins modify the config files through the volume mount, then launch sibling containers using those modified configs
- `DEMO_PROJECT_PATH` must be set correctly, otherwise the volume mounts will fail and the plugins won't be able to find the configuration files or launch containers

## Building Images
On a terminal in the demo_containerized directory:

```
./build_images.sh
```

This will build the `simulation`, `rcc`, and `dds_discovery_server` Docker images. The `ue_amr` Docker image is built automatically by the UE plugin at simulation runtime.

## Running simulation

1. On a terminal, launch the simulation with your chosen core network:

```bash
CORE_NETWORK=oai source launch.sh
```

(Replace `oai` with `free5gc` or `open5gs` as needed. If `CORE_NETWORK` is not set, it defaults to `oai`.)

Insert your password when required.

This command will launch the CN containers (from `core_network_setup/`), the Gazebo Simulation container with the gNB embedded, the robot-UE container, and the RCC container. It will take 40 seconds to start everything, finishing when Rviz opens.
The first time you launch the simulation, it might fail since the UE plugin will build the robot-UE image for a long time.

2. On Rviz click on "Nav2 goal" and then select a feasible location.

3. Whenever you want, look for the three vertical dots on the top right of Gazebo GUI, click it and in the search box of Gazebo search "UE_Power_Plugin" and "gNB_Power_Plugin". In the Gazebo GUI two buttons will pop up, that will switch on or off the UE and gNB of the robot.

4. Press the gNB button, that will switch both gNB and UE off.

5. The robot will continue its path (or it will stay still if it does not have one), you can try to give another goal but the communication won't be enabled.

6. Switch on the UE and gNB from the buttons.

7. Give another Nav2 Goal or wait for the robot to finish its path.

## Remove all the containers

In the demo_containerized folder, run the kill script with the same `CORE_NETWORK` value:

```bash
CORE_NETWORK=oai ./kill.sh
```

## Modify gNB parameters

Inside the world file for your chosen CN, change the gNB parameters. The world files are located in `images/simulation/gazebo_launch/src/ign_turtlebot/worlds/`:

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

After modifying world files, rebuild the simulation image:
```bash
./build_images.sh
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

## NVIDIA GPU Acceleration (Optional)

To enable hardware acceleration for the Gazebo simulation, you need an NVIDIA GPU and the **NVIDIA Container Toolkit** installed on your host machine. This significantly improves simulation frame rates and reduces CPU load.

### 1. Install the Toolkit
Follow the official [NVIDIA Container Toolkit Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) to set up the repository and install the `nvidia-container-toolkit` package.

### 2. Configure the Launch Script
The simulation container is designed to switch between the standard Linux runtime and the NVIDIA runtime dynamically. Set the `GPU_RUNTIME` variable before launching:

```bash
GPU_RUNTIME=nvidia CORE_NETWORK=oai source launch.sh
```

Set `GPU_RUNTIME` to `"nvidia"` to use your GPU, or leave it unset / set to `"runc"` for standard CPU execution.

## Configuring Containerized Simulation

To develop and deploy custom robotic logic within the existing containerized environment, follow these steps to integrate your ROS 2 workspaces and launch scripts.
To add custom code, it should be considered that the images have installed:

- Ubuntu 22.04

- ROS2 humble

- gazebo ignition fortress 2.6.9

### Directory Layout for Customization

The key directories for customization are all under `images/simulation/`:

| Directory | Purpose |
|-----------|---------|
| `images/simulation/images/ue_amr/` | Contains the Dockerfile and files for the **robot-UE container** image. Place additional ROS 2 packages or dependencies here. The UE plugin will copy your robot logic into this folder, build the image, and launch it. |
| `images/simulation/nav_stack/` | Contains the **robot logic** ROS 2 workspace (navigation stack, ros_gz_bridge, etc.). The UE plugin copies this workspace into the robot-UE container at runtime and builds it inside the container. Modify this to change what the robot does. |
| `images/simulation/gazebo_launch/` | Contains the **Gazebo world** workspace with world files, plugin declarations, and the launch file. Modify world files here to change gNB/UE parameters, add models, etc. |

The `oai/` folder (at `demo_containerized/oai/`, mounted into the simulation container as `/app/oai`) contains:
- `docker-compose-gNB.yml` / `docker-compose-ue.yml` — Control how the gNB and UE containers are launched, their networks, volumes, and entrypoints. Modify these to change container behavior (e.g., add environment variables, change network settings).
- `conf/gNB_config.yaml` / `conf/UE_config.yaml` — 5G configuration files modified at runtime by the plugins.

### Minimal Customization (Let the Plugins Handle Everything)

If your changes are limited to the robot logic and world parameters, you can simply:

1. Place your robot ROS 2 workspace in `images/simulation/nav_stack/`
2. Update the UE plugin parameters in the world file (`<robot_package_name>`, `<robot_project_name>`, `<robot_launch_file_name>`) to match your workspace
3. Rebuild the simulation image with `./build_images.sh`

The Gazebo plugins will handle the rest — copying the robot workspace into the UE container, building it, modifying the docker-compose and config files, and launching the gNB and UE containers automatically.


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

Locate the world file for your chosen CN in:

    images/simulation/gazebo_launch/src/ign_turtlebot/worlds/world_only_<cn>.sdf

Modify:

    <plugin name="phine_plugins::UE_plugin" filename="UE_plugin">
        ...
        <robot_project_name>your_custom_workspace</robot_project_name>
        <robot_package_name>your_custom_package_name</robot_package_name>
        <robot_launch_file_name>your_robot_logic.launch.py</robot_launch_file_name>
        ...
    </plugin>

---

## 3. Update the Launch Orchestration

To ensure the simulation container starts with your specific world and configurations, modify the `oai/docker-compose-ue.yml` file.

Update the `entrypoint` associated with the simulation service to call your custom launch file:

    # Inside oai/docker-compose-ue.yml
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

## Scripts in `launch.sh`

### `wait_for_core_network.sh`
Located in `core_network_setup/oai/`, this script ensures that the OAI core network is fully operational before proceeding with the simulation. It waits for the MySQL database container to become healthy and checks that all critical OAI services (`oai-nrf`, `oai-amf`, `oai-smf`, `oai-upf`) are running. The script is called automatically during the simulation launch process to avoid race conditions and ensure a reliable startup sequence.

### `ros_gz_net` bridge creation
The `launch.sh` script creates a custom Docker bridge network (`ros_gz_net`) with a specified subnet. It is used to enable communication between simulation containers (such as Gazebo, robot-UE, and network components) on a dedicated network, ensuring proper connectivity and isolation for the simulation environment. This is executed as part of the launch process and typically does not require manual intervention.

## Important

After making changes to the world files or the Dockerfile, remember to rebuild your images to apply the updates to the container layers:

    ./build_images.sh
