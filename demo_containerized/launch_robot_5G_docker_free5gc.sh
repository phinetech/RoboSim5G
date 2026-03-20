export DEMO_PROJECT_PATH=$(pwd)
export ROS_DOMAIN_ID=10
export IGN_PARTITION=domain1
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=~/workspace/install/lib
export ROS_DISCOVERY_SERVER="192.168.70.159:11811"
export GNB_NAME_FOR_BUTTON="gNB1"
export UE_NAME_FOR_BUTTON="ue_turtlebot"
export GPU_RUNTIME="nvidia"


cd free5gc_setup
docker compose up -d 
./wait_for_core_network.sh
docker compose -f docker-compose-ue.yml up dds_discovery_server -d
./create_physical_bridge.sh
xhost +local:docker
docker compose -f docker-compose-ue.yml up simulation -d
docker compose -f docker-compose-ue.yml up rcc -d
cd ..
