export DEMO_PROJECT_PATH=$(pwd)
export ROS_DOMAIN_ID=10
export IGN_PARTITION=domain1
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=~/workspace/install/lib
export ROS_DISCOVERY_SERVER="192.168.70.159:11811"
export GNB_NAME_FOR_BUTTON="gNB1"
export UE_NAME_FOR_BUTTON="ue_turtlebot"


cd oai_setup
docker compose up -d 
sleep 5


docker compose -f docker-compose-ue.yml up dds_discovery_server -d
docker network create \
  --driver bridge \
  --subnet=192.168.73.128/26 \
  --opt com.docker.network.bridge.name="ros_gz_net" \
  ros_gz_net

xhost +local:docker
docker compose -f docker-compose-ue.yml up simulation -d
sleep 10
docker compose -f docker-compose-ue.yml up rcc -d
cd ..
