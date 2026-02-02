export PROJECT_PATH=$(pwd)
export ROS_DOMAIN_ID=10
export IGN_PARTITION=domain1
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=~/workspace/install/lib
export ROS_DISCOVERY_SERVER="192.168.70.159:11811"
export GNB_NAME_FOR_BUTTON="gNB1"
export UE_NAME_FOR_BUTTON="ue_turtlebot"
source $PROJECT_PATH/gazebo_launch/install/setup.bash


cd oai_setup
docker compose up -d 
cd ..
sleep 5


sudo ip route add 10.0.0.0/16 via 192.168.70.134 dev phine-net
cd oai_setup
docker compose -f docker-compose-ue.yml up dds_discovery_server -d
cd ..
docker network create \
  --driver bridge \
  --subnet=192.168.73.128/26 \
  --opt com.docker.network.bridge.name="ros_gz_net" \
  ros_gz_net
  
export IGN_IP=192.168.73.129
ros2 launch ign_turtlebot gazebo_launch.launch.py &
sleep 10
cd oai_setup
xhost +local:docker
docker compose -f docker-compose-ue.yml up rcc -d
cd ..
