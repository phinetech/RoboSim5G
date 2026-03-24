export PROJECT_PATH=$(pwd)
export ROS_DOMAIN_ID=10
export IGN_PARTITION=domain1
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=~/workspace/install/lib
export ROS_DISCOVERY_SERVER="192.168.70.159:11811"
export GNB_NAME_FOR_BUTTON="gNB1"
export UE_NAME_FOR_BUTTON="ue_turtlebot"
source $PROJECT_PATH/gazebo_launch/install/setup.bash


cd open5gs
docker compose up -d 
sleep 10
sudo ip route add 10.0.0.0/16 via 192.168.70.134 dev phine-net
docker compose -f docker-compose-ue.yml up dds_discovery_server -d
./create_physical_bridge.sh
cd ..
  
export IGN_IP=192.168.73.129
ros2 launch ign_turtlebot gazebo_launch.launch.py &
cd open5gs
xhost +local:docker
docker compose -f docker-compose-ue.yml up rcc -d
cd ..
