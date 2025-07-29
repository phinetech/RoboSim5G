cd gazebo_launch/
colcon build
cd ..
cd images/dds_discovery_server
docker build -t dds_discovery_server .
cd ../..
cd images/rcc
docker build -t rcc .
cd ../..
