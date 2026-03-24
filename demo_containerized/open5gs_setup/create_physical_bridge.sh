docker network create \
  --driver bridge \
  --subnet=192.168.73.128/26 \
  --opt com.docker.network.bridge.name="ros_gz_net" \
  ros_gz_net