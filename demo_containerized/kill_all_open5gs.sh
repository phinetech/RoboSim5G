cd open5gs_setup
docker compose -f docker-compose-ue.yml down
docker ps -q --filter ancestor=oaisoftwarealliance/oai-gnb:2026.w04 | xargs -I {} docker stop {} && docker ps -a -q --filter ancestor=oaisoftwarealliance/oai-gnb:2026.w04 | xargs -I {} docker rm {}
docker compose down
docker network rm ros_gz_net
xhost -local:docker
cd ..
