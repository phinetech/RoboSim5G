pkill -f ign\ gazebo

cd oai_setup
docker compose -f docker-compose-ue.yml down
docker ps -q --filter ancestor=oaisoftwarealliance/oai-gnb:2024.w44 | xargs -I {} docker stop {} && docker ps -a -q --filter ancestor=oaisoftwarealliance/oai-gnb:2024.w44 | xargs -I {} docker rm {}
docker compose down
xhost -local:docker
cd ..
