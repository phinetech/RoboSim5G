#!/bin/bash
# Stop and rm all gNb containers
docker ps -q --filter ancestor=oaisoftwarealliance/oai-gnb:2024.w44 | xargs -I {} docker stop {} && docker ps -a -q --filter ancestor=oaisoftwarealliance/oai-gnb:2024.w44 | xargs -I {} docker rm {}
# Stop and rm CN containers and network
cd $PROJECT_PATH/oai_setup && docker-compose down
cd ..