cd images/simulation
docker build -t simulation .
cd ../..
cd images/rcc
docker build -t rcc .
cd ../..
cd images/dds_discovery_server
docker build -t dds_discovery_server .
cd ../..