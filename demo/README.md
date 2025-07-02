## Running the Demo
In one terminal, launch the core network and set the environment variables (plugin folder location and gazebo model folder location):
```bash
cd path/demo
source start_CN-set_env.sh
```
If you get an error, you can launch in the same terminal  `./kill_all.sh` and then relaunch the instruction.
In the same terminal, launch the gazebo world with 2 gNBs:
```bash
ign gazebo world-file.sdf
```
two gNBs should appear in the gazebo simulation and in the terminal 2 containers should start. If some parameter related of the gNB should be changed, change the gNB declaration in the world file. It is possible to add another gNB by adding another gNB declaration (ex. gN3).
To check if the gNBs are working properly, it is possible to see whether they connected to the amf by (wait 5-10 seconds):
```bash
docker logs oai-amf
```
If everything goes right you should see a table:
``` |------------------------------gNBs' Information-------------------------|

    Index    |    Status     |   Global Id    |      gNB Name   |        PLMN              
      1      |    Connected  |     0x01       |        gNB1     |       001,01      
      
      2      |    Connected  |     0x02       |        gNB2     |       001,01
    -------------------------------------------------------------------------  
```
When the simulation is stopped use the same terminal to stop and remove the containers:
```bash
./kill_all.sh
```
It is important to use the same terminal since this `.sh` file uses the environment variable exported by `start_CN-set_env.sh`.

## Modify gNB parameters 
Inside `world-file.sdf` the two gNB are declared, you can modify those parameters following the `World file declaration` section of [README.md](https://github.com/phinetech/RoboSim5G/blob/develop/README.md) in the main folder of this repository. You can add another gNB by copying the declaration and changing the name and pose fields.
