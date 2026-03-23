# free5GC Core Network Deployment for RoboSim5G

This deployment provides a free5GC 5G Core Network that integrates with RoboSim5G.

## About free5GC

free5GC is an open-source implementation of 5G Core Network (5GC) compliant with 3GPP Release 15 and beyond. Key features:
- **Web-based management interface** for subscriber configuration
- **Full 5G Core implementation** with all network functions (AMF, SMF, UPF, etc.)
- **MongoDB-based persistence** for subscriber and network data
- **Flexible architecture** supporting network slicing and multiple DNNs
- **Hybrid UPF support** - uses OAI UPF for better performance and eBPF data plane

## Architecture

This deployment uses a **hybrid approach**:
- **Control Plane**: free5GC network functions (AMF, SMF, UDM, UDR, etc.)
- **User Plane**: OAI UPF (User Plane Function) with eBPF-based data plane

This combination leverages free5GC's user-friendly WebUI while benefiting from OAI UPF's performance optimizations.

## Quick Start

### 1. Start the Core Network
```bash
docker compose up -d
```

Wait for all services to become healthy (this may take 10-20 seconds):
```bash
docker compose ps
```

All services should show "running" status.

### 2. Configure Subscribers via WebUI

Open http://localhost:5000 in your browser.

**Default credentials:**
- Username: `admin`
- Password: `free5gc`

**Add a subscriber:**
1. Navigate to "Subscribers" in the left menu
2. Click the "+ NEW SUBSCRIBER" button
3. Enter the following details:
   - **SUPI (IMSI)**: `imsi-208930000000001`
   - **Operator Key (K)**: `8baf473f2f8fd09487cccbd7097c6862`
   - **OPc Type**: Select `OPc`
   - **OPc**: `8e27b6af0e692e750f32667a3b14605d`
4. Scroll down to **S-NSSAI Configuration**:
   - Click "+ Add S-NSSAI"
   - **SST**: `1`
   - **SD**: `66051` (hex: `010203`)
   - **Default S-NSSAI**: ✓ (check this box)
5. Under **Data Network Name**:
   - Click the "+ DNN" button
   - **DNN**: `internet`
   - **Default DNN**: ✓ (check this box)
6. Click "CREATE" at the bottom

> **Note:** The first subscriber (`imsi-208930000000001`) with the credentials above should be pre-configured in the MongoDB dump. You can add additional subscribers for multiple robots by incrementing the IMSI (e.g., `imsi-208930000000002`, `imsi-208930000000003`).

### 3. Start gNB and UE

Follow the RoboSim5G demo instructions to start your Gazebo simulation with the robot. The gNB and UE will automatically connect to this core network.

Alternatively, for standalone testing:
```bash
# Start gNB
docker compose -f docker-compose-gNB.yml up -d

# Start UE
docker compose -f docker-compose-ue.yml up -d
```

## Managing the Core Network

### Check Service Status
```bash
docker compose ps
```

All services should show "running" status. The UPF runs in `host` network mode for performance reasons.

### View Logs
```bash
# View all services
docker compose logs -f

# View specific service
docker compose logs -f amf
docker compose logs -f smf
docker compose logs -f upf
```

### Verify Network Functions Registration
Check that all NFs have registered with NRF:
```bash
docker logs nrf
```

You should see registration messages from AMF, SMF, UDM, UDR, AUSF, PCF, and other network functions.

### Stop the Core Network
```bash
docker compose down
```

> **Important:** Do NOT use `docker compose down -v` as this will delete the MongoDB database with all subscriber configurations.

### Restart Services
```bash
# Restart all services
docker compose restart

# Restart specific service
docker compose restart amf
docker compose restart upf
```

## Subscriber Data Persistence

Subscriber data is stored in the MongoDB container and persists across container restarts. The database is initialized from the dump files in `./free5gc/dump/` on first startup.

To backup subscriber data:
```bash
docker exec mongodb mongodump --db=free5gc --out=/data/backup
docker cp mongodb:/data/backup ./backup_$(date +%Y%m%d)
```

To restore subscriber data:
```bash
docker cp ./backup_folder mongodb:/data/restore
docker exec mongodb mongorestore --db=free5gc /data/restore/free5gc
```

## Network Configuration

The deployment uses three Docker networks:

### Control Plane Network (public_net)
- **Subnet**: `192.168.70.128/26`
- **Bridge name**: `phine-net`
- All core network functions communicate on this network

### N3 Network (User Plane - gNB to UPF)
- **Subnet**: `192.168.71.128/26`
- **Bridge name**: `demo-n3`
- GTP-U tunnel between gNB and UPF

### N6 Network (Data Network)
- **Subnet**: `192.168.72.128/26`
- **Bridge name**: `demo-n6`
- Connection from UPF to external data network (internet)

### UE IP Address Pool
- **UE Data Network**: `10.60.0.0/24`
- IP addresses assigned to UE tunnel interfaces

### Key Service Endpoints
- **AMF (N2 interface)**: `192.168.70.132:38412` (SCTP)
- **UPF (N3 interface)**: Host network mode, GTP-U on port `2152`
- **SMF**: `192.168.70.133:8000`
- **NRF**: `192.168.70.130:8000`
- **WebUI**: `http://localhost:5000`
- **External Data Network**: `192.168.72.135` (traffic server container)

## Configuration Files

Configuration files are located in the following directories:

### free5GC Network Functions
All free5GC NF configs are in `./free5gc/config/`:
- `amfcfg.yaml` - Access and Mobility Management Function
- `smfcfg.yaml` - Session Management Function
- `uerouting.yaml` - UE routing information for SMF
- `ausfcfg.yaml` - Authentication Server Function
- `udmcfg.yaml` - Unified Data Management
- `udrcfg.yaml` - Unified Data Repository
- `nrfcfg.yaml` - Network Repository Function
- `pcfcfg.yaml` - Policy Control Function
- `nssfcfg.yaml` - Network Slice Selection Function
- `chfcfg.yaml` - Charging Function
- `nefcfg.yaml` - Network Exposure Function
- `webuicfg.yaml` - Web User Interface configuration

### OAI UPF Configuration
- `./oai/oai_upf_config.yaml` - OAI UPF configuration with eBPF data plane

### Radio Access Network
- `./oai/gNB_config.yaml` - gNB configuration
- `./oai/UE_config.yaml` - UE configuration with subscriber credentials

### Database Initialization
- `./free5gc/dump/` - MongoDB dump files for subscriber data initialization
- `./free5gc/mongorestore.sh` - Script to restore MongoDB data on startup

## Default Network Parameters

The deployment is pre-configured with the following network identifiers:

- **MCC (Mobile Country Code)**: `208` (France)
- **MNC (Mobile Network Code)**: `93`
- **PLMN ID**: `20893`
- **TAC (Tracking Area Code)**: `000001`
- **AMF ID**: `cafe00`

### Network Slices (S-NSSAI)
Two network slices are configured by default:
1. **SST**: `1`, **SD**: `010203` - Default slice for robot communication
2. **SST**: `1`, **SD**: `112233` - Alternative slice

### Data Networks (DNN)
- **internet** - Main data network for robot connectivity

## Troubleshooting

### UE Not Attaching
1. **Check subscriber exists in WebUI:**
   - Log into http://localhost:5000
   - Verify IMSI is configured with correct credentials
   - Ensure S-NSSAI and DNN match UE configuration

2. **Verify subscriber credentials match:**
   - IMSI, K, OPc in WebUI must match `./oai/UE_config.yaml`
   - Check that DNN (`internet`) is configured
   - Verify S-NSSAI (SST=1, SD=66051) is set

3. **Check AMF logs:**
   ```bash
   docker compose logs -f amf
   ```
   Look for:
   - Registration Request/Accept messages
   - Authentication success/failure
   - N2 connection from gNB

4. **Check authentication in AUSF logs:**
   ```bash
   docker compose logs -f ausf
   ```

### gNB Not Connecting to AMF
1. **Verify network connectivity:**
   ```bash
   docker exec oai-gNB1 ping -c 3 192.168.70.132
   ```

2. **Check AMF logs for NG Setup:**
   ```bash
   docker compose logs -f amf
   ```
   Look for "Handling NG Setup Request" messages

3. **Verify PLMN configuration:**
   - gNB PLMN (in `gNB_config.yaml`) must match AMF PLMN (208/93)
   - Check `./oai/gNB_config.yaml` and `./free5gc/config/amfcfg.yaml`

### No Internet Access from UE
1. **Check UE has tunnel interface:**
   ```bash
   docker exec <ue-container-name> ip addr show
   ```
   Should see `oaitun_ue1` with IP in `10.60.0.0/24` range

2. **Verify UPF routing:**
   ```bash
   docker logs upf
   ```
   Look for session establishment and GTP-U tunnel creation

3. **Check external data network:**
   ```bash
   docker exec oai-ext-dn ip route
   ```
   Should show route to `10.60.0.0/24` via UPF

4. **Test connectivity from UE:**
   ```bash
   docker exec <ue-container-name> ping -I oaitun_ue1 8.8.8.8
   docker exec <ue-container-name> ping -I oaitun_ue1 192.168.72.135
   ```

### UPF Not Connecting to SMF
The UPF runs in host network mode and must be able to reach the SMF:

1. **Check UPF logs:**
   ```bash
   docker logs upf
   ```
   Look for PFCP association with SMF

2. **Verify SMF configuration:**
   Check `./free5gc/config/smfcfg.yaml` for correct UPF endpoint

3. **Check UPF can reach SMF:**
   ```bash
   docker exec upf ping -c 3 192.168.70.133
   ```

### WebUI Connection Issues
If WebUI cannot connect to MongoDB:
```bash
docker compose restart mongodb webui
```

Wait 10 seconds for MongoDB to fully start, then restart WebUI.

### Database Initialization Failed
If subscriber data is not loaded on first startup:

1. **Check MongoDB logs:**
   ```bash
   docker compose logs mongodb
   ```

2. **Manually restore the database:**
   ```bash
   docker exec mongodb /docker-entrypoint-initdb.d/mongorestore.sh
   ```

## Advanced Configuration

### Adding Multiple Subscribers
For multi-robot simulations, add subscribers with incrementing IMSIs:

| Robot | IMSI | Use in UE config |
|-------|------|------------------|
| Robot 1 | `imsi-208930000000001` | `208930000000001` |
| Robot 2 | `imsi-208930000000002` | `208930000000002` |
| Robot 3 | `imsi-208930000000003` | `208930000000003` |

Use the WebUI to add each subscriber with the same K and OPc values.

### Changing Network Slice Configuration
To modify S-NSSAI values:

1. Update `./free5gc/config/amfcfg.yaml` (AMF supported slices)
2. Update `./free5gc/config/nssfcfg.yaml` (NSSF slice selection)
3. Update subscriber configuration in WebUI
4. Update UE configuration in `./oai/UE_config.yaml`
5. Restart affected services

### Customizing UE IP Pool
To change the UE IP address range:

1. Edit `./free5gc/config/smfcfg.yaml`:
   ```yaml
   userplaneInformation:
     upfInfo:
       ueIPPools:
         - cidr: 10.60.0.0/24  # Change this
   ```

2. Edit `./oai/oai_upf_config.yaml`:
   ```yaml
   upf:
     ue_ip_pools:
       - network: 10.60.0.0/24  # Change this
   ```

3. Update the external data network container environment:
   ```yaml
   oai-ext-dn:
     environment:
       - UE_IP_ADDRESS_POOL=10.60.0.0/24  # Change this
   ```

4. Restart services:
   ```bash
   docker compose restart smf upf
   docker compose restart oai-ext-dn
   ```

## Integration with RoboSim5G

When using this CN with RoboSim5G:

1. **Start the core network first:**
   ```bash
   cd demo/free5gc_setup
   docker compose up -d
   ```

2. **Configure subscribers** for your robots via WebUI at http://localhost:5000

3. **Launch your Gazebo simulation** with RoboSim5G plugins:
   - The gNB plugin will automatically connect to the AMF
   - The UE plugin will register with credentials from the plugin configuration

4. **Verify connectivity:**
   - Check AMF logs for successful UE registration
   - Verify UE has established PDU session
   - Test data connectivity through the 5G network

Each robot in your simulation can have its own IMSI, network slice, and DNN configuration, enabling realistic multi-robot 5G scenarios with network slicing and QoS differentiation.


## Performance Considerations

### UPF in Host Mode
The OAI UPF runs in host network mode for best performance:
- Reduces network stack overhead
- Enables eBPF-based fast path
- Requires proper host routing configuration

### eBPF Data Plane
The UPF uses eBPF for high-performance packet processing:
- Requires kernel >= 5.4
- Needs `SYS_ADMIN` and `NET_ADMIN` capabilities
- Check `/sys/fs/bpf` is mounted

## Additional Resources

- [free5GC Official Website](https://free5gc.org/)
- [free5GC Documentation](https://free5gc.org/guide/)
- [free5GC GitHub](https://github.com/free5gc/free5gc)
- [OAI UPF Documentation](https://gitlab.eurecom.fr/oai/cn5g/oai-cn5g-upf)
- [RoboSim5G Project](https://github.com/phinetech/RoboSim5G)

## Support

For issues specific to:
- **free5GC**: Check [free5GC forum](https://forum.free5gc.org/)
- **OAI UPF**: Check [OAI GitLab issues](https://gitlab.eurecom.fr/oai/cn5g/oai-cn5g-upf/-/issues)
- **RoboSim5G integration**: Use the [RoboSim5G Slack chat](https://join.slack.com/t/robosimworkspace/shared_invite/zt-38i7sbsit-FpsT6d7PU241~nGz0fcUig)

