# OAI Core Network Deployment for RoboSim5G

This deployment provides an OpenAirInterface (OAI) 5G Core Network that integrates with RoboSim5G.

## About OAI (OpenAirInterface)

OpenAirInterface is a leading open-source implementation of 5G Core Network (5GC) and Radio Access Network (RAN) developed by EURECOM. Key features:
- **Full 3GPP compliance** with 5G Core specifications
- **High performance** C++-based network functions
- **Modular architecture** with independent containerized NFs
- **Advanced features** including network slicing, QoS policies, and eBPF data plane
- **Active development** backed by research and industry collaboration
- **Database-driven** subscriber management using MySQL

## Architecture

This deployment includes:
- **Control Plane NFs**: AMF, SMF, UDM, UDR, AUSF, NRF, PCF
- **User Plane**: OAI UPF with eBPF-based high-performance data plane
- **Database**: MySQL for subscriber and network configuration
- **Traffic Server**: External data network for internet access simulation

## Quick Start

### 1. Start the Core Network
```bash
docker compose up -d
```

Wait for all services to become healthy (this may take 15-30 seconds):
```bash
docker compose ps
```

All services should show "healthy" status.

### 2. Verify Core Network is Running

Check that the UDR has completed initialization:
```bash
docker logs oai-udr
```

You should see at the end:
```
[udr_app] [debug] Status: REGISTERED
[udr_app] [debug] HeartBeat timer: 50
[udr_app] [debug] Priority: 1
[udr_app] [debug] Capacity: 100
```

### 3. Configure Subscribers

Subscribers are configured via the MySQL database. The default subscriber is already configured:

**Default subscriber credentials:**
- **IMSI**: `001010000000101`
- **K (Permanent Key)**: `fec86ba6eb707ed08905757b1bb44b8f`
- **OPc (Operator Key)**: `C42449363BBAD02B66D16BC975D77CC1`
- **DNN**: `oai`
- **S-NSSAI**: SST=`1`, SD=`FFFFFF` (no slice differentiator)

#### Adding Additional Subscribers

To add more subscribers for multi-robot scenarios, connect to the MySQL database:

```bash
docker exec -it mysql mysql -u root -plinux oai_db
```

Then run SQL commands (example for second robot):

```sql
-- Add authentication data
INSERT INTO AuthenticationSubscription (ueid, authenticationMethod, encPermanentKey, protectionParameterId, sequenceNumber, authenticationManagementField, algorithmId, encOpcKey, encTopcKey, vectorGenerationInHss, n5gcAuthMethod, rgAuthenticationInd, supi)
VALUES ('001010000000102', '5G_AKA', 'fec86ba6eb707ed08905757b1bb44b8f', 'fec86ba6eb707ed08905757b1bb44b8f', '{"sqn": "000000000020", "sqnScheme": "NON_TIME_BASED", "lastIndexes": {"ausf": 0}}', '8000', 'milenage', 'C42449363BBAD02B66D16BC975D77CC1', NULL, NULL, NULL, NULL, '001010000000102');

-- Add session management subscription data
INSERT INTO SessionManagementSubscriptionData (ueid, servingPlmnid, singleNssai, dnnConfigurations)
VALUES ('001010000000102', '00101', '{"sst": 1, "sd": "FFFFFF"}',
'{"oai":{"pduSessionTypes":{"defaultSessionType": "IPV4"},"sscModes": {"defaultSscMode": "SSC_MODE_1"},"5gQosProfile": {"5qi": 6,"arp":{"priorityLevel": 15,"preemptCap": "NOT_PREEMPT","preemptVuln":"PREEMPTABLE"},"priorityLevel":1},"sessionAmbr":{"uplink":"1000Mbps", "downlink":"1000Mbps"}}}');

-- Exit MySQL
exit;
```

> **Note:** Increment the IMSI for each additional robot: `001010000000102`, `001010000000103`, etc.

### 4. Start gNB and UE

Follow the RoboSim5G demo instructions to start your Gazebo simulation with the robot. The gNB and UE will automatically connect to this core network.

Alternatively, for standalone testing:
```bash
# Start gNB
docker compose -f docker-compose-gNB.yml up -d

# Verify gNB connected to AMF (wait 5-10 seconds)
docker logs oai-amf
```

Expected output showing gNB registration:
```
|------------------------------gNBs' Information-------------------------|
| Index | gNB Name | Status | PLMN | Global Id |
| 1 | gnb-rfsim | Connected | 001,01 | 0x0E00 |
```

```bash
# Start UE
docker compose -f docker-compose-ue.yml up oai-nr-ue -d

# Verify UE registered (wait 5-10 seconds)
docker logs oai-amf
```

Expected output showing UE registration:
```
|----------------------UEs' Information-----------------------------------------|
| Index | 5GMM State | IMSI | GUTI |
| 1 | 5GMM-REGISTERED | 001010000000101 | 00101010041631377652 |
```

## Managing the Core Network

### Check Service Status
```bash
docker compose ps
```

All services should show "healthy" status.

### View Logs
```bash
# View all services
docker compose logs -f

# View specific service
docker compose logs -f amf
docker compose logs -f smf
docker compose logs -f upf
docker compose logs -f mysql
```

### Verify Network Functions Registration
Check the NRF to see registered network functions:
```bash
docker logs oai-nrf
```

You should see heartbeat messages from AMF, SMF, UDM, UDR, AUSF, PCF, and UPF.

### Stop the Core Network
```bash
# Stop gNB and UE first (if running)
docker compose -f docker-compose-gNB.yml down
docker compose -f docker-compose-ue.yml down

# Stop core network
docker compose down
```

> **Important:** Do NOT use `docker compose down -v` as this will delete the MySQL database with all subscriber configurations.

### Restart Services
```bash
# Restart all services
docker compose restart

# Restart specific service
docker compose restart amf
docker compose restart upf
```

## Subscriber Data Persistence

Subscriber data is stored in the MySQL database container. The database is initialized from `./database/oai_db.sql` on first startup.

### Backup Database
```bash
docker exec mysql mysqldump -u root -plinux oai_db > backup_$(date +%Y%m%d).sql
```

### Restore Database
```bash
docker exec -i mysql mysql -u root -plinux oai_db < backup_file.sql
```

### View Current Subscribers
```bash
docker exec mysql mysql -u root -plinux oai_db -e "SELECT ueid, authenticationMethod, encPermanentKey, encOpcKey FROM AuthenticationSubscription;"
```

## Network Configuration

The deployment uses a single Docker network for all components:

### Control and User Plane Network (public_net)
- **Subnet**: `192.168.70.128/26`
- **Bridge name**: `phine-net`
- All core network functions and RAN communicate on this network

### UE IP Address Pool
- **UE Data Network**: `10.0.0.0/16`
- IP addresses assigned to UE tunnel interfaces (e.g., `10.0.0.2`, `10.0.0.3`)

### Key Service Endpoints

| Service | IP Address | Port | Protocol | Purpose |
|---------|------------|------|----------|---------|
| **MySQL** | 192.168.70.131 | 3306 | TCP | Subscriber database |
| **NRF** | 192.168.70.130 | 8080 | HTTP | NF discovery |
| **AMF** | 192.168.70.132 | 38412 | SCTP | N2 interface (gNB) |
| **AMF** | 192.168.70.132 | 8080 | HTTP | SBI interface |
| **SMF** | 192.168.70.133 | 8080 | HTTP | SBI interface |
| **SMF** | 192.168.70.133 | 8805 | UDP | PFCP (N4) |
| **UPF** | 192.168.70.134 | 2152 | UDP | GTP-U (N3) |
| **UPF** | 192.168.70.134 | 8805 | UDP | PFCP (N4) |
| **UDR** | 192.168.70.136 | 8080 | HTTP | SBI interface |
| **UDM** | 192.168.70.137 | 8080 | HTTP | SBI interface |
| **AUSF** | 192.168.70.138 | 8080 | HTTP | SBI interface |
| **PCF** | 192.168.70.139 | 8080 | HTTP | SBI interface |
| **Ext DN** | 192.168.70.135 | - | - | External data network |

## Configuration Files

All OAI network functions share a single unified configuration file:

### Main Configuration
- `./conf/config.yaml` - **Unified configuration for all OAI NFs**
  - Contains SBI interfaces for all network functions
  - Defines PLMN, TAC, and slice configurations
  - Configures UPF user plane parameters
  - Sets up MySQL database connection

### Database
- `./database/oai_db.sql` - Initial MySQL database schema and subscriber data

### Health Scripts
- `./healthscripts/mysql-healthcheck.sh` - MySQL container health check

### QoS Policies
- `./policies/qos/pcc_rules/pcc_rules.yaml` - PCC (Policy and Charging Control) rules
- `./policies/qos/policy_decisions/policy_decision.yaml` - Policy decisions
- `./policies/qos/qos_data/qos_data.yaml` - QoS flow configuration

### Radio Access Network (for testing)
- `./conf/gnb.sa.bandn78.fr1.106PRB.rfsim.conf` - gNB configuration (rfsimulator mode)
- `./conf/ue1.conf` - UE configuration with subscriber credentials

## Default Network Parameters

The deployment is pre-configured with the following network identifiers:

- **MCC (Mobile Country Code)**: `001` (Test network)
- **MNC (Mobile Network Code)**: `01`
- **PLMN ID**: `00101`
- **TAC (Tracking Area Code)**: `1`

### Network Slices (S-NSSAI)
One network slice is configured by default:
- **SST**: `1` (eMBB - Enhanced Mobile Broadband)
- **SD**: `FFFFFF` (no slice differentiator)

### Data Networks (DNN)
- **oai** - Main data network for robot connectivity
- **ims** - IP Multimedia Subsystem (available but not used by default)

## Troubleshooting

### UE Not Attaching

1. **Check subscriber exists in database:**
   ```bash
   docker exec mysql mysql -u root -plinux oai_db -e \
     "SELECT ueid FROM AuthenticationSubscription WHERE ueid='001010000000101';"
   ```

2. **Verify UE credentials match database:**
   - Check `./conf/ue1.conf` for IMSI, K, OPc
   - Ensure DNN (`oai`) matches database configuration
   - Verify S-NSSAI (SST=1) is configured

3. **Check AMF logs:**
   ```bash
   docker compose logs -f amf
   ```
   Look for:
   - "Receive Registration Request" messages
   - Authentication success/failure
   - "Send Registration Accept" confirmation

4. **Check AUSF logs for authentication:**
   ```bash
   docker compose logs -f oai-ausf
   ```

5. **Verify MySQL is accessible:**
   ```bash
   docker compose logs mysql
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
   Look for "Received NG SETUP REQUEST" and "Sending NG SETUP RESPONSE"

3. **Verify PLMN configuration:**
   - gNB PLMN must match AMF PLMN (001/01)
   - Check `./conf/gnb.sa.bandn78.fr1.106PRB.rfsim.conf`
   - Verify in `./conf/config.yaml` under AMF section

4. **Check gNB logs:**
   ```bash
   docker logs oai-gNB1
   ```
   Look for "Received NG Setup Response"

### No Internet Access from UE

1. **Check UE has tunnel interface:**
   ```bash
   docker exec <ue-container-name> ip addr show
   ```
   Should see `oaitun_ue1` with IP in `10.0.0.0/16` range

2. **Verify PDU Session established:**
   ```bash
   docker compose logs -f smf
   ```
   Look for "PDU Session Establishment Accept" messages

3. **Check UPF logs:**
   ```bash
   docker compose logs -f upf
   ```
   Look for:
   - PFCP association with SMF
   - Session creation messages
   - GTP-U tunnel establishment

4. **Verify external data network routing:**
   ```bash
   docker exec oai-ext-dn ip route | grep 10.0.0
   ```
   Should show route to `10.0.0.0/16` via UPF (192.168.70.134)

5. **Test connectivity from UE:**
   ```bash
   docker exec <ue-container-name> ping -I oaitun_ue1 192.168.70.135
   ```

### UPF Not Connecting to SMF

1. **Check UPF logs for PFCP association:**
   ```bash
   docker compose logs -f upf
   ```
   Look for "Received PFCP Association Setup Request"

2. **Verify SMF configuration:**
   Check `./conf/config.yaml` under `nfs.upf` section for correct IP and ports

3. **Check network connectivity:**
   ```bash
   docker exec oai-upf ping -c 3 192.168.70.133
   ```

### MySQL Connection Issues

1. **Check MySQL health:**
   ```bash
   docker compose ps mysql
   ```
   Should show "healthy" status

2. **Test database connection:**
   ```bash
   docker exec mysql mysql -u test -ptest oai_db -e "SELECT 1;"
   ```

3. **Check which NFs are connected:**
   ```bash
   docker compose logs mysql | grep "Connect"
   ```

4. **Restart MySQL and dependent services:**
   ```bash
   docker compose restart mysql
   sleep 10
   docker compose restart oai-udr oai-udm
   ```

### Services Not Registering with NRF

1. **Check NRF is running:**
   ```bash
   docker compose logs -f oai-nrf
   ```

2. **Verify NRF address in config:**
   Check `./conf/config.yaml` - ensure `register_nf.general` is set to `yes`

3. **Restart NRF and other services:**
   ```bash
   docker compose restart oai-nrf
   sleep 5
   docker compose restart oai-amf oai-smf oai-udm oai-udr oai-ausf oai-pcf oai-upf
   ```

## Advanced Configuration

### Modifying Network Slices

To add or modify network slices, edit `./conf/config.yaml`:

```yaml
amf:
  supported_slices:
    - sst: 1
      sd: 0xFFFFFF    # Default slice
    - sst: 2          # New slice for URLLC
      sd: 0x000002
```

Update the MySQL database with matching slice configurations for subscribers.

### Changing UE IP Pool

Edit `./conf/config.yaml` under the SMF/UPF section:

```yaml
upf:
  ue_ip_pools:
    - network: 10.0.0.0/16     # Change this
      ranges:
        - start: 10.0.0.2
          end: 10.0.255.254
```

Also update the external data network container:
```yaml
oai-traffic-server:
  entrypoint: /bin/bash -c \
    "ip route add 10.0.0.0/16 via 192.168.70.134 dev eth0; ip route; sleep infinity"
```

### Adjusting QoS Policies

Modify the policy files in `./policies/qos/`:
- `pcc_rules.yaml` - Define PCC rules (policy enforcement)
- `qos_data.yaml` - Set 5QI, bitrates, and packet delays
- `policy_decision.yaml` - Map policies to subscribers

Restart PCF after changes:
```bash
docker compose restart oai-pcf
```

### Enabling HTTP/2 for SBI

In `./conf/config.yaml`, change:
```yaml
http_version: 2
```

HTTP/2 provides better performance for service-based interfaces.

### Changing PLMN Configuration

Update all occurrences in `./conf/config.yaml`:

```yaml
# In AMF section
amf:
  guami:
    mcc: 208        # Change MCC
    mnc: 95         # Change MNC
  plmn_support_list:
    - mcc: 208
      mnc: 95
      tac: 0x0001
```

Also update:
- gNB configuration (`gnb.sa.bandn78.fr1.106PRB.rfsim.conf`)
- Subscriber IMSIs in the database (must start with new MCC/MNC)

## Integration with RoboSim5G

When using this CN with RoboSim5G:

1. **Start the core network first:**
   ```bash
   cd demo/oai_setup
   docker compose up -d
   ./wait_for_core_network.sh
   ```

2. **Configure subscribers** for your robots in the MySQL database

3. **Launch your Gazebo simulation** with RoboSim5G plugins:
   - The gNB plugin will automatically connect to the AMF at `192.168.70.132`
   - The UE plugin will register with credentials from the plugin configuration
   - Ensure plugin configuration matches subscriber data in MySQL

4. **Verify connectivity:**
   - Check AMF logs for successful UE registration
   - Verify PDU session establishment in SMF logs
   - Test data connectivity through the 5G network

### Multi-Robot Configuration

For multiple robots, add subscribers with sequential IMSIs:

| Robot | IMSI | K (same for all) | OPc (same for all) |
|-------|------|------------------|-------------------|
| Robot 1 | `001010000000101` | `fec86ba6eb707ed08905757b1bb44b8f` | `C42449363BBAD02B66D16BC975D77CC1` |
| Robot 2 | `001010000000102` | `fec86ba6eb707ed08905757b1bb44b8f` | `C42449363BBAD02B66D16BC975D77CC1` |
| Robot 3 | `001010000000103` | `fec86ba6eb707ed08905757b1bb44b8f` | `C42449363BBAD02B66D16BC975D77CC1` |

Each robot can use different network slices and QoS profiles for realistic 5G scenarios.

## Performance Considerations

### OAI Network Functions
- Written in C++ for high performance
- Multi-threaded architecture
- Optimized for low latency

### eBPF-based UPF
The OAI UPF uses Extended Berkeley Packet Filter (eBPF) for fast packet processing:
- Requires kernel >= 4.18 (5.4+ recommended)
- Needs `NET_ADMIN` and `SYS_ADMIN` capabilities
- Provides near line-rate throughput

### Resource Requirements
Minimum recommended resources for the full deployment:
- **CPU**: 4 cores
- **RAM**: 8 GB
- **Storage**: 10 GB
- **Network**: Bridge networking with kernel routing support


## Utility Scripts

### Wait for Core Network
```bash
./wait_for_core_network.sh
```
This script waits for all core network services to become healthy before proceeding.

### Create Physical Bridge
```bash
./create_physical_bridge.sh
```
Creates network bridges and sets up routing for the 5G network (if needed for host integration).

## Additional Resources

- [OAI GitLab Repository](https://gitlab.eurecom.fr/oai/cn5g)
- [OAI 5G Core Network Documentation](https://gitlab.eurecom.fr/oai/cn5g/oai-cn5g-fed/-/blob/master/docs/DEPLOY_HOME.md)
- [OAI Tutorials](https://gitlab.eurecom.fr/oai/cn5g/oai-cn5g-fed/-/tree/master/docs/TUTORIALS)
- [3GPP 5G Specifications](https://www.3gpp.org/specifications-technologies/specifications-by-series)
- [RoboSim5G Project](https://github.com/phinetech/RoboSim5G)

## Support

For issues specific to:
- **OAI Core Network**: Check [OAI GitLab issues](https://gitlab.eurecom.fr/oai/cn5g/oai-cn5g-fed/-/issues)
- **OAI Community**: Join the [OAI Slack workspace](https://openairinterface.slack.com/)
- **RoboSim5G integration**: Use the [RoboSim5G Slack chat](https://join.slack.com/t/robosimworkspace/shared_invite/zt-38i7sbsit-FpsT6d7PU241~nGz0fcUig)

