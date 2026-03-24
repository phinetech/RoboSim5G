# Open5GS Core Network Deployment for RoboSim5G

This deployment provides an Open5GS 5G Core Network that integrates with RoboSim5G.

## About Open5GS

Open5GS is an open-source implementation of 5G Core and EPC (Evolved Packet Core). Key features:
- **User-friendly WebUI** for subscriber management
- **Modular architecture** with independent network functions
- **Persistent storage** using MongoDB for subscriber data
- **Active community** and good documentation

## Quick Start

### 1. Start the Core Network
```bash
docker compose up -d
```

Wait for all services to become healthy:
```bash
docker compose ps
```

### 2. Configure Subscribers via WebUI

Open http://localhost:9999 in your browser.

**Default credentials:**
- Username: `admin`
- Password: `1423`

**Add a subscriber:**
1. Navigate to "Subscribers" → "New Subscriber"
2. Enter the following details:
   - **IMSI**: `001010000000101` (must match UE configuration)
   - **Security:**
     - K: `8baf473f2f8fd09487cccbd7097c6862`
     - OPc: `8e27b6af0e692e750f32667a3b14605d`
     - AMF: `8000`
   - **S-NSSAI (Network Slice):**
     - SST: `1`
     - SD: `1` (or `000001`)
   - **DNN/APN**: `internet`
3. Click "Save"

> **Note:** First user (001010000000101) is already configured. You can pre-configure multiple additional subscribers for different robots by incrementing the IMSI (e.g., `001010000000102`, `001010000000103`).

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

All services should show "healthy" or "running" status.

### View Logs
```bash
# View all services
docker compose logs -f

# View specific service
docker compose logs -f amf
docker compose logs -f upf
```

### Stop the Core Network
```bash
docker compose down
```

> **Important:** Do NOT use `docker compose down -v` as this will delete subscriber data.

### Restart Services
```bash
docker compose restart
```

## Subscriber Data Persistence

Subscriber data is stored in the local `./db_data` directory and persists across container restarts. This makes the deployment portable:
- Copy the entire folder to another machine
- Subscriber configurations remain intact
- No need to reconfigure subscribers after restart

## Network Configuration

The core network uses the following subnet:
- **Open5GS CN**: `10.33.33.0/24`
- **UE Data Network (DN)**: `10.45.0.0/24` (assigned via UPF)

### Key Service Endpoints
- **AMF (N2 interface)**: `10.33.33.5:38412` (SCTP)
- **UPF (N3 interface)**: `10.33.33.10:2152` (GTP-U)
- **WebUI**: `http://localhost:9999`

## Troubleshooting

### UE Not Attaching
1. **Check subscriber exists:** Log into WebUI and verify IMSI is configured
2. **Verify credentials match:** K, OPc values in WebUI must match UE config
3. **Check AMF logs:**
   ```bash
   docker compose logs -f amf
   ```
   Look for authentication failures or registration issues

### gNB Not Connecting to AMF
1. **Check network connectivity:**
   ```bash
   docker exec amf ping -c 3 oai-gNB
   ```
2. **Verify AMF logs:**
   ```bash
   docker compose logs -f amf
   ```
   Look for NG Setup Request/Response messages

### No Internet Access from UE
1. **Check UPF logs:**
   ```bash
   docker compose logs -f upf
   ```
2. **Verify UE has tunnel interface:**
   ```bash
   docker exec oai-nr-ue1 ip addr show
   ```
   Should see `oaitun_ue1` with IP in `10.45.0.0/24` range

### WebUI Connection Issues
If WebUI cannot connect to MongoDB:
```bash
docker compose restart db webui
```

## Configuration Files

All Open5GS network function configs are in `./configs/`:
- `amf.yaml` - Access and Mobility Management Function
- `smf.yaml` - Session Management Function  
- `upf.yaml` - User Plane Function
- `ausf.yaml`, `udm.yaml`, `udr.yaml` - Authentication functions
- `nrf.yaml` - Network Repository Function
- `pcf.yaml` - Policy Control Function

## Integration with RoboSim5G

When using this CN with RoboSim5G:
1. Start this core network first
2. Configure subscribers for your robots via WebUI
3. Launch your Gazebo simulation with the RoboSim5G plugins
4. The robot's OAI UE will automatically attach to the network

Each robot in your simulation can have its own IMSI and network slice configuration, enabling realistic multi-robot 5G scenarios.

## Additional Resources

- [Open5GS Documentation](https://open5gs.org/open5gs/docs/)
- [Open5GS GitHub](https://github.com/open5gs/open5gs)
- [RoboSim5G Project](https://github.com/phinetech/RoboSim5G)
