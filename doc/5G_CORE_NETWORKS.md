# 5G Core Network Options for RoboSim5G

RoboSim5G supports **three different 5G Core Network implementations**, allowing you to choose the one that best fits your needs. Each core network has its own strengths and is suitable for different use cases.

## Available Core Networks

### 1. **OAI (OpenAirInterface)**
Research-focused, high-performance 5G core network developed by EURECOM.

**Best for:**
- Performance-critical applications
- Research and development
- Advanced 5G features testing

📁 **README:** `demo/oai_setup/README.md`

---

### 2. **free5GC**
User-friendly 5G core network with a comprehensive web-based management interface.

**Best for:**
- Ease of use and quick setup
- Subscriber management via GUI
- Educational purposes
- Rapid prototyping

📁 **README:** `demo/free5gc_setup/README.md`

---

### 3. **Open5GS** *(Coming Soon)*
Mature open-source implementation with excellent documentation and active community support.

**Best for:**
- Production-like deployments
- Long-term stability
- Well-documented configurations

📁 **README:** `demo/open5gs_setup/README.md` *(to be added)*

---

## Feature Comparison

| Feature | OAI | free5GC | Open5GS |
|---------|-----|---------|---------|
| **Implementation Language** | C++ | Go | C |
| **Web UI** | ❌ No | ✅ Yes (comprehensive) | ✅ Yes (user-friendly) |
| **Subscriber Management** | MySQL (SQL commands) | WebUI + MongoDB | WebUI + MongoDB |
| **Database** | MySQL | MongoDB | MongoDB |
| **Configuration** | Single unified YAML | Per-NF YAML files | Per-NF YAML files |
| **Default PLMN** | 001/01 | 208/93 | 001/01 |
| **IMSI Format** | `001010000000101` | `imsi-208930000000001` | `001010000000101` |
| **WebUI Port** | N/A | 5000 | 9999 |
| **User Plane (UPF)** | OAI UPF (eBPF) | OAI UPF (eBPF, hybrid) | Open5GS UPF |
| **Performance** | ⚡ Excellent (C++) | ✅ Good (Go) | ✅ Good (C) |
| **HTTP Version** | 1 or 2 (configurable) | 1 | 1 |
| **Development Focus** | Research (EURECOM) | Community-driven | Community-driven |
| **Documentation** | Technical, research-oriented | Good | Excellent |
| **Community** | Academic/research | Growing | Large & active |

---

## Quick Comparison: Which Should I Choose?

### Choose **OAI** if you need:
- Maximum performance
- Advanced 5G features (network slicing, QoS policies)
- Research-grade implementation
- Fine-grained control over network functions
- Experience with 3GPP specifications

### Choose **free5GC** if you want:
- Easy subscriber management via web interface
- Quick setup and deployment
- Hybrid approach (free5GC control plane + OAI UPF)
- Good balance between features and usability
- Educational purposes

### Choose **Open5GS** if you prefer:
- User-friendly WebUI with persistent storage
- Extensive documentation and tutorials
- Active community support
- Production-ready stability
- Mature codebase

---

## Network Configuration Differences

### IP Address Pools for UEs

| Core Network | UE IP Pool | Configured In |
|--------------|------------|---------------|
| **OAI** | `10.0.0.0/16` | `conf/config.yaml` |
| **free5GC** | `10.0.0.0/24` | `free5gc/config/smfcfg.yaml` & `oai/oai_upf_config.yaml` |
| **Open5GS** | `10.45.0.0/24` | `configs/upf.yaml` |

### Network Names (DNN)

| Core Network | Default DNN | Alternative DNNs |
|--------------|-------------|------------------|
| **OAI** | `oai` | `ims` |
| **free5GC** | `internet` | - |
| **Open5GS** | `internet` | - |

### Network Slicing (S-NSSAI)

All three core networks support network slicing, but with different default configurations:

- **OAI**: SST=1, SD=FFFFFF (no slice differentiator)
- **free5GC**: SST=1, SD=010203 (hex: 66051)
- **Open5GS**: SST=1, SD=1 or SST=1, SD=000001

---

## Architecture Notes

### Hybrid Deployments

**free5GC setup uses a hybrid approach:**
- **Control Plane**: free5GC network functions (AMF, SMF, etc.)
- **User Plane**: OAI UPF with eBPF data plane

This combines free5GC's user-friendly management with OAI's high-performance user plane.

### Network Topology

All deployments use Docker networks:
- **public_net** (`phine-net`): Control plane communication (192.168.70.128/26)
- **n3_net** (OAI/free5GC only): gNB ↔ UPF GTP-U tunnel
- **n6_net** (free5GC only): UPF ↔ Data network

---

## Getting Started

### 1. Choose Your Core Network

Review the comparison above and select the core network that fits your requirements.

### 2. Read the Detailed README

Navigate to the corresponding setup folder and read the README:

```bash
# For OAI
cat demo/oai_setup/README.md

# For free5GC
cat demo/free5gc_setup/README.md

# For Open5GS (coming soon)
cat demo/open5gs_setup/README.md
```

### 3. Start the Core Network

Follow the instructions in the respective README to:
1. Start the core network
2. Configure subscribers
3. Launch gNB and UE containers
4. Verify connectivity

---

## Switching Between Core Networks

You can switch between different core networks by:

1. **Stop the current core network:**
   ```bash
   cd demo/<current_setup>
   docker compose down
   ```

2. **Start the new core network:**
   ```bash
   cd demo/<new_setup>
   docker compose up -d
   ```

3. **Update subscriber credentials** in the UE configuration to match the new core network's format (IMSI, DNN, PLMN, etc.)

4. **Restart gNB and UE containers** with updated configurations

---

## Additional Resources

- **OAI Documentation**: [GitLab](https://gitlab.eurecom.fr/oai/cn5g)
- **free5GC Documentation**: [free5gc.org](https://free5gc.org/guide/)
- **Open5GS Documentation**: [open5gs.org](https://open5gs.org/open5gs/docs/)
- **RoboSim5G Project**: [GitHub](https://github.com/phinetech/RoboSim5G)
- **RoboSim5G Support**: [Slack Chat](https://join.slack.com/t/robosimworkspace/shared_invite/zt-38i7sbsit-FpsT6d7PU241~nGz0fcUig)

---

## Contributing

If you have experience with these core networks or encounter issues, please contribute to improving the documentation:
- See [CONTRIBUTING.md](../CONTRIBUTING.md)
- Join our [Slack community](https://join.slack.com/t/robosimworkspace/shared_invite/zt-38i7sbsit-FpsT6d7PU241~nGz0fcUig)

