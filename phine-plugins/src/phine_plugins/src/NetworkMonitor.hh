/* Copyright 2025 phine.tech GmbH

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.*/

#ifndef NETWORK_MONITOR_HH_
#define NETWORK_MONITOR_HH_

#include <QString>
#include <atomic>
#include <ignition/gui/Plugin.hh>
#include <string>
#include <thread>

/**
 * @namespace phine_plugins
 * @brief Contains plugins for the RoboSim5G simulation environment.
 */
namespace phine_plugins {

/**
 * @class NetworkMonitor
 * @brief Ignition Gazebo GUI plugin for monitoring 5G network performance
 * between a User Equipment (UE) container and a Data Network (DN) container.
 *
 * This plugin provides real-time uplink/downlink latency and bandwidth
 * measurements using ping and iperf3 commands executed via docker exec
 * on the respective containers.
 */
class NetworkMonitor : public ignition::gui::Plugin {
    Q_OBJECT

    Q_PROPERTY(QString ueIp READ getUeIp WRITE setUeIp NOTIFY ueIpChanged)
    Q_PROPERTY(QString dnIp READ getDnIp WRITE setDnIp NOTIFY dnIpChanged)
    Q_PROPERTY(
	QString uplinkLatency READ getUplinkLatency NOTIFY uplinkLatencyChanged)
    Q_PROPERTY(QString downlinkLatency READ getDownlinkLatency NOTIFY
		   downlinkLatencyChanged)
    Q_PROPERTY(QString uplinkBandwidth READ getUplinkBandwidth NOTIFY
		   uplinkBandwidthChanged)
    Q_PROPERTY(QString downlinkBandwidth READ getDownlinkBandwidth NOTIFY
		   downlinkBandwidthChanged)
    Q_PROPERTY(bool running READ isRunning NOTIFY runningChanged)

  public:
    /// \brief Constructor
    NetworkMonitor();

    /// \brief Destructor
    ~NetworkMonitor() override;

    /// \brief Load plugin configuration from XML element
    void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    /// \brief Get UE IP address
    /// \return Current UE IP as QString
    QString getUeIp() const;

    /// \brief Set UE IP address
    /// \param ip The IP address to set
    void setUeIp(const QString &ip);

    /// \brief Get DN IP address
    /// \return Current DN IP as QString
    QString getDnIp() const;

    /// \brief Set DN IP address
    /// \param ip The IP address to set
    void setDnIp(const QString &ip);

    /// \brief Get uplink latency result
    /// \return Uplink latency string (e.g. "12.34 ms")
    QString getUplinkLatency() const;

    /// \brief Get downlink latency result
    /// \return Downlink latency string (e.g. "15.67 ms")
    QString getDownlinkLatency() const;

    /// \brief Get uplink bandwidth result
    /// \return Uplink bandwidth string (e.g. "94.1 Mbits/sec")
    QString getUplinkBandwidth() const;

    /// \brief Get downlink bandwidth result
    /// \return Downlink bandwidth string (e.g. "98.2 Mbits/sec")
    QString getDownlinkBandwidth() const;

    /// \brief Check if test is currently running
    /// \return true if test loop is active
    bool isRunning() const;

  public slots:
    /// \brief Start the network performance test loop
    void startTest();

    /// \brief Stop the network performance test loop
    void stopTest();

  signals:
    void ueIpChanged();
    void dnIpChanged();
    void uplinkLatencyChanged();
    void downlinkLatencyChanged();
    void uplinkBandwidthChanged();
    void downlinkBandwidthChanged();
    void runningChanged();

  private:
    /// \brief Execute a shell command and return its stdout output
    /// \param cmd The shell command to execute
    /// \return Captured stdout output
    std::string executeCommand(const std::string &cmd);

    /// \brief Parse ping output to extract average round-trip time
    /// \param output Raw ping command output
    /// \return Parsed latency string or "N/A"
    std::string parsePingOutput(const std::string &output);

    /// \brief Parse iperf3 output to extract bandwidth measurement
    /// \param output Raw iperf3 command output
    /// \return Parsed bandwidth string or "N/A"
    std::string parseIperfOutput(const std::string &output);

    /// \brief UE IP address (5G tunnel IP, e.g. 10.0.0.1)
    QString ue_ip;

    /// \brief DN container IP address (e.g. 192.168.70.135)
    QString dn_ip;

    /// \brief Uplink latency measurement result
    QString uplink_latency{"N/A"};

    /// \brief Downlink latency measurement result
    QString downlink_latency{"N/A"};

    /// \brief Uplink bandwidth measurement result
    QString uplink_bandwidth{"N/A"};

    /// \brief Downlink bandwidth measurement result
    QString downlink_bandwidth{"N/A"};

    /// \brief Flag indicating whether the test loop is running
    std::atomic<bool> is_running{false};

    /// \brief Background thread for running measurement loop
    std::thread test_thread;

    /// \brief UE container name (read from UE_NAME_FOR_BUTTON env var)
    std::string ue_container_name;

    /// \brief DN container name (default: oai-ext-dn)
    std::string dn_container_name{"oai-ext-dn"};
};

} // namespace phine_plugins

#endif // NETWORK_MONITOR_HH_
