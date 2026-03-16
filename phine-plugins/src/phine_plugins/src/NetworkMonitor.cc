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

#include "NetworkMonitor.hh"
#include <QMetaObject>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>
#include <regex>
#include <sstream>
#include <tinyxml2.h>

// Register this plugin with Ignition GUI
IGNITION_ADD_PLUGIN(phine_plugins::NetworkMonitor, ignition::gui::Plugin)

using namespace phine_plugins;

NetworkMonitor::NetworkMonitor() {
    ignmsg << "[NetworkMonitor] constructor called" << std::endl;

    // Read UE container name from environment variable, fallback to default
    const char *ue_name_env = std::getenv("UE_NAME_FOR_BUTTON");
    if (ue_name_env != nullptr) {
	this->ue_container_name = std::string(ue_name_env);
    } else {
	this->ue_container_name = "ue_turtlebot";
    }

    ignmsg << "[NetworkMonitor] UE container: " << this->ue_container_name
	   << std::endl;
    ignmsg << "[NetworkMonitor] DN container: " << this->dn_container_name
	   << std::endl;
}

NetworkMonitor::~NetworkMonitor() {
    ignmsg << "[NetworkMonitor] destructor called" << std::endl;
    // Stop the test loop if still running
    if (this->is_running.load()) {
	this->is_running.store(false);
    }
    if (this->test_thread.joinable()) {
	this->test_thread.join();
    }
}

/**
 * @brief Loads plugin configuration from an XML element.
 *
 * Optionally reads default IP addresses and container names from the plugin
 * configuration in the SDF or GUI config file.
 *
 * @param _pluginElem Pointer to the XML element containing plugin config.
 */
void NetworkMonitor::LoadConfig(const tinyxml2::XMLElement *_pluginElem) {
    if (this->title.empty()) {
	this->title = "Network Monitor";
    }

    if (_pluginElem == nullptr) {
	return;
    }

    // Read optional default UE IP from config
    auto *ue_ip_elem = _pluginElem->FirstChildElement("ue_ip");
    if (ue_ip_elem != nullptr && ue_ip_elem->GetText() != nullptr) {
	this->ue_ip = QString::fromStdString(ue_ip_elem->GetText());
    }

    // Read optional default DN IP from config
    auto *dn_ip_elem = _pluginElem->FirstChildElement("dn_ip");
    if (dn_ip_elem != nullptr && dn_ip_elem->GetText() != nullptr) {
	this->dn_ip = QString::fromStdString(dn_ip_elem->GetText());
    }

    // Allow overriding container names via config
    auto *ue_container_elem =
	_pluginElem->FirstChildElement("ue_container_name");
    if (ue_container_elem != nullptr &&
	ue_container_elem->GetText() != nullptr) {
	this->ue_container_name = ue_container_elem->GetText();
    }

    auto *dn_container_elem =
	_pluginElem->FirstChildElement("dn_container_name");
    if (dn_container_elem != nullptr &&
	dn_container_elem->GetText() != nullptr) {
	this->dn_container_name = dn_container_elem->GetText();
    }
}

QString NetworkMonitor::getUeIp() const { return this->ue_ip; }

void NetworkMonitor::setUeIp(const QString &ip) {
    if (this->ue_ip != ip) {
	this->ue_ip = ip;
	emit ueIpChanged();
    }
}

QString NetworkMonitor::getDnIp() const { return this->dn_ip; }

void NetworkMonitor::setDnIp(const QString &ip) {
    if (this->dn_ip != ip) {
	this->dn_ip = ip;
	emit dnIpChanged();
    }
}

QString NetworkMonitor::getUplinkLatency() const {
    return this->uplink_latency;
}

QString NetworkMonitor::getDownlinkLatency() const {
    return this->downlink_latency;
}

QString NetworkMonitor::getUplinkBandwidth() const {
    return this->uplink_bandwidth;
}

QString NetworkMonitor::getDownlinkBandwidth() const {
    return this->downlink_bandwidth;
}

bool NetworkMonitor::isRunning() const { return this->is_running.load(); }

/**
 * @brief Executes a shell command and captures its stdout output.
 *
 * Uses popen() to run the command and reads the output into a string.
 *
 * @param cmd The command string to execute.
 * @return std::string The captured stdout output.
 */
std::string NetworkMonitor::executeCommand(const std::string &cmd) {
    std::string result;
    FILE *pipe = popen(cmd.c_str(), "r");
    if (pipe == nullptr) {
	ignerr << "[NetworkMonitor] Failed to run command: " << cmd
	       << std::endl;
	return result;
    }
    char buffer[256];
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
	result += buffer;
    }
    pclose(pipe);
    return result;
}

/**
 * @brief Parses ping output to extract the average round-trip time.
 *
 * Looks for the "rtt min/avg/max/mdev" summary line and extracts the
 * average value. Falls back to extracting a single "time=X ms" value
 * if the summary is not found.
 *
 * @param output The raw output from the ping command.
 * @return std::string The average latency (e.g. "12.34 ms"), or "N/A".
 */
std::string NetworkMonitor::parsePingOutput(const std::string &output) {
    // Match the rtt summary line: rtt min/avg/max/mdev = X/Y/Z/W ms
    std::regex rtt_regex(
	R"(rtt min/avg/max/mdev\s*=\s*[\d.]+/([\d.]+)/[\d.]+/[\d.]+\s*ms)");
    std::smatch match;
    if (std::regex_search(output, match, rtt_regex)) {
	return match[1].str() + " ms";
    }
    // Fallback: extract a single time= value from a reply line
    std::regex time_regex(R"(time=([\d.]+)\s*ms)");
    if (std::regex_search(output, match, time_regex)) {
	return match[1].str() + " ms";
    }
    return "N/A";
}

/**
 * @brief Parses iperf3 text output to extract the bandwidth measurement.
 *
 * Looks for the receiver summary line and extracts the bitrate value.
 * Falls back to the sender summary if the receiver line is not found.
 *
 * @param output The raw output from the iperf3 command.
 * @return std::string The bandwidth (e.g. "94.1 Mbits/sec"), or "N/A".
 */
std::string NetworkMonitor::parseIperfOutput(const std::string &output) {
    // Match receiver summary line bitrate
    std::regex receiver_regex(
	R"(([\d.]+)\s+(Mbits/sec|Kbits/sec|Gbits/sec)\s+.*receiver)");
    std::smatch match;
    if (std::regex_search(output, match, receiver_regex)) {
	return match[1].str() + " " + match[2].str();
    }
    // Fallback: match sender summary line bitrate
    std::regex sender_regex(
	R"(([\d.]+)\s+(Mbits/sec|Kbits/sec|Gbits/sec)\s+.*sender)");
    if (std::regex_search(output, match, sender_regex)) {
	return match[1].str() + " " + match[2].str();
    }
    return "N/A";
}

/**
 * @brief Starts the network performance test in a background thread.
 *
 * Launches an iperf3 server on the UE container, then continuously measures
 * uplink/downlink latency (via ping) and bandwidth (via iperf3) between
 * the UE and DN containers. Results are pushed to the GUI thread via
 * queued connections.
 *
 * Uplink = UE -> DN direction, Downlink = DN -> UE direction.
 * The UE acts as iperf3 server. The DN acts as iperf3 client.
 * - Default iperf3 (client sends to server): DN -> UE = Downlink
 * - Reverse iperf3 (-R flag): UE -> DN = Uplink
 */
void NetworkMonitor::startTest() {
    if (this->is_running.load()) {
	return;
    }

    if (this->ue_ip.isEmpty() || this->dn_ip.isEmpty()) {
	ignerr << "[NetworkMonitor] UE IP or DN IP is empty. Cannot start test."
	       << std::endl;
	return;
    }

    this->is_running.store(true);
    emit runningChanged();

    std::string ue_ip_str = this->ue_ip.toStdString();
    std::string dn_ip_str = this->dn_ip.toStdString();
    std::string ue_name = this->ue_container_name;
    std::string dn_name = this->dn_container_name;

    // Join any previous thread before launching a new one
    if (this->test_thread.joinable()) {
	this->test_thread.join();
    }

    this->test_thread = std::thread([this, ue_ip_str, dn_ip_str, ue_name,
				     dn_name]() {
	ignmsg << "[NetworkMonitor] Starting network performance test"
	       << std::endl;

	// Start iperf3 server on UE container in daemon mode
	std::string start_server =
	    "docker exec -d " + ue_name + " iperf3 -s 2>&1";
	executeCommand(start_server);

	// Allow iperf3 server to initialize
	std::this_thread::sleep_for(std::chrono::seconds(1));

	while (this->is_running.load()) {
	    // Downlink latency: ping from DN to UE (through 5G)
	    std::string ping_dl = "docker exec " + dn_name +
				  " ping -c 3 -W 2 " + ue_ip_str + " 2>&1";
	    std::string dl_lat = parsePingOutput(executeCommand(ping_dl));

	    if (!this->is_running.load()) {
		break;
	    }

	    // Uplink latency: ping from UE to DN
	    std::string ping_ul = "docker exec " + ue_name +
				  " ping -c 3 -W 2 " + dn_ip_str + " 2>&1";
	    std::string ul_lat = parsePingOutput(executeCommand(ping_ul));

	    if (!this->is_running.load()) {
		break;
	    }

	    // Downlink bandwidth: DN client sends to UE server
	    std::string iperf_dl = "docker exec " + dn_name + " iperf3 -c " +
				   ue_ip_str + " -t 2 -f m 2>&1";
	    std::string dl_bw = parseIperfOutput(executeCommand(iperf_dl));

	    if (!this->is_running.load()) {
		break;
	    }

	    // Uplink bandwidth: reverse mode (UE sends to DN via
	    // DN client with -R flag)
	    std::string iperf_ul = "docker exec " + dn_name + " iperf3 -c " +
				   ue_ip_str + " -t 2 -R -f m 2>&1";
	    std::string ul_bw = parseIperfOutput(executeCommand(iperf_ul));

	    // Update GUI properties from the main Qt thread
	    QMetaObject::invokeMethod(
		this,
		[this, dl_lat, ul_lat, dl_bw, ul_bw]() {
		    this->downlink_latency = QString::fromStdString(dl_lat);
		    this->uplink_latency = QString::fromStdString(ul_lat);
		    this->downlink_bandwidth = QString::fromStdString(dl_bw);
		    this->uplink_bandwidth = QString::fromStdString(ul_bw);
		    emit downlinkLatencyChanged();
		    emit uplinkLatencyChanged();
		    emit downlinkBandwidthChanged();
		    emit uplinkBandwidthChanged();
		},
		Qt::QueuedConnection);

	    // Wait before next measurement cycle
	    std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	// Stop iperf3 server on UE container
	std::string stop_server =
	    "docker exec " + ue_name + " pkill iperf3 2>&1";
	executeCommand(stop_server);

	ignmsg << "[NetworkMonitor] Network performance test stopped"
	       << std::endl;
    });
}

/**
 * @brief Stops the running network performance test.
 *
 * Sets the running flag to false and waits for the background thread
 * to complete. The iperf3 server on the UE container is cleaned up
 * by the thread before it exits.
 */
void NetworkMonitor::stopTest() {
    if (!this->is_running.load()) {
	return;
    }
    this->is_running.store(false);

    if (this->test_thread.joinable()) {
	this->test_thread.join();
    }

    emit runningChanged();
}
