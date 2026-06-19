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

#include "UePowerPlugin.hh"
#include <QTimer>
#include <cstdio>
#include <cstdlib>
#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>
#include <regex>
#include <sstream>
#include <string>
#include <tinyxml2.h>
#include <vector>

IGNITION_ADD_PLUGIN(phine_plugins::UePowerPlugin, ignition::gui::Plugin)

using namespace phine_plugins;

// Helper function to ask Docker for data without freezing Gazebo
static void triggerDockerCheck(const std::string &name) {
    std::string file_path = "/tmp/ue_" + name + "_pids.txt";
    std::string tmp_path = file_path + ".tmp";
    // Redirects Docker output to a temporary file, then instantly swaps it. The
    // '&' ensures 0 freezing.
    std::string cmd = "docker exec " + name +
		      " ps -eo pid,stat,args --no-headers 2>/dev/null > " +
		      tmp_path + " && mv " + tmp_path + " " + file_path + " &";
    system(cmd.c_str());
}

UePowerPlugin::UePowerPlugin() {
    const char *ue_name_env = std::getenv("UE_NAME_FOR_BUTTON");
    if (ue_name_env != nullptr) {
	this->container_name = QString::fromStdString(std::string(ue_name_env));
    }

    status_timer = new QTimer(this);
    status_timer->setInterval(2000);
    connect(status_timer, &QTimer::timeout, this,
	    &UePowerPlugin::checkAndUpdateStatus);
}

UePowerPlugin::~UePowerPlugin() {}

void UePowerPlugin::LoadConfig(const tinyxml2::XMLElement *_pluginElem) {
    if (this->title.empty())
	this->title = "UE Power Control";

    if (_pluginElem != nullptr) {
	auto *name_elem = _pluginElem->FirstChildElement("container_name");
	if (name_elem != nullptr && name_elem->GetText() != nullptr) {
	    container_name = QString::fromStdString(name_elem->GetText());
	}

	auto *ip_elem = _pluginElem->FirstChildElement("gnb_ip");
	if (ip_elem != nullptr && ip_elem->GetText() != nullptr) {
	    gnb_ip = QString::fromStdString(ip_elem->GetText());
	}

	auto *freq_elem = _pluginElem->FirstChildElement("carrier_freq");
	if (freq_elem != nullptr && freq_elem->GetText() != nullptr) {
	    carrier_freq = QString::fromStdString(freq_elem->GetText());
	}
    }
}

QString UePowerPlugin::getContainerName() const { return container_name; }
void UePowerPlugin::setContainerName(const QString &name) {
    if (container_name != name) {
	container_name = name;
	emit containerNameChanged();
    }
}

QString UePowerPlugin::getGnbIp() const { return gnb_ip; }
void UePowerPlugin::setGnbIp(const QString &ip) {
    if (gnb_ip != ip) {
	gnb_ip = ip;
	emit gnbIpChanged();
    }
}

QString UePowerPlugin::getCarrierFreq() const { return carrier_freq; }
void UePowerPlugin::setCarrierFreq(const QString &freq) {
    if (carrier_freq != freq) {
	carrier_freq = freq;
	emit carrierFreqChanged();
    }
}

bool UePowerPlugin::isProcessRunning() const { return process_running; }
bool UePowerPlugin::isConnected() const { return connected; }

bool UePowerPlugin::isValidContainerName(const std::string &name) {
    if (name.empty() || name.size() > 128)
	return false;
    static const std::regex valid_name(R"(^[a-zA-Z0-9][a-zA-Z0-9_.\-]*$)");
    return std::regex_match(name, valid_name);
}

bool UePowerPlugin::isValidIpAddress(const std::string &ip) {
    static const std::regex ip_regex(
	R"(^(\d{1,3})\.(\d{1,3})\.(\d{1,3})\.(\d{1,3})$)");
    std::smatch match;
    if (!std::regex_match(ip, match, ip_regex))
	return false;
    for (int i = 1; i <= 4; ++i) {
	int octet = std::stoi(match[i].str());
	if (octet < 0 || octet > 255)
	    return false;
    }
    return true;
}

bool UePowerPlugin::isValidFrequency(const std::string &freq) {
    if (freq.empty() || freq.size() < 8 || freq.size() > 12)
	return false;
    for (char c : freq) {
	if (!std::isdigit(c))
	    return false;
    }
    return true;
}

std::vector<int> UePowerPlugin::getProcessPids() const {
    std::vector<int> pids;
    std::string name = container_name.toStdString();

    if (!isValidContainerName(name))
	return pids;

    // Reads the file locally. This is instantaneous and immune to Docker
    // lock-ups.
    std::string file_path = "/tmp/ue_" + name + "_pids.txt";
    std::ifstream file(file_path);
    if (file.is_open()) {
	std::string line;
	while (std::getline(file, line)) {
	    std::istringstream iss(line);
	    std::string pid_str, stat;

	    if (iss >> pid_str >> stat) {
		try {
		    int pid = std::stoi(pid_str);
		    if (line.find("nr-uesoftmodem") != std::string::npos &&
			stat.front() != 'Z') {
			pids.push_back(pid);
		    }
		} catch (...) {
		}
	    }
	}
    }
    return pids;
}

std::string UePowerPlugin::buildStartCommand() const {
    std::string name = container_name.toStdString();
    std::string ip = gnb_ip.toStdString();
    std::string freq = carrier_freq.toStdString();

    return "docker exec " + name +
	   " /bin/bash -c"
	   " '/opt/oai-nr-ue/bin/nr-uesoftmodem"
	   " -O /opt/oai-nr-ue/etc/nr-ue.yaml"
	   " -E --rfsim -r 106 --numerology 1"
	   " -C " +
	   freq + " --rfsimulator.serveraddr " + ip + " > /dev/null 2>&1 &'";
}

void UePowerPlugin::checkAndUpdateStatus() {
    if (!connected)
	return;

    std::string name = container_name.toStdString();

    if (skip_checks_count > 0) {
	skip_checks_count--;
	triggerDockerCheck(name); // Keep background updates going
	return;
    }

    std::vector<int> live_pids = getProcessPids();
    bool is_actually_running = !live_pids.empty();

    if (is_actually_running) {
	failed_checks = 0;
	ignmsg << "[UePowerPlugin] Polling: PROCESS FOUND (" << live_pids.size()
	       << " PIDs)" << std::endl;

	if (!process_running) {
	    process_running = true;
	    emit processRunningChanged();
	}
    } else {
	failed_checks++;
	ignmsg << "[UePowerPlugin] Polling: PROCESS NOT FOUND (Strike "
	       << failed_checks << "/2)" << std::endl;

	if (failed_checks >= 2 && process_running) {
	    process_running = false;
	    emit processRunningChanged();
	}
    }

    // Ping Docker to gather data for the *next* cycle
    triggerDockerCheck(name);
}

void UePowerPlugin::toggleProcess() {
    std::string name = container_name.toStdString();
    if (!isValidContainerName(name)) {
	ignerr << "[UePowerPlugin] Invalid container name." << std::endl;
	return;
    }

    // --- 1. FIRST CLICK ---
    if (!connected) {
	ignmsg << "[UePowerPlugin] === INITIALIZING CONNECTION ==="
	       << std::endl;
	connected = true;

	triggerDockerCheck(
	    name); // Fire the very first check to build the text file
	status_timer->start();
	emit connectedChanged();
	return;
    }

    // --- 2. SUBSEQUENT CLICKS ---
    skip_checks_count =
	3; // Block UI from flickering for 6 seconds while processes boot/die

    if (process_running) {
	ignmsg << "[UePowerPlugin] TURN OFF Clicked. Executing KILL..."
	       << std::endl;
	std::vector<int> current_live_pids = getProcessPids();
	for (int pid : current_live_pids) {
	    std::string cmd = "docker exec " + name + " kill -9 " +
			      std::to_string(pid) + " > /dev/null 2>&1 &";
	    system(cmd.c_str());
	}

	// Immediately delete the text file so the timer doesn't read ghost
	// processes
	system(("rm -f /tmp/ue_" + name + "_pids.txt").c_str());

	process_running = false;
	emit processRunningChanged();
    } else {
	ignmsg << "[UePowerPlugin] TURN ON Clicked. Executing START..."
	       << std::endl;
	std::string ip = gnb_ip.toStdString();
	std::string freq = carrier_freq.toStdString();
	if (!isValidIpAddress(ip) || !isValidFrequency(freq))
	    return;

	// Ensure the host doesn't freeze waiting for Docker to parse the
	// command
	std::string cmd = buildStartCommand() + " > /dev/null 2>&1 &";
	system(cmd.c_str());

	process_running = true;
	emit processRunningChanged();
    }
}