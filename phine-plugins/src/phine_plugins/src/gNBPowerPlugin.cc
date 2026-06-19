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

#include "gNBPowerPlugin.hh"
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

IGNITION_ADD_PLUGIN(phine_plugins::gNBPowerPlugin, ignition::gui::Plugin)

using namespace phine_plugins;

// Helper function to ask Docker for data without freezing Gazebo
static void triggerDockerCheck(const std::string &name) {
    std::string file_path = "/tmp/gnb_" + name + "_pids.txt";
    std::string tmp_path = file_path + ".tmp";
    // Redirects Docker output to a temporary file, then instantly swaps it. The
    // '&' ensures 0 freezing.
    std::string cmd = "docker exec " + name +
		      " ps -eo pid,stat,args --no-headers 2>/dev/null > " +
		      tmp_path + " && mv " + tmp_path + " " + file_path + " &";
    system(cmd.c_str());
}

gNBPowerPlugin::gNBPowerPlugin() {
    ignmsg << "[gNBPowerPlugin] constructor called" << std::endl;

    status_timer = new QTimer(this);
    status_timer->setInterval(2000); // 2 seconds
    connect(status_timer, &QTimer::timeout, this,
	    &gNBPowerPlugin::checkAndUpdateStatus);
}

gNBPowerPlugin::~gNBPowerPlugin() {}

void gNBPowerPlugin::LoadConfig(const tinyxml2::XMLElement *_pluginElem) {
    if (this->title.empty()) {
	this->title = "gNB Power Control";
    }

    if (_pluginElem != nullptr) {
	auto *name_elem = _pluginElem->FirstChildElement("container_name");
	if (name_elem != nullptr && name_elem->GetText() != nullptr) {
	    container_name = QString::fromStdString(name_elem->GetText());
	}
    }
}

QString gNBPowerPlugin::getContainerName() const { return container_name; }
void gNBPowerPlugin::setContainerName(const QString &name) {
    if (container_name != name) {
	container_name = name;
	emit containerNameChanged();
    }
}

bool gNBPowerPlugin::isProcessRunning() const { return process_running; }
bool gNBPowerPlugin::isConnected() const { return connected; }

bool gNBPowerPlugin::isValidContainerName(const std::string &name) {
    if (name.empty() || name.size() > 128)
	return false;
    static const std::regex valid_name(R"(^[a-zA-Z0-9][a-zA-Z0-9_.\-]*$)");
    return std::regex_match(name, valid_name);
}

std::vector<int> gNBPowerPlugin::getProcessPids() const {
    std::vector<int> pids;
    std::string name = container_name.toStdString();

    if (!isValidContainerName(name))
	return pids;

    // Reads the file locally. This is instantaneous and immune to Docker
    // lock-ups.
    std::string file_path = "/tmp/gnb_" + name + "_pids.txt";
    std::ifstream file(file_path);
    if (file.is_open()) {
	std::string line;
	while (std::getline(file, line)) {
	    std::istringstream iss(line);
	    std::string pid_str, stat;

	    if (iss >> pid_str >> stat) {
		try {
		    int pid = std::stoi(pid_str);
		    // Search specifically for nr-softmodem
		    if (line.find("nr-softmodem") != std::string::npos &&
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

std::string gNBPowerPlugin::buildStartCommand() const {
    std::string name = container_name.toStdString();

    return "docker exec " + name +
	   " /bin/bash -c"
	   " '/opt/oai-gnb/bin/nr-softmodem"
	   " -O /opt/oai-gnb/etc/gnb.yaml"
	   " -E --rfsim"
	   " --log_config.global_log_options level,nocolor,time > /dev/null "
	   "2>&1 &'";
}

void gNBPowerPlugin::checkAndUpdateStatus() {
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
	ignmsg << "[gNBPowerPlugin] Polling: PROCESS FOUND ("
	       << live_pids.size() << " PIDs)" << std::endl;

	if (!process_running) {
	    process_running = true;
	    emit processRunningChanged();
	}
    } else {
	failed_checks++;
	ignmsg << "[gNBPowerPlugin] Polling: PROCESS NOT FOUND (Strike "
	       << failed_checks << "/2)" << std::endl;

	if (failed_checks >= 2 && process_running) {
	    process_running = false;
	    emit processRunningChanged();
	}
    }

    // Ping Docker to gather data for the *next* cycle
    triggerDockerCheck(name);
}

void gNBPowerPlugin::toggleProcess() {
    std::string name = container_name.toStdString();
    if (!isValidContainerName(name)) {
	ignerr << "[gNBPowerPlugin] Invalid container name." << std::endl;
	return;
    }

    // --- 1. FIRST CLICK ---
    if (!connected) {
	ignmsg << "[gNBPowerPlugin] === INITIALIZING CONNECTION ==="
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
	ignmsg << "[gNBPowerPlugin] TURN OFF Clicked. Executing KILL..."
	       << std::endl;
	std::vector<int> current_live_pids = getProcessPids();
	for (int pid : current_live_pids) {
	    std::string cmd = "docker exec " + name + " kill -9 " +
			      std::to_string(pid) + " > /dev/null 2>&1 &";
	    system(cmd.c_str());
	}

	// Immediately delete the text file so the timer doesn't read ghost
	// processes
	system(("rm -f /tmp/gnb_" + name + "_pids.txt").c_str());

	process_running = false;
	emit processRunningChanged();
    } else {
	ignmsg << "[gNBPowerPlugin] TURN ON Clicked. Executing START..."
	       << std::endl;

	// Ensure the host doesn't freeze waiting for Docker to parse the
	// command
	std::string cmd = buildStartCommand() + " > /dev/null 2>&1 &";
	system(cmd.c_str());

	process_running = true;
	emit processRunningChanged();
    }
}