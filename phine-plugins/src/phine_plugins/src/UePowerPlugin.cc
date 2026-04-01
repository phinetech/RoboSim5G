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
#include <cstdlib>
#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>
#include <regex>
#include <tinyxml2.h>

IGNITION_ADD_PLUGIN(phine_plugins::UePowerPlugin, ignition::gui::Plugin)

using namespace phine_plugins;

UePowerPlugin::UePowerPlugin() {
	ignmsg << "[UePowerPlugin] constructor called" << std::endl;

	const char *ue_name_env = std::getenv("UE_NAME_FOR_BUTTON");
	if (ue_name_env != nullptr) {
		this->container_name = QString::fromStdString(std::string(ue_name_env));
	}

	ignmsg << "[UePowerPlugin] UE container: "
	       << this->container_name.toStdString() << std::endl;
}

UePowerPlugin::~UePowerPlugin() {}

void UePowerPlugin::LoadConfig(const tinyxml2::XMLElement *_pluginElem) {
	if (this->title.empty()) {
		this->title = "UE Power Control";
	}

	if (_pluginElem != nullptr) {
		auto *name_elem = _pluginElem->FirstChildElement("container_name");
		if (name_elem != nullptr && name_elem->GetText() != nullptr) {
			container_name = QString::fromStdString(name_elem->GetText());
		}

		auto *ver_elem = _pluginElem->FirstChildElement("version");
		if (ver_elem != nullptr && ver_elem->GetText() != nullptr) {
			std::string ver = ver_elem->GetText();
			if (ver == "v24") {
				version_index = 0;
			} else if (ver == "v26") {
				version_index = 1;
			}
		}

		auto *ip_elem = _pluginElem->FirstChildElement("gnb_ip");
		if (ip_elem != nullptr && ip_elem->GetText() != nullptr) {
			gnb_ip = QString::fromStdString(ip_elem->GetText());
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

int UePowerPlugin::getVersionIndex() const { return version_index; }

void UePowerPlugin::setVersionIndex(int index) {
	if (version_index != index) {
		version_index = index;
		emit versionIndexChanged();
	}
}

QString UePowerPlugin::getGnbIp() const { return gnb_ip; }

void UePowerPlugin::setGnbIp(const QString &ip) {
	if (gnb_ip != ip) {
		gnb_ip = ip;
		emit gnbIpChanged();
	}
}

bool UePowerPlugin::isProcessRunning() const { return process_running; }

bool UePowerPlugin::isConnected() const { return connected; }

bool UePowerPlugin::isValidContainerName(const std::string &name) {
	if (name.empty() || name.size() > 128) {
		return false;
	}
	static const std::regex valid_name(R"(^[a-zA-Z0-9][a-zA-Z0-9_.\-]*$)");
	return std::regex_match(name, valid_name);
}

bool UePowerPlugin::isValidIpAddress(const std::string &ip) {
	static const std::regex ip_regex(
	    R"(^(\d{1,3})\.(\d{1,3})\.(\d{1,3})\.(\d{1,3})$)");
	std::smatch match;
	if (!std::regex_match(ip, match, ip_regex)) {
		return false;
	}
	for (int i = 1; i <= 4; ++i) {
		int octet = std::stoi(match[i].str());
		if (octet < 0 || octet > 255) {
			return false;
		}
	}
	return true;
}

bool UePowerPlugin::checkProcessRunning() {
	std::string name = container_name.toStdString();
	if (!isValidContainerName(name)) {
		return false;
	}
	std::string cmd =
	    "docker exec " + name + " pgrep nr-uesoftmodem > /dev/null 2>&1";
	int ret = system(cmd.c_str());
	return (ret == 0);
}

std::string UePowerPlugin::buildStartCommand() const {
	std::string name = container_name.toStdString();
	std::string ip = gnb_ip.toStdString();

	if (version_index == 0) {
		// v24: uses .conf file with --sa flag, freq 3619200000
		return "docker exec " + name +
		       " /bin/bash -c"
		       " '/opt/oai-nr-ue/bin/nr-uesoftmodem"
		       " -O /opt/oai-nr-ue/etc/nr-ue.conf"
		       " -E --sa --rfsim -r 106 --numerology 1"
		       " -C 3619200000"
		       " --rfsimulator.serveraddr " +
		       ip +
		       " --log_config.global_log_options level,nocolor,time &'";
	}
	// v26: uses .yaml file without --sa flag, freq 3319680000
	return "docker exec " + name +
	       " /bin/bash -c"
	       " '/opt/oai-nr-ue/bin/nr-uesoftmodem"
	       " -O /opt/oai-nr-ue/etc/nr-ue.yaml"
	       " -E --rfsim -r 106 --numerology 1"
	       " -C 3319680000"
	       " --rfsimulator.serveraddr " +
	       ip + " &'";
}

void UePowerPlugin::toggleProcess() {
	std::string name = container_name.toStdString();
	if (!isValidContainerName(name)) {
		ignerr << "[UePowerPlugin] Invalid container name: " << name
		       << std::endl;
		return;
	}

	// First click: check actual state, then become connected
	if (!connected) {
		process_running = checkProcessRunning();
		connected = true;
		emit connectedChanged();
		emit processRunningChanged();
		ignmsg << "[UePowerPlugin] Connected to " << name << " — process is "
		       << (process_running ? "running" : "stopped") << std::endl;
		return;
	}

	// Subsequent clicks: toggle without checking
	if (process_running) {
		std::string cmd =
		    "docker exec " + name + " pkill nr-uesoftmodem 2>&1";
		ignmsg << "[UePowerPlugin] Running: " << cmd << std::endl;
		int ret = system(cmd.c_str());
		ignmsg << "[UePowerPlugin] Kill exit=" << ret << std::endl;
		process_running = false;
		emit processRunningChanged();
	} else {
		std::string ip = gnb_ip.toStdString();
		if (!isValidIpAddress(ip)) {
			ignerr << "[UePowerPlugin] Invalid gNB IP address: " << ip
			       << std::endl;
			return;
		}
		std::string cmd = buildStartCommand();
		ignmsg << "[UePowerPlugin] Running: " << cmd << std::endl;
		int ret = system(cmd.c_str());
		ignmsg << "[UePowerPlugin] Start exit=" << ret << std::endl;
		process_running = true;
		emit processRunningChanged();
	}
}
