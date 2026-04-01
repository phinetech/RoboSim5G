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
#include <cstdlib>
#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>
#include <regex>
#include <tinyxml2.h>

IGNITION_ADD_PLUGIN(phine_plugins::gNBPowerPlugin, ignition::gui::Plugin)

using namespace phine_plugins;

gNBPowerPlugin::gNBPowerPlugin() {
    ignmsg << "[gNBPowerPlugin] constructor called" << std::endl;
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

	auto *ver_elem = _pluginElem->FirstChildElement("version");
	if (ver_elem != nullptr && ver_elem->GetText() != nullptr) {
	    std::string ver = ver_elem->GetText();
	    if (ver == "v24") {
		version_index = 0;
	    } else if (ver == "v26") {
		version_index = 1;
	    }
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

int gNBPowerPlugin::getVersionIndex() const { return version_index; }

void gNBPowerPlugin::setVersionIndex(int index) {
    if (version_index != index) {
	version_index = index;
	emit versionIndexChanged();
    }
}

bool gNBPowerPlugin::isProcessRunning() const { return process_running; }

bool gNBPowerPlugin::isConnected() const { return connected; }

bool gNBPowerPlugin::isValidContainerName(const std::string &name) {
    if (name.empty() || name.size() > 128) {
	return false;
    }
    static const std::regex valid_name(R"(^[a-zA-Z0-9][a-zA-Z0-9_.\-]*$)");
    return std::regex_match(name, valid_name);
}

bool gNBPowerPlugin::checkProcessRunning() {
    std::string name = container_name.toStdString();
    if (!isValidContainerName(name)) {
	return false;
    }
    std::string cmd =
	"docker exec " + name + " pgrep nr-softmodem > /dev/null 2>&1";
    int ret = system(cmd.c_str());
    return (ret == 0);
}

std::string gNBPowerPlugin::buildStartCommand() const {
    std::string name = container_name.toStdString();
    if (version_index == 0) {
	// v24: uses .conf file with --sa flag
	return "docker exec " + name +
	       " /bin/bash -c"
	       " '/opt/oai-gnb/bin/nr-softmodem"
	       " -O /opt/oai-gnb/etc/gnb.conf"
	       " --sa -E --rfsim"
	       " --log_config.global_log_options level,nocolor,time &'";
    }
    // v26: uses .yaml file without --sa flag
    return "docker exec " + name +
	   " /bin/bash -c"
	   " '/opt/oai-gnb/bin/nr-softmodem"
	   " -O /opt/oai-gnb/etc/gnb.yaml"
	   " -E --rfsim"
	   " --log_config.global_log_options level,nocolor,time &'";
}

void gNBPowerPlugin::toggleProcess() {
    std::string name = container_name.toStdString();
    if (!isValidContainerName(name)) {
	ignerr << "[gNBPowerPlugin] Invalid container name: " << name
	       << std::endl;
	return;
    }

    // First click: check actual state, then become connected
    if (!connected) {
	process_running = checkProcessRunning();
	connected = true;
	emit connectedChanged();
	emit processRunningChanged();
	ignmsg << "[gNBPowerPlugin] Connected to " << name << " — process is "
	       << (process_running ? "running" : "stopped") << std::endl;
	return;
    }

    // Subsequent clicks: toggle without checking
    if (process_running) {
	std::string cmd = "docker exec " + name + " pkill nr-softmodem 2>&1";
	ignmsg << "[gNBPowerPlugin] Running: " << cmd << std::endl;
	int ret = system(cmd.c_str());
	ignmsg << "[gNBPowerPlugin] Kill exit=" << ret << std::endl;
	process_running = false;
	emit processRunningChanged();
    } else {
	std::string cmd = buildStartCommand();
	ignmsg << "[gNBPowerPlugin] Running: " << cmd << std::endl;
	int ret = system(cmd.c_str());
	ignmsg << "[gNBPowerPlugin] Start exit=" << ret << std::endl;
	process_running = true;
	emit processRunningChanged();
    }
}
