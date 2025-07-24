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

// Include plugin header and auxiliary functions
#include "RS5G_plugins.hh"
#include "aux_functions.hh"
#include <gz/plugin/RegisterMore.hh>

using namespace phine_plugins;

// Register the plugin with Ignition Gazebo
IGNITION_ADD_PLUGIN(phine_plugins::UE_plugin, gz::sim::System,
		    phine_plugins::UE_plugin::ISystemConfigure)

// Constructor: logs when the plugin is created
UE_plugin::UE_plugin() {
    RoboSimLogger::Log(LogLevel::INFO, "UE_plugin constructor called\n");
}

// Destructor: logs when the plugin is destroyed
UE_plugin::~UE_plugin() {
    RoboSimLogger::Log(LogLevel::INFO, "UE_plugin destructor called\n");
}

// Configure function: called when the plugin is loaded and configured
/**
 * @brief Configures the UE_plugin instance using parameters from the SDF and
 * environment.
 *
 * This method performs the following steps:
 * - Clones the SDF element to safely parse plugin parameters.
 * - Retrieves all required configuration parameters from the SDF; aborts if any
 * are missing.
 * - Parses the debug flag and converts it to a boolean.
 * - Obtains the PROJECT_PATH environment variable; aborts if not set.
 * - Constructs file paths for configuration and resources based on
 * PROJECT_PATH.
 * - Updates DDS XML interface whitelist addresses.
 * - Builds the Docker image for the UE robot using the specified project and
 * configuration.
 * - Modifies the UE configuration file with new values for IMSI, key, OPC, DNN,
 * NSSAI SST, and NSSAI SD.
 * - Composes and runs the Docker container for the UE robot with the
 * appropriate environment variables.
 *
 * @param[in] _entity The simulation entity associated with this plugin.
 * @param[in] _sdf Shared pointer to the SDF element containing plugin
 * configuration.
 * @param[in,out] _ecm EntityComponentManager for managing simulation entities
 * and components.
 * @param[in] _eventMgr EventManager for simulation events (unused).
 *
 * @note If any required parameter is missing or PROJECT_PATH is not set, the
 * method aborts early.
 */
void UE_plugin::Configure(const gz::sim::Entity &_entity,
			  const std::shared_ptr<const sdf::Element> &_sdf,
			  gz::sim::EntityComponentManager &_ecm,
			  gz::sim::EventManager & /*_eventMgr*/) {
    RoboSimLogger::Log(LogLevel::INFO, "UE_plugin::Configure called\n");

    // Clone the SDF to parse plugin parameters
    auto sdfClone = _sdf->Clone();

    // Retrieve all required parameters from SDF, abort if any are missing
    if (!GetParamFromSDF(sdfClone, "robot_container_name",
			 this->robot_container_name))
	return;
    if (!GetParamFromSDF(sdfClone, "robot_id", this->robot_id))
	return;
    if (!GetParamFromSDF(sdfClone, "net_name", this->netName))
	return;
    if (!GetParamFromSDF(sdfClone, "ip_robotUE", this->ip_robotUE))
	return;
    if (!GetParamFromSDF(sdfClone, "subnet_5G", this->subnet_5G))
	return;
    if (!GetParamFromSDF(sdfClone, "imsi", this->imsi))
	return;
    if (!GetParamFromSDF(sdfClone, "key", this->key))
	return;
    if (!GetParamFromSDF(sdfClone, "opc", this->opc))
	return;
    if (!GetParamFromSDF(sdfClone, "dnn", this->dnn))
	return;
    if (!GetParamFromSDF(sdfClone, "nssai_sst", this->nssai_sst))
	return;
    if (!GetParamFromSDF(sdfClone, "nssai_sd", this->nssai_sd))
	return;
    if (!GetParamFromSDF(sdfClone, "robot_project_path",
			 this->robot_project_path))
	return;
    if (!GetParamFromSDF(sdfClone, "ip_gnb", this->ip_gNB))
	return;
    if (!GetParamFromSDF(sdfClone, "robot_project_name",
			 this->robot_project_name))
	return;
    if (!GetParamFromSDF(sdfClone, "ros_gz_bridge_name",
			 this->ros_gz_bridge_name))
	return;
    if (!GetParamFromSDF(sdfClone, "robot_launch_file_name",
			 this->robot_launch_file_name))
	return;
    if (!GetParamFromSDF(sdfClone, "robot_package_name",
			 this->robot_package_name))
	return;
    if (!GetParamFromSDF(sdfClone, "execute_robot_launch_file",
			 this->execute_robot_launch_file))
	return;
    if (!GetParamFromSDF(sdfClone, "ros_discovery_server",
			 this->ros_discovery_server))
	return;

    // Get the debug flag from the SDF and convert to bool
    std::string temp_debug;
    if (!GetParamFromSDF(sdfClone, "debug", temp_debug)) {
	return;
    }
    this->debug_logs = string_to_bool(temp_debug);

    // Get PROJECT_PATH environment variable
    const char *project_path = std::getenv("PROJECT_PATH");
    if (project_path == nullptr) {
	RoboSimLogger::Log(
	    LogLevel::ERR,
	    "Error: PROJECT_PATH environment variable is not set.");
	return;
    }

    // Define file paths using PROJECT_PATH
    std::string file_path1 =
	std::string(project_path) + "/oai_setup/docker-compose-ue.yml";
    std::string file_path2 = std::string(project_path) + "/oai_setup/conf/ue" +
			     this->robot_id + ".conf";
    std::string file_path3 =
	std::string(project_path) + "/images/ue_amr/dds.xml";
    std::string file_path4 = std::string(project_path) + "/images/ue_amr";

    // Replace interface whitelist addresses in DDS XML
    replace_interface_whitelist_addresses(this->subnet_5G, file_path3,
					  this->debug_logs);

    // Build Docker image for UE robot
    std::string docker_build_command =
	"cp -r " + this->robot_project_path + " " + file_path4 +
	" && docker build --build-arg ROBOT_PROJECT_NAME=" +
	this->robot_project_name +
	" --build-arg ROS_DISCOVERY_SERVER=" + this->ros_discovery_server +
	" -t ue_amr " + file_path4;
    system(docker_build_command.c_str());

    // Prepare configuration file modifications for UE
    const std::string key1 = "imsi";
    const std::string new_value1 = "\"" + this->imsi + "\";";
    const std::string key2 = "key";
    const std::string new_value2 = "\"" + this->key + "\";";
    const std::string key3 = "opc";
    const std::string new_value3 = "\"" + this->opc + "\";";
    const std::string key4 = "dnn";
    const std::string new_value4 = "\"" + this->dnn + "\";";
    const std::string key5 = "nssai_sst";
    const std::string new_value5 = "\"" + this->nssai_sst + "\";";
    const std::string key6 = "nssai_sd";
    const std::string new_value6 = "\"" + this->nssai_sd + "\";";

    // Modify UE configuration file with new values
    modify_conf(file_path2, key1, new_value1, this->debug_logs);
    modify_conf(file_path2, key2, new_value2, this->debug_logs);
    modify_conf(file_path2, key3, new_value3, this->debug_logs);
    modify_conf(file_path2, key4, new_value4, this->debug_logs);
    modify_conf(file_path2, key5, new_value5, this->debug_logs);
    modify_conf(file_path2, key6, new_value6, this->debug_logs);

    // Compose and run Docker container for UE robot
    std::string folder_path = std::string(project_path) + "/oai_setup";
    std::string docker_compose_command =
	"cd " + folder_path + " && " + "NAME_ROBOT_" + this->robot_id + "=" +
	this->robot_container_name + " " + "IP_GNB=" + this->ip_gNB + " " +
	"IP_ROBOT_" + this->robot_id + "=" + this->ip_robotUE + " " +
	"ROBOT_PACKAGE_NAME_" + this->robot_id + "=" +
	this->robot_package_name + " " + "ROS_GZ_BRIDGE_NAME_" +
	this->robot_id + "=" + this->ros_gz_bridge_name + " " +
	"ROBOT_LAUNCH_FILE_NAME_" + this->robot_id + "=" +
	this->robot_launch_file_name + " " + "EXECUTE_ROBOT_LAUNCH_FILE_" +
	this->robot_id + "=" + this->execute_robot_launch_file + " " +
	"ROS_DISCOVERY_SERVER=" + this->ros_discovery_server + " " +
	"docker compose -f docker-compose-ue.yml up robot_" + this->robot_id +
	" -d";
    system(docker_compose_command.c_str());
}
