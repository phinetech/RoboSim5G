/* Copyright 2025 RoboSim5G

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.*/

// including header file for the plugin and auxiliary functions
#include "RS5G_plugins.hh"
#include "aux_functions.hh"
// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
IGNITION_ADD_PLUGIN(
    phine_plugins::gNB_plugin,
    gz::sim::System,
    phine_plugins::gNB_plugin::ISystemConfigure,
    phine_plugins::gNB_plugin::ISystemPostUpdate)

using namespace phine_plugins;

gNB_plugin::gNB_plugin()
{
    RoboSimLogger::Log(LogLevel::INFO, "gNB_plugin constructor called\n");
}

gNB_plugin::~gNB_plugin()
{
    // Shutdown ROS node and publisher
    if (this->node)
    {
        this->publisher.reset();
        this->node.reset();
        rclcpp::shutdown();
    }
}

/**
 * @brief Configures the gNB_plugin with parameters from the simulation and SDF.
 *
 * This method initializes the plugin by:
 * - Validating the attached model entity.
 * - Initializing the ROS node and publisher for pose messages.
 * - Parsing required parameters from the SDF, such as link name, network name, IP addresses, MCC, MNC, and debug flag.
 * - Fetching the PROJECT_PATH environment variable to locate configuration files.
 * - Modifying Docker Compose and configuration files with the parsed parameters to set up the gNB instance.
 * - Launching the Docker Compose setup for the gNB.
 * - Restoring the original service name in the Docker Compose file for subsequent gNBs.
 *
 * @param[in] _entity The simulation entity to which the plugin is attached.
 * @param[in] _sdf Shared pointer to the SDF element containing plugin parameters.
 * @param[in,out] _ecm The EntityComponentManager for accessing and modifying simulation entities.
 * @param[in,out] _eventMgr The EventManager (unused).
 *
 * @note The method logs errors and returns early if required parameters are missing or if initialization fails.
 * @note The method modifies files on disk and launches system commands; use with appropriate permissions.
 */
void gNB_plugin::Configure(const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/)
{
    RoboSimLogger::Log(LogLevel::INFO, "gNB_plugin::Configure called\n");

    this->model = gz::sim::Model(_entity);

    // Check if the plugin is attached to a model entity
    if (!this->model.Valid(_ecm))
    {
        RoboSimLogger::Log(LogLevel::ERR,  "gNB_plugin plugin should be attached to a model entity. Failed to initialize.\n" );
        return;
    }

    // Get the model's name (this should be "gNB{i}" with i the number of the gNB as in the SDF definition)
    this->model_name = model.Name(_ecm);
    // Initialize the ROS node and publisher
    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    this->node = rclcpp::Node::make_shared(this->model_name + "_node");

    // Create a publisher for the String message
    this->publisher = this->node->create_publisher<std_msgs::msg::String>("pose_of_" + this->model_name, 10);

    // Clone the SDF to parse the plugin parameters
    auto sdfClone = _sdf->Clone();
    // Get the link name from the SDF
    if (!GetParamFromSDF(sdfClone, "link_name", this->linkName))
        return;
    //  Get the network name from the SDF
    if (!GetParamFromSDF(sdfClone, "net_name", this->netName))
        return;
    // Get the IP addresses of the AMF from the SDF
    if (!GetParamFromSDF(sdfClone, "IP_AMF", this->ip_amf))
        return;
    // Get the IP addresses of the gNB from the SDF
    if (!GetParamFromSDF(sdfClone, "IP_GNB", this->ip_gnb))
        return;
    // Get the mobile country code from the SDF
    if (!GetParamFromSDF(sdfClone, "mobile_country_code", this->mcc))
        return;
    // Get the mobile network code from the SDF
    if (!GetParamFromSDF(sdfClone, "mobile_network_code", this->mnc))
        return;
    // Get the debug flag from the SDF
    std::string temp_debug; 
    if (!GetParamFromSDF(sdfClone, "debug", temp_debug)) {
        return;
    }
    this->debug_logs = string_to_bool(temp_debug);

    // get the value of the PROJECT_PATH environment variable
    const char* project_path = std::getenv("PROJECT_PATH");
    if (project_path == nullptr) {
        RoboSimLogger::Log(LogLevel::ERR, "Error: PROJECT_PATH environment variable is not set." );
        return;
    }

    // Define the docker compose filefile path using PROJECT_PATH
    std::string file_path1 = std::string(project_path) + "/oai_setup/docker-compose-gNB.yml"; 
    // Define the configuration file file path using PROJECT_PATH
    std::string file_path2 = std::string(project_path) + "/oai_setup/conf/gnb.sa.bandn78.fr1.106PRB.rfsim.conf";  

    // Define the key-value pairs for modifications in the Docker Compose file and configuration file
    const std::string key1 = "name";  
    const std::string new_value1 = this->netName;  // Set the network name

    const std::string key2 = "ipv4_address";  // The parameter key to search for
    const std::string new_value2 = this->ip_gnb;  // Set the gNB's IPv4 address

    const std::string key3 = "container_name";  // The parameter key to search for
    const std::string new_value3 = "oai-" + this->model_name;  // Set the container name based on the model name

    const std::string key4 = "ipv4";  // The parameter key to search for in the configuration file
    const std::string new_value4 = "\""+this->ip_amf+"\";";  // Set the AMF's IPv4 address

    const std::string key5 = "GNB_IPV4_ADDRESS_FOR_NG_AMF";  // The parameter key to search for
    const std::string new_value5 = "\""+this->ip_gnb+"\";";  // Set the gNB's IPv4 address for NG-AMF

    const std::string key6 = "GNB_IPV4_ADDRESS_FOR_NGU";  // The parameter key to search for
    const std::string new_value6 = "\""+this->ip_gnb+"\";";  // Set the gNB's IPv4 address for NG-U

    const std::string key7 = "Active_gNBs";  // The parameter key to search for
    const std::string new_value7 = "(\""+this->model_name+"\");";  // SChanging gNB name in config file

    const std::string key8 = "gNB_name";  // The parameter key to search for
    const std::string new_value8 = "\""+this->model_name+"\";";  // Changing gNB name in config file

    std::string gNB_ID = extractAndConvertToHex(this->model_name); // Extract and convert the gNB number to hexadecimal
    const std::string key9 = "gNB_ID";  // The parameter key to search for
    const std::string new_value9 = gNB_ID+";";  // Changing gNB ID number (hexadecimal) in config file

    const std::string key10 = "mcc";  // The parameter key to search for
    const std::string new_value10 = this->mcc+";";  // Changing the mcc in config file

    const std::string key11 = "mnc";  // The parameter key to search for
    const std::string new_value11 = this->mnc+";";  // Changing the mnc in config file

    const std::string old_key = "oai_gNB";  // The old service name in the Docker Compose file
    const std::string new_key = "oai_" + this->model_name;  // The new service name based on the model name

    // Call modifyFile for each key-value pair to update the Docker Compose file
    modify_dockerC(file_path1, key1, new_value1, this->debug_logs);
    modify_dockerC(file_path1, key2, new_value2, this->debug_logs);
    modify_dockerC(file_path1, key3, new_value3, this->debug_logs);

    // Call modifyFile for each key-value pair to update the configuration file
    modify_conf(file_path2, key4, new_value4, this->debug_logs);
    modify_conf(file_path2, key5, new_value5, this->debug_logs);
    modify_conf(file_path2, key6, new_value6, this->debug_logs);
    modify_conf(file_path2, key7, new_value7, this->debug_logs);
    modify_conf(file_path2, key8, new_value8, this->debug_logs);
    modify_conf(file_path2, key9, new_value9, this->debug_logs);
    modify_conf(file_path2, key10, new_value10, this->debug_logs);
    modify_conf(file_path2, key11, new_value11, this->debug_logs);
    // Update the service name in the Docker Compose file
    modify_service_name(file_path1, old_key, new_key, this->debug_logs);

    // Define the Docker Compose command to run in that specific folder
    std::string folder_path = std::string(project_path) + "/oai_setup";  // Update with your path
    std::string docker_compose_command = "cd " + folder_path + " && docker compose -f docker-compose-gNB.yml up -d";
    system(docker_compose_command.c_str());
    // Reset the old service name in the Docker Compose file to be access by other gNBs
    modify_service_name(file_path1, new_key, old_key,this->debug_logs);
}

// Here we implement the PostUpdate function, which is called at every
// iteration after the physics updates the world
/**
 * @brief Called after each simulation update to publish the pose of the gNB link.
 *
 * This method ensures that ROS is initialized, locates the specified link entity
 * if it has not already been found, retrieves its world pose, formats the pose
 * information into a string, and publishes it as a ROS message.
 *
 * @param[in] _info Simulation update information.
 * @param[in] _ecm Entity-component manager providing access to simulation entities.
 */
void gNB_plugin::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
    // Ensure ROS is initialized
    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    // If the link entity is not yet initialized, find it by name
    if (this->linkEntity == ignition::gazebo::v6::kNullEntity)
    {
        this->linkEntity =
            this->model.LinkByName(_ecm, this->linkName);
    }

    // If the link entity is still invalid, exit the function
    if (this->linkEntity == ignition::gazebo::v6::kNullEntity)
        return;

    // Get the world pose of the link entity
    const gz::math::Pose3d rawPose = worldPose(this->linkEntity, _ecm);

    // Create a ROS message to publish the pose
    auto msg = std_msgs::msg::String();

    // Format the pose data into a string
    std::ostringstream oss;
    oss << "Position: [" << rawPose.Pos().X() << ", " << rawPose.Pos().Y() << ", " << rawPose.Pos().Z() << "], "
        << "Orientation: [" << rawPose.Rot().Roll() << ", " << rawPose.Rot().Pitch() << ", " << rawPose.Rot().Yaw() << "]";
    std::string poseStr = oss.str();

    // Assign the formatted pose string to the message
    msg.data =  poseStr;

    // Publish the message
    this->publisher->publish(msg);
}
