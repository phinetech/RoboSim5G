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

#ifndef RS5G_PLUGINS_HH_
#define RS5G_PLUGINS_HH_

// The only required include in the header is this one.
// All others will depend on what your plugin does.
#include <string>
#include <gz/common/Console.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <ignition/math/Vector3.hh>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <regex>
#include <cstdlib>
#include <gz/math/Pose3.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <memory>
#include <sdf/sdf.hh>

/**
 * @namespace phine_plugins
 * @brief Contains plugins for the RoboSim5G simulation environment.
 *
 * This namespace encapsulates all plugin classes and related functionality
 * for extending and interacting with the RoboSim5G simulation, particularly
 * for 5G network simulation components.
 */
namespace phine_plugins
{
// The plugin class inherits from the System class, which provides the
// functionality to load the plugin and interact with the simulation.
// The ISystemConfigure and ISystemPostUpdate interfaces are used to
// configure the plugin and update the simulation, respectively.
class gNB_plugin :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate {
  /// \brief Constructor
  public:
    gNB_plugin();

    /// \brief Destructor
  public:
    ~gNB_plugin();

    /// \brief Configure the plugin 
  public:
    void Configure(const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &/*_eventMgr*/) override;

    /// \brief Update the plugin at each gazebo iteration
  public:
    void PostUpdate(const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm) override;
    // \brief ROS node
  private:
    rclcpp::Node::SharedPtr node;
  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;

    // \brief plugin parameters
  public:
    std::string model_name;
    std::string linkName;
    std::string netName;
    std::string ip_gnb;
    std::string ip_amf;
    bool debug_logs;
    std::string mnc;
    std::string mcc;
    gz::sim::Model model{ gz::sim::kNullEntity };
    gz::sim::Entity linkEntity{ gz::sim::kNullEntity };
  };

}

#endif // RS5G_PLUGINS_HH_
