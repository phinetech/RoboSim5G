# generated from colcon_powershell/shell/template/prefix_chain.ps1.em

# This script extends the environment with the environment of other prefix
# paths which were sourced when this file was generated as well as all packages
# contained in this prefix path.

# function to source another script with conditional trace output
# first argument: the path of the script
function _colcon_prefix_chain_powershell_source_script {
  param (
    $_colcon_prefix_chain_powershell_source_script_param
  )
  # source script with conditional trace output
  if (Test-Path $_colcon_prefix_chain_powershell_source_script_param) {
    if ($env:COLCON_TRACE) {
      echo ". '$_colcon_prefix_chain_powershell_source_script_param'"
    }
    . "$_colcon_prefix_chain_powershell_source_script_param"
  } else {
    Write-Error "not found: '$_colcon_prefix_chain_powershell_source_script_param'"
  }
}

# source chained prefixes
_colcon_prefix_chain_powershell_source_script "/opt/ros/humble\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/RB/study/gazebo_ign/gaz-ros-bridge_plug/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/RB/study/gazebo_ign/bridge_rosgz/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/RB/workspace/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/RB/study/gazebo_ign/myrosplugin/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/RB/study/ros2_net_slicing/install\local_setup.ps1"
_colcon_prefix_chain_powershell_source_script "/home/RB/study/docker_study/ros_dock_pubsub/install\local_setup.ps1"

# source this prefix
$env:COLCON_CURRENT_PREFIX=(Split-Path $PSCommandPath -Parent)
_colcon_prefix_chain_powershell_source_script "$env:COLCON_CURRENT_PREFIX\local_setup.ps1"
