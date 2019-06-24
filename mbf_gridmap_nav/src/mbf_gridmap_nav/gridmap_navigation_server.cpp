/*
 *  Copyright 2017, Magazino GmbH, Sebastian Pütz, Jorge Santos Simón
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  move_base_navigation_server.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include "mbf_gridmap_nav/gridmap_navigation_server.h"

namespace mbf_gridmap_nav
{

GridmapNavigationServer::GridmapNavigationServer(const TFPtr &tf_listener_ptr) :
  AbstractNavigationServer(tf_listener_ptr),
  private_nh_("~"),
  planner_plugin_loader_("mbf_gridmap_core", "mbf_gridmap_core::GridmapPlanner"),
  controller_plugin_loader_("mbf_gridmap_core", "mbf_gridmap_core::GridmapController"),
  recovery_plugin_loader_("mbf_gridmap_core", "mbf_gridmap_core::GridmapRecovery"),
  file_name_(private_nh_.param<std::string>("map_file", "/local/ceres_ws/src/uos_tools/uos_maps/maps/avz5floor_gazebo.yaml")),
  global_gridmap_ptr_(boost::make_shared<grid_map::GridMap>(std::vector<std::string> {"global_gridmap"})),
  local_gridmap_ptr_(boost::make_shared<grid_map::GridMap>(std::vector<std::string> {"local_gridmap"})),
  grid_map_handler_(file_name_,
    private_nh_.param<double>("resolution", 0.1),
    private_nh_.param<double>("local_map_length", 5.0),
    private_nh_.param<double>("inflation_distance", 0.35),
    private_nh_.param<bool>("use_polygon", false),
    tf_listener_ptr_,
    *global_gridmap_ptr_,
    *local_gridmap_ptr_
  )
{
  // shutdown gridmaps
  private_nh_.param("shutdown_gridmaps", shutdown_gridmaps_, false);

  // stop updating gridmaps when not planning, moving, or recovering
  if (!shutdown_gridmaps_)
  {
    local_gridmap_active_ = true;
    global_gridmap_active_ = true;
  }
  else
  {
    grid_map_handler_.stop();
    local_gridmap_active_ = false;
    global_gridmap_active_ = false;
  }

  // advertise services and current goal topic
//TODO These services are not part of this project; same goes for dynamic reconfigure. Thus haven't been tested
  check_pose_cost_srv_ = private_nh_.advertiseService("check_pose_cost",
                                                     &GridmapNavigationServer::callServiceCheckPoseCost, this);
//  clear_gridmaps_srv_ = private_nh_.advertiseService("clear_gridmaps",
//                                                    &GridmapNavigationServer::callServiceClearGridmaps, this); //TODO most likely unneeded needs further inspection

  // dynamic reconfigure server for mbf_gridmap_nav specific config
  dsrv_gridmap_ = boost::make_shared<dynamic_reconfigure::Server<mbf_gridmap_nav::MoveBaseFlexConfig> >(private_nh_);
  dsrv_gridmap_->setCallback(boost::bind(&GridmapNavigationServer::reconfigure, this, _1, _2));

  // initialize all plugins
  initializeServerComponents();

  // start all action servers
  startActionServers();
}

mbf_abstract_nav::AbstractPlannerExecution::Ptr GridmapNavigationServer::newPlannerExecution(
    const std::string name, const mbf_abstract_core::AbstractPlanner::Ptr plugin_ptr)
{
  return boost::make_shared<mbf_gridmap_nav::GridmapPlannerExecution>(
      name,
      boost::static_pointer_cast<mbf_gridmap_core::GridmapPlanner>(plugin_ptr),
      boost::ref(global_gridmap_ptr_),
      last_config_);
}

mbf_abstract_nav::AbstractControllerExecution::Ptr GridmapNavigationServer::newControllerExecution(
    const std::string name, const mbf_abstract_core::AbstractController::Ptr plugin_ptr)
{
  return boost::make_shared<mbf_gridmap_nav::GridmapControllerExecution>(
      name,
      boost::static_pointer_cast<mbf_gridmap_core::GridmapController>(plugin_ptr),
      vel_pub,
      goal_pub,
      tf_listener_ptr_,
      last_config_,
      boost::ref(local_gridmap_ptr_));
}

mbf_abstract_nav::AbstractRecoveryExecution::Ptr GridmapNavigationServer::newRecoveryExecution(
    const std::string name, const mbf_abstract_core::AbstractRecovery::Ptr plugin_ptr)
{
return boost::make_shared<mbf_gridmap_nav::GridmapRecoveryExecution>(
      name,
      boost::static_pointer_cast<mbf_gridmap_core::GridmapRecovery>(plugin_ptr),
      tf_listener_ptr_,
      boost::ref(global_gridmap_ptr_),
      boost::ref(local_gridmap_ptr_),
      last_config_);
}

mbf_abstract_core::AbstractPlanner::Ptr GridmapNavigationServer::loadPlannerPlugin(const std::string& planner_type)
{
  mbf_abstract_core::AbstractPlanner::Ptr planner_ptr;

  // try to load and init global planner
  ROS_DEBUG("Load planner plugin.");
  try
  {
    planner_ptr = boost::static_pointer_cast<mbf_abstract_core::AbstractPlanner>(
        planner_plugin_loader_.createInstance(planner_type));
    std::string planner_name = planner_plugin_loader_.getName(planner_type);
    ROS_INFO_STREAM("MBF_core-based global planner plugin " << planner_name << " loaded");
  }
  catch (const pluginlib::PluginlibException &ex)
  {
    ROS_INFO_STREAM("Failed to load the " << planner_type << " planner as a mbf_gridmap_core-based plugin." << ex.what());
  }
  return planner_ptr;
}

bool GridmapNavigationServer::initializePlannerPlugin(
    const std::string& name,
    const mbf_abstract_core::AbstractPlanner::Ptr& planner_ptr
)
{
  mbf_gridmap_core::GridmapPlanner::Ptr gridmap_planner_ptr
      = boost::static_pointer_cast<mbf_gridmap_core::GridmapPlanner>(planner_ptr);
  ROS_INFO_STREAM("Initialize planner \"" << name << "\".");

  gridmap_planner_ptr->initialize(name, global_gridmap_ptr_);

  ROS_INFO("Planner plugin initialized.");
  return true;
}

mbf_abstract_core::AbstractController::Ptr
GridmapNavigationServer::loadControllerPlugin(const std::string &controller_type)
{
  mbf_abstract_core::AbstractController::Ptr controller_ptr;
  // try to load and init controller
  try {
    controller_ptr = controller_plugin_loader_.createInstance(controller_type);
    std::string controller_name = controller_plugin_loader_.getName(controller_type);
    ROS_INFO_STREAM("MBF_core-based local planner plugin " << controller_name << " loaded");
  }
  catch (const pluginlib::PluginlibException &ex) {
    ROS_INFO_STREAM("Failed to load the " << controller_type << " local planner as a mbf_gridmap_core-based plugin; "
                                          << ex.what());
  }
  return controller_ptr;
}

bool GridmapNavigationServer::initializeControllerPlugin(
    const std::string& name,
    const mbf_abstract_core::AbstractController::Ptr& controller_ptr
)
{
  ROS_INFO_STREAM("Initialize controller \"" << name << "\".");

  if (!tf_listener_ptr_) {
    ROS_ERROR_STREAM("The tf listener pointer has not been initialized!");
    return false;
  }

  if (!local_gridmap_ptr_) {
    ROS_ERROR_STREAM("The gridmap pointer has not been initialized!");
    return false;
  }

  mbf_gridmap_core::GridmapController::Ptr gridmap_controller_ptr
      = boost::static_pointer_cast<mbf_gridmap_core::GridmapController>(controller_ptr);
  gridmap_controller_ptr->initialize(name, tf_listener_ptr_, local_gridmap_ptr_);
  ROS_INFO_STREAM("Controller plugin \"" << name << "\" initialized.");
  return true;
}

mbf_abstract_core::AbstractRecovery::Ptr GridmapNavigationServer::loadRecoveryPlugin(
    const std::string& recovery_type)
{
  mbf_abstract_core::AbstractRecovery::Ptr recovery_ptr;
  try
  {
    recovery_ptr = boost::static_pointer_cast<mbf_abstract_core::AbstractRecovery>(
        recovery_plugin_loader_.createInstance(recovery_type));
  }
  catch (pluginlib::PluginlibException &ex)
  {
    ROS_DEBUG_STREAM("Failed to load the " << recovery_type << " recovery behavior as a mbf_abstract_core-based plugin;"
                                           << " Exception: " << ex.what());
  }

  return recovery_ptr;
}

bool GridmapNavigationServer::initializeRecoveryPlugin(
    const std::string& name,
    const mbf_abstract_core::AbstractRecovery::Ptr& recovery_ptr
)
{
  mbf_gridmap_core::GridmapRecovery::Ptr gridmap_recovery_ptr =
        boost::static_pointer_cast<mbf_gridmap_core::GridmapRecovery>(recovery_ptr);
  gridmap_recovery_ptr->initialize(name, tf_listener_ptr_, global_gridmap_ptr_, local_gridmap_ptr_);
  return true;
}


void GridmapNavigationServer::stop()
{
  AbstractNavigationServer::stop();
  grid_map_handler_.stop();
}


GridmapNavigationServer::~GridmapNavigationServer()
{
}

void GridmapNavigationServer::reconfigure(mbf_gridmap_nav::MoveBaseFlexConfig &config, uint32_t level)
{
  //boost::recursive_mutex::scoped_lock sl(configuration_mutex_);

  // Make sure we have the original configuration the first time we're called, so we can restore it if needed
  if (!setup_reconfigure_)
  {
    default_config_ = config;
    setup_reconfigure_ = true;
  }

  // fill the abstract configuration common to all MBF-based navigation
  mbf_abstract_nav::MoveBaseFlexConfig abstract_config;
  abstract_config.planner_frequency = config.planner_frequency;
  abstract_config.planner_patience = config.planner_patience;
  abstract_config.planner_max_retries = config.planner_max_retries;
  abstract_config.controller_frequency = config.controller_frequency;
  abstract_config.controller_patience = config.controller_patience;
  abstract_config.controller_max_retries = config.controller_max_retries;
  abstract_config.recovery_enabled = config.recovery_enabled;
  abstract_config.oscillation_timeout = config.oscillation_timeout;
  abstract_config.oscillation_distance = config.oscillation_distance;
  abstract_config.restore_defaults = config.restore_defaults;
  mbf_abstract_nav::AbstractNavigationServer::reconfigure(abstract_config, level);

  last_config_ = config;
}

bool GridmapNavigationServer::callServiceCheckPoseCost(mbf_msgs::CheckPose::Request &request,
                                                        mbf_msgs::CheckPose::Response &response)
{
  return true;
}

bool GridmapNavigationServer::callServiceClearGridmaps(std_srvs::Empty::Request &request,
                                                        std_srvs::Empty::Response &response)
{
//  grid_map_handler_.stop();
//  grid_map_handler_.resetLayers(); //TODO could be implemented if needed.
  return true;
}

} /* namespace mbf_gridmap_nav */
