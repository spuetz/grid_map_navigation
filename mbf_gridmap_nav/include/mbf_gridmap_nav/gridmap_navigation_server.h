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
 *  move_base_navigation_server.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MBF_GRIDMAP_NAV__GRIDMAP_NAVIGATION_SERVER_H_
#define MBF_GRIDMAP_NAV__GRIDMAP_NAVIGATION_SERVER_H_

#include <mbf_abstract_nav/abstract_navigation_server.h>

#include "gridmap_planner_execution.h"
#include "gridmap_controller_execution.h"
#include "gridmap_recovery_execution.h"

#include "mbf_gridmap_nav/MoveBaseFlexConfig.h"
#include <std_srvs/Empty.h>
#include <mbf_msgs/CheckPose.h>
#include <mbf_msgs/CheckPath.h>

#include <mbf_gridmap_core/gridmap_planner.h>
#include <mbf_gridmap_core/gridmap_controller.h>
#include <mbf_gridmap_core/gridmap_recovery.h>

#include <grid_map_handler/grid_map_handler.h>

namespace mbf_gridmap_nav
{
/**
 * @defgroup move_base_server Move Base Server
 * @brief Classes belonging to the Move Base Server level.
 */

typedef boost::shared_ptr<dynamic_reconfigure::Server<mbf_gridmap_nav::MoveBaseFlexConfig> > DynamicReconfigureServerGridmapNav;

/**
 * @brief The GridmapNavigationServer makes Move Base Flex backwards compatible to the old move_base. It combines the
 *        execution classes which use the nav_core/BaseLocalPlanner, nav_core/BaseCostmapPlanner and the
 *        nav_core/RecoveryBehavior base classes as plugin interfaces. These plugin interface are the same for the
 *        old move_base
 *
 * @ingroup navigation_server move_base_server
 */
class GridmapNavigationServer : public mbf_abstract_nav::AbstractNavigationServer
{
public:

  typedef boost::shared_ptr<grid_map::GridMap> GridmapPtr;

  /**
   * @brief Constructor
   * @param tf_listener_ptr Shared pointer to a common TransformListener
   */
  GridmapNavigationServer(const TFPtr &tf_listener_ptr);

  /**
   * @brief Destructor
   */
  virtual ~GridmapNavigationServer();

  virtual void stop();

private:

  //! shared pointer to a new @ref planner_execution "PlannerExecution"
  virtual mbf_abstract_nav::AbstractPlannerExecution::Ptr newPlannerExecution(
      const std::string name,
      const mbf_abstract_core::AbstractPlanner::Ptr plugin_ptr);

  //! shared pointer to a new @ref controller_execution "ControllerExecution"
  virtual mbf_abstract_nav::AbstractControllerExecution::Ptr newControllerExecution(
      const std::string name,
      const mbf_abstract_core::AbstractController::Ptr plugin_ptr);

  //! shared pointer to a new @ref recovery_execution "RecoveryExecution"
  virtual mbf_abstract_nav::AbstractRecoveryExecution::Ptr newRecoveryExecution(
      const std::string name,
      const mbf_abstract_core::AbstractRecovery::Ptr plugin_ptr);

  /**
   * @brief Loads the plugin associated with the given planner_type parameter.
   * @param planner_type The type of the planner plugin to load.
   * @return true, if the local planner plugin was successfully loaded.
   */
  virtual mbf_abstract_core::AbstractPlanner::Ptr loadPlannerPlugin(const std::string& planner_type);

  /**
   * @brief Initializes the controller plugin with its name and pointer to the gridmap
   * @param name The name of the planner
   * @param planner_ptr pointer to the planner object which corresponds to the name param
   * @return true if init succeeded, false otherwise
   */
  virtual bool initializePlannerPlugin(
      const std::string& name,
      const mbf_abstract_core::AbstractPlanner::Ptr& planner_ptr
  );

  /**
   * @brief Loads the plugin associated with the given controller type parameter
   * @param controller_type The type of the controller plugin
   * @return A shared pointer to a new loaded controller, if the controller plugin was loaded successfully,
   *         an empty pointer otherwise.
   */
  virtual mbf_abstract_core::AbstractController::Ptr loadControllerPlugin(const std::string& controller_type);

  /**
   * @brief Initializes the controller plugin with its name, a pointer to the TransformListener
   *        and pointer to the gridmap
   * @param name The name of the controller
   * @param controller_ptr pointer to the controller object which corresponds to the name param
   * @return true if init succeeded, false otherwise
   */
  virtual bool initializeControllerPlugin(
      const std::string& name,
      const mbf_abstract_core::AbstractController::Ptr& controller_ptr
  );

  /**
   * @brief Loads a Recovery plugin associated with given recovery type parameter
   * @param recovery_name The name of the Recovery plugin
   * @return A shared pointer to a Recovery plugin, if the plugin was loaded successfully, an empty pointer otherwise.
   */
  virtual mbf_abstract_core::AbstractRecovery::Ptr loadRecoveryPlugin(const std::string& recovery_type);

  /**
   * @brief Initializes a recovery behavior plugin with its name and pointers to the global and local gridmaps
   * @param name The name of the recovery behavior
   * @param behavior_ptr pointer to the recovery behavior object which corresponds to the name param
   * @return true if init succeeded, false otherwise
   */
  virtual bool initializeRecoveryPlugin(
      const std::string& name,
      const mbf_abstract_core::AbstractRecovery::Ptr& behavior_ptr);



  /**
   * @brief Callback method for the check_pose_cost service
   * @param request Request object, see the mbf_msgs/CheckPose service definition file.
   * @param response Response object, see the mbf_msgs/CheckPose service definition file.
   * @return true, if the service completed successfully, false otherwise
   */
  bool callServiceCheckPoseCost(mbf_msgs::CheckPose::Request &request,
                                mbf_msgs::CheckPose::Response &response);

  /**
   * @brief Callback method for the check_path_cost service
   * @param request Request object, see the mbf_msgs/CheckPath service definition file.
   * @param response Response object, see the mbf_msgs/CheckPath service definition file.
   * @return true, if the service completed successfully, false otherwise
   */
  bool callServiceCheckPathCost(mbf_msgs::CheckPath::Request &request,
                                mbf_msgs::CheckPath::Response &response);

  /**
   * @brief Callback method for the make_plan service
   * @param request Empty request object.
   * @param response Empty response object.
   * @return true, if the service completed successfully, false otherwise
   */
  bool callServiceClearGridmaps(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

  /**
   *
   * @brief Reconfiguration method called by dynamic reconfigure.
   * @param config Configuration parameters. See the MoveBaseFlexConfig definition.
   * @param level bit mask, which parameters are set.
   */
  void reconfigure(mbf_gridmap_nav::MoveBaseFlexConfig &config, uint32_t level);

  pluginlib::ClassLoader<mbf_gridmap_core::GridmapPlanner> planner_plugin_loader_;
  pluginlib::ClassLoader<mbf_gridmap_core::GridmapController> controller_plugin_loader_;
  pluginlib::ClassLoader<mbf_gridmap_core::GridmapRecovery> recovery_plugin_loader_;

  //! Dynamic reconfigure server for the mbf_gridmap specific part
  DynamicReconfigureServerGridmapNav dsrv_gridmap_;

  //! last configuration save
  mbf_gridmap_nav::MoveBaseFlexConfig last_config_;

  //! the default parameter configuration save
  mbf_gridmap_nav::MoveBaseFlexConfig default_config_;

  ros::NodeHandle private_nh_;

  //! Shared pointer to the common local gridmap
  GridmapPtr local_gridmap_ptr_;

  //! Shared pointer to the common global gridmap
  GridmapPtr global_gridmap_ptr_;

  // filename to the map pgm
  std::string file_name_;

  // handles the grid_map and keeps it up to date
  grid_map_handler::GridMapHandler grid_map_handler_;

  //! true, if the local gridmap is active
  bool local_gridmap_active_;

  //! true, if the global gridmap is active
  bool global_gridmap_active_;

  //! Service Server for the check_pose_cost service
  ros::ServiceServer check_pose_cost_srv_;

  //! Service Server for the check_path_cost service
  ros::ServiceServer check_path_cost_srv_;

  //! Service Server for the clear_gridmap service
  ros::ServiceServer clear_gridmaps_srv_;

  ros::Publisher vel_pub;
  ros::Publisher goal_pub;

  //! Stop updating gridmaps when not planning or controlling, if true //TODO really needed?
  bool shutdown_gridmaps_;
  ros::Timer shutdown_gridmaps_timer_;    //!< delayed shutdown timer
  ros::Duration shutdown_gridmaps_delay_; //!< delayed shutdown delay

};

} /* namespace mbf_gridmap_nav */

#endif /* MBF_GRIDMAP_NAV__GRIDMAP_NAVIGATION_SERVER_H_ */
