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
 *  move_base_planner_execution.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MBF_GRIDMAP_NAV__GRIDMAP_PLANNER_EXECUTION_H_
#define MBF_GRIDMAP_NAV__GRIDMAP_PLANNER_EXECUTION_H_

#include <mbf_abstract_nav/abstract_planner_execution.h>
#include <mbf_gridmap_nav/MoveBaseFlexConfig.h>
#include <costmap_2d/costmap_2d_ros.h> //TODO delete
#include <mbf_gridmap_core/gridmap_planner.h>

namespace mbf_gridmap_nav
{
/**
 * @brief The GridmapPlannerExecution binds a global costmap to the AbstractPlannerExecution and uses the
 *        nav_core/BaseCostmapPlanner class as base plugin interface. This class makes move_base_flex compatible to the old move_base.
 *
 * @ingroup planner_execution move_base_server
 */
class GridmapPlannerExecution : public mbf_abstract_nav::AbstractPlannerExecution
{
public:
  typedef boost::shared_ptr<grid_map::GridMap> GridmapPtr;

  /**
   * @brief Constructor
   * @param gridmap Shared pointer to the gridmap.
   */
  GridmapPlannerExecution(
      const std::string name,
      const mbf_gridmap_core::GridmapPlanner::Ptr &planner_ptr,
      GridmapPtr &gridmap,
      const MoveBaseFlexConfig &config,
      boost::function<void()> setup_fn = boost::function<void()>(),
      boost::function<void()> cleanup_fn = boost::function<void()>());

  /**
   * @brief Destructor
   */
  virtual ~GridmapPlannerExecution();

private:

  mbf_abstract_nav::MoveBaseFlexConfig toAbstract(const MoveBaseFlexConfig &config);

  //! Shared pointer to the global planner gridmap
  GridmapPtr &gridmap_ptr_;
};

} /* namespace mbf_gridmap_nav */

#endif /* MBF_GRIDMAP_NAV__GRIDMAP_PLANNER_EXECUTION_H_ */
