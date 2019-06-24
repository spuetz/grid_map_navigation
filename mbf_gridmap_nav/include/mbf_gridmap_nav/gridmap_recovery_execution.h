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
 *  move_base_recovery_execution.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MBF_GRIDMAP_NAV__GRIDMAP_RECOVERY_EXECUTION_H_
#define MBF_GRIDMAP_NAV__GRIDMAP_RECOVERY_EXECUTION_H_

#include <costmap_2d/costmap_2d_ros.h>
#include <mbf_gridmap_core/gridmap_recovery.h>
#include <mbf_abstract_nav/abstract_recovery_execution.h>
#include <mbf_gridmap_nav/MoveBaseFlexConfig.h>


namespace mbf_gridmap_nav
{
/**
 * @brief The GridmapRecoveryExecution binds a local and a global costmap to the AbstractRecoveryExecution and uses the
 *        nav_core/CostmapRecovery class as base plugin interface. This class makes move_base_flex compatible to the old move_base.
 *
 * @ingroup recovery_execution move_base_server
 */
class GridmapRecoveryExecution : public mbf_abstract_nav::AbstractRecoveryExecution
{

public:
  typedef boost::shared_ptr<grid_map::GridMap> GridmapPtr;
  typedef boost::shared_ptr<GridmapRecoveryExecution> Ptr;

  /**
   * @brief Constructor
   * @param condition Thread sleep condition variable, to wake up connected threads
   * @param tf_listener_ptr Shared pointer to a common tf listener
   * @param global_costmap Shared pointer to the global costmap.
   * @param local_costmap Shared pointer to the local costmap.
   */
  GridmapRecoveryExecution(
      const std::string name,
      const mbf_gridmap_core::GridmapRecovery::Ptr &recovery_ptr,
      const TFPtr &tf_listener_ptr,
      GridmapPtr &global_gridmap, GridmapPtr &local_gridmap,
      const MoveBaseFlexConfig &config);

  /**
   * Destructor
   */
  virtual ~GridmapRecoveryExecution();

protected:

  //! Shared pointer to the global gridmap
  GridmapPtr &global_gridmap_;
  
  //! Shared pointer to thr local gridmap
  GridmapPtr &local_gridmap_;

private:

  mbf_abstract_nav::MoveBaseFlexConfig toAbstract(const MoveBaseFlexConfig &config);
};

} /* namespace mbf_gridmap_nav */

#endif /* MBF_GRIDMAP_NAV__GRIDMAP_RECOVERY_EXECUTION_H_ */
