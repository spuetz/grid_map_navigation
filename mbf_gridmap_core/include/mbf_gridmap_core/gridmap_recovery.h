/*
 *  Copyright 2017, Sebastian Pütz
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
 *  abstract_global_planner.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *
 */

#ifndef MBF_GRIDMAP_CORE__GRIDMAP_RECOVERY_H_
#define MBF_GRIDMAP_CORE__GRIDMAP_RECOVERY_H_

#include <grid_map_core/GridMap.hpp>
#include <tf/transform_listener.h>
#include "mbf_abstract_core/abstract_recovery.h"
#include <mbf_utility/types.h>

namespace mbf_gridmap_core {
  //TODO
  /**
   * @class GridmapRecovery
   * @brief Provides an interface for recovery behaviors used in navigation.
   * All recovery behaviors written to work as MBF plugins must adhere to this interface. Alternatively, this
   * class can also operate as a wrapper for old API nav_core-based plugins, providing backward compatibility.
   */
  class GridmapRecovery : public mbf_abstract_core::AbstractRecovery{
    public:

      typedef boost::shared_ptr< ::mbf_gridmap_core::GridmapRecovery> Ptr;

      /**
       * @brief Initialization function for the GridmapRecovery
       * @param tf A pointer to a transform listener
       * @param global_gridmap_ptr A pointer to the global_gridmap used by the navigation stack //TODO merge both maps
       * @param local_gridmap_ptr A pointer to the local_gridmap used by the navigation stack
       */
      virtual void initialize(std::string name,
                              const TFPtr& tf,
                              boost::shared_ptr<grid_map::GridMap>& global_grid_map_ptr,
                              boost::shared_ptr<grid_map::GridMap>& local_grid_map_ptr) = 0;

  /**
   * @brief Runs the GridmapRecovery
   * @param message The recovery behavior could set, the message should correspond to the return value
   * @return An outcome which will be hand over to the action result.
   */
  virtual uint32_t runBehavior(std::string& message) = 0;

  /**
   * @brief Requests the planner to cancel, e.g. if it takes too much time
   * @remark New on MBF API
   * @return True if a cancel has been successfully requested, false if not implemented.
   */
  virtual bool cancel() = 0;

      /**
       * @brief Virtual destructor for the interface
       */
      virtual ~GridmapRecovery(){}

    protected:
      GridmapRecovery(){}

  };
};  /* namespace mbf_gridmap_core */

#endif /* MBF_GRIDMAP_CORE__GRIDMAP_RECOVERY_H_ */
