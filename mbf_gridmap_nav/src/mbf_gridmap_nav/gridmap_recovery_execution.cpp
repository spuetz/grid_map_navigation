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
 *  move_base_recovery_execution.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */
#include <nav_core/recovery_behavior.h>
#include <mbf_gridmap_nav/MoveBaseFlexConfig.h>
#include "mbf_gridmap_nav/gridmap_recovery_execution.h"

namespace mbf_gridmap_nav
{



GridmapRecoveryExecution::GridmapRecoveryExecution(
    const std::string name,
    const mbf_gridmap_core::GridmapRecovery::Ptr &recovery_ptr,
    const TFPtr &tf_listener_ptr,
    GridmapPtr &global_gridmap, GridmapPtr &local_gridmap,
    const MoveBaseFlexConfig &config)
    : AbstractRecoveryExecution(name, recovery_ptr, tf_listener_ptr, toAbstract(config), boost::function<void()>(), boost::function<void()>()),
      global_gridmap_(global_gridmap), local_gridmap_(local_gridmap)
{
}

GridmapRecoveryExecution::~GridmapRecoveryExecution()
{
}

mbf_abstract_nav::MoveBaseFlexConfig GridmapRecoveryExecution::toAbstract(const MoveBaseFlexConfig &config)
{
  // copy the recovery-related abstract configuration common to all MBF-based navigation
  mbf_abstract_nav::MoveBaseFlexConfig abstract_config;
  abstract_config.recovery_enabled = config.recovery_enabled;
  abstract_config.recovery_patience = config.recovery_patience;
  return abstract_config;
}

} /* namespace mbf_gridmap_nav */
