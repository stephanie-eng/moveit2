/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <pilz_industrial_motion_planner/joint_limits_aggregator.hpp>
#include <pilz_industrial_motion_planner/joint_limits_interface_extension.hpp>

#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/utils/logger.hpp>

#include <vector>
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.planners.pilz.joint_limits_aggregator");
}
}  // namespace

pilz_industrial_motion_planner::JointLimitsContainer
pilz_industrial_motion_planner::JointLimitsAggregator::getAggregatedLimits(
    const rclcpp::Node::SharedPtr& node, const std::string& param_namespace,
    const std::vector<const moveit::core::JointModel*>& joint_models)
{
  JointLimitsContainer container;

  RCLCPP_INFO_STREAM(getLogger(), "Reading limits from namespace " << param_namespace);

  // Iterate over all joint models and generate the map
  for (auto joint_model : joint_models)
  {
    for (const auto& variable_name : joint_model->getVariableNames())
    {
      pilz_industrial_motion_planner::JointLimit joint_limit;

      // NOTE: declareParameters fails (=returns false) if the parameters have already been declared.
      // The function should be checked in the if condition below when we disable
      // 'NodeOptions::automatically_declare_parameters_from_overrides(true)'
      joint_limits_interface::declareParameters(variable_name, param_namespace, node);

      // If there is something defined for the joint in the node parameters
      if (joint_limits_interface::getJointLimits(variable_name, param_namespace, node, joint_limit))
      {
        if (joint_limit.has_position_limits)
        {
          checkPositionBoundsThrowing(joint_model, variable_name, joint_limit);
        }
        else
        {
          updatePositionLimitFromJointModel(joint_model, variable_name, joint_limit);
        }

        if (joint_limit.has_velocity_limits)
        {
          checkVelocityBoundsThrowing(joint_model, variable_name, joint_limit);
        }
        else
        {
          updateVelocityLimitFromJointModel(joint_model, variable_name, joint_limit);
        }
      }
      else
      {
        // If there is nothing defined for this joint in the node parameters just
        // update the values by the values of
        // the urdf

        updatePositionLimitFromJointModel(joint_model, variable_name, joint_limit);
        updateVelocityLimitFromJointModel(joint_model, variable_name, joint_limit);
      }

      // Update max_deceleration if no max_acceleration has been set
      if (joint_limit.has_acceleration_limits && !joint_limit.has_deceleration_limits)
      {
        joint_limit.max_deceleration = -joint_limit.max_acceleration;
        joint_limit.has_deceleration_limits = true;
      }

      // Insert the joint limit into the map
      container.addLimit(variable_name, joint_limit);
    }
  }

  return container;
}

void pilz_industrial_motion_planner::JointLimitsAggregator::updatePositionLimitFromJointModel(
    const moveit::core::JointModel* joint_model, const std::string& variable_name, JointLimit& joint_limit)
{
  const auto& bounds = joint_model->getVariableBounds(variable_name);
  joint_limit.has_position_limits = bounds.position_bounded_;
  joint_limit.min_position = bounds.min_position_;
  joint_limit.max_position = bounds.max_position_;

  RCLCPP_DEBUG_STREAM(getLogger(), "Limit(" << variable_name << " min:" << joint_limit.min_position
                                            << " max:" << joint_limit.max_position);
}

void pilz_industrial_motion_planner::JointLimitsAggregator::updateVelocityLimitFromJointModel(
    const moveit::core::JointModel* joint_model, const std::string& variable_name, JointLimit& joint_limit)
{
  const auto& bounds = joint_model->getVariableBounds(variable_name);
  joint_limit.has_velocity_limits = bounds.velocity_bounded_;
  joint_limit.max_velocity = bounds.max_velocity_;
}

void pilz_industrial_motion_planner::JointLimitsAggregator::checkPositionBoundsThrowing(
    const moveit::core::JointModel* joint_model, const std::string& variable_name, const JointLimit& joint_limit)
{
  const auto& bounds = joint_model->getVariableBounds(variable_name);

  // Check min position
  if (bounds.position_bounded_ && joint_limit.min_position < bounds.min_position_)
  {
    throw AggregationBoundsViolationException("min_position of " + variable_name + " violates min limit from URDF");
  }

  // Check max position
  if (bounds.position_bounded_ && joint_limit.max_position > bounds.max_position_)
  {
    throw AggregationBoundsViolationException("max_position of " + variable_name + " violates max limit from URDF");
  }
}

void pilz_industrial_motion_planner::JointLimitsAggregator::checkVelocityBoundsThrowing(
    const moveit::core::JointModel* joint_model, const std::string& variable_name, const JointLimit& joint_limit)
{
  const auto& bounds = joint_model->getVariableBounds(variable_name);

  // Check velocity
  if (bounds.velocity_bounded_ && joint_limit.max_velocity > bounds.max_velocity_)
  {
    throw AggregationBoundsViolationException("max_velocity of " + variable_name + " violates velocity limit from URDF");
  }
}
