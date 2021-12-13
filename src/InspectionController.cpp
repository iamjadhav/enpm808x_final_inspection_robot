/**
 * @file InspectionController.cpp
 * @author Robert Vandemark
 * @brief The main controller for the inspection process pipeline, which
 * helps to communicate objectives and status on the ROS network.
 * @version 0.1
 * @date 2021-12-05
 *
 * @copyright Copyright 2021 Robert Vandemark, Aditya Jadhav, Abhishek Nalawade
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "enpm808x_final_inspection_robot/InspectionController.hpp"

void InspectionController::requestMoveBaseActionGoal(
        const geometry_msgs::Pose& pose) {
    (void)pose;
}

void InspectionController::finishPipelineIteration() {
}

InspectionController::InspectionController(
        ros::NodeHandle& nh,
        const geometry_msgs::Pose& home_position,
        const tf::Transform& detection_pose_offset) {
    (void)nh;
    (void)home_position;
    (void)detection_pose_offset;
}

InspectionController::~InspectionController() {
}

void InspectionController::handleArmTrajectoryResult(
        const control_msgs::FollowJointTrajectoryActionResultConstPtr& msg) {
    (void)msg;
}

void InspectionController::handleMoveBaseResult(
        const actionlib::SimpleClientGoalState& state,
        const move_base_msgs::MoveBaseResultConstPtr& msg) {
    (void)state;
    (void)msg;
}

void InspectionController::handleRgbImageUpdate(
        const sensor_msgs::ImageConstPtr& msg) {
    (void)msg;
}

void InspectionController::handlePointCloudUpdate(
        const sensor_msgs::PointCloud2ConstPtr& msg) {
    (void)msg;
}

void InspectionController::inspect(
        const std::vector<tf::Vector3>& new_expected_can_positions) {
    (void)new_expected_can_positions;
}

bool InspectionController::isArmTucked() const {
    return false;
}
