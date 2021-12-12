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

#include <std_msgs/Empty.h>

#include "enpm808x_final_inspection_robot/InspectionController.hpp"
#include "enpm808x_final_inspection_robot/InspectionMetrics.h"
#include "enpm808x_final_inspection_robot/InspectCan.h"
#include "enpm808x_final_inspection_robot/LocalizeCan.h"

#define STREAM_TF(title, t) \
    ROS_INFO_STREAM(title << ": [" \
                          << t.getOrigin().getX() << "," \
                          << t.getOrigin().getY() << "," \
                          << t.getOrigin().getZ() << "], [" \
                          << t.getRotation().getX() << "," \
                          << t.getRotation().getY() << "," \
                          << t.getRotation().getZ() << "," \
                          << t.getRotation().getW() << "]")

// Help shorten typenames via namespace alias
namespace enpm808x = enpm808x_final_inspection_robot;

namespace {
    void resetMetrics(enpm808x::InspectionMetrics* im) {
        static const geometry_msgs::Pose identity;
        im->move_sucessful = false;
        im->can_detection_sucessful = false;
        im->can_nominal = false;
        im->transform_WC_measured = identity;
        im->transform_WDi_expected = identity;
        im->transform_WDi_measured = identity;
    }
}

void InspectionController::requestMoveBaseActionGoal(
        const geometry_msgs::Pose& pose) {
    // Populate the action goal given the target pose
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose = pose;
    // Send the goal, using the preallocated result callback
    move_base_cli->sendGoal(goal, move_base_result_cb);
}

void InspectionController::requestMoveBaseActionGoalFromCanPosition(
        const tf::Vector3& position_WDi) {
    // Calculate the robot's desired base position relative to the world frame:
    // T_W0' = inv(T_0'C * T_CDi * inv(T_WDi={I,P_WDi}))
    const tf::Transform transform_0primeC = transform_0C;  // Rename
    const tf::Transform transform_CDi(tf::Quaternion::getIdentity(),
                                      detection_pose_offset.getOrigin());
    const tf::Transform transform_WDi(tf::Quaternion::getIdentity(),
                                      position_WDi);
    const tf::Transform transform_W0_desired_prime = (
        transform_0primeC * transform_CDi * transform_WDi.inverse()
    ).inverse();
    const tf::Transform transform_W0_desired(
        detection_pose_offset.getRotation(),
        tf::Vector3(
            transform_W0_desired_prime.getOrigin().getX(),
            transform_W0_desired_prime.getOrigin().getY(),
            0.0));
    // Now convert to a pose and send
    STREAM_TF(" - tf_0'C", transform_0primeC);
    STREAM_TF(" - tf_CDi", transform_CDi);
    STREAM_TF(" - tf_WDi", transform_WDi);
    STREAM_TF(" - tf_DiW", transform_WDi.inverse());
    STREAM_TF(" - tf_W0_desired_prime", transform_W0_desired_prime);
    STREAM_TF(" - tf_W0_desired", transform_W0_desired);
    const tf::Vector3 t = transform_W0_desired.getOrigin();
    const tf::Quaternion q = transform_W0_desired.getRotation();
    geometry_msgs::Pose pose;
    pose.position.x = t.getX();
    pose.position.y = t.getY();
    pose.position.z = t.getZ();
    pose.orientation.x = q.getX();
    pose.orientation.y = q.getY();
    pose.orientation.z = q.getZ();
    pose.orientation.w = q.getW();
    current_metrics.transform_WDi_expected = pose;
    requestMoveBaseActionGoal(pose);
}

void InspectionController::finishPipelineIteration() {
    // Publish the metrics of this iteration then reset the container
    inspection_metrics_pub->publish(current_metrics);
    resetMetrics(&current_metrics);
    // Is the controller done?
    if (is_going_home) {
        // The TIAGo just got to its home position
        inspection_finished_pub->publish(std_msgs::Empty());
    } else {
        // The goal to this motion wasn't the home position
        expected_can_positions.pop();
        if (expected_can_positions.empty()) {
            // That was the last inspection station, time to go home
            is_going_home = true;
            requestMoveBaseActionGoal(home_position);
        } else {
            // There are more cans, move on to the next
            requestMoveBaseActionGoalFromCanPosition(
                expected_can_positions.front());
        }
    }
}

InspectionController::InspectionController(
        ros::NodeHandle& nh,
        const geometry_msgs::Pose& home_position,
        const tf::Transform& detection_pose_offset) :
            // Create all the members for communication over the ROS network
            arm_trajectory_result_sub(new ros::Subscriber(
                nh.subscribe(
                    "/arm_controller/follow_joint_trajectory/result",
                    1,
                    &InspectionController::handleArmTrajectoryResult,
                    this))),
            move_base_cli(new MoveBaseActionCli(
                nh,
                "/move_base",
                false)),
            inspect_can_cli(new ros::ServiceClient(
                nh.serviceClient<enpm808x::InspectCan>("inspect_can"))),
            localize_can_cli(new ros::ServiceClient(
                nh.serviceClient<enpm808x::LocalizeCan>("localize_can"))),
            rgb_image_sub(new ros::Subscriber(
                nh.subscribe(
                    "/xtion/rgb/image_raw",
                    1,
                    &InspectionController::handleRgbImageUpdate,
                    this))),
            point_cloud_sub(new ros::Subscriber(
                nh.subscribe(
                    "/xtion/depth_registered/points",
                    1,
                    &InspectionController::handlePointCloudUpdate,
                    this))),
            inspection_metrics_pub(new ros::Publisher(
                nh.advertise<enpm808x::InspectionMetrics>(
                    "inspection_metrics",
                    1))),
            inspection_finished_pub(new ros::Publisher(
                nh.advertise<std_msgs::Empty>(
                    "inspection_finished",
                    1))),
            // Create the rest of the values
            tf_listener(new tf::TransformListener),
            home_position(home_position),
            detection_pose_offset(detection_pose_offset),
            move_base_result_cb(boost::bind(
                &InspectionController::handleMoveBaseResult,
                this,
                _1,
                _2)),
            arm_tucked(false),
            is_going_home(false) {
    resetMetrics(&current_metrics);
    // Wait until we have access to a required (static) transform
    const std::string t2 = "/base_footprint", t1 = "/xtion_rgb_optical_frame";
    ROS_INFO_STREAM("Waiting for transform_0C = " << t2 << " -> " << t1);
    if (tf_listener->waitForTransform(t2,
                                      t1,
                                      ros::Time(0),
                                      ros::Duration(120))) {
        ROS_INFO_STREAM("Successfully populated transform_0C!");
    }
    tf_listener->lookupTransform(t2,
                                 t1,
                                 ros::Time(0),
                                 transform_0C);
}

InspectionController::~InspectionController() {
}

void InspectionController::handleArmTrajectoryResult(
        const control_msgs::FollowJointTrajectoryActionResultConstPtr& msg) {
    // We only expect this callback once, it determines whether the arm will
    // ever be tucked successfully
    const int ec = msg->result.error_code;
    arm_tucked = (ec == control_msgs::FollowJointTrajectoryResult::SUCCESSFUL);
    if (arm_tucked) {
        ROS_INFO_STREAM("Arm is tucked.");
    } else {
        ROS_ERROR_STREAM("Arm failed to tuck: " << msg->result.error_string);
    }
}

void InspectionController::handleMoveBaseResult(
        const actionlib::SimpleClientGoalState& state,
        const move_base_msgs::MoveBaseResultConstPtr& msg) {
    // Handle any move result
    (void)msg;
    bool continue_inspection =
        state.state_ == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED;
    current_metrics.move_sucessful = continue_inspection;
    finishPipelineIteration();
}

void InspectionController::handleRgbImageUpdate(
        const sensor_msgs::ImageConstPtr& msg) {
    last_rgb_image = *msg;
}

void InspectionController::handlePointCloudUpdate(
        const sensor_msgs::PointCloud2ConstPtr& msg) {
    last_point_cloud = *msg;
}

void InspectionController::inspect(
        const std::vector<tf::Vector3>& new_expected_can_positions) {
    // Make sure our queue is empty, then add each element
    // There's no range-based algorithm, so insert each manually
    while (!expected_can_positions.empty()) {
        expected_can_positions.pop();
    }
    for (auto iter = new_expected_can_positions.begin();
         iter != new_expected_can_positions.end();
         ++iter) {
        expected_can_positions.push(*iter);
    }
    // Now kickstart the first objective pose, ensuring we were given any
    if (!new_expected_can_positions.empty()) {
        requestMoveBaseActionGoalFromCanPosition(
            expected_can_positions.front());
    }
}

bool InspectionController::isArmTucked() const {
    return arm_tucked;
}
