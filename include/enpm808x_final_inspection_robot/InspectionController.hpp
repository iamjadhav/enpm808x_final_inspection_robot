/**
 * @file InspectionController.hpp
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

#pragma once

#include <ros/service_client.h>
#include <ros/subscriber.h>
#include <actionlib/client/simple_action_client.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>

#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>

#include <memory>
#include <vector>

// Definitions to make common names shorter
template <typename T> using uptr = std::unique_ptr<T>;
template <typename T> using ActionClient = actionlib::SimpleActionClient<T>;

/**
 * The main controller for the inspection process pipeline, which helps to
 * communicate objectives and status on the ROS network.
 */
class InspectionController {
 private:
    /**
     * Subscriber for results of arm trajectory actions. The TIAGo's arm is
     * first requested to tuck in on start, so this listens for that process to
     * complete.
     */
    uptr<ros::Subscriber> arm_trajectory_result_sub;

    /**
     * Client to the action that handles requests to move the base of the TIAGo
     * robot to an objective pose.
     */
    uptr<ActionClient<move_base_msgs::MoveBaseAction>> move_base_cli;

    /**
     * Subscriber for results of move base actions. This describes whether or
     * not objective poses were reached.
     */
    uptr<ros::Subscriber> move_base_result_sub;

    /**
     * Client to the service which uses a given image to find a can and
     * determine whether or not it is defective.
     */
    uptr<ros::ServiceClient> inspect_can_cli;

    /**
     * Client to the service which uses a given depth field to get the position
     * of a can relative to the sensor that the depth field was captured by.
     */
    uptr<ros::ServiceClient> localize_can_cli;

    /**
     * Subscriber for RBG image updates. The most recent update is captured
     * from this subscription into @a last_rgb_image.
     */
    uptr<ros::Subscriber> rgb_image_sub;

    /**
     * Subscriber for depth field updates. The most recent update is captured
     * from this subscription into @a last_point_cloud.
     */
    uptr<ros::Subscriber> point_cloud_sub;

    /**
     * The TIAGo robot's home position.
     */
    const geometry_msgs::Pose home_position;

    /**
     * The offset that the TIAGo robot should have from a can for inspection.
     */
    const tf::Transform detection_pose_offset;

    /**
     * The approximate expected positions of all cans that should be inspected.
     */
    std::vector<tf::Vector3> expected_can_positions;

    /**
     * Whether or not the TIAGo has finished tucking its arm in.
     */
    bool arm_tucked;

    /**
     * Helper function to package an objective pose into a goal for the move
     * base action server.
     * @param pose The objective pose for the TIAGo robot.
     */
    void requestMoveBaseActionGoal(const geometry_msgs::Pose& pose);

 public:
    /**
     * Sole constructor.
     * @param home_position The TIAGo robot's home position.
     * @param detection_pose_offset The constant offset for the detection pose.
     */
    InspectionController(const geometry_msgs::Pose& home_position,
                         const tf::Transform& detection_pose_offset);

    /**
     * Destructor.
     */
    ~InspectionController();

    /**
     * Callback for the TIAGo's arm trajectory action's results.
     * @param msg The result of the requested arm trajectory action.
     */
    void handleArmTrajectoryResult(
        const control_msgs::FollowJointTrajectoryActionResultConstPtr& msg);

    /**
     * Callback for the TIAGo's arm trajectory action's results.
     * @param msg The result of the requested arm trajectory action.
     */
    void handleMoveBaseResult(
        const move_base_msgs::MoveBaseActionResultConstPtr& msg);

    /**
     * Callback for an updated RGB image captured by the TIAGo robot.
     * @param msg The updated RGB image.
     */
    void handleRgbImageUpdate(const sensor_msgs::ImageConstPtr& msg);

    /**
     * Callback for an updated depth field captured by the TIAGo robot.
     * @param msg The updated depth field.
     */
    void handlePointCloudUpdate(const sensor_msgs::PointCloud2ConstPtr& msg);

    /**
     * Getter for whether the TIAGo arm has been tucked in yet or not.
     * @return Whether or not the TIAGo has finished tucking its arm in.
     */
    bool isArmTucked() const;
};
