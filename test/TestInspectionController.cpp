/**
 * @file TestInspectionController.cpp
 * @author Robert Vandemark
 * @brief A testing suite dedicated to unit tests of the inspection controller.
 * @version 0.1
 * @date 2021-12-13
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

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <actionlib/server/simple_action_server.h>
#include <std_msgs/Empty.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <memory>

#include "enpm808x_final_inspection_robot/InspectionController.hpp"
#include "enpm808x_final_inspection_robot/InspectionMetrics.h"

// Help shorten typename
template <typename T> using ActionSrv = actionlib::SimpleActionServer<T>;
template <typename T> using ActionCli = actionlib::SimpleActionClient<T>;

void armCallbackBad(
        const control_msgs::FollowJointTrajectoryGoalConstPtr& goal,
        ActionSrv<control_msgs::FollowJointTrajectoryAction>* sas) {
    sas->setAborted();
}
void armCallbackGood(
        const control_msgs::FollowJointTrajectoryGoalConstPtr& goal,
        ActionSrv<control_msgs::FollowJointTrajectoryAction>* sas) {
    sas->setSucceeded();
}

std::unique_ptr<ros::NodeHandle> nh;

TEST(TESTSuite, testArmTrajectoryResult) {
    // The values of these don't matter
    const geometry_msgs::Pose hp;
    const tf::Transform dpo;
    const control_msgs::FollowJointTrajectoryGoal goal;
    // Set a somewhat small sleep rate
    const ros::Duration sleep_duration(0.1);

#define DO_ARM_TEST(cb) \
    ActionSrv<control_msgs::FollowJointTrajectoryAction> \
        arm_traj_action_srv(*nh, \
                            "/arm_controller/follow_joint_trajectory", \
                            boost::bind(&cb, \
                                        _1, \
                                        &arm_traj_action_srv), \
                            true); \
    ActionCli<control_msgs::FollowJointTrajectoryAction> \
        arm_traj_action_cli(*nh, \
                            "/arm_controller/follow_joint_trajectory", \
                            false); \
    ros::Duration(2).sleep(); \
    InspectionController cont(*nh, hp, dpo); \
    arm_traj_action_cli.sendGoal(goal); \
    while (!arm_traj_action_cli.waitForResult(sleep_duration)) { \
        ros::spinOnce(); \
    } \
    EXPECT_TRUE(cont.isArmTucked()); \
    arm_traj_action_cli.stopTrackingGoal(); \
    arm_traj_action_srv.shutdown() \

    // Create scoped tests to help destroy the objects
    {
        // The arm should NOT be marked as 'tucked' after this tests' callback
        DO_ARM_TEST(armCallbackBad);
    }
    {
        // The arm should be marked as 'tucked' after this tests' callback
        DO_ARM_TEST(armCallbackGood);
    }

#undef DO_ARM_TEST
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_inspection_controller");
    ros::start();
    nh.reset(new ros::NodeHandle("/enpm808x"));
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
