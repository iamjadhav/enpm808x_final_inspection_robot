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
using Enpm808xIm = enpm808x_final_inspection_robot::InspectionMetrics;
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

void baseCallbackGood(
        const move_base_msgs::MoveBaseGoalConstPtr& goal,
        ActionSrv<move_base_msgs::MoveBaseAction>* sas) {
    static const ros::Duration sleep_duration(0.5);
    sleep_duration.sleep();
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
    arm_traj_action_srv.shutdown(){ \
    // Create scoped tests to help destroy the objects

        // The arm should NOT be marked as 'tucked' after this tests' callback
        DO_ARM_TEST(armCallbackBad);
    }
    {
        // The arm should be marked as 'tucked' after this tests' callback
        DO_ARM_TEST(armCallbackGood);
    }

#undef DO_ARM_TEST
}

TEST(TESTSuite, testInspect) {
    // The values of these don't matter
    const geometry_msgs::Pose hp;
    const tf::Transform dpo;
    // Set constants
    const double timeout = 5.0;
    const ros::Duration sleep_duration(0.1);

    ActionSrv<move_base_msgs::MoveBaseAction> move_base_action_srv(
        *nh,
        "/move_base",
        boost::bind(&baseCallbackGood, _1, &move_base_action_srv),
        true);
    ActionCli<move_base_msgs::MoveBaseAction> move_base_action_cli(
        *nh,
        "/move_base",
        false);

#define DO_INSPECT_TEST(positions) \
    InspectionController cont(*nh, hp, dpo); \
    bool finished = false; \
    std::vector<Enpm808xIm> results; \
    ros::Subscriber inspection_metrics_sub = nh->subscribe<Enpm808xIm>( \
        "inspection_metrics", \
        1000, \
        [&](const Enpm808xIm::ConstPtr& msg) { \
            results.push_back(*msg); \
        }); \
    ros::Subscriber finished_sub = nh->subscribe<std_msgs::Empty>( \
        "inspection_finished", \
        1, \
        [&](const std_msgs::EmptyConstPtr& msg) { \
            (void)msg; \
            finished = true; \
        }); \
    const std::vector<tf::Vector3> can_positions = positions; \
    cont.inspect(can_positions); \
    const ros::Time start = ros::Time::now(); \
    while (!finished && (timeout > (ros::Time::now() - start).toSec())) { \
        sleep_duration.sleep(); \
        ros::spinOnce(); \
    } \
    EXPECT_TRUE(finished); \
    ASSERT_EQ(results.size(), (can_positions.size() + 1)); \
    int idx = 0; \
    for (auto iter = results.begin(); iter != results.end(); ++iter) { \
        const geometry_msgs::Pose result = iter->transform_WDi_expected; \
        const tf::Vector3 position = can_positions[idx++]; \
        EXPECT_NEAR(result.position.x, position.getX(), 0.000001); \
        EXPECT_NEAR(result.position.y, position.getY(), 0.000001); \
        EXPECT_NEAR(result.position.z, 0.0, 0.000001); \
    } \
    inspection_metrics_sub.shutdown(); \
    finished_sub.shutdown(){
    // Create scoped tests to help destroy the objects with these random values
    // (the exact values shouldn't matter, just that the inputs and outputs
    // agree)

        DO_INSPECT_TEST(std::vector<tf::Vector3>({
            tf::Vector3(1.0, 2.0, 3.0)
        }));
    }
    {
        DO_INSPECT_TEST(std::vector<tf::Vector3>({
            tf::Vector3(1.0, 2.0, 3.0),
            tf::Vector3(-3.0, -2.0, -1.0),
            tf::Vector3(0.0, 0.0, 0.0),
            tf::Vector3(1.2, 3.4, 5.6)
        }));
    }
    {
        DO_INSPECT_TEST(std::vector<tf::Vector3>({
            tf::Vector3(42.42, 24.24, -42.24),
            tf::Vector3(-6.7, 0.0, 0.0),
            tf::Vector3(1.1, 2.3, 5.8),
            tf::Vector3(-1.1, 3.3, 5.5),
            tf::Vector3(100.1, -99.9, 6.66),
            tf::Vector3(33.3, 6.67, 100.0),
            tf::Vector3(-3.14, 6.022, 6.634)
        }));
    }

#undef DO_INSPECT_TEST
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_inspection_controller");
    ros::start();
    nh.reset(new ros::NodeHandle("/enpm808x"));
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
