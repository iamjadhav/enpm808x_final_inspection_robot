/**
 * @file TestMain.cpp
 * @author Robert Vandemark
 * @brief The testing suite's entry point.
 * @version 0.1
 * @date 2021-12-06
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

#include <std_msgs/Empty.h>

#include <memory>

#include "enpm808x_final_inspection_robot/LocalizeCan.h"
#include "enpm808x_final_inspection_robot/InspectCan.h"
#include "enpm808x_final_inspection_robot/InspectionMetrics.h"

/**
 * The macros below are inspired by test/TestHelpers.hpp in:
 * https://github.com/rnvandemark/enpm808x_midterm_path_planner
 *
 * The value of 'prefix' in each can be either 'EXPECT' or 'ASSERT', to expand
 * to 'EXPECT_NEAR' or 'ASSERT_NEAR'.
 */

#define POPULATE_POSE(xxx, px, py, pz, ox, oy, oz, ow) \
    xxx.position.x = px; \
    xxx.position.y = py; \
    xxx.position.z = pz; \
    xxx.orientation.x = ox; \
    xxx.orientation.y = oy; \
    xxx.orientation.z = oz; \
    xxx.orientation.w = ow

#define DEFINE_POSE(xxx, px, py, pz, ox, oy, oz, ow) \
    geometry_msgs::Pose xxx; \
    POPULATE_POSE(xxx, px, py, pz, ox, oy, oz, ow)

#define MY_VALUE_NEAR(prefix, expected, actual, eps) \
    prefix##_NEAR(expected, actual, eps)

#define POSE_EQ(prefix, expected, actual, pos_eps, ori_eps) \
    { \
        const geometry_msgs::Point pos_expected = expected.position; \
        const geometry_msgs::Quaternion ori_expected = expected.orientation; \
        const geometry_msgs::Point pos_actual = actual.position; \
        const geometry_msgs::Quaternion ori_actual = actual.orientation; \
        MY_VALUE_NEAR(prefix, pos_expected.x, pos_actual.x, pose_eps); \
        MY_VALUE_NEAR(prefix, pos_expected.y, pos_actual.y, pose_eps); \
        MY_VALUE_NEAR(prefix, pos_expected.z, pos_actual.z, pose_eps); \
        MY_VALUE_NEAR(prefix, ori_expected.x, ori_actual.x, ori_eps); \
        MY_VALUE_NEAR(prefix, ori_expected.y, ori_actual.y, ori_eps); \
        MY_VALUE_NEAR(prefix, ori_expected.z, ori_actual.z, ori_eps); \
        MY_VALUE_NEAR(prefix, ori_expected.w, ori_actual.w, ori_eps); \
    }

#define INSPECTION_METRICS_FAILED_MOVE_EQ(prefix, \
                                          tf_WDi_exp_expected, \
                                          actual, \
                                          pos_eps, \
                                          ori_eps) \
    prefix##_FALSE(actual.move_successful); \
    POSE_EQ(prefix, \
            tf_WDi_exp_expected, \
            actual.transform_WDi_expected, \
            pos_eps, \
            ori_eps)

#define INSPECTION_METRICS_SUCCESSFUL_MOVE_EQ(prefix, \
                                              can_detected_expected, \
                                              tf_WC_meas_expected, \
                                              tf_WDi_exp_expected, \
                                              actual, \
                                              pos_eps, \
                                              ori_eps) \
    prefix##_TRUE(actual.move_successful); \
    prefix##_EQ(actual.move_successful, can_detected_expected); \
    POSE_EQ(prefix, \
            tf_WC_meas_expected, \
            actual.transform_WC_measured, \
            pos_eps, \
            ori_eps); \
    POSE_EQ(prefix, \
            tf_WDi_exp_expected, \
            actual.transform_WDi_expected, \
            pos_eps, \
            ori_eps)

#define INSPECTION_METRICS_NO_DETECTED_CAN_EQ(prefix, \
                                              tf_WC_meas_expected, \
                                              tf_WDi_exp_expected, \
                                              actual, \
                                              pos_eps, \
                                              ori_eps) \
    INSPECTION_METRICS_SUCCESSFUL_MOVE_EQ(prefix, \
                                          false, \
                                          tf_WC_meas_expected, \
                                          tf_WDi_exp_expected, \
                                          actual, \
                                          pos_eps, \
                                          ori_eps)

#define INSPECTION_METRICS_NOMINAL_CAN_EQ(prefix, \
                                          tf_WC_meas_expected, \
                                          tf_WDi_exp_expected, \
                                          actual, \
                                          pos_eps, \
                                          ori_eps) \
    INSPECTION_METRICS_SUCCESSFUL_MOVE_EQ(prefix, \
                                          true, \
                                          tf_WC_meas_expected, \
                                          tf_WDi_exp_expected, \
                                          actual, \
                                          pos_eps, \
                                          ori_eps)

#define INSPECTION_METRICS_DEFECTIVE_CAN_EQ(prefix, \
                                            tf_WC_meas_expected, \
                                            tf_WDi_exp_expected, \
                                            tf_WDi_meas_expected, \
                                            actual, \
                                            pos_eps, \
                                            ori_eps) \
    INSPECTION_METRICS_SUCCESSFUL_MOVE_EQ(prefix, \
                                          true, \
                                          tf_WC_meas_expected, \
                                          tf_WDi_exp_expected, \
                                          actual, \
                                          pos_eps, \
                                          ori_eps); \
    POSE_EQ(prefix, \
            tf_WDi_meas_expected, \
            actual.transform_WDi_measured, \
            pos_eps, \
            ori_eps)

// Help shorten typename
using Enpm808xIm = enpm808x_final_inspection_robot::InspectionMetrics;

std::unique_ptr<ros::NodeHandle> nh;

TEST(ServiceTest, ServiceExistence) {
  //ros::NodeHandle n;
  auto client = nh->serviceClient<enpm808x_final_inspection_robot::InspectCan>
  ("inspectcan");
  bool exists(client.waitForExistence(ros::Duration(5)));
  EXPECT_FALSE(exists);
  enpm808x_final_inspection_robot::InspectCan srv;
}

/**
*  Simple Existence Test for the LocalizeCan Service.
*/
TEST(LocalizeCanTest, Test_LocalizeCan_Existence) {
    // client for the LocalizeCan Service
    auto client = nh->serviceClient<enpm808x_final_inspection_robot::
                                LocalizeCan>("LocalizeCanExistence");

    // Test to validate service existence
    bool exists(client.waitForExistence(ros::Duration(2)));
    EXPECT_FALSE(exists);
}

TEST(TESTSuite, testFullPipeline1) {
    bool finished = false;
    std::vector<Enpm808xIm> results;
    ros::Subscriber inspection_metrics_sub = nh->subscribe<Enpm808xIm>(
        "inspection_metrics",
        1,
        [&](const Enpm808xIm::ConstPtr& msg) {
            results.push_back(*msg);
        });
    ros::Subscriber finished_sub = nh->subscribe<std_msgs::Empty>(
        "inspection_finished",
        1,
        [&](const std_msgs::EmptyConstPtr& msg) {
            (void)msg;
            finished = true;
        });

    // Loop until finished inspecting, given a VERY high timeout (this time is
    // the REAL time, not the time in the simulated environment, so this could
    // run slow on less powerful machines, hence extra time)
    const double timeout = 4000.0;
    const ros::Time start = ros::Time::now();
    const ros::Duration sleep_duration(0.1);
    while (!finished && (timeout > (ros::Time::now() - start).toSec())) {
        sleep_duration.sleep();
        ros::spinOnce();
    }

    // Make sure we finished and that our metrics match our expectations (to a
    // reasonable degree, when considering poses)
    EXPECT_TRUE(finished);
    ASSERT_EQ(results.size(), 6);

    // The last 'metric' is from going to the home position, separate it
    const Enpm808xIm home_result = results.back();
    results.pop_back();
    for (auto iter = results.begin(); iter != results.end(); ++iter) {
        EXPECT_TRUE(iter->move_sucessful);
    }

    // TODO: compare more results to the captured metrics once CanCharacterizer
    // and the rest of the pipeline is implemented
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_main");
    nh.reset(new ros::NodeHandle("/enpm808x"));
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#undef INSPECTION_METRICS_DEFECTIVE_CAN_EQ
#undef INSPECTION_METRICS_NOMINAL_CAN_EQ
#undef INSPECTION_METRICS_NO_DETECTED_CAN_EQ
#undef INSPECTION_METRICS_SUCCESSFUL_MOVE_EQ
#undef INSPECTION_METRICS_FAILED_MOVE_EQ
#undef POSE_EQ
#undef MY_VALUE_NEAR
#undef DEFINE_POSE
#undef POPULATE_POSE
