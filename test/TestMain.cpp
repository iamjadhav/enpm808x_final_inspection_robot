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
#include<ros/service_client.h>
#include <memory>
#include"enpm808x_final_inspection_robot/InspectCan.h"

std::unique_ptr<ros::NodeHandle> nh;

TEST(ServiceTest, ServiceExistence) {
  //ros::NodeHandle n;
  auto client = nh->serviceClient<enpm808x_final_inspection_robot::InspectCan>
  ("inspectcan");
  bool exists(client.waitForExistence(ros::Duration(5)));
  EXPECT_FALSE(exists);
  enpm808x_final_inspection_robot::InspectCan srv;
}


TEST(TESTSuite, testMain) {
    const bool dummy = true;
    EXPECT_TRUE(dummy);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_main");
    nh.reset(new ros::NodeHandle);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
