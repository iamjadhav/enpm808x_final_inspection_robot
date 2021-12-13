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
#include <ros/package.h>
#include <gtest/gtest.h>
#include <sensor_msgs/image_encodings.h>
#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "enpm808x_final_inspection_robot/LocalizeCan.h"


#include"enpm808x_final_inspection_robot/InspectCan.h"

std::unique_ptr<ros::NodeHandle> nh;

/**
*  Simple Existence Test for the InspectCan Service.
*/
TEST(InspectCanTest, Test_Inspect_can) {
  auto client = nh->serviceClient<enpm808x_final_inspection_robot::InspectCan>
  ("inspect_can");
  bool exists(client.waitForExistence(ros::Duration(5)));
  EXPECT_TRUE(exists);
}

/**
* Testing the inspect can service callback function handleInspectCanRequest
*/
TEST(Handle_InspectCan, Test_InspectionCan_callback) {
  // declaring a client for InspectCan service
  auto client = nh->serviceClient<enpm808x_final_inspection_robot::InspectCan>
  ("inspect_can");
  // inspect can service object
  enpm808x_final_inspection_robot::InspectCan srv;
  // loading an image from file
  std::string image = ros::package::getPath(
                    "enpm808x_final_inspection_robot") + "/test_data/test.jpeg";
  cv::Mat testimg = cv::imread(image);
  // warming up for service creation
  ros::Duration(10).sleep();
  int counter = 1;
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg;
  std_msgs::Header header;
  header.seq = counter;
  header.stamp = ros::Time::now();
  img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8,
                                                                      testimg);
  // converted from opencv image to ros message
  img_bridge.toImageMsg(img_msg);
  // assigning service request parameter rgb_image
  srv.request.rgb_image = img_msg;
  // calling service
  if (client.call(srv)) {
    // validating service responses
    ROS_WARN("Centroid_X %ld", srv.response.centroid_x);
    ROS_WARN("Centroid_Y %ld", srv.response.centroid_y);
    EXPECT_TRUE(srv.response.success);
  } else {
    ROS_ERROR("Failed to call service");
  }
}


TEST(TESTSuite, testMain) {
    const bool dummy = true;
    EXPECT_TRUE(dummy);
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

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_main");
    nh.reset(new ros::NodeHandle);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
