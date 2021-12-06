/**
 * @file CanCharacterizer.hpp
 * @author Aditya Jadhav
 * @brief The Class which handles the inspection (Detection) and localization
 * of the Cans present in front of TIAGo in each iteration.
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

#include <sensor_msgs/Image.h>
#include <vector>
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "enpm808x_final_inspection_robot/InspectCan.h"
#include "enpm808x_final_inspection_robot/LocalizeCan.h"


/**
* @brief  The class concerned with the Detection and Localization of the can
* in front of the TIAGo using two services.
*/
class CanCharacterizer {
 private:
  // The Server for the InspectCan Service
  ros::ServiceServer inspect_can_srv;
  // The Server for the LocalizeCan Service
  ros::ServiceServer localize_can_srv;
  // Shape of the OpenCV Frame having Width and Height
  cv::Size frameShape;
  // Maksed Frame which has the masked output of RED HSV values
  cv::Mat maskedFrame;
  // OpenCV HSV frame converted from the RGB frame
  cv::Mat hsvFrame;
  // Raw ROS frame from the RGBD sensor onboard the TIAGo
  cv::Mat rosRawFrame;
  // Storing Contours obtained by OpenCV
  std::vector<std::vector<cv::Point>> contours;
  // HSV Lower Limit
  const cv::Scalar hsvLower = {160, 50, 50};
  // HSV Higher Limit
  const cv::Scalar hsvHigher = {180, 255, 255};
  // Final boolean indication of the success of inspection and localization
  bool detect;

 public:
/**
* @brief  Constructor for Can Characterizer class
* @param  none
* @return none
*/
  CanCharacterizer();
/**
* @brief Destructor
*/
  ~CanCharacterizer();
  // OpenCV Image converted from the raw ROS image
  cv::Mat ros_to_cv_image;
/**
* @brief Method which handles the Inspection of the Can using InspectCan
* @param inspectReq The rgb_image obtained from sensor_msgs topic
* @param inspectRes Booleans success, nominal and the can's centroid coordinates
* @return none
*/
  bool handleInspectCanRequest(enpm808x_final_inspection_robot::InspectCan::
      InspectCanRequest &req, enpm808x_final_inspection_robot::InspectCan::
          InspectCanResponse &res);
/**
* @brief Method which handles the Localization of the Can using LocalizeCan
* service request and response.
* @param localizeReq Point cloud data and can's centroid coordinates
* @param localizeRes Boolean success and the Can's transform wrt world frame
* @return none
*/
  bool handleLocalizeCanRequest(enpm808x_final_inspection_robot::LocalizeCan::
      LocalizeCanRequest &req, enpm808x_final_inspection_robot::LocalizeCan::
          LocalizeCanResponse &res);
};
