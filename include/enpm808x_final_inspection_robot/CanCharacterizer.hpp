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
  // ros::ServiceServer inspect_can_srv;
  // The Server for the LocalizeCan Service
  ros::ServiceServer localize_can_srv;

 public:
/**
* @brief  Constructor for Can Characterizer class
*/
  CanCharacterizer();
/**
* @brief Destructor
*/
  ~CanCharacterizer();
/**
* @brief Method which handles the Inspection of the Can using InspectCan
* @param req The rgb_image obtained from sensor_msgs topic
* @param res Booleans success, nominal and the can's centroid coordinates
* @return Whether or not the request was successfully serviced.
*/
  bool handleInspectCanRequest(
    enpm808x_final_inspection_robot::InspectCan::InspectCan::Request &req,
    enpm808x_final_inspection_robot::InspectCan::InspectCan::Response &res);
/**
* @brief Method which handles the Localization of the Can using LocalizeCan
* service request and response.
* @param req Point cloud data and can's centroid coordinates
* @param res Boolean success and the Can's transform wrt world frame
* @return Whether or not the request was successfully serviced.
*/
  bool handleLocalizeCanRequest(
    enpm808x_final_inspection_robot::LocalizeCan::LocalizeCan::Request &req,
    enpm808x_final_inspection_robot::LocalizeCan::LocalizeCan::Response &res);
};
