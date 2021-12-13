/**
 * @file CanCharacterizerNode.cpp
 * @author Aditya Jadhav
 * @brief The node used for inspection and localization of the Can.
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

#include "ros/ros.h"
#include "enpm808x_final_inspection_robot/CanCharacterizer.hpp"
#include <iostream>

/**
 * @brief Inspector Node overseeing the Can Detection and Localization
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "can_characterizer");
  ros::NodeHandle nh;
  ros::ServiceServer inspect_can_srv;
  CanCharacterizer inspect;
  inspect_can_srv = nh.advertiseService("inspect_can",
                          &CanCharacterizer::handleInspectCanRequest, &inspect);
  ROS_INFO("Inspect can Service Initialized ");
  ros::spin();
  return 0;
}
