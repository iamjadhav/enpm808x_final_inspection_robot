/**
 * @file CanCharacterizer.cpp
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

#include "enpm808x_final_inspection_robot/CanCharacterizer.hpp"

CanCharacterizer::CanCharacterizer() {
    // Initializing...
    ROS_INFO_STREAM("Can Inspection Initiated...");
}

CanCharacterizer::~CanCharacterizer() {
    // Finishing...
    ROS_INFO_STREAM("All done, Moving On...");
}

bool CanCharacterizer::handleInspectCanRequest(
    enpm808x_final_inspection_robot::InspectCan::InspectCan::Request &req,
    enpm808x_final_inspection_robot::InspectCan::InspectCan::Response &res) {
  cv_bridge::CvImagePtr convert_cv;
  // source image, hsv image and masked image
  cv::Mat cvImage, hsvFrame, maskedFrame;
  // HSV Lower Limit blue
  const cv::Scalar hsvLower = {115, 36, 0};
  // HSV Higher Limit blue
  const cv::Scalar hsvHigher = {165, 255, 255};

  // // HSV Lower Limit red
  // const cv::Scalar hsvLower = {0, 230, 0};
  // // HSV Higher Limit red
  // const cv::Scalar hsvHigher = {37, 255, 255};

  // converting ros image to an opencv image with cv bridge
  try {
  convert_cv = cv_bridge::toCvCopy(req.rgb_image,
                                          sensor_msgs::image_encodings::BGR8);
  cvImage = convert_cv->image;
  cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e) {
  ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
  }
  // converting to HSV image and thresholding to get masked image
  cv::cvtColor(cvImage, hsvFrame, CV_BGR2HSV);
  cv::inRange(hsvFrame, hsvLower, hsvHigher, maskedFrame);
  // calculating the centroid
  cv::Moments m = cv::moments(maskedFrame, true);
  cv::Point p(m.m10/m.m00, m.m01/m.m00);
  // assigning and converting centroid coordinates
  res.centroid_x = static_cast<std::int64_t>(p.x);
  res.centroid_y = static_cast<std::int64_t>(p.y);
  if (res.centroid_x && res.centroid_y) {
  res.success = true;
  res.nominal = false;
  } else {
  res.success = false;
  res.nominal = true;
  }
  return true;
}


bool CanCharacterizer::handleLocalizeCanRequest(
    enpm808x_final_inspection_robot::LocalizeCan::LocalizeCan::Request &req,
    enpm808x_final_inspection_robot::LocalizeCan::LocalizeCan::Response &res) {
return true;
}
