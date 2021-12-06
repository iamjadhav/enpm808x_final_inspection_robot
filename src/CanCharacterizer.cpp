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

#include "../include/CanCharacterizer.hpp"

CanCharacterizer::CanCharacterizer() {
    // Initializing...
    ROS_INFO_STREAM("Can Inspection Initiated...");
}

CanCharacterizer::~CanCharacterizer() {
    // Finishing...
    ROS_INFO_STREAM("All done, Moving On...");
}

bool CanCharacterizer::handleInspectCanRequest(enpm808x_final_inspection_robot::InspectCan::
    InspectCanRequest &req, enpm808x_final_inspection_robot::InspectCan::
        InspectCanResponse &res) {
return true;
}

bool CanCharacterizer::handleLocalizeCanRequest(enpm808x_final_inspection_robot::LocalizeCan::
    LocalizeCanRequest &req, enpm808x_final_inspection_robot::LocalizeCan::
        LocalizeCanResponse &res) {
return true;
}
