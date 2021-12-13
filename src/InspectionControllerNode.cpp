/**
 * @file InspectionControllerNode.cpp
 * @author Robert Vandemark
 * @brief The node encapsulating the inspection process's main controller.
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

#include <string>
#include <regex>
#include <stdexcept>

#include <ros/ros.h>

#include "enpm808x_final_inspection_robot/InspectionController.hpp"

namespace {
    /**
     * Attempt to convert a string to an double.
     * @param s The string that is supposed to be a double.
     * @param d A pointer to a double to write the result to.
     * @return Whether or not the conversion process was successful.
     */
    bool strToDouble(const std::string& s, double* d) {
        try {
            // try to convert the string to a double
            *d = std::stod(s);
            return true;
        } catch (const std::invalid_argument&) {
            return false;
        }
    }

    /**
     * Attempt to get a pose (x, y, z, roll, pitch, and yaw) from the parameter
     * server.
     * @param nh The node handle which gives access to the parameter server.
     * @param name The name of the parameter on the parameter server.
     * @param x The x coordinate of the position.
     * @param y The y coordinate of the position.
     * @param z The z coordinate of the position.
     * @param R The roll of the orientation.
     * @param P The pitch of the orientation.
     * @param Y The yaw of the orientation.
     * @return Whether or not the conversion process was successful.
     */
    bool parsePoseParameterString(const ros::NodeHandle& nh,
                                  const char* name,
                                  double* x,
                                  double* y,
                                  double* z,
                                  double* R,
                                  double* P,
                                  double* Y) {
        bool rc = true;  // Prove this value otherwise
        std::string value;
        if (nh.getParam(name, value)) {
            ROS_INFO_STREAM("Got " << name << "='" << value << "'");
        } else {
            ROS_FATAL_STREAM("Failed to find parameter " << name);
            rc = false;
        }
        if (rc) {
            // Split all the words (items delimited by spaces)
            const std::regex regex("[^\\s]+");
            const std::sregex_iterator iter_begin(value.begin(),
                                                  value.end(),
                                                  regex);
            const auto split_end = std::sregex_iterator();

            // Ensure we have each required value
            bool set_x = false, set_y = false, set_z = false,
                 set_R = false, set_P = false, set_Y = false;
            for (auto iter = iter_begin; rc && (iter != split_end); ++iter) {
                const std::string curr = iter->str(), next = (++iter)->str();
                if (0 == curr.compare("-x")) {
                    set_x = strToDouble(next, x);
                    rc = set_x;
                } else if (0 == curr.compare("-y")) {
                    set_y = strToDouble(next, y);
                    rc = set_y;
                } else if (0 == curr.compare("-z")) {
                    set_z = strToDouble(next, z);
                    rc = set_z;
                } else if (0 == curr.compare("-R")) {
                    set_R = strToDouble(next, R);
                    rc = set_R;
                } else if (0 == curr.compare("-P")) {
                    set_P = strToDouble(next, P);
                    rc = set_P;
                } else if (0 == curr.compare("-Y")) {
                    set_Y = strToDouble(next, Y);
                    rc = set_Y;
                }
            }
            rc = set_x && set_y && set_z && set_R && set_P && set_Y;
        }
        return rc;
    }

    /**
     * Attempt to get a pose (x, y, z, roll, pitch, and yaw) from the parameter
     * server, then populate a tf Transform with any valid results.
     * @param nh The node handle which gives access to the parameter server.
     * @param name The name of the parameter on the parameter server.
     * @param tf The output transform.
     * @return Whether or not the conversion process was successful.
     */
    bool parsePoseParameterStringToTf(const ros::NodeHandle& nh,
                                      const char* name,
                                      tf::Transform* tf) {
        double x, y, z, R, P, Y;
        if (!parsePoseParameterString(nh, name, &x, &y, &z, &R, &P, &Y)) {
            return false;
        }
        tf::Quaternion ori;
        ori.setRPY(R, P, Y);  // Can't set RPY in constructor :(
        tf->setOrigin(tf::Vector3(x, y, z));
        tf->setRotation(ori);
        return true;
    }

    /**
     * Attempt to get a pose (x, y, z, roll, pitch, and yaw) from the parameter
     * server, then populate a geometry_msgs; Pose with any valid results.
     * @param nh The node handle which gives access to the parameter server.
     * @param name The name of the parameter on the parameter server.
     * @param pose The output pose.
     * @return Whether or not the conversion process was successful.
     */
    bool parsePoseParameterStringToPose(const ros::NodeHandle& nh,
                                        const char* name,
                                        geometry_msgs::Pose* pose) {
        double x, y, z, R, P, Y;
        if (!parsePoseParameterString(nh, name, &x, &y, &z, &R, &P, &Y)) {
            return false;
        }
        pose->position.x = x;
        pose->position.y = y;
        pose->position.z = z;
        // Use tf's Quaternion to help us set this
        tf::Quaternion ori;
        ori.setRPY(R, P, Y);  // Can't set RPY in constructor :(
        pose->orientation.x = ori.x();
        pose->orientation.y = ori.y();
        pose->orientation.z = ori.z();
        pose->orientation.w = ori.w();
        return true;
    }

    /**
     * Attempt to parse the list of arguments that describe the spawned cans.
     * @param nh The node handle which gives access to the parameter server.
     * @param name The name of the parameter on the parameter server.
     * @param cans The output vector of can descriptions.
     * @return Whether or not the conversion process was successful.
     */
    bool parseCanArgsList(const ros::NodeHandle& nh,
                          const char* name,
                          std::vector<tf::Vector3>* cans) {
        std::string value;
        if (nh.getParam(name, value)) {
            ROS_INFO_STREAM("Got " << name << "='" << value << "'");
        } else {
            // Don't log an error otherwise
            return false;
        }
        // Split all the colons to get an iterator of can args
        const std::regex regex1("[^:]+");
        const std::sregex_iterator iter1_begin(value.begin(),
                                               value.end(),
                                               regex1);
        const auto iter_end = std::sregex_iterator();
        bool rc = true;
        for (auto iter = iter1_begin; rc && (iter != iter_end); ++iter) {
            const std::string args = std::regex_replace(iter->str(),
                                                        std::regex("\\s+"),
                                                        "");
            // Split the comma-separated values
            // Skip the 1st param, which is whether or not the can is defective
            const std::regex regex2("[^,]+");
            std::sregex_iterator iter2_begin(args.begin(),
                                             args.end(),
                                             regex2);
            iter2_begin++;

            // Get the expected/approximate [x,y,z] coords
            double x, y, z;
            rc = strToDouble((iter2_begin++)->str(), &x)
                 && strToDouble((iter2_begin++)->str(), &y)
                 && strToDouble((iter2_begin)->str(), &z);
            if (rc) {
                cans->push_back(tf::Vector3(x, y, z));
            }
        }
        return rc;
    }
}

/**
 * The main entry point of our inspection controller node.
 * @param argc The number of program arguments.
 * @param argv The list of program argument strings.
 * @return The return code of the node's operation.
 */
int main(int argc, char** argv) {
    // Create our node's handle
    ros::init(argc, argv, "inspection_controller");
    ros::NodeHandle nh("/enpm808x");

    // Get parameters that are available as the TIAGo robot launches
    tf::Transform detection_pose_offset;
    if (!parsePoseParameterStringToTf(nh,
                                      "detection_pose_offset",
                                      &detection_pose_offset)) {
        ROS_FATAL_STREAM("Unable to set detection_pose_offset!");
        return -1;
    }
    geometry_msgs::Pose home_position;
    if (!parsePoseParameterStringToPose(nh,
                                        "home_position",
                                        &home_position)) {
        ROS_FATAL_STREAM("Unable to set home_position!");
        return -2;
    }

    // With these parameters, create our inspection controller
    InspectionController cont(nh, home_position, detection_pose_offset);

    // Before the robot can start moving, we need to get parameters from the
    // parameter server. Ideally these just come in as a message, this is left
    // as future work in the interest of time. Furthermore, we also have to
    // wait for the robot's arm to be tucked in. For now, just sleep briefly
    // and spin once while waiting for these.
    tf::TransformListener tf_listener;
    std::vector<tf::Vector3> expected_can_positions;
    bool has_expected_can_positions = false;
    bool base_transform_is_ready = false;
    bool ready_to_start = false;
    ros::Rate sleep_rate(10);
    while (!ready_to_start) {
        if (!has_expected_can_positions) {
            has_expected_can_positions = parseCanArgsList(
                nh,
                "can_args_list",
                &expected_can_positions);
            if (has_expected_can_positions) {
                ROS_INFO_STREAM("Successfully parsed can positions.");
            }
        }
        if (!base_transform_is_ready) {
            base_transform_is_ready = tf_listener.waitForTransform(
                "/map",
                "/base_footprint",
                ros::Time(0),
                ros::Duration(1));
            if (base_transform_is_ready) {
                ROS_INFO_STREAM("Confirmed existence of transform between /map"
                                " and /base_footprint");
            }
        }
        ready_to_start = has_expected_can_positions
                         && base_transform_is_ready
                         && cont.isArmTucked();
        if (!ready_to_start) {
            sleep_rate.sleep();
            ros::spinOnce();
        }
    }

    // Print the can attributes (expected/approximate x, y, and z coordinates)
    ROS_INFO_STREAM("Expecting " << expected_can_positions.size() << " cans at:");
    for (auto iter = expected_can_positions.begin();
            iter != expected_can_positions.end();
            ++iter) {
        ROS_INFO_STREAM("  x=" << iter->x() << ", y=" << iter->y() << ", z=" << iter->z());
    }

    // Pass this onto the controller and let it drive the rest of the program
    cont.inspect(expected_can_positions);
    ros::spin();

    return 0;
}
