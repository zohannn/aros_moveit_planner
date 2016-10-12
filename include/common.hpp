#ifndef COMMON_HPP
#define COMMON_HPP


#define ATTEMPTS 5
#define TIMEOUT 0.1
#define CAN_LOOK false
#define ALLOW_REPLAN false
#define FRAME_ID "base_link"

#include <sstream>
#include <ros/ros.h>
#include <moveit_msgs/RobotState.h>

using namespace std;

namespace common {

/**
 * @brief getJointPositionFromState *
 * This method retrieves the position of the joint with name "joint" in the robot state
 * @param joint
 * @param state
 * @return
 */
double getJointPositionFromState(const string &joint, const moveit_msgs::RobotState &state);

/**
 * @brief getJointPositionsFromState
 * This method retrieves all the values of the joints in the robot state
 * @param joints
 * @param state
 * @param values
 */
void getJointPositionsFromState(const vector<string> &joints, const moveit_msgs::RobotState &state, vector<double>&values);

/**
 * @brief getArmJointNames
 * This method retrieves the names of the joints in the arm selected
 * @param arm
 * @param names
 */
void getArmJointNames(const string &arm, vector<string> &names);

/**
 * @brief getArmValuesFromState
 * This method retrieves the values of the joints in the arm selected in the robot state
 * @param arm
 * @param state
 * @param values
 */
void getArmValuesFromState(const string &arm, const moveit_msgs::RobotState &state, vector<double> &values);

/**
 * @brief printJointsValues
 * This method prints the values of the joints of the arm selected in the robot state
 * @param state
 * @param arm
 */
void printJointsValues(const moveit_msgs::RobotState &state, const string &arm);

}



#endif // COMMON_HPP
