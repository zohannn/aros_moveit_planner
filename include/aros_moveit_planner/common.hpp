#ifndef MOVEIT_COMMON_HPP
#define MOVEIT_COMMON_HPP


#define ATTEMPTS 5
#define TIMEOUT 0.1
#define CAN_LOOK false
#define ALLOW_REPLAN false
#define FRAME_ID "base_link"
#define SUPPORT_SURFACE "table_surface_link"

/** definitions for printing output */
#define SEP ", "
#define METERS " [m]"
#define MILLIMETERS " [mm]"
#define DEG " [deg]"
#define RAD " [rad]"
#define COLUMN ":"
#define SPACE " "
#define XposSTR "Xpos = "
#define YposSTR "Ypos = "
#define ZposSTR "Zpos = "
#define RollSTR "Roll = "
#define PitchSTR "Pitch = "
#define YawSTR "Yaw = "
#define XsizeSTR "Xsize = "
#define YsizeSTR "Ysize = "
#define ZsizeSTR "Zsize = "

#include <sstream>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/format.hpp>



using namespace std;
using namespace moveit_msgs;
using namespace Eigen;

namespace moveit_common {


/** this struct defines the tolerances that have to be set before planning the trajectory*/
typedef struct{
    std::vector<float> tolsArm; /**< radius of the spheres along the arm in [mm] */
    MatrixXf tolsHand; /**< radius of the spheres along the fingers in [mm] */
    MatrixXf final_tolsObstacles; /**< tolerances of the final posture against the obstacles in [mm] */
    std::vector< MatrixXf > singleArm_tolsTarget; /**< tolerances of the trajectory against the target in [mm] */
    std::vector< MatrixXf > singleArm_tolsObstacles; /**< tolerances of the trajectory against the obstacles in [mm] */
    float tolTarPos; /**< tolerance of the final position of the hand against the target in [mm] */
    float tolTarOr; /**< tolerance of the final orientation of the hand against the target in [mm] */
    //boundaryConditions bounds; /**< boundary condistions of the bounce problem */
    int steps; /**< number of steps for the trajectory */
    //float totalTime; // normalized time of the movement (0 < t <= 1)
    std::vector<float> lambda_final; /**< weights for the final posture optimization */
    std::vector<float> lambda_bounce; /**< weights for the bounce posture optimization */
    std::vector<float> w_max; /**< maximum angular velocity for each joint [rad/s] */
    //std::vector<float> tols_table; // tolernaces for the table
    bool obstacle_avoidance; /**< true to avoid obstacle */
    bool target_avoidance; /**< true to avoid the target during the motion */
    //float eng_dist; /**< distance in [mm] of tolerance from the engage point along the axis  indicated by the eng_dir parameter.
      //                  It defines the target for the sub-engage movement */
    //int eng_dir; /**< direction of tolerance. eng_dir=0 means no direction; eng_dir=1 means the x axis; eng_dir=2 means the y axis; eng_dir=3 means the z axis;*/
    //std::vector<float> eng_tols; /**< tolerances in reaching the engage point in [mm].
      //                              eng_tols(0) along the x axis; eng_tols(1) along the y axis; eng_tols(2) along the z axis;*/
    //float diseng_dist; /**< distance in [mm] of tolerance from the engage point along the axis  indicated by the diseng_dir parameter.
      //                  It defines the target for the sub-disengage movement */
    //int diseng_dir; /**< direction of tolerance. diseng_dir=0 means no direction; diseng_dir=1 means the x axis; diseng_dir=2 means the y axis; diseng_dir=3 means the z axis;*/

    float tol_stop; /**< this tolerance defines the error between the norm of the final posture and the norm the current posture.
                    It has to be set to stop the movement when the final posture is reached. A tipical value is 0.1  */
} moveit_params;

/**
 * @brief The PlanningResult struct
 * This structure lists the result of planning
 */
typedef struct{
        int type;/**< type of the trajectory */
        int status;/**< status code of the planning */
        string status_msg;/**< status message of the planning */
        string object_id;/**< identity of the object involved on the movement */
        RobotState start_state;/**< start state of the robot*/
        vector<RobotTrajectory> trajectory_stages;/**< sequence of robot trajectories */
        vector<string> trajectory_descriptions;/**< description of the trajectories */
}PlanningResult;


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



#endif // MOVEIT_COMMON_HPP
