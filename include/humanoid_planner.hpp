#ifndef HUMANOID_PLANNER_HPP
#define HUMANOID_PLANNER_HPP

#include <ros/ros.h>
#include <string>

#include <boost/scoped_ptr.hpp>

#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit_msgs/Grasp.h>

#include "../include/common.hpp"

using namespace std;
using namespace moveit_msgs;
using namespace common;

namespace humanoid_planning {

/**
 * @brief The PlanningResult struct
 * This structure lists the result of planning
 */
struct PlanningResult {
        int type;/**< type of the trajectory */
        int status;/**< status code of the planning */
        string status_msg;/**< status message of the planning */
        string object_id;/**< identity of the object involved on the movement */
        RobotState start_state;/**< start state of the robot*/
        vector<RobotTrajectory> trajectory_stages;/**< sequence of robot trajectories */
        vector<string> trajectory_descriptions;/**< description of the trajectories */
};
typedef boost::shared_ptr<PlanningResult> PlanningResultPtr;

//! The HumanoidPlanner class
/**
 * @brief The HumanoidPlanner class
 * This class
 */
class HumanoidPlanner{

private:

    string group_name;/**< planning group */
    string end_effector;/**< group of the end effector */
    string end_effector_link;/**< end effector that is linking to the planning group */
    double planning_time;/**< maximum allowed planning time */
    int planning_attempts;/**< number of attempts of planning */
    double goal_joint_tolerance;/**< tolerance on the goal joint position [rad]*/
    double goal_position_tolerance;/**< tolerance on the goal end-effector position [m] */
    double goal_orientation_tolerance;/**< tolerance on the goal end-effector orientation [rad]*/
    string planner_id;/**< identity of the planner */
    string support_surface;/**< support surface for pick and place */

    ros::NodeHandle nh;/**< ros node handle */
    ros::Publisher pub;/**< ros publisher for pickup messages */
    ros::ServiceClient execution_client;/**< ros service client for execution of the trajectory */
    ros::Publisher attached_object_pub;/**< ros publisher of the attached collision object */

    robot_model::RobotModelConstPtr robot_model;/**< model of the loaded robot*/
    const robot_model::JointModelGroup *joint_model_group;/**< joint group of the planning group */

    boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::PickupAction> > pick_action_client; /**< pickup action client */
    boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::PlaceAction> > place_action_client; /**< place action client */
    boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> > move_action_client;/**< move group action client */
    //boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> > execute_action_client;/**< execute action client */

    /**
     * @brief execute
     * It executes the robot trajectory
     * @param trajectory
     * @return
     */
    bool execute(const RobotTrajectory &trajectory);

    /**
     * @brief attachObject
     * This method attaches the object from the end effector link
     * @param object
     * @return
     */
    bool attachObject(const string &object);

    /**
     * @brief detachObject
     * This method detaches the object from the end effector link
     * @param object
     * @return
     */
    bool detachObject(const string &object);

public:

    static const int SUCCESS = 1; /**< success status code */
    static const int FAILURE = 0;/**< failure status code */
    static const int PICK = 0;/**< pick type code */
    static const int PLACE = 1;/**< place type code */
    static const int GOAL = 2;/**< goal type code */

    /**
     * @brief HumanoidPlanner, a constructor
     * @param arm
     */
    HumanoidPlanner(const string &arm);

    /**
     * @brief ~HumanoidPlanner, a destructor
     */
    ~HumanoidPlanner();

    /**
     * @brief plan_pick
     * Plan a pickup movement with the list of given grasps
     * @param object_id
     * @param grasps
     * @return
     */
    PlanningResultPtr plan_pick(const string &object_id, const vector<Grasp> &grasps);

    /**
     * @brief plan_place
     * Plan the place movement with the list of given locations
     * @param object_id
     * @param locations
     * @return
     */
    PlanningResultPtr plan_place(const string &object_id, const vector<PlaceLocation> &locations);

    /**
     * @brief plan
     * Plan the movement to reach the pose of the given goal
     * @param pose_goal
     * @return
     */
    PlanningResultPtr plan(const geometry_msgs::Pose &pose_goal);

    /**
     * @brief plan
     * Plan the movement to reach the given posture
     * @param posture
     * @return
     */
    PlanningResultPtr plan(const std::vector<double> posture);

    /**
     * @brief plan_to_park
     * Plan the trajectory to go to the park posture
     * @return
     */
    PlanningResultPtr plan_to_park();

    /**
     * @brief plan_to_home
     * Plan the trajectory to go to the home posture
     * @return
     */
    PlanningResultPtr plan_to_home();

    /**
     * @brief execute
     * It executes the planned trajectory
     * @param plan
     * @return
     */
    bool execute(const PlanningResultPtr &plan);

    /**
     * @brief setPlanningGroupName
     * It sets the name of the planning group
     * @param name
     */
    void setPlanningGroupName(const string &name);

    /**
     * @brief getPlanningGroupName
     * It gets the name of the planning group
     * @return
     */
    string getPlanningGroupName();

    /**
     * @brief setPlannerId
     * It sets the identity of the planner
     * @param id
     */
    void setPlannerId(const string &id);

    /**
     * @brief getPlannerId
     * It gets the identity of the planner
     * @return
     */
    string getPlannerId();

    /**
     * @brief setAllowedPlanningTime
     * It sets the maximum allowed planning time
     * @param value
     */
    void setAllowedPlanningTime(double value);

    /**
     * @brief getAllowedPlanningTime
     * It gets the maximum allowed planning time
     * @return
     */
    double getAllowedPlanningTime();

    /**
     * @brief setPlanningAttempts
     * It sets the attempts of planning
     * @param value
     */
    void setPlanningAttempts(int value);

    /**
     * @brief getPlanningAttempts
     * It gets the attempts of planning
     * @return
     */
    int getPlanningAttempts();

    /**
     * @brief setSupportSurfaceName
     * It sets the name of the support surface
     * @param name
     */
    void setSupportSurfaceName(const string name);

    /**
     * @brief getSupportSurfaceName
     * It gets the name of the support surface
     * @return
     */
    string getSupportSurfaceName();


};


} // namespace humanoid_planning

#endif // HUMANOID_PLANNER_HPP
