#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "../include/humanoid_planner.hpp"

using namespace std;

namespace humanoid_planning {

typedef boost::shared_ptr<HumanoidPlanner> HumanoidPlannerPtr;

HumanoidPlanner::HumanoidPlanner(const string &arm)
{
    pub = nh.advertise<moveit_msgs::PickupGoal>("pickup_message", 1, true);
    /* Load the robot model */
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    /* Get a shared pointer to the model */
    // instance of our robot model loaded from URDF
    ROS_INFO("Loading robot model from URDF");
    robot_model = robot_model_loader.getModel();
    if(!robot_model) {
        ROS_FATAL_STREAM("Unable to construct robot model!");
        throw runtime_error("Unable to construct robot model!");
    }
    string name = arm + "_arm";
    if (!robot_model->hasJointModelGroup(name)) {
        std::string error = "Group '" + name + "' was not found.";
        ROS_FATAL_STREAM(error);
        throw std::runtime_error(error);
    }
    joint_model_group = robot_model->getJointModelGroup(name);

    // try to find the name of the end effector...
    const vector<const robot_model::JointModelGroup *> eefs = robot_model->getEndEffectors();
    for(size_t i = 0; i < eefs.size(); ++i) {
        string eefName = eefs[i]->getName();
        string parent_name = eefs[i]->getEndEffectorParentGroup().first;
        if(parent_name == name) {
            end_effector = eefName;
            break;
        }
    }

    // get the name of the end effector link in the arm...
    end_effector_link = joint_model_group->getLinkModelNames().back();
    ROS_INFO("End effector link: %s", end_effector_link.c_str());
    group_name_arm = name;
    group_name_hand = arm + "_hand";
    group_name_arm_hand = arm + "_arm_hand";
    planning_time = 5.0;
    planning_attempts = 5;
    goal_joint_tolerance = 1e-4;
    goal_position_tolerance = 1e-4; // 0.1 mm
    goal_orientation_tolerance = 1e-3; // ~0.1 deg
    planner_id = "";


    ROS_INFO("Connecting to pickup action...");
    string pickup_topic = "pickup";
    pick_action_client.reset(new actionlib::SimpleActionClient<moveit_msgs::PickupAction>(pickup_topic, true));

    if (!pick_action_client->waitForServer(ros::Duration(5.0)))
    {
        ROS_ERROR("Pick up action client NOT available!");
    }else{
        ROS_INFO("Pick up action client available!");
    }

    ROS_INFO("Connecting to place action...");
    string place_topic = "place";
    place_action_client.reset(new actionlib::SimpleActionClient<moveit_msgs::PlaceAction>(place_topic, true));

    if (!place_action_client->waitForServer(ros::Duration(5.0)))
    {
        ROS_ERROR("Place action client NOT available!");
    }else{
        ROS_INFO("Place action client available!");
    }

    ROS_INFO("Connecting to move_group action...");
    string move_group_topic = "move_group";
    move_action_client.reset(new actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>(move_group_topic, true));

    if(!move_action_client->waitForServer(ros::Duration(5.0)))
    {
        ROS_ERROR("Move action client NOT available!");
    }else{
        ROS_INFO("Move action client available!");
    }


    /*
    ROS_INFO("Connecting to execute trajectory action...");
    string execute_topic = "execute_traj";
    execute_action_client.reset(new actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>(execute_topic,true));

    if(!execute_action_client->waitForServer(ros::Duration(5.0)))
    {
        ROS_ERROR("Execute action client NOT available!");
    }else{
        ROS_INFO("Execute action client available!");
    }
    */
    execution_client = nh.serviceClient<ExecuteKnownTrajectory>("execute_kinematic_path");
    attached_object_pub = nh.advertise<AttachedCollisionObject>("attached_collision_object", 1, false);
    get_planning_scene_client = nh.serviceClient<GetPlanningScene>("get_planning_scene");
    apply_planning_scene_client = nh.serviceClient<ApplyPlanningScene>("apply_planning_scene");

    // set the default park state of the planning groups
    std::vector<double> group_variable_values;
    robot_state::RobotState start_state(robot_model);
    const robot_state::JointModelGroup *joint_model_group = start_state.getJointModelGroup("arms_hands");
    start_state.setToDefaultValues(joint_model_group,"park_arms_hands");
    start_state.copyJointGroupPositions(joint_model_group,group_variable_values);

    GetPlanningScene msg;
    PlanningScene scene;
    if(get_planning_scene_client.call(msg)){
        ROS_INFO("Got the planning scene");
        scene = msg.response.scene;
        JointState jstate = scene.robot_state.joint_state;
        jstate.name = joint_model_group->getActiveJointModelNames();
        jstate.position = group_variable_values;
        scene.robot_state.joint_state = jstate;
        scene.is_diff=true;

        ApplyPlanningScene msg_apply;
        msg_apply.request.scene=scene;
        if(apply_planning_scene_client.call(msg_apply)){
            ROS_INFO("Robot start state applied");
        }else{
            ROS_WARN("Applying the planning scene failure");
        }
    }else{
       ROS_WARN("Getting the planning scene failue");
    }



}

HumanoidPlanner::~HumanoidPlanner()
{

}

void HumanoidPlanner::setPlanningGroupName(const string &name)
{
    if (!robot_model->hasJointModelGroup(name)) {
        std::string error =	"No JointModel group with given name exists!";
        ROS_FATAL_STREAM(error);
        throw std::runtime_error(error);
    }
    group_name_arm = name;
}

string HumanoidPlanner::getPlanningGroupName()
{
    return group_name_arm;
}

void HumanoidPlanner::setPlannerId(const string &id)
{
    planner_id=id;
}

string HumanoidPlanner::getPlannerId()
{
    return planner_id;
}

void HumanoidPlanner::setAllowedPlanningTime(double value)
{
    planning_time=value;
}

double HumanoidPlanner::getAllowedPlanningTime()
{
    return planning_time;
}

void HumanoidPlanner::setPlanningAttempts(int value)
{
    planning_attempts=value;
}

int HumanoidPlanner::getPlanningAttempts()
{
    return planning_attempts;
}

void HumanoidPlanner::setSupportSurfaceName(const string name)
{
    support_surface = name;
}

string HumanoidPlanner::getSupportSurfaceName()
{
    return support_surface;
}

PlanningResultPtr HumanoidPlanner::plan_arm_to_default(const std::string def_posture)
{


    std::vector<double> group_variable_values;

    robot_state::RobotState robot_state(robot_model);
    const robot_state::JointModelGroup *joint_model_group = robot_state.getJointModelGroup(group_name_arm);
    robot_state.setToDefaultValues(joint_model_group,def_posture+"_"+group_name_arm);

    robot_state.copyJointGroupPositions(joint_model_group,group_variable_values);

    PlanningResultPtr result = plan(group_name_arm,group_variable_values);

    return result;
}

PlanningResultPtr HumanoidPlanner::plan_hand_to_default(const std::string def_posture)
{


    std::vector<double> group_variable_values;

    robot_state::RobotState robot_state(robot_model);
    const robot_state::JointModelGroup *joint_model_group = robot_state.getJointModelGroup(group_name_hand);
    robot_state.setToDefaultValues(joint_model_group,def_posture+"_"+group_name_hand);

    robot_state.copyJointGroupPositions(joint_model_group,group_variable_values);

    PlanningResultPtr result = plan(group_name_hand,group_variable_values);

    return result;
}

PlanningResultPtr HumanoidPlanner::plan_arm_hand_to_default(const std::string def_posture)
{


    std::vector<double> group_variable_values;

    robot_state::RobotState robot_state(robot_model);
    const robot_state::JointModelGroup *joint_model_group = robot_state.getJointModelGroup(group_name_arm_hand);
    robot_state.setToDefaultValues(joint_model_group,def_posture+"_"+group_name_arm_hand);

    robot_state.copyJointGroupPositions(joint_model_group,group_variable_values);

    PlanningResultPtr result = plan(group_name_arm_hand,group_variable_values);

    return result;
}



PlanningResultPtr HumanoidPlanner::plan_pick(const string &object_id, const vector<Grasp> &grasps)
{
    PlanningResultPtr result;

    result.reset(new PlanningResult);
    result->type = HumanoidPlanner::PICK;

    if (!pick_action_client) {
        ROS_ERROR("Pick action client not found");

        result->status = HumanoidPlanner::FAILURE;
        result->status_msg = "Pick action client not found";
        return result;
    }

    if (!pick_action_client->isServerConnected()) {
        ROS_ERROR("Pick action server not connected");

        result->status = HumanoidPlanner::FAILURE;
        result->status_msg = "Pick action server not connected";
        return result;
    }

    moveit_msgs::PickupGoal goal;

    goal.target_name = object_id;
    goal.group_name = group_name_arm;
    goal.end_effector = end_effector;
    goal.allowed_planning_time = planning_time;
    goal.support_surface_name = support_surface;
    goal.planner_id = planner_id;

    if (!support_surface.empty()) {
        goal.allow_gripper_support_collision = true;
    }

    ROS_INFO("Pickup goal constructed for group '%s' and end effector '%s'", goal.group_name.c_str(), goal.end_effector.c_str());

    goal.possible_grasps = grasps;
    goal.planning_options.look_around = CAN_LOOK;
    goal.planning_options.replan = ALLOW_REPLAN;
    goal.planning_options.replan_delay = 2.0;
    goal.planning_options.plan_only = true;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

    ROS_INFO("Publishing pickup message");
    pub.publish(goal);

    pick_action_client->sendGoal(goal);

    if (!pick_action_client->waitForResult()) {
        ROS_INFO_STREAM("Pickup action returned early");
    }

    moveit_msgs::PickupResultConstPtr res = pick_action_client->getResult();

    if (pick_action_client->getState()	== actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Call to pick action server succeeded!");

        result->status = HumanoidPlanner::SUCCESS;
        result->status_msg = "Call to pick action server succeeded!";
        result->object_id = object_id;
        result->start_state = res->trajectory_start;
        result->trajectory_descriptions = res->trajectory_descriptions;
        result->trajectory_stages = res->trajectory_stages;

    } else {
        ROS_WARN_STREAM("Fail: " << pick_action_client->getState().toString() << ": " << pick_action_client->getState().getText());

        result->status = HumanoidPlanner::FAILURE;
        stringstream ss;
        ss << "Planning failed with status code '" << res->error_code.val << "'";
        result->status_msg = ss.str();
    }

    return result;
}

PlanningResultPtr HumanoidPlanner::plan_place(const string &object_id, const vector<PlaceLocation> &locations)
{
    PlanningResultPtr result;
    result.reset(new PlanningResult);
    result->type = HumanoidPlanner::PLACE;

    if (!place_action_client) {
        ROS_ERROR_STREAM("Place action client not found");

        result->status = HumanoidPlanner::FAILURE;
        result->status_msg = "Place action client not found";
        return result;
    }
    if (!place_action_client->isServerConnected()) {
        ROS_ERROR_STREAM("Place action server not connected");

        result->status = HumanoidPlanner::FAILURE;
        result->status_msg = "Place action server not connected";
        return result;
    }

    moveit_msgs::PlaceGoal goal;

    goal.attached_object_name = object_id;
    goal.group_name = group_name_arm;
    goal.allowed_planning_time = planning_time;
    goal.support_surface_name = support_surface;
    goal.planner_id = planner_id;

    if (!support_surface.empty()) {
        goal.allow_gripper_support_collision = true;
    }

    goal.place_locations = locations;
    goal.planning_options.plan_only = true;
    goal.planning_options.look_around = CAN_LOOK;
    goal.planning_options.replan = ALLOW_REPLAN;
    goal.planning_options.replan_delay = 2;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

    place_action_client->sendGoal(goal);

    ROS_DEBUG("Sent place goal with %d locations", (int ) goal.place_locations.size());

    if (!place_action_client->waitForResult()) {
        ROS_INFO_STREAM("Place action returned early");
    }

    moveit_msgs::PlaceResultConstPtr res = place_action_client->getResult();

    if (place_action_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Call to place action server succeeded!");

        result->status = HumanoidPlanner::SUCCESS;
        result->status_msg = "Call to pick action server succeeded!";
        result->object_id = object_id;
        result->start_state = res->trajectory_start;
        result->trajectory_descriptions = res->trajectory_descriptions;
        result->trajectory_stages = res->trajectory_stages;

    } else {
        ROS_WARN_STREAM("Fail: " << place_action_client->getState().toString() << ": " << place_action_client->getState().getText());

        result->status = HumanoidPlanner::FAILURE;
        stringstream ss;
        ss << "Planning failed with status code '" << res->error_code.val << "'";
        result->status_msg = ss.str();
    }

    return result;
}

PlanningResultPtr HumanoidPlanner::plan(const std::string group, const std::vector<double> posture)
{
    PlanningResultPtr result;
    result.reset(new PlanningResult);
    result->type = HumanoidPlanner::GOAL;

    moveit_msgs::MoveGroupGoal goal;
    goal.request.group_name = group;
    goal.request.num_planning_attempts = planning_attempts;
    goal.request.allowed_planning_time = planning_time;
    goal.request.planner_id = planner_id;


    robot_state::RobotState robot_state(robot_model);
    const robot_state::JointModelGroup *joint_model_group = robot_state.getJointModelGroup(group);

    std::vector<string> joint_names = joint_model_group->getActiveJointModelNames();

    moveit_msgs::Constraints c;

    for (size_t i=0; i<posture.size(); ++i)
    {
        moveit_msgs::JointConstraint jcm;
        jcm.joint_name = joint_names.at(i);
        jcm.position = posture.at(i);
        jcm.tolerance_above = 0.05;
        jcm.tolerance_below = 0.05;
        jcm.weight = 1.0;
        c.joint_constraints.push_back(jcm);
    }

    goal.planning_options.plan_only = true;
    goal.planning_options.look_around = false;
    goal.planning_options.replan = false;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
    goal.request.goal_constraints.push_back(c);

    move_action_client->sendGoal(goal);

    ROS_DEBUG("Sent planning request for posture");

    if (!move_action_client->waitForResult()) {
        ROS_INFO_STREAM("MoveGroup action returned early");
    }

    moveit_msgs::MoveGroupResultConstPtr res = move_action_client->getResult();

    if (move_action_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Call to move_group action server succeeded!");

        result->status = HumanoidPlanner::SUCCESS;
        result->status_msg = "Call to pick action server succeeded!";
        result->start_state = res->trajectory_start;
        result->trajectory_descriptions.push_back("target");
        result->trajectory_stages.push_back(res->planned_trajectory);

    } else {
        ROS_WARN_STREAM("Fail: " << move_action_client->getState().toString() << ": "
                        << move_action_client->getState().getText());

        result->status = HumanoidPlanner::FAILURE;
        stringstream ss;
        ss << "Planning failed with status code '" << res->error_code.val << "'";
        result->status_msg = ss.str();
    }



    return result;


}

PlanningResultPtr HumanoidPlanner::plan(const geometry_msgs::Pose &pose_goal)
{
    PlanningResultPtr result;
    result.reset(new PlanningResult);
    result->type = HumanoidPlanner::GOAL;

    moveit_msgs::MoveGroupGoal goal;
    goal.request.group_name = group_name_arm;
    goal.request.num_planning_attempts = planning_attempts;
    goal.request.allowed_planning_time = planning_time;
    goal.request.planner_id = planner_id;

    geometry_msgs::PoseStamped p;
    p.header.frame_id = FRAME_ID;
    p.header.stamp = ros::Time::now();
    p.pose = pose_goal;

    //vector<Constraints> constraints;
    Constraints c =	kinematic_constraints::constructGoalConstraints(end_effector_link,	p,
                                                                    goal_position_tolerance,
                                                                    goal_orientation_tolerance);

    goal.planning_options.plan_only = true;
    goal.planning_options.look_around = false;
    goal.planning_options.replan = false;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
    goal.request.goal_constraints.push_back(c);

    move_action_client->sendGoal(goal);

    ROS_DEBUG("Sent planning request for pose goal");

    if (!move_action_client->waitForResult()) {
        ROS_INFO_STREAM("MoveGroup action returned early");
    }

    moveit_msgs::MoveGroupResultConstPtr res = move_action_client->getResult();

    if (move_action_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Call to move_group action server succeeded!");

        result->status = HumanoidPlanner::SUCCESS;
        result->status_msg = "Call to pick action server succeeded!";
        result->start_state = res->trajectory_start;
        result->trajectory_descriptions.push_back("target");
        result->trajectory_stages.push_back(res->planned_trajectory);

    } else {
        ROS_WARN_STREAM("Fail: " << move_action_client->getState().toString() << ": "
                        << move_action_client->getState().getText());

        result->status = HumanoidPlanner::FAILURE;
        stringstream ss;
        ss << "Planning failed with status code '" << res->error_code.val << "'";
        result->status_msg = ss.str();
    }


    return result;
}

bool HumanoidPlanner::execute(const PlanningResultPtr &plan)
{
    for (size_t i = 0; i < plan->trajectory_stages.size(); ++i) {
        string &description = plan->trajectory_descriptions[i];

        ROS_INFO_STREAM("Executing stage '" << description << "'...");
        if(!execute(plan->trajectory_stages[i])) {
            return false;
        }

        if(description == "grasp" && plan->type == HumanoidPlanner::PICK) {
            attachObject(plan->object_id);
        }
        else if(description == "grasp" && plan->type == HumanoidPlanner::PLACE) {
            detachObject(plan->object_id);
        }
    }

    return true;
}


bool HumanoidPlanner::execute(const RobotTrajectory &trajectory)
{

    ExecuteKnownTrajectory msg;
    ExecuteKnownTrajectoryRequest &request = msg.request;

    request.wait_for_execution = true;
    request.trajectory = trajectory;

    bool success = execution_client.call(msg);


    if (success) {
        MoveItErrorCodes &code = msg.response.error_code;

        if (code.val == MoveItErrorCodes::SUCCESS) {
            ROS_INFO("Execution finished successfully.");
        } else {
            ROS_ERROR("Execution finished with error_code '%d'", code.val);
            return false;
        }

    } else {
        ROS_ERROR("Execution failed!");
        return false;
    }

    return true;
}

bool HumanoidPlanner::attachObject(const string &object)
{
    moveit_msgs::AttachedCollisionObject aco;
    aco.object.id = object;
    aco.link_name = end_effector_link;
    aco.touch_links = robot_model->getEndEffector(end_effector)->getLinkModelNames();
    aco.object.operation = moveit_msgs::CollisionObject::ADD;
    attached_object_pub.publish(aco);

    return true;
}

bool HumanoidPlanner::detachObject(const string &object)
{
    moveit_msgs::AttachedCollisionObject aco;
    aco.object.id = object;
    aco.object.operation = moveit_msgs::CollisionObject::REMOVE;
    aco.link_name = end_effector_link;

    attached_object_pub.publish(aco);

    return true;
}



} // namespace humanoid_planning
