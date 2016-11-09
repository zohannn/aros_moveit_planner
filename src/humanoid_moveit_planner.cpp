#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "../include/aros_moveit_planner/humanoid_moveit_planner.hpp"

using namespace std;

namespace moveit_planning {

typedef boost::shared_ptr<HumanoidPlanner> HumanoidPlannerPtr;

HumanoidPlanner::HumanoidPlanner(const string &name)
{

    scenario_path="";
    planner_name = name;

    init();

    // set the default park state of the planning groups
    std::vector<double> group_variable_values;
    robot_state::RobotState start_state(robot_model);
    const robot_state::JointModelGroup *joint_model_group = start_state.getJointModelGroup("arms_hands");
    start_state.setToDefaultValues(joint_model_group,"park_arms_hands");
    start_state.copyJointGroupPositions(joint_model_group,group_variable_values);

    GetPlanningScene msg;
    moveit_msgs::PlanningScene scene_msg;

    if(get_planning_scene_client.call(msg)){
        ROS_INFO("Got the planning scene");
        scene_msg = msg.response.scene;
        JointState jstate = scene_msg.robot_state.joint_state;
        jstate.name = joint_model_group->getActiveJointModelNames();
        jstate.position = group_variable_values;
        scene_msg.robot_state.joint_state = jstate;
        scene_msg.is_diff=true;

        ApplyPlanningScene msg_apply;
        msg_apply.request.scene=scene_msg;
        if(apply_planning_scene_client.call(msg_apply)){
            ROS_INFO("Robot start state applied");
        }else{
            ROS_WARN("Applying the planning scene failure");
        }
    }else{
       ROS_WARN("Getting the planning scene failue");
    }


}



HumanoidPlanner::HumanoidPlanner(const string &name,const string &path)
{


    scenario_path = path;
    planner_name = name;


    init();

    // set the default park state of the planning groups and load the selected scene
    std::vector<double> group_variable_values;
    robot_state::RobotState start_state(robot_model);
    const robot_state::JointModelGroup *joint_model_group = start_state.getJointModelGroup("arms_hands");
    start_state.setToDefaultValues(joint_model_group,"park_arms_hands");
    start_state.copyJointGroupPositions(joint_model_group,group_variable_values);

    moveit_msgs::PlanningScene scene_msg;
    planning_scene::PlanningScene scene(robot_model);
    std::ifstream f(scenario_path);
    if (f.good() && !f.eof())
    {
        ROS_INFO("Loading the selected scene ...");
        scene.loadGeometryFromStream(f);
    }
    scene.getPlanningSceneMsg(scene_msg);
    JointState jstate = scene_msg.robot_state.joint_state;
    jstate.name = joint_model_group->getActiveJointModelNames();
    jstate.position = group_variable_values;
    scene_msg.robot_state.joint_state = jstate;
    scene_msg.is_diff=true;

    ApplyPlanningScene msg_apply;
    msg_apply.request.scene=scene_msg;
    if(apply_planning_scene_client.call(msg_apply)){
        ROS_INFO("scene applied");
    }else{
        ROS_WARN("Applying the planning scene failure");
    }




}


HumanoidPlanner::HumanoidPlanner(const HumanoidPlanner &hp)
{
    this->group_name_arm = hp.group_name_arm;
    this->group_name_hand = hp.group_name_hand;
    this->group_name_arm_hand = hp.group_name_arm_hand;
    this->end_effector = hp.end_effector;
    this->end_effector_link = hp.end_effector_link;
    this->planning_time = hp.planning_time;
    this->planning_attempts = hp.planning_attempts;
    this->goal_joint_tolerance = hp.goal_joint_tolerance;
    this->goal_position_tolerance = hp.goal_position_tolerance;
    this->goal_orientation_tolerance = hp.goal_orientation_tolerance;
    this->planner_id = hp.planner_id;
    this->support_surface = hp.support_surface;
    this->planner_name = hp.planner_name;
    this->scenario_path = hp.scenario_path;

    this->nh = hp.nh;
    this->pub = hp.pub;
    this->execution_client = hp.execution_client;
    this->attached_object_pub = hp.attached_object_pub;
    this->get_planning_scene_client = hp.get_planning_scene_client;
    this->apply_planning_scene_client = hp.apply_planning_scene_client;
    this->robot_model = hp.robot_model;
    this->joint_model_group = hp.joint_model_group;

    init();
    if (scenario_path.empty()){
        // set the default park state of the planning groups
        std::vector<double> group_variable_values;
        robot_state::RobotState start_state(robot_model);
        const robot_state::JointModelGroup *joint_model_group = start_state.getJointModelGroup("arms_hands");
        start_state.setToDefaultValues(joint_model_group,"park_arms_hands");
        start_state.copyJointGroupPositions(joint_model_group,group_variable_values);

        GetPlanningScene msg;
        moveit_msgs::PlanningScene scene_msg;

        if(get_planning_scene_client.call(msg)){
            ROS_INFO("Got the planning scene");
            scene_msg = msg.response.scene;
            JointState jstate = scene_msg.robot_state.joint_state;
            jstate.name = joint_model_group->getActiveJointModelNames();
            jstate.position = group_variable_values;
            scene_msg.robot_state.joint_state = jstate;
            scene_msg.is_diff=true;

            ApplyPlanningScene msg_apply;
            msg_apply.request.scene=scene_msg;
            if(apply_planning_scene_client.call(msg_apply)){
                ROS_INFO("Robot start state applied");
            }else{
                ROS_WARN("Applying the planning scene failure");
            }
        }else{
           ROS_WARN("Getting the planning scene failue");
        }
    }else{
        // set the default park state of the planning groups and load the selected scene
        std::vector<double> group_variable_values;
        robot_state::RobotState start_state(robot_model);
        const robot_state::JointModelGroup *joint_model_group = start_state.getJointModelGroup("arms_hands");
        start_state.setToDefaultValues(joint_model_group,"park_arms_hands");
        start_state.copyJointGroupPositions(joint_model_group,group_variable_values);

        moveit_msgs::PlanningScene scene_msg;
        planning_scene::PlanningScene scene(robot_model);
        std::ifstream f(scenario_path);
        if (f.good() && !f.eof())
        {
            ROS_INFO("Loading the selected scene ...");
            scene.loadGeometryFromStream(f);
        }
        scene.getPlanningSceneMsg(scene_msg);
        JointState jstate = scene_msg.robot_state.joint_state;
        jstate.name = joint_model_group->getActiveJointModelNames();
        jstate.position = group_variable_values;
        scene_msg.robot_state.joint_state = jstate;
        scene_msg.is_diff=true;

        ApplyPlanningScene msg_apply;
        msg_apply.request.scene=scene_msg;
        if(apply_planning_scene_client.call(msg_apply)){
            ROS_INFO("scene applied");
        }else{
            ROS_WARN("Applying the planning scene failure");
        }

    }

}

HumanoidPlanner::~HumanoidPlanner()
{

}

void HumanoidPlanner::init()
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
    string name = "right_arm"; // default arm planning group
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
    group_name_hand = "right_hand";// default hand planning group
    group_name_arm_hand = "right_arm_hand"; // default arm_hand planning group
    planning_time = 5.0;
    planning_attempts = 5;
    goal_joint_tolerance = 1e-4; // ~0.01 deg
    goal_position_tolerance = 1e-3; // 1 mm
    goal_orientation_tolerance = 1e-3; // ~0.1 deg
    planner_id = "RRTConnectkConfigDefault";
    support_surface = SUPPORT_SURFACE;



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

    execution_client = nh.serviceClient<ExecuteKnownTrajectory>("execute_kinematic_path");
    attached_object_pub = nh.advertise<AttachedCollisionObject>("attached_collision_object", 1, false);
    get_planning_scene_client = nh.serviceClient<GetPlanningScene>("get_planning_scene");
    apply_planning_scene_client = nh.serviceClient<ApplyPlanningScene>("apply_planning_scene");



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


PlanningResultPtr HumanoidPlanner::pick(moveit_params& params)
{
  PlanningResultPtr result;
  int arm_code = params.arm_code;
  this->planner_id = params.config;

  switch(arm_code){
  case 0:// both arms
      // TO DO
      break;
  case 1: // right arm
      this->group_name_arm = "right_arm";
      this->group_name_hand = "right_hand";
      this->group_name_arm_hand = "right_arm_hand";
      break;
  case 2:// left arm
      this->group_name_arm = "left_arm";
      this->group_name_hand = "left_hand";
      this->group_name_arm_hand = "left_arm_hand";
      break;
  }

  // try to find the name of the end effector...
  joint_model_group = robot_model->getJointModelGroup(this->group_name_arm);
  const vector<const robot_model::JointModelGroup *> eefs = robot_model->getEndEffectors();
  for(size_t i = 0; i < eefs.size(); ++i) {
      string eefName = eefs[i]->getName();
      string parent_name = eefs[i]->getEndEffectorParentGroup().first;
      if(parent_name == this->group_name_arm) {
          this->end_effector = eefName;
          break;
      }
  }
  // get the name of the end effector link in the arm...
  this->end_effector_link = joint_model_group->getLinkModelNames().back();

  // target
  std::vector<double> tar = params.target;
  Vector3d tar_pos(tar.at(0),tar.at(1),tar.at(2));
  std::vector<double> rpy = {tar.at(3),tar.at(4),tar.at(5)};
  Matrix3d Rot_tar; this->RPY_matrix(rpy,Rot_tar);
  Vector3d z_tar = Rot_tar.col(2);

  // approach
  std::vector<double> approach = params.pre_grasp_approach;
  Vector3d v_app(approach.at(0),approach.at(1),approach.at(2));
  Vector3d app_vv =Rot_tar*v_app;
  double dist_app = approach.at(3);

  // retreat
  std::vector<double> retreat = params.post_grasp_retreat;
  Vector3d v_ret(retreat.at(0),retreat.at(1),retreat.at(2));
  Vector3d ret_vv =Rot_tar*v_ret;
  double dist_ret = retreat.at(3);

  // Pick grasp
  std::vector<Grasp> grasps; Grasp g; geometry_msgs::PoseStamped p;
  p.header.frame_id = FRAME_ID;
  // position of the pose
  double dHO = params.dHO;
  Vector3d pose_position = app_vv*dHO+tar_pos;
  p.pose.position.x = pose_position(0);
  p.pose.position.y = pose_position(1);
  p.pose.position.z = pose_position(2);
  // orientation of the pose
  Vector3d z_hand = -app_vv; // z axis of the hand;
  Vector3d x_hand; Vector3d y_hand;
  int griptype = params.griptype;
  switch(griptype){
  case 211: case 111:// Side thumb left
      x_hand = z_tar;
      break;
  case 112: case 212:// Side thumb right
      // TO DO
      break;
  case 113: case 213:// Side thumb up
      // TO DO
      break;
  case 114: case 214:// Side thumb down
      // TO DO
      break;
  case 121: case 221:// Above
      // TO DO
      break;
  case 122: case 222: // Below
      // TO DO
      break;
  }
  y_hand = z_hand.cross(x_hand);
  Matrix3d Rot_hand = Matrix3d::Zero();
  Rot_hand.col(0)=x_hand; Rot_hand.col(1)=y_hand; Rot_hand.col(2)=z_hand;
  Quaterniond q(Rot_hand);
  p.pose.orientation.x = q.x();
  p.pose.orientation.y = q.y();
  p.pose.orientation.z = q.z();
  p.pose.orientation.w = q.w();
  g.grasp_pose = p;
  //approach
  // the components of the vector of approach renge from -1 and +1 included.
  // The direction of approahc must always be the opposite of the typed values.
  // this guarantees that the point of contact is calculated correctly and that the hand goes towards it.
  g.pre_grasp_approach.direction.header.frame_id = FRAME_ID;
  g.pre_grasp_approach.direction.vector.x = -app_vv(0);
  g.pre_grasp_approach.direction.vector.y = -app_vv(1);
  g.pre_grasp_approach.direction.vector.z = -app_vv(2);
  g.pre_grasp_approach.min_distance = dist_app/2;
  g.pre_grasp_approach.desired_distance = dist_app;
  // retreat
  g.post_grasp_retreat.direction.header.frame_id = FRAME_ID;
  g.post_grasp_retreat.direction.vector.x = ret_vv(0);
  g.post_grasp_retreat.direction.vector.y = ret_vv(1);
  g.post_grasp_retreat.direction.vector.z = ret_vv(2);
  g.post_grasp_retreat.min_distance = dist_ret/2;
  g.post_grasp_retreat.desired_distance = dist_ret;

  int hand_code = params.hand_code;
  switch(hand_code){
  case 0:// human hand
      // TO DO
      break;
  case 1:// barrett hand
      this->openBarrettHand(params.finalHand,g.pre_grasp_posture);
      this->closedBarrettHand(params.finalHand,g.grasp_posture);
      break;

  }
  std::string obj_name = params.obj_name;
  //std::vector<std::string> allowed_touch_objects; allowed_touch_objects.push_back(obj_name);
  //g.allowed_touch_objects = allowed_touch_objects;

  grasps.push_back(g);

  result = this->plan_pick(obj_name,grasps);


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

/*
    moveit_msgs::PlanningScene currentScene;
    moveit_msgs::PlanningScene newSceneDiff;


    moveit_msgs::GetPlanningScene scene_srv;
    scene_srv.request.components.components = scene_srv.request.components.ALLOWED_COLLISION_MATRIX;
    moveit_msgs::AllowedCollisionMatrix currentACM;

    if(!get_planning_scene_client.call(scene_srv))
    {
        ROS_WARN("Failed to call service /get_planning_scene");
    }
    else
    {
        ROS_INFO_STREAM("Initial scene!");
        currentScene = scene_srv.response.scene;
        currentACM = currentScene.allowed_collision_matrix;

        ROS_ERROR_STREAM("size of acm_entry_names before " << currentACM.entry_names.size());
        ROS_ERROR_STREAM("size of acm_entry_values before " << currentACM.entry_values.size());
        ROS_ERROR_STREAM("size of acm_entry_values[0].entries before " << currentACM.entry_values[0].enabled.size());


        moveit_msgs::AllowedCollisionMatrix currentACM_init = currentACM;
        currentACM.entry_names.push_back(object_id);
        moveit_msgs::AllowedCollisionEntry entry;
        entry.enabled.resize(currentACM.entry_names.size());

        int index;
        for(size_t i=0; i < currentACM.entry_names.size(); ++i){
            //ROS_ERROR_STREAM("acm_entry_names before " << i << " "<< currentACM.entry_names.at(i));
            std::string name = currentACM.entry_names.at(i);
            if(strcmp(name.c_str(),"right_hand_barrett_link")==0){
                index = i;
            }
        }

        for(size_t i = 0; i < entry.enabled.size(); i++){
            if(i==index){
                entry.enabled[i] = true;
            }else{entry.enabled[i] = false;}
        }

        //add new row to allowed collsion matrix
        currentACM.entry_values.push_back(entry);
        for(int i = 0; i < currentACM.entry_values.size(); i++)
        {
            //extend the last column of the matrix
            if(i==index){
                currentACM.entry_values[i].enabled.push_back(true);
            }else{
               currentACM.entry_values[i].enabled.push_back(false);
            }
        }

        newSceneDiff.is_diff = true;
        newSceneDiff.allowed_collision_matrix = currentACM;
        ApplyPlanningScene msg_apply;
        msg_apply.request.scene=newSceneDiff;
        apply_planning_scene_client.call(msg_apply);

       if(!get_planning_scene_client.call(scene_srv))
       {
         ROS_WARN("Failed to call service /get_planning_scene");
       }
       else
       {
           ROS_INFO_STREAM("Modified scene!");
           currentScene = scene_srv.response.scene;
           moveit_msgs::AllowedCollisionMatrix currentACM = currentScene.allowed_collision_matrix;

           ROS_ERROR_STREAM("size of acm_entry_names after " << currentACM.entry_names.size());
           ROS_ERROR_STREAM("size of acm_entry_values after " << currentACM.entry_values.size());
           ROS_ERROR_STREAM("size of acm_entry_values[0].entries after " << currentACM.entry_values[0].enabled.size());
       }
    }
*/

    //std::vector<std::string> attached_object_touch_links;
    //const robot_model::JointModelGroup * joint_model_group = robot_model->getJointModelGroup(this->end_effector);
    //std::vector<std::string> names = joint_model_group->getJointModelNames();
    //attached_object_touch_links = names; attached_object_touch_links.push_back("right_hand_barrett_link");
    //std::vector<std::string> allowed_touch_objects; allowed_touch_objects.push_back(object_id);

    moveit_msgs::PickupGoal goal;
    goal.target_name = object_id;
    goal.group_name = group_name_arm;
    goal.end_effector = end_effector;
    //goal.allowed_touch_objects = allowed_touch_objects;
    //goal.attached_object_touch_links = attached_object_touch_links;
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

void HumanoidPlanner::RPY_matrix(std::vector<double> rpy, Matrix3d &Rot)
{
    Rot = Matrix3d::Zero();

    if(!rpy.empty()){
        double roll = rpy.at(0); // around z
        double pitch = rpy.at(1); // around y
        double yaw = rpy.at(2); // around x

        // Rot = Rot_z * Rot_y * Rot_x

        Rot(0,0) = cos(roll)*cos(pitch);  Rot(0,1) = cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw); Rot(0,2) = sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw);
        Rot(1,0) = sin(roll)*cos(pitch);  Rot(1,1) = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw); Rot(1,2) = sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw);
        Rot(2,0) = -sin(pitch);           Rot(2,1) = cos(pitch)*sin(yaw);                              Rot(2,2) = cos(pitch)*cos(yaw);

    }
}

void HumanoidPlanner::openBarrettHand(std::vector<double> finalHand, trajectory_msgs::JointTrajectory &posture)
{

    const robot_model::JointModelGroup * joint_model_group = robot_model->getJointModelGroup(this->group_name_hand);
    std::vector<std::string> names = joint_model_group->getJointModelNames();
    //posture.joint_names.resize(names.size());
    posture.points.resize(1);
    //posture.points[0].positions.resize(names.size());
    for(size_t i =0; i < names.size(); ++i){
        //std::cout << names.at(i).c_str() << endl;
        //posture.joint_names[i] = names.at(i);
        if(i==2 || i==5){
            posture.joint_names.push_back(names.at(i));
            posture.points[0].positions.push_back(finalHand.at(0));
        }else if(i==3){
            posture.joint_names.push_back(names.at(i));
            if((finalHand.at(1)-AP) < 0){
                posture.points[0].positions.push_back(0.0);
            }else{
                posture.points[0].positions.push_back(finalHand.at(1)-AP);
            }
        }else if(i==6){
            posture.joint_names.push_back(names.at(i));
            if((finalHand.at(2)-AP) < 0){
                posture.points[0].positions.push_back(0.0);
            }else{
                posture.points[0].positions.push_back(finalHand.at(2)-AP);
            }
        }else if(i==8){
            posture.joint_names.push_back(names.at(i));
            if((finalHand.at(3)-AP) < 0){
                posture.points[0].positions.push_back(0.0);
            }else{
                posture.points[0].positions.push_back(finalHand.at(3)-AP);
            }
        }
    }
}

void HumanoidPlanner::closedBarrettHand(std::vector<double> finalHand, trajectory_msgs::JointTrajectory& posture)
{
    const robot_model::JointModelGroup * joint_model_group = robot_model->getJointModelGroup(this->group_name_hand);
    std::vector<std::string> names = joint_model_group->getJointModelNames();
    //posture.joint_names.resize(names.size());
    posture.points.resize(1);
    //posture.points[0].positions.resize(names.size());
    for(size_t i =0; i < names.size(); ++i){
        //std::cout << names.at(i).c_str() << endl;
        //posture.joint_names[i] = names.at(i);
        if(i==2 || i==5){
            posture.joint_names.push_back(names.at(i));
            posture.points[0].positions.push_back(finalHand.at(0));
        }else if(i==3){
            posture.joint_names.push_back(names.at(i));
            posture.points[0].positions.push_back(finalHand.at(1));
        }else if(i==6){
            posture.joint_names.push_back(names.at(i));
            posture.points[0].positions.push_back(finalHand.at(2));
        }else if(i==8){
            posture.joint_names.push_back(names.at(i));
            posture.points[0].positions.push_back(finalHand.at(3));
        }
    }
}



} // namespace moveit_planning
