#include <gtest/gtest.h>


#include "../../include/kinematics_helper.hpp"


using namespace trajectory_planner_moveit;
using namespace common;
using namespace std;

int argcc;
char** argvv;

TEST(TestSuite_kin_helper, Test_kin_goal1)
{
    ros::init(argcc, argvv, "kinematics_helper_test_goal1");
    ros::NodeHandle nh;

    geometry_msgs::Pose goal1;

    goal1.position.x = -0.26;
    goal1.position.y = 0.50;
    goal1.position.z = 1.25;
    goal1.orientation.x = 0.755872;
    goal1.orientation.y = -0.612878;
    goal1.orientation.z = -0.0464803;
    goal1.orientation.w = 0.22556;

    KinematicsHelper helper(nh);

    moveit_msgs::RobotState solution;

    geometry_msgs::PoseStamped pose;
    pose.header.stamp=ros::Time::now();
    pose.header.frame_id = FRAME_ID;
    pose.pose=goal1;

    bool success=false;

    ROS_INFO("Kinematic helper test goal 1: Computing test pose");
    if(helper.computeIK("right", pose, solution)) {
        printJointsValues(solution,"right");
        geometry_msgs::Pose pose;
        ROS_INFO("Kinematic helper test goal 1: Computing FK result for given solution");
        if(helper.computeFK(solution, "right_hand_sensor_link", pose)) {
            ROS_INFO(" Kinematic helper test goal 1: Calculated pose: [%.4f, %.4f, %.4f][%.4f, %.4f, %.4f, %.4f]", pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
            success=true;
        } else {
            ROS_ERROR("Kinematic helper test goal 1: FK computation failed.");
        }
    } else {
        ROS_ERROR("Kinematic helper test goal 1: Computation failed");

    }

    EXPECT_TRUE(success);

}


int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc,argv);
    argcc=argc;
    argvv=argv;
    return RUN_ALL_TESTS();
}
