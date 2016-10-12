#ifndef TRAJECTORY_PLANNER_HPP
#define TRAJECTORY_PLANNER_HPP

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/GetMotionPlan.h>

#include "kinematics_helper.hpp"
#include "base_planner.hpp"

using namespace std;

namespace trajectory_planner_moveit {

//! The TrajectoryPlanner class
/**
 * @brief The TrajectoryPlanner class
 * This class defines the functionalities of the planning process
 *
 */
class TrajectoryPlanner: public BasePlanner {

private:

    double planning_time; /**< maximum time allowed for planning [sec] */
    int planning_attempts; /**< numbers of planning attempts */
    string planner_id; /**< identity of the planner */
    int max_traj_pts; /**< maximum points of the trajectory*/


public:


};

}

#endif // TRAJECTORY_PLANNER_HPP
