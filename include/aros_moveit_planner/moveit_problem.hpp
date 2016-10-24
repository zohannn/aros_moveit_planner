#ifndef MOVEIT_PROBLEM_HPP
#define MOVEIT_PROBLEM_HPP

#include "common.hpp"
#include "moveit_movement.hpp"

using namespace Eigen;
using namespace common;

namespace humanoid_planning{

typedef boost::shared_ptr<Movement> movementPtr; /**< shared pointer to a movement */

//! The Problem class
/**
 * @brief The Problem class
 */
class Problem
{
public:

    /**
     * @brief Problem, a constructor
     */
    Problem();

    /**
     * @brief Problem, a constructor
     * @param mov
     */
    Problem(Movement* mov);

    /**
     * @brief Problem, a copy constructor
     * @param s
     */
    Problem(const Problem& s);

    /**
     * @brief ~Problem, a desctructor
     */
    ~Problem();


private:

    bool solved; /**< true if the problem has been solved */

   /**
     * @brief error log of the problem TO DO!!!!!!!!!!!!!!!!
     * <table>
     * <caption id="multi_row">Types of the errors</caption>
     * <tr><th>Type      <th>Code <th>Description
     * <tr><td>Any <td>0 <td>the problems have been successfully solved
     * <tr><td>Reach-to-grasp <td>10 <td>reach-to-grasp failure
     * <tr><td rowspan="4">Engage <td>130 <td>final posture engage sub-disengage not solved
     * <tr><td>13 <td>final posture engage not solved
     * <tr><td>131 <td>final posture engage sub-engage not solved
     * <tr><td>23 <td>bounce posture engage not solved
     * <tr><td>Go Park <td>25 <td> Go Park failure
     * </table>
     */
    int err_log;
    bool part_of_task; /**< true if the problem is part of a task */
    float dHOr; /**< distance between the right hand and the center of the object that is being manipulated */
    float dHOl; /**< distance between the left hand and the center of the object that is being manipulated */
    float dFF; /**< distance between the fingertip F3 and the fingertips F1 and F2 */
    float dFH; /**< distance between the fingers and the palm of the hand */
    MatrixXf optimalTraj; /**< planned trajectory */
    Tols tolerances; /**< tolerances and parameters of the optimization problem */
    movementPtr mov; /**< movement to be planned */
    int targetAxis; /**< approaching direction towards the target: 0 = none , 1 = x axis , 2 = y axis, 3 = z axis*/
    string obj_curr; /**< current object being manipulated */


};

}// namespace humanoid_planning

#endif // MOVEIT_PROBLEM_HPP
