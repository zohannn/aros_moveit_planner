#ifndef MOVEIT_ENGAGEPOINT_HPP
#define MOVEIT_ENGAGEPOINT_HPP

#include "moveit_point.hpp"

namespace moveit_planning{

//! The EngagePoint class
/**
 * @brief This class defines the concept of point of engagement on a object.
 * A point of engagement is a point that represents a point of contact and a reference when engaging/disengaging movements are planned.
 * An engaging movement is a movement that causes the combination (engagement) of two or more objects, consequently forming one single structure.
 * On the contrary a disengaging movement is a movement that causes the disengagement of two or more objects that are forming one single structure.
 */
class EngagePoint: public Point
{
public:
    /**
     * @brief EngagePoint, default constructor.
     */
    EngagePoint();

    /**
     * @brief EngagePoint, a constructor.
     * @param name
     * @param ppos
     * @param oor
     */
    EngagePoint(string name, pos ppos, orient oor);

    /**
     * @brief EngagePoint, a copy constructor.
     * @param eng
     */
    EngagePoint(const EngagePoint& eng);


    /**
     * @brief ~EngagePoint, a destructor.
     */
    ~EngagePoint();

};

}// namespace moveit_planning

#endif // MOVEIT_ENGAGEPOINT_HPP
