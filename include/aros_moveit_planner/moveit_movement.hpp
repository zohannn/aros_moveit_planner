#ifndef MOVEIT_MOVEMENT_HPP
#define MOVEIT_MOVEMENT_HPP

#include "common.hpp"

using namespace std;
using namespace common;

namespace humanoid_planning{

class Movement
{
public:
    /**
     * @brief Movement, a constructor
     * @param type
     * @param arm
     */
    Movement(int type, int arm);

    /**
     * @brief Movement, a constructor
     * @param type
     * @param arm
     * @param obj
     */
    Movement(int type, int arm, string obj);

    /**
     * @brief Movement, a constructor
     * @param type
     * @param arm
     * @param obj
     * @param grip_id
     * @param prec
     */
    Movement(int type, int arm, string obj, int grip_id, bool prec);

    /**
     * @brief Movement, a constructor
     * @param type
     * @param arm
     * @param obj
     * @param obj_eng
     * @param grip_id
     * @param prec
     */
    Movement(int type, int arm, string obj, string obj_eng, int grip_id, bool prec);

    /**
     * @brief Movement, a copy constructor
     * @param mov
     */
    Movement(const Movement& mov);

    /**
     * @brief ~Movement, a destructor
     */
    ~Movement();

    /**
    * @brief This method sets the type of the movement.
    *  @param t
    */
    void setType(int t);

    /**
     * @brief This method sets the type of grip related to the movement.
     * @param index
     * @param prec
     */
    void setGrip(int index, bool prec);

    /**
     * @brief This method sets the object that is manipulated during the movement
     * @param obj
     */
    void setObject(objectPtr obj);

    /**
     * @brief This method sets the object that is manipulated before the movement starts
     * @param obj
     */
    void setObjectInit(objectPtr obj);

    /**
     * @brief This method sets the object that is involved in engaging/disengaging movements
     * @param obj_eng
     */
    void setObjectEng(objectPtr obj_eng);

    /**
     * @brief This method sets the arm/s in movement
     * @param a
     */
    void setArm(int a);

    /**
     * @brief This method gets the the status of the movement
     * @param exec
     */
    void setExecuted(bool exec);

    /**
     * @brief This method gets the code of the movement
     * @return
     */
    int getType();

    /**
     * @brief This method gets the description of the movement
     * @return
     */
    string getStrType();

    /**
     * @brief This method gets the code of the grip
     * @return
     */
    int getGrip();

    /**
     * @brief This method gets the description of the grip
     * @return
     */
    string getGripStr();

    /**
     * @brief This method gets the name of the object that is being manipulated during the movement
     * @return
     */
    string getObject();

    /**
     * @brief This method gets the name of the object that is being manipulated before the movement starts
     * @return
     */
    string getObjectInit();

    /**
     * @brief This method gets the name of the object that is involved in engaging/disengaging movements
     * @return
     */
    string getObjectEng();

    /**
     * @brief This method gets information about the movement
     * @return
     */
    string getInfoLine();

    /**
     * @brief This method gets the arm/s in movement
     * @return
     */
    int getArm();

    /**
     * @brief This method gets the the status of the movement
     * @return
     */
    bool getExecuted();

private:
    int arm;  /**< 0 = both arms, 1 = right-arm, 2 = left-arm */


   /**
     * @brief type of the movement (code)
     * <table>
     * <caption id="multi_row">Types of the movement</caption>
     * <tr><th>Type      <th>Code
     * <tr><td>Reach-to-grasp <td>0
     * <tr><td>Reaching <td>1
     * <tr><td>Transport <td>2
     * <tr><td>Engage <td>3
     * <tr><td>Disengage <td>4
     * <tr><td>Go park <td>5
     * </table>
     */
    int type;
    string strType; /**< type of the movement (string) */

    /**
     * @brief type of the grip (code)
     * <table>
     * <caption id="multi_row">Types of grip</caption>
     * <tr><th>Type      <th>Precision (prec = true) <th>Full (prec = false) <th> Index
     * <tr><td>Side thumb left <td>111 <td>211 <td>0
     * <tr><td>Side thumb right <td>112 <td>212 <td>1
     * <tr><td>Side thumb up <td>113 <td>213 <td>2
     * <tr><td>Side thumb down <td>114 <td>214 <td>3
     * <tr><td>Above <td>121 <td>221 <td>4
     * <tr><td>Below <td>122 <td>222 <td>5
     * <tr><td>No Grip <td>- <td>- <td>6
     * </table>
     */
    int grip_code;
    string grip_str; /**< type of the grip (string) */
    string obj; /**< name of the object being manipulated in the movement */
    string obj_init; /**< name of the object being manipulated before the movement starts */
    string obj_eng; /**< name of the object involved in engaging/disengaging movements */
    bool executed; /**< true if the movement has been executed, false otherwise */

    // Types of movements
    // ||||||||||||||||||||||||||
    // |||||||||||||||||| Code ||
    // ||Reach-to-grasp |   0  ||
    // ||Reaching       |   1  ||
    // ||Transport      |   2  ||
    // ||Engage         |   3  ||
    // ||Disengage      |   4  ||
    // ||Go park        |   5  ||
    // ||||||||||||||||||||||||||

    // Types of Grip and related code
    // |||||||||||||||||||||||||||||||||||||||||||||||||
    // |||||||||||||||||||| Precision |  Full | Index ||
    // ||Side thumb left  |    111    |  211  |   0   ||
    // ||Side thumb right |    112    |  212  |   1   ||
    // ||Side thumb up    |    113    |  213  |   2   ||
    // ||Side thumb down  |    114    |  214  |   3   ||
    // ||Above            |    121    |  221  |   4   ||
    // ||Below            |    122    |  222  |   5   ||
    // ||No Grip          |     -     |   -   |   6   ||
    // |||||||||||||||||||||||||||||||||||||||||||||||||


};

}//namespace humanoid_planning
#endif // MOVEIT_MOVEMENT_HPP
