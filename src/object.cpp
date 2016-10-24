#include "../include/aros_moveit_planner/object.hpp"
#include <boost/format.hpp>


namespace humanoid_planning{

Object::Object()
{

    this->m_name = "";
    this->m_pos.Xpos=0;this->m_pos.Ypos=0;this->m_pos.Zpos=0;
    this->m_or.pitch = 0; this->m_or.roll = 0; this->m_or.yaw = 0;
    this->m_size.Xsize = 0; this->m_size.Ysize = 0; this->m_size.Zsize = 0;
    this->handle = -1;
    this->handle_body = -1;
    this->p_targetRight = targetPtr(new Target());
    this->p_targetLeft = targetPtr(new Target());
    this->p_engage = engagePtr(new EngagePoint());

    this->m_targetRightEnabled = false;
    this->m_targetLeftEnabled = false;

    this->setup_features = true;

}


Object::Object(string name)
{

    this->m_name = name;
    this->m_pos.Xpos=0;this->m_pos.Ypos=0;this->m_pos.Zpos=0;
    this->m_or.pitch = 0; this->m_or.roll = 0; this->m_or.yaw = 0;
    this->m_size.Xsize = 0; this->m_size.Ysize = 0; this->m_size.Zsize = 0;
    this->handle = -1;
    this->handle_body = -1;
    this->p_targetRight = targetPtr(new Target());
    this->p_targetLeft = targetPtr(new Target());
    this->p_engage = engagePtr(new EngagePoint());

    this->m_targetRightEnabled = false;
    this->m_targetLeftEnabled = false;

    this->setup_features = true;

}

Object::Object(string name, pos ppos, orient oor, dim ssize,
               Target* pTR, Target* pTL,
               EngagePoint* pEng)
{

    this->m_name = name;
    this->m_pos = ppos;
    this->m_or = oor;
    this->m_size = ssize;
    this->handle = -1;
    this->handle_body = -1;
    this->p_targetRight = targetPtr(pTR);
    this->p_targetLeft = targetPtr(pTL);
    this->p_engage = engagePtr(pEng);

    this->m_targetRightEnabled = false;
    this->m_targetLeftEnabled = false;

    this->setup_features = false;

}


Object::Object(const Object &obj)
{

    this->m_name = obj.m_name;
    this->m_pos = obj.m_pos;
    this->m_or = obj.m_or;
    this->m_size = obj.m_size;

    this->p_targetRight = targetPtr(new Target(*obj.p_targetRight.get()));
    this->p_targetLeft = targetPtr(new Target(*obj.p_targetLeft.get()));
    this->p_engage = engagePtr(new EngagePoint(*obj.p_engage.get()));

    this->handle = obj.handle;
    this->handle_body = obj.handle_body;

    this->m_targetRightEnabled = obj.m_targetRightEnabled;
    this->m_targetLeftEnabled = obj.m_targetLeftEnabled;

    this->setup_features = obj.setup_features;


}


Object::~Object()
{

}




void Object::setPos(pos& ppos, bool update_features)
{


    if (update_features){

        float x;
        float y;
        float z;
        Matrix4f trans_tar_right;
        Matrix4f trans_tar_left;
        Matrix4f trans_engage;

        Matrix4f trans_obj;
        Matrix4f inv_trans_obj;

        Matrix4f trans_obj_tar_right;
        Matrix4f trans_obj_tar_left;
        Matrix4f trans_obj_engage;

        x = this->m_pos.Xpos;
        y = this->m_pos.Ypos;
        z = this->m_pos.Zpos;
        Matrix3f Rot;
        this->RPY_matrix(Rot);

        trans_obj(0,0) = Rot(0,0); trans_obj(0,1) = Rot(0,1); trans_obj(0,2) = Rot(0,2); trans_obj(0,3) = x;
        trans_obj(1,0) = Rot(1,0); trans_obj(1,1) = Rot(1,1); trans_obj(1,2) = Rot(1,2); trans_obj(1,3) = y;
        trans_obj(2,0) = Rot(2,0); trans_obj(2,1) = Rot(2,1); trans_obj(2,2) = Rot(2,2); trans_obj(2,3) = z;
        trans_obj(3,0) = 0;        trans_obj(3,1) = 0;        trans_obj(3,2) = 0;        trans_obj(3,3) = 1;

        this->getTar_right_matrix(trans_tar_right);
        this->getTar_left_matrix(trans_tar_left);
        this->getEngage_matrix(trans_engage);


        inv_trans_obj = trans_obj.inverse();
        trans_obj_tar_right=inv_trans_obj * trans_tar_right; // transformation matrix object - target right
        trans_obj_tar_left=inv_trans_obj * trans_tar_left; // transformation matrix object - target left
        trans_obj_engage=inv_trans_obj * trans_engage; // transformation matrix object - engage


        // set the new position of the object and update the features
        this->m_pos = ppos;
        x = this->m_pos.Xpos; trans_obj(0,3)=x;
        y = this->m_pos.Ypos; trans_obj(1,3)=y;
        z = this->m_pos.Zpos; trans_obj(2,3)=z;

        trans_tar_right = trans_obj * trans_obj_tar_right; // updated matrix of target right
        pos new_tar_right_pos;
        new_tar_right_pos.Xpos = trans_tar_right(0,3);
        new_tar_right_pos.Ypos = trans_tar_right(1,3);
        new_tar_right_pos.Zpos = trans_tar_right(2,3);
        this->p_targetRight->setPos(new_tar_right_pos);

        trans_tar_left = trans_obj * trans_obj_tar_left; // updated matrix of target left
        pos new_tar_left_pos;
        new_tar_left_pos.Xpos = trans_tar_left(0,3);
        new_tar_left_pos.Ypos = trans_tar_left(1,3);
        new_tar_left_pos.Zpos = trans_tar_left(2,3);
        this->p_targetLeft->setPos(new_tar_left_pos);

        trans_engage = trans_obj * trans_obj_engage; // updated matrix of the engage point
        pos new_engage_pos;
        new_engage_pos.Xpos = trans_engage(0,3);
        new_engage_pos.Ypos = trans_engage(1,3);
        new_engage_pos.Zpos = trans_engage(2,3);
        this->p_engage->setPos(new_engage_pos);


    }else{
        this->m_pos = ppos;

    }
}

void Object::setOr(orient& oor, bool update_features)
{

    if (update_features){

        Matrix4f trans_tar_right;
        Matrix4f trans_tar_left;
        Matrix4f trans_engage;

        Matrix4f trans_obj;
        Matrix4f inv_trans_obj;

        Matrix4f trans_obj_tar_right;
        Matrix4f trans_obj_tar_left;
        Matrix4f trans_obj_engage;

        this->Trans_matrix(trans_obj);
        this->getTar_right_matrix(trans_tar_right);
        this->getTar_left_matrix(trans_tar_left);
        this->getEngage_matrix(trans_engage);


        inv_trans_obj = trans_obj.inverse();
        trans_obj_tar_right=inv_trans_obj * trans_tar_right; // transformation matrix object - target right
        trans_obj_tar_left=inv_trans_obj * trans_tar_left; // transformation matrix object - target left
        trans_obj_engage=inv_trans_obj * trans_engage; // transformation matrix object - engage

        // update the new orientation of the object and update the features
        this->m_or = oor;
        this->Trans_matrix(trans_obj);

        trans_tar_right = trans_obj * trans_obj_tar_right; // updated matrix of target right
        std::vector<float> rpy_tar_right;
        this->getRPY(trans_tar_right,rpy_tar_right);
        orient new_tar_right_or;
        new_tar_right_or.roll = rpy_tar_right.at(0); new_tar_right_or.pitch = rpy_tar_right.at(1); new_tar_right_or.yaw = rpy_tar_right.at(2);
        this->p_targetRight->setOr(new_tar_right_or);

        trans_tar_left = trans_obj * trans_obj_tar_left; // updated matrix of target left
        std::vector<float> rpy_tar_left;
        this->getRPY(trans_tar_left,rpy_tar_left);
        orient new_tar_left_or;
        new_tar_left_or.roll = rpy_tar_left.at(0); new_tar_left_or.pitch = rpy_tar_left.at(1); new_tar_left_or.yaw = rpy_tar_left.at(2);
        this->p_targetLeft->setOr(new_tar_left_or);

        trans_engage = trans_obj * trans_obj_engage; // updated matrix of engage point
        std::vector<float> rpy_engage;
        this->getRPY(trans_engage,rpy_engage);
        orient new_engage_or;
        new_engage_or.roll = rpy_engage.at(0); new_engage_or.pitch = rpy_engage.at(1); new_engage_or.yaw = rpy_engage.at(2);
        this->p_engage->setOr(new_engage_or);



    }else{
        this->m_or = oor;

    }
}



void Object::setSize(dim ssize)
{

    this->m_size = ssize;
}

bool Object::setTargetRight(targetPtr tr)
{

    if (this->setup_features){
        this->p_targetRight = targetPtr(new Target(*tr.get()));
        return true;
    }else{
        return false;
    }
}

void Object::setHandle(int h)
{

    this->handle = h;
}

void Object::setHandleBody(int h)
{

    this->handle_body = h;
}


bool Object::setTargetLeft(targetPtr tl)
{

    if (this->setup_features){
        this->p_targetLeft = targetPtr(new Target(*tl.get()));
        return true;
    }else{
        return false;
    }
}

bool Object::setEngagePoint(engagePtr eng)
{

    if (this->setup_features){
        this->p_engage = engagePtr(new EngagePoint(*eng.get()));
        return true;
    }else{
        return false;
    }
}


void Object::setTargetRightEnabled(bool c)
{

    this->m_targetRightEnabled = c;
}

void Object::setTargetLeftEnabled(bool c)
{

    this->m_targetLeftEnabled = c;
}


dim Object::getSize()
{

    return this->m_size;
}

int Object::getHandle()
{

    return this->handle;
}


int Object::getHandleBody()
{

    return this->handle_body;
}


targetPtr Object::getTargetRight()
{

    return p_targetRight;
}

targetPtr Object::getTargetLeft()
{

    return p_targetLeft;
}

engagePtr Object::getEngagePoint()
{

   return p_engage;
}


float Object::getRadius()
{

    return (max(this->m_size.Xsize,this->m_size.Ysize)/2.0);
}

bool Object::isTargetRightEnabled()
{

    return this->m_targetRightEnabled;
}

bool Object::isTargetLeftEnabled()
{

    return this->m_targetLeftEnabled;
}



string Object::getInfoLine()
{

    return  this->m_name + COLUMN + SPACE +
            XposSTR + str(boost::format("%d") % this->m_pos.Xpos) + MILLIMETERS + SEP +
            YposSTR + str(boost::format("%d") % this->m_pos.Ypos) + MILLIMETERS + SEP+
            ZposSTR + str(boost::format("%d") % this->m_pos.Zpos) + MILLIMETERS + SEP+
            RollSTR + str(boost::format("%d") % this->m_or.roll) + RAD + SEP+
            PitchSTR + str(boost::format("%d") % this->m_or.pitch) + RAD + SEP+
            YawSTR + str(boost::format("%d") % this->m_or.yaw) + RAD + SEP+
            XsizeSTR + str(boost::format("%d") % this->m_size.Xsize) + MILLIMETERS + SEP+
            YsizeSTR + str(boost::format("%d") % this->m_size.Ysize) + MILLIMETERS + SEP+
            ZsizeSTR + str(boost::format("%d") % this->m_size.Zsize)+ MILLIMETERS;


}


void Object::getTar_right_matrix(Matrix4f& mat)
{
    this->p_targetRight->Trans_matrix(mat);
}


void Object::getTar_left_matrix(Matrix4f &mat)
{

    this->p_targetLeft->Trans_matrix(mat);

}


void Object::getEngage_matrix(Matrix4f &mat)
{

    this->p_engage->Trans_matrix(mat);
}



void Object::getRPY(Matrix4f Trans, std::vector<float>& rpy)
{

    rpy = std::vector<float>(3);

    if(abs(Trans(0,0) < 1e-10) && (abs(Trans(1,0) < 1e-10))){

        rpy.at(0) = 0; // [rad]
        rpy.at(1) = atan2(-Trans(2,0),Trans(0,0)); // [rad]
        rpy.at(2) = atan2(-Trans(1,2),Trans(1,1)); // [rad]

    }else{

        rpy.at(0) = atan2(Trans(1,0),Trans(0,0)); // [rad]
        float sp = sin(rpy.at(0));
        float cp = cos(rpy.at(0));
        rpy.at(1) = atan2(-Trans(2,0), cp*Trans(0,0)+sp*Trans(1,0)); // [rad]
        rpy.at(2) = atan2(sp*Trans(0,2)-cp*Trans(1,2),cp*Trans(1,1)-sp*Trans(0,1)); // [rad]

    }


}

}// namespace humanoid_planning
