#include "../include/aros_moveit_planner/moveit_scenario.hpp"


namespace moveit_planning{


Scenario::Scenario()
{
    this->m_name = "";
    this->m_sceneID = -1;
}

Scenario::Scenario(string name, int id)
{
    this->m_name = name;
    this->m_sceneID = id;
}


Scenario::Scenario(const Scenario &scene)
{

    this->m_name=scene.m_name;
    this->m_sceneID=scene.m_sceneID;

    if(!scene.objs_list.empty()){
        this->objs_list = std::vector<objectPtr>(scene.objs_list.size());
        std::copy(scene.objs_list.begin(),scene.objs_list.end(),this->objs_list.begin());
    }

}


Scenario::~Scenario()
{

}


void Scenario::setName(string &name)
{

    this->m_name = name;
}

void Scenario::setID(int id)
{

    this->m_sceneID = id;
}

void Scenario::setObject(int pos, objectPtr obj)
{

    this->objs_list.at(pos) = objectPtr(new Object(*obj.get()));

}


string Scenario::getName()
{

    return this->m_name;
}

int Scenario::getID()
{

    return this->m_sceneID;
}


void Scenario::getObjects(std::vector<objectPtr> &objs)
{

    objs=this->objs_list;

}


void Scenario::addObject(Object* ob)
{

    this->objs_list.push_back(objectPtr(ob));
}

objectPtr Scenario::getObject(int pos)
{

    std::vector<objectPtr>::iterator ii = this->objs_list.begin();
    advance(ii,pos);

    return (*ii);

}

objectPtr Scenario::getObject(std::string obj_name)
{

    objectPtr obj = NULL;

    for(std::size_t i=0; i<this->objs_list.size();++i){
        string name = this->objs_list.at(i)->getName();
        if(boost::iequals(name,obj_name)){
            obj=this->objs_list.at(i);
            break;
        }
    }
    return obj;
}




}// namespace moveit_planning
