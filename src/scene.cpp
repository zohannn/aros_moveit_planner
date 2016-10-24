#include "../include/aros_moveit_planner/scene.hpp"


namespace humanoid_planning{


Scene::Scene()
{
    this->m_name = "";
    this->m_sceneID = -1;
}

Scene::Scene(string name, int id)
{
    this->m_name = name;
    this->m_sceneID = id;
}


Scene::Scene(const Scene &scene)
{

    this->m_name=scene.m_name;
    this->m_sceneID=scene.m_sceneID;

    if(!scene.objs_list.empty()){
        this->objs_list = std::vector<objectPtr>(scene.objs_list.size());
        std::copy(scene.objs_list.begin(),scene.objs_list.end(),this->objs_list.begin());
    }

}


Scene::~Scene()
{

}


void Scene::setName(string &name)
{

    this->m_name = name;
}

void Scene::setID(int id)
{

    this->m_sceneID = id;
}

void Scene::setObject(int pos, objectPtr obj)
{

    this->objs_list.at(pos) = objectPtr(new Object(*obj.get()));

}


string Scene::getName()
{

    return this->m_name;
}

int Scene::getID()
{

    return this->m_sceneID;
}


void Scene::getObjects(std::vector<objectPtr> &objs)
{

    objs=this->objs_list;

}


void Scene::addObject(Object* ob)
{

    this->objs_list.push_back(objectPtr(ob));
}

objectPtr Scene::getObject(int pos)
{

    std::vector<objectPtr>::iterator ii = this->objs_list.begin();
    advance(ii,pos);

    return (*ii);

}

objectPtr Scene::getObject(std::string obj_name)
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




}// namespace humanoid_planning
