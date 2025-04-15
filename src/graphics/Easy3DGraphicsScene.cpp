#include "graphics/Easy3DGraphicsScene.hpp"
#include "graphics/Easy3DMeshGraphicsObject.hpp"
#include "graphics/Easy3DSphereGraphicsObject.hpp"
#include "graphics/Easy3DBoxGraphicsObject.hpp"
#include "graphics/Easy3DCylinderGraphicsObject.hpp"
#include "graphics/Easy3DVirtuosoArmGraphicsObject.hpp"
#include "graphics/Easy3DVirtuosoRobotGraphicsObject.hpp"
#include "config/MeshObjectConfig.hpp"

#include "simobject/MeshObject.hpp"
#include "simobject/RigidPrimitives.hpp"
#include "simobject/VirtuosoArm.hpp"
#include "simobject/VirtuosoRobot.hpp"

#include <easy3d/viewer/viewer.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/renderer/camera.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/core/model.h>
#include <easy3d/core/types.h>
#include <easy3d/util/initializer.h>

namespace Graphics
{

Easy3DGraphicsScene::Easy3DGraphicsScene(const std::string& name)
    : GraphicsScene(name)
{

}

Easy3DGraphicsScene::~Easy3DGraphicsScene()
{

}

void Easy3DGraphicsScene::init()
{
    // initialize easy3d
    easy3d::initialize();

    // now we can create the Viewer
    _viewer = std::make_unique<Easy3DTextRenderingViewer>(_name);
    _easy3d_viewer = dynamic_cast<Easy3DTextRenderingViewer*>(_viewer.get());
    _easy3d_viewer->set_usage("");
}

void Easy3DGraphicsScene::update()
{
    // make sure graphics are up to date for all the objects in the scene
    for (auto& obj : _graphics_objects)
    {
        obj->update();
    }

    // update the viewer
    _viewer->update();
}

int Easy3DGraphicsScene::run()
{
    // run the viewer (will run indefinitely)
    return _easy3d_viewer->run();
}

int Easy3DGraphicsScene::addObject(const Sim::Object* obj, const ObjectConfig* obj_config)
{

    // make sure object with name doesn't already exist in the scene
    if (getObject(obj->name()))
    {
        std::cout << "GraphicsObject with name " << obj->name() << " already exists in this GraphicsScene!" << std::endl;
        assert(0);
    }

    std::unique_ptr<GraphicsObject> new_graphics_obj;

    // try downcasting to MeshObject
    if (const Sim::MeshObject* mo = dynamic_cast<const Sim::MeshObject*>(obj))
    {
        // if the downcast was successful, we should be able to downcast the ObjectConfig object to a MeshObjectConfig
        const MeshObjectConfig* mo_config = dynamic_cast<const MeshObjectConfig*>(obj_config);
        assert(mo_config);

        // create a new MeshGraphicsObject for visualizing this MeshObject
        // std::unique_ptr<Easy3DMeshGraphicsObject> e3d_mgo = std::make_unique<Easy3DMeshGraphicsObject>(obj->name(), mo->mesh(), mo_config);
        new_graphics_obj = std::make_unique<Easy3DMeshGraphicsObject>(obj->name(), mo->mesh(), mo_config);
    }

    // try downcasting to a RigidSphere
    else if (const Sim::RigidSphere* sphere = dynamic_cast<const Sim::RigidSphere*>(obj))
    {
        // create a new SphereGraphicsObject for visualizing the sphere
        // std::unique_ptr<Easy3DSphereGraphicsObject> e3d_sphere = std::make_unique<Easy3DSphereGraphicsObject>(sphere->name(), sphere);
        new_graphics_obj = std::make_unique<Easy3DSphereGraphicsObject>(sphere->name(), sphere);
        
    }

    // try downcasting to a RigidBox
    else if (const Sim::RigidBox* box = dynamic_cast<const Sim::RigidBox*>(obj))
    {
        new_graphics_obj = std::make_unique<Easy3DBoxGraphicsObject>(box->name(), box);
    }

    // try downcasting to a RigidCylinder
    else if (const Sim::RigidCylinder* cyl = dynamic_cast<const Sim::RigidCylinder*>(obj))
    {
        new_graphics_obj = std::make_unique<Easy3DCylinderGraphicsObject>(cyl->name(), cyl);
    }

    // try downcasting to a VirtuosoArm
    else if (const Sim::VirtuosoArm* arm = dynamic_cast<const Sim::VirtuosoArm*>(obj))
    {
        new_graphics_obj = std::make_unique<Easy3DVirtuosoArmGraphicsObject>(arm->name(), arm);
    }

    else if (const Sim::VirtuosoRobot* robot = dynamic_cast<const Sim::VirtuosoRobot*>(obj))
    {
        new_graphics_obj = std::make_unique<Easy3DVirtuosoRobotGraphicsObject>(robot->name(), robot);

        if (robot->hasArm1())
        {
            std::unique_ptr<Easy3DVirtuosoArmGraphicsObject> arm1_graphics_obj = std::make_unique<Easy3DVirtuosoArmGraphicsObject>(robot->arm1()->name(), robot->arm1());
            _addAllDrawablesForModel(arm1_graphics_obj.get());
            _graphics_objects.push_back(std::move(arm1_graphics_obj));
        }
        if (robot->hasArm2())
        {
            std::unique_ptr<Easy3DVirtuosoArmGraphicsObject> arm2_graphics_obj = std::make_unique<Easy3DVirtuosoArmGraphicsObject>(robot->arm2()->name(), robot->arm2());
            _addAllDrawablesForModel(arm2_graphics_obj.get());
            _graphics_objects.push_back(std::move(arm2_graphics_obj));
        }
        
    }

    // all Easy3D graphics objects should be easy3d::Models themselves
    easy3d::Model* model = dynamic_cast<easy3d::Model*>(new_graphics_obj.get());
    assert(model);

    _addAllDrawablesForModel(model);

    

    // finally add the GraphicsObject to the list of GraphicsObjects
    _graphics_objects.push_back(std::move(new_graphics_obj));


    
    return _graphics_objects.size() - 1;
}

void Easy3DGraphicsScene::_addAllDrawablesForModel(const easy3d::Model* model)
{
    // add the easy3d::Drawables created by the Easy3DMeshGraphicsObject to the easy3d::Viewer
    for (const auto& pt_drawable : model->renderer()->points_drawables())
    {
        _easy3d_viewer->add_drawable(pt_drawable);
    }
    for (const auto& line_drawable : model->renderer()->lines_drawables())
    {
        _easy3d_viewer->add_drawable(line_drawable);
    }
    for (const auto& tri_drawable : model->renderer()->triangles_drawables())
    {
        _easy3d_viewer->add_drawable(tri_drawable);
    }
}

void Easy3DGraphicsScene::setCameraOrthographic()
{
    _easy3d_viewer->camera()->setType(easy3d::Camera::ORTHOGRAPHIC);
}

void Easy3DGraphicsScene::setCameraPerspective()
{
    _easy3d_viewer->camera()->setType(easy3d::Camera::PERSPECTIVE);
}

void Easy3DGraphicsScene::setCameraFOV(Real fov)
{
    _easy3d_viewer->camera()->setFieldOfView(fov);
}

Vec3r Easy3DGraphicsScene::cameraViewDirection() const
{
    easy3d::vec3 view_dir = _easy3d_viewer->camera()->viewDirection();
    return Vec3r(view_dir.x, view_dir.y, view_dir.z);
}
void Easy3DGraphicsScene::setCameraViewDirection(const Vec3r& view_dir)
{
    easy3d::vec3 e3d_view_dir(view_dir(0), view_dir(1), view_dir(2));
    _easy3d_viewer->camera()->setViewDirection(e3d_view_dir);
}

Vec3r Easy3DGraphicsScene::cameraUpDirection() const
{
    easy3d::vec3 up_dir = _easy3d_viewer->camera()->upVector();
    return Vec3r(up_dir.x, up_dir.y, up_dir.z);
}

void Easy3DGraphicsScene::setCameraUpDirection(const Vec3r& up_dir)
{
    easy3d::vec3 e3d_up_dir(up_dir(0), up_dir(1), up_dir(2));
    _easy3d_viewer->camera()->setUpVector(e3d_up_dir);
}

Vec3r Easy3DGraphicsScene::cameraRightDirection() const
{
    easy3d::vec3 right_dir = _easy3d_viewer->camera()->rightVector();
    return Vec3r(right_dir.x, right_dir.y, right_dir.z);
}

Vec3r Easy3DGraphicsScene::cameraPosition() const
{
    easy3d::vec3 camera_position = _easy3d_viewer->camera()->position();
    return Vec3r(camera_position.x, camera_position.y, camera_position.z);
}
void Easy3DGraphicsScene::setCameraPosition(const Vec3r& position)
{
    easy3d::vec3 e3d_position(position(0), position(1), position(2));
    _easy3d_viewer->camera()->setPosition(e3d_position);
}



} // namespace Graphics