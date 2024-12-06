#include "graphics/Easy3DGraphicsScene.hpp"
#include "graphics/Easy3DMeshGraphicsObject.hpp"
#include "config/MeshObjectConfig.hpp"

#include <easy3d/viewer/viewer.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/renderer/camera.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/renderer/text_renderer.h>
#include <easy3d/core/types.h>
#include <easy3d/util/resource.h>
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

size_t Easy3DGraphicsScene::addMeshObject(std::shared_ptr<MeshObject> obj, MeshObjectConfig* obj_config)
{

    // make sure object with name doesn't already exist in the scene
    if (getObject(obj->name()))
    {
        std::cout << "GraphicsObject with name " << obj->name() << " already exists in this GraphicsScene!" << std::endl;
        assert(0);
    }

    // create a new MeshGraphicsObject for visualizing this MeshObject
    std::unique_ptr<Easy3DMeshGraphicsObject> e3d_mgo = std::make_unique<Easy3DMeshGraphicsObject>(obj->name(), obj, obj_config);
    
    // add the easy3d::Drawables created by the Easy3DMeshGraphicsObject to the easy3d::Viewer
    for (const auto& pt_drawable : e3d_mgo->renderer()->points_drawables())
    {
        _easy3d_viewer->add_drawable(pt_drawable);
    }
    for (const auto& line_drawable : e3d_mgo->renderer()->lines_drawables())
    {
        _easy3d_viewer->add_drawable(line_drawable);
    }
    for (const auto& tri_drawable : e3d_mgo->renderer()->triangles_drawables())
    {
        _easy3d_viewer->add_drawable(tri_drawable);
    }

    // store the MeshGraphicsObject and return its position in the vector
    _graphics_objects.push_back(std::move(e3d_mgo));
    return _graphics_objects.size() - 1;
}

void Easy3DGraphicsScene::setCameraOrthographic()
{
    _easy3d_viewer->camera()->setType(easy3d::Camera::ORTHOGRAPHIC);
}

void Easy3DGraphicsScene::setCameraPerspective()
{
    _easy3d_viewer->camera()->setType(easy3d::Camera::PERSPECTIVE);
}

Eigen::Vector3d Easy3DGraphicsScene::cameraViewDirection() const
{
    easy3d::vec3 view_dir = _easy3d_viewer->camera()->viewDirection();
    return Eigen::Vector3d(view_dir.x, view_dir.y, view_dir.z);
}
void Easy3DGraphicsScene::setCameraViewDirection(const Eigen::Vector3d& view_dir)
{
    easy3d::vec3 e3d_view_dir(view_dir(0), view_dir(1), view_dir(2));
    _easy3d_viewer->camera()->setViewDirection(e3d_view_dir);
}

Eigen::Vector3d Easy3DGraphicsScene::cameraUpDirection() const
{
    easy3d::vec3 up_dir = _easy3d_viewer->camera()->upVector();
    return Eigen::Vector3d(up_dir.x, up_dir.y, up_dir.z);
}

Eigen::Vector3d Easy3DGraphicsScene::cameraRightDirection() const
{
    easy3d::vec3 right_dir = _easy3d_viewer->camera()->rightVector();
    return Eigen::Vector3d(right_dir.x, right_dir.y, right_dir.z);
}

Eigen::Vector3d Easy3DGraphicsScene::cameraPosition() const
{
    easy3d::vec3 camera_position = _easy3d_viewer->camera()->position();
    return Eigen::Vector3d(camera_position.x, camera_position.y, camera_position.z);
}
void Easy3DGraphicsScene::setCameraPosition(const Eigen::Vector3d& position)
{
    easy3d::vec3 e3d_position(position(0), position(1), position(2));
    _easy3d_viewer->camera()->setPosition(e3d_position);
}



} // namespace Graphics