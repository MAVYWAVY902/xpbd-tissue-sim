#include "graphics/GraphicsScene.hpp"

namespace Graphics
{

GraphicsScene::GraphicsScene(const std::string& name, const Config::SimulationRenderConfig& sim_render_config)
    : _name(name), _sim_render_config(sim_render_config)
{

}

GraphicsObject* GraphicsScene::getObject(const int index)
{
    return _graphics_objects[index].get();
}

GraphicsObject* GraphicsScene::getObject(const std::string& name)
{
    for (const auto& obj : _graphics_objects)
    {
        if (obj->name() == name)
        {
            return obj.get();
        }
    }

    return nullptr;
}

} // namespace Graphics