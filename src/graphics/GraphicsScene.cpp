#include "graphics/GraphicsScene.hpp"

namespace Graphics
{

GraphicsScene::GraphicsScene(const std::string& name)
    : _name(name)
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