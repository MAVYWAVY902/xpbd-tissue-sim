#ifndef __OPENGL_GRAPHICS_SCENE_HPP
#define __OPENGL_GRAPHICS_SCENE_HPP

#include "graphics/GraphicsScene.hpp"
#include "graphics/opengl/OpenGLViewer.hpp"

#include "simobject/Object.hpp"
#include "config/simobject/MeshObjectConfig.hpp"
#include "config/simobject/ObjectConfig.hpp"

#include <memory>

namespace Sim { class RigidSphere; }

namespace Graphics
{

class OpenGLStaticModel;  // forward declare

class OpenGLGraphicsScene : public GraphicsScene
{
    public:
    explicit OpenGLGraphicsScene(const std::string& name, const Config::SimulationRenderConfig& sim_render_config);
    virtual ~OpenGLGraphicsScene();

    virtual void init() override;
    virtual void update() override;
    virtual int run() override;

    virtual int addObject(const Sim::Object* obj, const Config::ObjectRenderConfig& render_config) override;

    /** Add a static decorative model (no physics). */
    void addStaticModel(const std::string& filepath, const Eigen::Matrix4f& transform);

    virtual void setCameraOrthographic() override;
    virtual void setCameraPerspective() override;
    virtual void setCameraFOV(Real fov) override;

    virtual Vec3r cameraViewDirection() const override;
    virtual void setCameraViewDirection(const Vec3r& view_dir) override;

    virtual Vec3r cameraUpDirection() const override;
    virtual void setCameraUpDirection(const Vec3r& up_dir) override;

    virtual Vec3r cameraRightDirection() const override;

    virtual Vec3r cameraPosition() const override;
    virtual void setCameraPosition(const Vec3r& position) override;

    protected:
    /** Convenience typed pointer to avoid casts everywhere. */
    OpenGLViewer* _opengl_viewer = nullptr;

    /** Mesh shader program (Blinn-Phong). */
    unsigned int _mesh_shader = 0;
    /** Static model shader (Blinn-Phong + normal mapping). */
    unsigned int _static_model_shader = 0;
    /** Line/point shader program (unlit, uniform color). */
    unsigned int _simple_shader = 0;

    private:
    void _initShaders();
    static void _drawSceneCallback(void* user_data);
    void _drawScene() const;

    /** Static decorative models (surgical table, etc). */
    std::vector<std::unique_ptr<OpenGLStaticModel>> _static_models;

    /** Tracked sphere objects (grasp cursor, etc). */
    struct SphereEntry {
        const Sim::RigidSphere* sphere;
        std::array<float, 4> color;
    };
    std::vector<SphereEntry> _sphere_objects;

    /** Sphere GL data (generated once). */
    mutable unsigned int _sphere_vao = 0, _sphere_vbo = 0, _sphere_nbo = 0, _sphere_ebo = 0;
    mutable int _sphere_num_indices = 0;
    void _ensureSphereGL() const;
};

} // namespace Graphics

#endif // __OPENGL_GRAPHICS_SCENE_HPP
