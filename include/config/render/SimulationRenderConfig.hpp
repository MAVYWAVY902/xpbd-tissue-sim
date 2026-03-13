#ifndef __SIMULATION_RENDER_CONFIG_HPP
#define __SIMULATION_RENDER_CONFIG_HPP

#include "config/Config.hpp"

#include <optional>

namespace Config
{

class SimulationRenderConfig : public Config
{
    public:
    explicit SimulationRenderConfig()
        : Config()
    {
        
    }

    explicit SimulationRenderConfig(const YAML::Node& node)
        : Config(node)
    {
        _extractParameter("hdr-image-filename", node, _hdr_image_filename);
        _extractParameter("create-skybox", node, _create_skybox);
        _extractParameter("exposure", node, _exposure);

        _extractParameter("background-image", node, _background_image);

        _extractParameter("window-width", node, _window_width);
        _extractParameter("window-height", node, _window_height);

        _extractParameter("background-u-offset", node, _background_u_offset);
        _extractParameter("background-v-offset", node, _background_v_offset);

        _extractParameter("camera-position", node, _camera_position);
        _extractParameter("camera-view-direction", node, _camera_view_direction);
        _extractParameter("camera-up-direction", node, _camera_up_direction);
        _extractParameter("camera-fov", node, _camera_fov);
    }

    const std::optional<std::string>& hdrImageFilename() const { return _hdr_image_filename.value; }
    bool createSkybox() const { return _create_skybox.value; }
    Real exposure() const { return _exposure.value; }
    const std::optional<std::string>& backgroundImage() const { return _background_image.value; }

    int windowWidth() const { return _window_width.value; }
    int windowHeight() const { return _window_height.value; }

    Real backgroundUOffset() const { return _background_u_offset.value; }
    Real backgroundVOffset() const { return _background_v_offset.value; }

    const std::optional<Vec3r>& cameraPosition() const { return _camera_position.value; }
    const std::optional<Vec3r>& cameraViewDirection() const { return _camera_view_direction.value; }
    const std::optional<Vec3r>& cameraUpDirection() const { return _camera_up_direction.value; }
    const std::optional<Real>& cameraFOV() const { return _camera_fov.value; }

    protected:
    ConfigParameter<std::optional<std::string>> _hdr_image_filename;
    ConfigParameter<std::optional<std::string>> _background_image;
    ConfigParameter<bool> _create_skybox = ConfigParameter<bool>(true);
    ConfigParameter<Real> _exposure = ConfigParameter<Real>(0.5);

    ConfigParameter<int> _window_width = ConfigParameter<int>(600);
    ConfigParameter<int> _window_height = ConfigParameter<int>(600);

    ConfigParameter<Real> _background_u_offset = ConfigParameter<Real>(0.0);
    ConfigParameter<Real> _background_v_offset = ConfigParameter<Real>(0.0);

    ConfigParameter<std::optional<Vec3r>> _camera_position;
    ConfigParameter<std::optional<Vec3r>> _camera_view_direction;
    ConfigParameter<std::optional<Vec3r>> _camera_up_direction;
    ConfigParameter<std::optional<Real>> _camera_fov;

};

} // namespace Config

#endif // __SIMULATION_RENDER_CONFIG_HPP