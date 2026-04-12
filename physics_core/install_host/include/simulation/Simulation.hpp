#ifndef __SIMULATION_HPP
#define __SIMULATION_HPP

// Minimal stub for physics-only build.
// Provides only what SimObject code actually calls.

#include "common/types.hpp"
#include "simobject/ElasticMaterial.hpp"
#include "geometry/embree/EmbreeScene.hpp"

#include <vector>
#include <string>
#include <iostream>
#include <cassert>

namespace Sim
{

class Simulation
{
public:
    Simulation(Real dt, const Vec3r& g_accel)
        : _time_step(dt), _g_accel(g_accel) {}

    Real dt() const { return _time_step; }
    const Vec3r& gAccel() const { return _g_accel; }
    const Geometry::EmbreeScene* embreeScene() const { return _embree_scene; }

    // Minimal config stub so CollisionScene can call _sim->config()->collisionAlgorithm()
    struct ConfigStub {
        std::string collisionAlgorithm() const { return "auto"; }
    };
    const ConfigStub* config() const { return &_config_stub; }

    void setDt(Real dt) { _time_step = dt; }
    void setGravity(const Vec3r& g) { _g_accel = g; }
    void setEmbreeScene(const Geometry::EmbreeScene* scene) { _embree_scene = scene; }

    void addMaterial(const ElasticMaterial& mat) { _materials.push_back(mat); }

    const ElasticMaterial& getMaterial(const std::string& name) const
    {
        for (const auto& mat : _materials)
        {
            if (mat.name() == name)
                return mat;
        }
        std::cerr << "Material with name " << name << " does not exist!" << std::endl;
        assert(0);
        return _materials[0];
    }

private:
    ConfigStub _config_stub;
    Real _time_step = 0.001;
    Vec3r _g_accel = Vec3r(0.0, -9.81, 0.0);
    const Geometry::EmbreeScene* _embree_scene = nullptr;
    std::vector<ElasticMaterial> _materials;
};

} // namespace Sim

#endif // __SIMULATION_HPP
