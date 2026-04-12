#ifndef __PHYSICS_CONTEXT_HPP
#define __PHYSICS_CONTEXT_HPP

#include "common/types.hpp"
#include <string>

// Forward declarations - no Graphics dependency
class ElasticMaterial;
namespace Geometry { class EmbreeScene; }
namespace Config { class SimulationConfig; }

namespace Sim
{

/**
 * Lightweight interface providing the runtime state that physics objects need.
 * This decouples XPBDMeshObject (and other physics objects) from the full
 * Simulation class, allowing them to be used without Graphics dependencies.
 */
class PhysicsContext
{
public:
    virtual ~PhysicsContext() = default;

    /** Time step size */
    virtual Real dt() const = 0;

    /** Gravity acceleration vector */
    virtual const Vec3r& gAccel() const = 0;

    /** Look up an elastic material by name */
    virtual const ElasticMaterial& getMaterial(const std::string& name) const = 0;

    /** Embree scene for ray-tracing queries (may return nullptr) */
    virtual const Geometry::EmbreeScene* embreeScene() const = 0;

    /** Simulation config (may return nullptr for standalone usage) */
    virtual const Config::SimulationConfig* config() const = 0;
};

} // namespace Sim

#endif // __PHYSICS_CONTEXT_HPP
