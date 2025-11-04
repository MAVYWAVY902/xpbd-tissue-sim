// #ifndef __MESH_OBJECT_HPP
// #define __MESH_OBJECT_HPP

// #include "geometry/Mesh.hpp"
// #include "geometry/TetMesh.hpp"
// #include "utils/MeshUtils.hpp"
// #include "config/simobject/ObjectConfig.hpp"
// #include "config/simobject/MeshObjectConfig.hpp"

// namespace Sim
// {

// class MeshObject
// {
//     // public typedefs
//     public:
//     using ConfigType = Config::MeshObjectConfig;

//     public:
//     MeshObject(const ConfigType* mesh_config, const Config::ObjectConfig* obj_config)
//     {
//         _filename = mesh_config->filename();

//         _initial_position = obj_config->initialPosition();
//         _initial_rotation = obj_config->initialRotation();

//         _initial_size = mesh_config->size();
//         _max_size = mesh_config->maxSize();
//     }

//     const Geometry::Mesh* mesh() const { return _mesh.get(); }

//     Geometry::Mesh* mesh() { return _mesh.get(); }

//     void loadAndConfigureMesh()
//     {
//         _loadMeshFromFile(_filename);

//         // order matters here...
//         // first apply scaling before rotating - either through the max-size criteria or a user-specified size
//         if (_max_size.has_value())
//         {
//             _mesh->resize(_max_size.value());
//         }

//         if (_initial_size.has_value())
//         {
//             _mesh->resize(_initial_size.value());
//         }

//         const Vec3r center_of_mass = _mesh->massCenter();

//         // move center of mass of the mesh to the specified initial position
//         _mesh->moveTogether(-center_of_mass + _initial_position);

//         // then do rigid transformation - rotation and translation
//         _mesh->rotateAbout(_initial_position, _initial_rotation);

//         // important: the current state of the mesh is the "initial" state that we would like to treat as the undeformed state
//         // as such, tell the mesh to recompute quantities so that it treats this state as the undeformed state
//         _mesh->setCurrentStateAsUndeformedState();
//     }

//     protected:

//     virtual void _loadMeshFromFile(const std::string& fname)
//     {
//         _mesh = std::make_unique<Geometry::Mesh>(MeshUtils::loadSurfaceMeshFromFile(fname));
//     }

//     void _scaleMesh()
//     {
        
//     }

//     protected:
//     std::unique_ptr<Geometry::Mesh> _mesh;

//     private:
//     std::string _filename;
//     Vec3r _initial_position;
//     Vec3r _initial_rotation;
//     std::optional<Vec3r> _initial_size;
//     std::optional<Real> _max_size;
    

// };

// ////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////

// class TetMeshObject : public MeshObject
// {
//     public:
//     TetMeshObject(const ConfigType* mesh_config, const Config::ObjectConfig* obj_config)
//         : MeshObject(mesh_config, obj_config)
//     {

//     }

//     const Geometry::TetMesh* tetMesh() const { return dynamic_cast<Geometry::TetMesh*>(_mesh.get()); }
//     Geometry::TetMesh* tetMesh() { return dynamic_cast<Geometry::TetMesh*>(_mesh.get()); }

//     protected:
//     virtual void _loadMeshFromFile(const std::string& fname)
//     {
//         _mesh = std::make_unique<Geometry::TetMesh>(MeshUtils::loadTetMeshFromGmshFile(fname));
//     }
// };

// } //namespace Sim

// #endif // __MESH_OBJECT_HPP


#ifndef __MESH_OBJECT_HPP
#define __MESH_OBJECT_HPP

#include "geometry/Mesh.hpp"
#include "geometry/TetMesh.hpp"
#include "utils/MeshUtils.hpp"
#include "config/simobject/ObjectConfig.hpp"
#include "config/simobject/MeshObjectConfig.hpp"

#include <unordered_map>

namespace Sim
{

class MeshObject
{
public:
    using ConfigType = Config::MeshObjectConfig;

public:
    MeshObject(const ConfigType* mesh_config, const Config::ObjectConfig* obj_config)
    {
        _filename = mesh_config->filename();

        _initial_position = obj_config->initialPosition();
        _initial_rotation = obj_config->initialRotation();

        _initial_size = mesh_config->size();
        _max_size = mesh_config->maxSize();
    }

    const Geometry::Mesh* mesh() const { return _mesh.get(); }
    Geometry::Mesh* mesh() { return _mesh.get(); }

    void loadAndConfigureMesh()
    {
        _loadMeshFromFile(_filename);

        // IMPORTANT: preserve gmsh node tag -> vertex index map across geometry ops.
        // Some mesh ops (resize/move/rotate/setCurrentStateAsUndeformedState) may
        // rebuild internal buffers and drop auxiliary maps.
        std::unordered_map<int, int> savedTagMap;
        if (auto* tet = dynamic_cast<Geometry::TetMesh*>(_mesh.get()))
        {
            // Take a snapshot of the current tag map (filled by MeshUtils::loadTetMeshFromGmshFile)
            savedTagMap = tet->tagMap();  // copy
        }

        // Order matters: resize (by max-size or explicit size) -> recenter -> rotate -> mark undeformed.
        if (_max_size.has_value())
        {
            _mesh->resize(_max_size.value());
        }

        if (_initial_size.has_value())
        {
            _mesh->resize(_initial_size.value());
        }

        const Vec3r center_of_mass = _mesh->massCenter();

        // Move COM to desired position first
        _mesh->moveTogether(-center_of_mass + _initial_position);

        // Then rotate about desired origin
        _mesh->rotateAbout(_initial_position, _initial_rotation);

        // Tell the mesh to treat the current configuration as the undeformed state
        _mesh->setCurrentStateAsUndeformedState();

        // --- Restore the tag map after geometry ops so Simulation can look up tags.
        if (auto* tet = dynamic_cast<Geometry::TetMesh*>(_mesh.get()))
        {
            auto& dst = tet->mutableTagMap();
            dst = std::move(savedTagMap);
        }
    }

protected:
    virtual void _loadMeshFromFile(const std::string& fname)
    {
        _mesh = std::make_unique<Geometry::Mesh>(MeshUtils::loadSurfaceMeshFromFile(fname));
    }

    void _scaleMesh() { /* intentionally empty */ }

protected:
    std::unique_ptr<Geometry::Mesh> _mesh;

private:
    std::string _filename;
    Vec3r _initial_position;
    Vec3r _initial_rotation;
    std::optional<Vec3r> _initial_size;
    std::optional<Real> _max_size;
};

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

class TetMeshObject : public MeshObject
{
public:
    TetMeshObject(const ConfigType* mesh_config, const Config::ObjectConfig* obj_config)
        : MeshObject(mesh_config, obj_config)
    {
    }

    const Geometry::TetMesh* tetMesh() const { return dynamic_cast<Geometry::TetMesh*>(_mesh.get()); }
    Geometry::TetMesh* tetMesh() { return dynamic_cast<Geometry::TetMesh*>(_mesh.get()); }

protected:
    // Ensure we actually load a TetMesh (so tagMap exists)
    virtual void _loadMeshFromFile(const std::string& fname) override
    {
        _mesh = std::make_unique<Geometry::TetMesh>(MeshUtils::loadTetMeshFromGmshFile(fname));
    }
};

} // namespace Sim

#endif // __MESH_OBJECT_HPP