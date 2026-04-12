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

#include <memory>
#include <optional>
#include <unordered_map>
#include <iostream>

#include "geometry/Mesh.hpp"
#include "geometry/TetMesh.hpp"
#include "utils/MeshUtils.hpp"
#include "config/simobject/ObjectConfig.hpp"
#include "config/simobject/MeshObjectConfig.hpp"

namespace Sim
{

class MeshObject
{
public:
    using ConfigType = Config::MeshObjectConfig;

    MeshObject(const ConfigType* mesh_config, const Config::ObjectConfig* obj_config)
    {
        _filename = mesh_config->filename();
        _initial_position = obj_config->initialPosition();
        _initial_rotation = obj_config->initialRotation();
        _initial_size = mesh_config->size();
        _max_size     = mesh_config->maxSize();
        _use_original_coords = mesh_config->useOriginalCoords();
    }

    const Geometry::Mesh* mesh() const { return _mesh.get(); }
    Geometry::Mesh*       mesh()       { return _mesh.get(); }

    void loadAndConfigureMesh()
    {
        _loadMeshFromFile(_filename);

        // Debug: after load
        if (_mesh) {
            if (_mesh->numVertices() > 0) {
            }
        }

        // Preserve gmsh tag map across geometry ops (some ops may rebuild internals)
        std::unordered_map<int, int> savedTagMap;
        if (auto* tet = dynamic_cast<Geometry::TetMesh*>(_mesh.get()))
            savedTagMap = tet->tagMap();  // copy snapshot

        // Check if this is a 1D line mesh (no faces/tetrahedra, only edges)
        bool isLineMesh = (_mesh->numFaces() == 0 && _mesh->numVertices() > 0);
        if (auto* tetMesh = dynamic_cast<Geometry::TetMesh*>(_mesh.get())) {
            isLineMesh = (tetMesh->numElements() == 0 && tetMesh->numFaces() == 0 && tetMesh->numVertices() > 0);
        }
        
        if (isLineMesh) {
            
            // For line meshes:
            // - Skip: resize (needs volume/area)
            // - Skip: massCenter (needs volume/area) 
            // - APPLY: moveTogether (works for point clouds!)
            // - APPLY: rotateAbout (works for point clouds!)
            // - APPLY: setCurrentStateAsUndeformedState
            
            // Compute geometric center instead of mass center (simple average of vertices)
            Vec3r geometric_center = Vec3r::Zero();
            const int nv = _mesh->numVertices();
            for (int i = 0; i < nv; ++i) {
                geometric_center += _mesh->vertex(i);
            }
            geometric_center /= nv;
            
            // Move mesh from geometric center to desired position
            _mesh->moveTogether(-geometric_center + _initial_position);
            
            // Apply rotation around the target position
            _mesh->rotateAbout(_initial_position, _initial_rotation);
            
            // Set current state as undeformed (reference configuration)
            _mesh->setCurrentStateAsUndeformedState();
        } else {
            // Check if we should use original coordinates (like MeshLab)
            if (_use_original_coords) {
                // ROOT FIX: use-original-coords means mesh stays EXACTLY where it was loaded
                // NO transformations applied - not even position offset!
                // The mesh's mass center BECOMES the rigid body position
                // This ensures globalToBody() transformation works correctly
                
                if (_initial_position.norm() > 1e-6 || _initial_rotation.norm() > 1e-6) {
                    std::cout << "[MeshObject WARNING] use-original-coords=true but position="
                              << _initial_position.transpose() << " rotation=" << _initial_rotation.transpose()
                              << " specified.\n  These are IGNORED - mesh stays at loaded coordinates!\n";
                }
                
                // Do nothing - mesh stays exactly as loaded
            } else {
                // Standard pipeline: resize -> recenter mass -> rotate
                if (_max_size.has_value()) {
                    _mesh->resize(_max_size.value());
                }
                if (_initial_size.has_value()) {
                    _mesh->resize(_initial_size.value());
                }

                const Vec3r com = _mesh->massCenter();
                _mesh->moveTogether(-com + _initial_position);
                _mesh->rotateAbout(_initial_position, _initial_rotation);
            }
            
            _mesh->setCurrentStateAsUndeformedState();
        }
        
        // Restore tag map for both cases
        if (auto* tet = dynamic_cast<Geometry::TetMesh*>(_mesh.get()))
            tet->mutableTagMap() = std::move(savedTagMap);
    }

protected:
    // Surface-mesh loader (no tag map expected)
    virtual void _loadMeshFromFile(const std::string& fname)
    {
        _mesh = std::make_unique<Geometry::Mesh>(MeshUtils::loadSurfaceMeshFromFile(fname));
    }

    void _scaleMesh() { /* intentionally empty */ }

protected:
    std::unique_ptr<Geometry::Mesh> _mesh;
    
    std::string _filename;
    Vec3r _initial_position;
    Vec3r _initial_rotation;
    std::optional<Vec3r> _initial_size;
    std::optional<Real>  _max_size;
    bool _use_original_coords;
};

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

class TetMeshObject : public MeshObject
{
public:
    TetMeshObject(const ConfigType* mesh_config, const Config::ObjectConfig* obj_config)
        : MeshObject(mesh_config, obj_config)
    {}

    const Geometry::TetMesh* tetMesh() const { return dynamic_cast<Geometry::TetMesh*>(_mesh.get()); }
    Geometry::TetMesh*       tetMesh()       { return dynamic_cast<Geometry::TetMesh*>(_mesh.get()); }

protected:
    void _loadMeshFromFile(const std::string& fname) override
    {
        // Load TetMesh from Gmsh; MeshUtils already fills tagMap().
        Geometry::TetMesh tmp = MeshUtils::loadTetMeshFromGmshFile(fname);
        
        // Debug: check vertices before move
        if (tmp.numVertices() > 0) {
            const auto& V_before = tmp.vertices();
        }

        // Copy out tagMap before move.
        auto tagMapCopy = tmp.tagMap();

        // Move into owned mesh.
        _mesh = std::make_unique<Geometry::TetMesh>(std::move(tmp));
        
        // Debug: check vertices after move
        if (_mesh->numVertices() > 0) {
            const auto& V_after = _mesh->vertices();
        }

        // Re-inject tagMap and debug print.
        if (auto* tm = dynamic_cast<Geometry::TetMesh*>(_mesh.get())) {
            tm->mutableTagMap() = std::move(tagMapCopy);
        } else {
        }
    }
};

} // namespace Sim

#endif // __MESH_OBJECT_HPP
