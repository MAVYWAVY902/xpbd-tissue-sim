#ifndef __MESH_OBJECT_HPP
#define __MESH_OBJECT_HPP

#include "geometry/Mesh.hpp"
#include "geometry/TetMesh.hpp"
#include "utils/MeshUtils.hpp"
#include "config/ObjectConfig.hpp"
#include "config/MeshObjectConfig.hpp"

namespace Sim
{

class MeshObject
{
    public:
    MeshObject(const MeshObjectConfig* mesh_config, const ObjectConfig* obj_config)
    {
        _filename = mesh_config->filename();

        _initial_position = obj_config->initialPosition();
        _initial_rotation = obj_config->initialRotation();

        _initial_size = mesh_config->size();
        _max_size = mesh_config->maxSize();
    }

    const Geometry::Mesh* mesh() const { return _mesh.get(); }

    Geometry::Mesh* mesh() { return _mesh.get(); }

    protected:
    void _loadAndConfigureMesh()
    {
        _loadMeshFromFile(_filename);

        // order matters here...
        // first apply scaling before rotating - either through the max-size criteria or a user-specified size
        if (_max_size.has_value())
        {
            _mesh->resize(_max_size.value());
        }

        if (_initial_size.has_value())
        {
            _mesh->resize(_initial_size.value());
        }

        const Vec3r center_of_mass = _mesh->massCenter();

        // move center of mass of the mesh to the specified initial position
        _mesh->moveTogether(-center_of_mass + _initial_position);

        // then do rigid transformation - rotation and translation
        _mesh->rotateAbout(_initial_position, _initial_rotation);
    }

    virtual void _loadMeshFromFile(const std::string& fname)
    {
        _mesh = std::make_unique<Geometry::Mesh>(MeshUtils::loadSurfaceMeshFromFile(fname));
    }

    void _scaleMesh()
    {
        
    }

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
    TetMeshObject(const MeshObjectConfig* mesh_config, const ObjectConfig* obj_config)
        : MeshObject(mesh_config, obj_config)
    {

    }

    const Geometry::TetMesh* tetMesh() const { return dynamic_cast<Geometry::TetMesh*>(_mesh.get()); }
    Geometry::TetMesh* tetMesh() { return dynamic_cast<Geometry::TetMesh*>(_mesh.get()); }

    protected:
    virtual void _loadMeshFromFile(const std::string& fname)
    {
        _mesh = std::make_unique<Geometry::TetMesh>(MeshUtils::loadTetMeshFromGmshFile(fname));
    }
};

} //namespace Sim

#endif // __MESH_OBJECT_HPP