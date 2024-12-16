#ifndef __COLLISION_SCENE_HPP
#define __COLLISION_SCENE_HPP

#include "simobject/MeshObject.hpp"

/** Represents a collision between a vertex and a face */
struct Collision
{
    MeshObject* obj1;
    unsigned vertex_ind;
    MeshObject* obj2;
    unsigned face_ind;
};

struct CollisionObject
{
    MeshObject* obj_ptr;
    MeshObject::VerticesMat prev_vertices;
};

/** Represents a collision bucket */
struct CollisionBucket
{
    double time;
    // first index is object index, second index is vertex/face vertex
    std::vector<std::pair<unsigned, unsigned>> vertices;
    std::vector<std::pair<unsigned, unsigned>> faces;
};

class CollisionScene
{
    public:
    explicit CollisionScene(const double dt, const double cell_size, const unsigned num_buckets);

    void setCellSize(const double cell_size) { _cell_size = cell_size; }

    void addObject(std::shared_ptr<MeshObject> new_obj);

    void collideObjects(const double sim_time);

    std::vector<Collision> potentialCollisions() const { return _potential_collisions; };

    void updatePrevPositions();

    protected:

    inline int _hash(int i, int j, int k) const;

    inline bool _rayTriangleIntersection(const Eigen::Vector3d& ray_origin, const Eigen::Vector3d& ray_vector, const Eigen::Vector3d& A, const Eigen::Vector3d& B, const Eigen::Vector3d& C) const;

    inline double _distanceToFace(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c);

    private:

    inline void _addVerticesToBuckets(const double sim_time);

    /** Time step */
    double _dt;

    /** Cell size for 3D space discretization */
    double _cell_size;

    /** Number of collision buckets */
    unsigned _num_buckets;

    /** Stores the mesh objects in the scene. */
    // std::vector<std::shared_ptr<MeshObject>> _objects;
    std::vector<CollisionObject> _objects;

    /** Collision buckets */
    std::vector<CollisionBucket> _buckets;

    /** Stores potential collisions (continuous collision detection) */
    std::vector<Collision> _potential_collisions;


};

#endif // __COLLISION_SCENE_HPP