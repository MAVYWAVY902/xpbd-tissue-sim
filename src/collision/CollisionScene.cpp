#include "collision/CollisionScene.hpp"

CollisionScene::CollisionScene(const double dt, const double cell_size, const unsigned num_buckets)
    : _dt(dt), _cell_size(cell_size), _num_buckets(num_buckets)
{
    _buckets.resize(_num_buckets);
}

void CollisionScene::addObject(std::shared_ptr<MeshObject> new_obj)
{
    std::cout << "Adding object: " << new_obj->name() << std::endl;
    _objects.push_back(new_obj);
}

void CollisionScene::collideObjects()
{
    if (_objects.size() == 0)
        return;
    // clear potential collisions
    _potential_collisions.clear();

    // clear buckets
    for (auto& b : _buckets)
    {
        b.faces.clear();
        b.vertices.clear();
    }

    for (int oi = 0; oi < _objects.size(); oi++)
    {
        // add vertices for this object to their corresponding buckets
        MeshObject::VerticesMat obj_vertices = _objects[oi]->vertices();
        MeshObject::VerticesMat obj_velocities = _objects[oi]->velocities();
        for (int vi = 0; vi < obj_vertices.rows(); vi++)
        {
            // only consider surface vertices
            if (!_objects[oi]->vertexOnSurface(vi))
                continue;

            // current cell
            int cur_i = static_cast<int>(obj_vertices(vi,0)/_cell_size);
            int cur_j = static_cast<int>(obj_vertices(vi,1)/_cell_size);
            int cur_k = static_cast<int>(obj_vertices(vi,2)/_cell_size);

            // "next" cell - predict where vertex will be after one time step
            int next_i = static_cast<int>( (obj_vertices(vi,0)+obj_velocities(vi,0)*_dt)/_cell_size );
            int next_j = static_cast<int>( (obj_vertices(vi,1)+obj_velocities(vi,1)*_dt)/_cell_size );
            int next_k = static_cast<int>( (obj_vertices(vi,2)+obj_velocities(vi,2)*_dt)/_cell_size );

            // add this vertex to "box" of cells between the current cell and the next cell
            for (int i = std::min(cur_i,next_i); i <= std::max(cur_i,next_i); i++)
            {
                for (int j = std::min(cur_j,next_j); j <= std::max(cur_j,next_j); j++)
                {
                    for (int k = std::min(cur_k,next_k); k <= std::max(cur_k,next_k); k++)
                    {
                        // find the bucket index for this cell
                        int bucket_index = _hash(i, j, k);
                        // add it the appropriate bucket
                        _buckets[bucket_index].vertices.push_back(std::make_pair(oi, vi));
                    }
                }
            }
        }
    }

    for (int oi = 0; oi < _objects.size(); oi++)
    {
        // check collisions with faces
        MeshObject::VerticesMat obj_vertices = _objects[oi]->vertices();
        MeshObject::FacesMat obj_faces = _objects[oi]->faces();
        for (int fi = 0; fi < obj_faces.rows(); fi++)
        {
            // find cells that this face is a part of
            int i1 = static_cast<int>(obj_vertices(obj_faces(fi,0),0)/_cell_size);
            int i2 = static_cast<int>(obj_vertices(obj_faces(fi,1),0)/_cell_size);
            int i3 = static_cast<int>(obj_vertices(obj_faces(fi,2),0)/_cell_size);
            int min_i = std::min(std::min(i1, i2), i3);
            int max_i = std::max(std::max(i1, i2), i3);

            int j1 = static_cast<int>(obj_vertices(obj_faces(fi,0),1)/_cell_size);
            int j2 = static_cast<int>(obj_vertices(obj_faces(fi,1),1)/_cell_size);
            int j3 = static_cast<int>(obj_vertices(obj_faces(fi,2),1)/_cell_size);
            int min_j = std::min(std::min(j1, j2), j3);
            int max_j = std::max(std::max(j1, j2), j3);

            int k1 = static_cast<int>(obj_vertices(obj_faces(fi,0),2)/_cell_size);
            int k2 = static_cast<int>(obj_vertices(obj_faces(fi,1),2)/_cell_size);
            int k3 = static_cast<int>(obj_vertices(obj_faces(fi,2),2)/_cell_size);
            int min_k = std::min(std::min(k1, k2), k3);
            int max_k = std::max(std::max(k1, k2), k3);
            
            // add this vertex to "box" of cells between the current cell and the next cell
            for (int i = min_i; i <= max_i; i++)
            {
                for (int j = min_j; j <= max_j; j++)
                {
                    for (int k = min_k; k <= max_k; k++)
                    {
                        // find the bucket index for this cell
                        int bucket_index = _hash(i, j, k);
                        
                        // do collision detection for any other vertices (not part of the triangle)
                        for (int vi = 0; vi < _buckets[bucket_index].vertices.size(); vi++)
                        {
                            int obj_ind = _buckets[bucket_index].vertices[vi].first;
                            int vert_ind = _buckets[bucket_index].vertices[vi].second;
                            // skip vertices that are part of this triangle
                            if (obj_ind == oi && (vert_ind == obj_faces(fi,0) || vert_ind == obj_faces(fi,1) || vert_ind == obj_faces(fi,2)) )
                            {
                                continue;
                            }

                            // do collision detection
                            const Eigen::Vector3d ray_origin = _objects[obj_ind]->vertices().row(vert_ind);
                            const Eigen::Vector3d ray_vector = _objects[obj_ind]->velocities().row(vert_ind) * _dt * 2; // multiply by 2 as a factor of safety
                            bool predictive_collision = _rayTriangleIntersection(ray_origin, ray_vector, obj_vertices.row(obj_faces(fi,0)), obj_vertices.row(obj_faces(fi,1)), obj_vertices.row(obj_faces(fi,2)));
                            if (predictive_collision)
                            {
                                std::cout << "POTENTIAL COLLISION! Obj1 " << obj_ind << " Vert: " << vert_ind << " Obj2 " << oi << " Face: " << fi << std::endl;
                                std::cout << "\tRay origin: " << ray_origin(0) << ", " << ray_origin(1) << ", " << ray_origin(2) << std::endl;
                                std::cout << "\tRay vector: " << ray_vector(0) << ", " << ray_vector(1) << ", " << ray_vector(2) << std::endl;
                                std::cout << "\tA: " << obj_vertices.row(obj_faces(fi,0))(0) << ", " << obj_vertices.row(obj_faces(fi,0))(1) << ", " << obj_vertices.row(obj_faces(fi,0))(2) << std::endl;
                                std::cout << "\tB: " << obj_vertices.row(obj_faces(fi,1))(0) << ", " << obj_vertices.row(obj_faces(fi,1))(1) << ", " << obj_vertices.row(obj_faces(fi,1))(2) << std::endl;
                                std::cout << "\tC: " << obj_vertices.row(obj_faces(fi,2))(0) << ", " << obj_vertices.row(obj_faces(fi,2))(1) << ", " << obj_vertices.row(obj_faces(fi,2))(2) << std::endl;

                                Collision collision;
                                collision.obj1_ind = obj_ind;
                                collision.vertex_ind = vert_ind;
                                collision.obj2_ind = oi;
                                collision.face_ind = fi;
                                _potential_collisions.push_back(collision);
                            }


                        }
                    }
                }
            }
        }
    }
}

// implements Moller-Trumbore intersection algorithm: https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
inline bool CollisionScene::_rayTriangleIntersection(const Eigen::Vector3d& ray_origin, const Eigen::Vector3d& ray_vector, const Eigen::Vector3d& A, const Eigen::Vector3d& B, const Eigen::Vector3d& C) const
{
    constexpr double epsilon = 1e-12;

    const Eigen::Vector3d edge1 = B - A;
    const Eigen::Vector3d edge2 = C - A;
    const Eigen::Vector3d ray_cross_edge2 = ray_vector.cross(edge2);
    double det = edge1.dot(ray_cross_edge2);

    if (det > -epsilon && det < epsilon)
        return false;   // ray is parallel to this triangle

    double inv_det = 1.0 / det;
    const Eigen::Vector3d s = ray_origin - A;
    double u = inv_det * s.dot(ray_cross_edge2);

    if (u < 0 || u > 1)
        return false;
    
    const Eigen::Vector3d s_cross_edge1 = s.cross(edge1);
    double v = inv_det * ray_vector.dot(s_cross_edge1);

    if (v < 0 || u + v > 1)
        return false;

    // At this stage we can compute t to find out where the intersection point is on the line.
    double t = inv_det * edge2.dot(s_cross_edge1);

    if (t > epsilon && t <= 1) // ray intersection, make sure t is less than one unit along the line
    {
        return  true;
    }
    else // This means that there is a line intersection but not a ray intersection.
        return false;

}

inline int CollisionScene::_hash(int i, int j, int k) const
{
    // from https://matthias-research.github.io/pages/publications/tetraederCollision.pdf
    return (i*73856093 ^ j*19349663 ^ k*83492791) % _num_buckets;
}