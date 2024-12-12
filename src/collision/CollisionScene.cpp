#include "collision/CollisionScene.hpp"

#include "rootparitycollisiontest.h"

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

void CollisionScene::collideObjects(const double sim_time)
{
    if (_objects.size() == 0)
        return;
    // clear potential collisions
    _potential_collisions.clear();

    int num_tests = 0;
    for (int oi = 0; oi < _objects.size(); oi++)
    {
        // add vertices for this object to their corresponding buckets
        const MeshObject::VerticesMat& obj_vertices = _objects[oi]->vertices();
        const MeshObject::VerticesMat& obj_velocities = _objects[oi]->velocities();
        for (int vi = 0; vi < obj_vertices.rows(); vi++)
        {
            // only consider surface vertices
            if (!_objects[oi]->vertexOnSurface(vi))
                continue;

            // current cell
            int cur_i = std::floor(obj_vertices(vi,0)/_cell_size);
            int cur_j = std::floor(obj_vertices(vi,1)/_cell_size);
            int cur_k = std::floor(obj_vertices(vi,2)/_cell_size);

            // "next" cell - predict where vertex will be after one time step
            int next_i = std::floor( (obj_vertices(vi,0)+obj_velocities(vi,0)*_dt)/_cell_size );
            int next_j = std::floor( (obj_vertices(vi,1)+obj_velocities(vi,1)*_dt)/_cell_size );
            int next_k = std::floor( (obj_vertices(vi,2)+obj_velocities(vi,2)*_dt)/_cell_size );

            // add this vertex to "box" of cells between the current cell and the next cell
            for (int i = std::min(cur_i,next_i); i <= std::max(cur_i,next_i); i++)
            {
                for (int j = std::min(cur_j,next_j); j <= std::max(cur_j,next_j); j++)
                {
                    for (int k = std::min(cur_k,next_k); k <= std::max(cur_k,next_k); k++)
                    {
                        // find the bucket index for this cell
                        int bucket_index = _hash(i, j, k);

                        // clear the bucket if it's stale
                        if (_buckets[bucket_index].time != sim_time)
                        {
                            _buckets[bucket_index].vertices.clear();
                            _buckets[bucket_index].time = sim_time;
                        }
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
        const MeshObject::VerticesMat& obj_vertices = _objects[oi]->vertices();
        const MeshObject::FacesMat& obj_faces = _objects[oi]->faces();
        for (int fi = 0; fi < obj_faces.rows(); fi++)
        {
            // find cells that this face is a part of
            int i1 = std::floor(obj_vertices(obj_faces(fi,0),0)/_cell_size);
            int i2 = std::floor(obj_vertices(obj_faces(fi,1),0)/_cell_size);
            int i3 = std::floor(obj_vertices(obj_faces(fi,2),0)/_cell_size);
            int min_i = std::min(std::min(i1, i2), i3);
            int max_i = std::max(std::max(i1, i2), i3);

            int j1 = std::floor(obj_vertices(obj_faces(fi,0),1)/_cell_size);
            int j2 = std::floor(obj_vertices(obj_faces(fi,1),1)/_cell_size);
            int j3 = std::floor(obj_vertices(obj_faces(fi,2),1)/_cell_size);
            int min_j = std::min(std::min(j1, j2), j3);
            int max_j = std::max(std::max(j1, j2), j3);

            int k1 = std::floor(obj_vertices(obj_faces(fi,0),2)/_cell_size);
            int k2 = std::floor(obj_vertices(obj_faces(fi,1),2)/_cell_size);
            int k3 = std::floor(obj_vertices(obj_faces(fi,2),2)/_cell_size);
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
                            if (obj_ind == oi)
                            {
                                continue;
                            }
                            // skip vertices that are part of this triangle
                            if (obj_ind == oi && (vert_ind == obj_faces(fi,0) || vert_ind == obj_faces(fi,1) || vert_ind == obj_faces(fi,2)) )
                            {
                                continue;
                            }

                            // std::cout << "Running collision test: Vertex obj: " << obj_ind << ", vertex ind: " << vert_ind << ", Face obj: " << oi << ", face ind: " << fi << std::endl;

                            // do collision detection
                            // const Eigen::Vector3d ray_origin = _objects[obj_ind]->vertices().row(vert_ind);
                            // const Eigen::Vector3d ray_vector = _objects[obj_ind]->velocities().row(vert_ind) * _dt * 2; // multiply by 2 as a factor of safety
                            // bool predictive_collision = _rayTriangleIntersection(ray_origin, ray_vector, obj_vertices.row(obj_faces(fi,0)), obj_vertices.row(obj_faces(fi,1)), obj_vertices.row(obj_faces(fi,2)));

                            const Eigen::Vector3d& v0 = _objects[obj_ind]->velocities().row(vert_ind);
                            const Eigen::Vector3d& v1 = _objects[oi]->velocities().row(obj_faces(fi,0));
                            const Eigen::Vector3d& v2 = _objects[oi]->velocities().row(obj_faces(fi,1));
                            const Eigen::Vector3d& v3 = _objects[oi]->velocities().row(obj_faces(fi,2));

                            const Eigen::Vector3d& p0 = _objects[obj_ind]->vertices().row(vert_ind);
                            const Eigen::Vector3d& p1 = _objects[oi]->vertices().row(obj_faces(fi,0));
                            const Eigen::Vector3d& p2 = _objects[oi]->vertices().row(obj_faces(fi,1));
                            const Eigen::Vector3d& p3 = _objects[oi]->vertices().row(obj_faces(fi,2));
                            bool collision = false;
                            // real CCD
                            num_tests++;
                            Vec3d x0(_objects[obj_ind]->getVertexPointer(vert_ind));
                            Vec3d x0_new = x0; x0_new[0] += v0[0]*_dt; x0_new[1] += v0[1]*_dt; x0_new[2] += v0[2]*_dt;

                            Vec3d x1(_objects[oi]->getVertexPointer(obj_faces(fi,0)));
                            Vec3d x1_new = x1; x1_new[0] += v1[0]*_dt; x1_new[1] += v1[1]*_dt; x1_new[2] += v1[2]*_dt;

                            Vec3d x2(_objects[oi]->getVertexPointer(obj_faces(fi,1)));
                            Vec3d x2_new = x2; x2_new[0] += v2[0]*_dt; x2_new[1] += v2[1]*_dt; x2_new[2] += v2[2]*_dt;

                            Vec3d x3(_objects[oi]->getVertexPointer(obj_faces(fi,2)));
                            Vec3d x3_new = x3; x3_new[0] += v3[0]*_dt; x3_new[1] += v3[1]*_dt; x3_new[2] += v3[2]*_dt;

                            rootparity::RootParityCollisionTest test(x0, x1, x2, x3, x0_new, x1_new, x2_new, x3_new, false);
                            collision = test.run_test();
                            

                            if (collision)
                            {
                                // std::cout << "POTENTIAL COLLISION! Obj1 " << obj_ind << " Vert: " << vert_ind << " Obj2 " << oi << " Face: " << fi << std::endl;
                                // std::cout << "\tRay origin: " << ray_origin(0) << ", " << ray_origin(1) << ", " << ray_origin(2) << std::endl;
                                // std::cout << "\tRay vector: " << ray_vector(0) << ", " << ray_vector(1) << ", " << ray_vector(2) << std::endl;
                                // std::cout << "\tA: " << obj_vertices.row(obj_faces(fi,0))(0) << ", " << obj_vertices.row(obj_faces(fi,0))(1) << ", " << obj_vertices.row(obj_faces(fi,0))(2) << std::endl;
                                // std::cout << "\tB: " << obj_vertices.row(obj_faces(fi,1))(0) << ", " << obj_vertices.row(obj_faces(fi,1))(1) << ", " << obj_vertices.row(obj_faces(fi,1))(2) << std::endl;
                                // std::cout << "\tC: " << obj_vertices.row(obj_faces(fi,2))(0) << ", " << obj_vertices.row(obj_faces(fi,2))(1) << ", " << obj_vertices.row(obj_faces(fi,2))(2) << std::endl;

                                const Eigen::Vector3d n = (p2 - p1).cross(p3 - p1);
                                const double d = (p0 - p1).dot(n);
                                if (d < 0)
                                    continue;

                                Collision collision;
                                collision.obj1 = _objects[obj_ind].get();
                                collision.vertex_ind = vert_ind;
                                collision.obj2 = _objects[oi].get();
                                collision.face_ind = fi;
                                _potential_collisions.push_back(collision);
                            }

                            

                        }
                    }
                }
            }
        }
    }
    std::cout << "Num collision tests: " << num_tests << std::endl;
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

inline double CollisionScene::_distanceToFace(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c)
{
    const Eigen::Vector3d ab = b - a;
    const Eigen::Vector3d ac = c - a;
    const Eigen::Vector3d ap = p - a;

    const double d1 = ab.dot(ap);
    const double d2 = ac.dot(ap);
    if (d1 <= 0.0 && d2 <= 0.0) { return (p-a).norm(); } //#1

    const Eigen::Vector3d bp = p - b;
    const double d3 = ab.dot(bp);
    const double d4 = ac.dot(bp);
    if (d3 >= 0.f && d4 <= d3) { return (p-b).norm(); } //#2

    const Eigen::Vector3d cp = p - c;
    const double d5 = ab.dot(cp);
    const double d6 = ac.dot(cp);
    if (d6 >= 0.0 && d5 <= d6) { return (p-c).norm(); } //#3

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
    {
        const double v = d1 / (d1 - d3);
        return (p - (a + v * ab)).norm(); //#4
    }

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
    {
        const double v = d2 / (d2 - d6);
        return (p - (a + v * ac)).norm(); //#5
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
    {
        const double v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (p - (b + v * (c - b))).norm(); //#6
    }

    const float denom = 1.f / (va + vb + vc);
    const float v = vb * denom;
    const float w = vc * denom;
    return (p - (a + v * ab + w * ac)).norm(); //#0
}

inline int CollisionScene::_hash(int i, int j, int k) const
{
    // std::cout << "i: " << i << ", j: " << j << ", k: " << k << ", hash: " << (i*73856093 ^ j*19349663 ^ k*83492791) % _num_buckets << std::endl;
    // from https://matthias-research.github.io/pages/publications/tetraederCollision.pdf
    return (i*73856093 ^ j*19349663 ^ k*83492791) % _num_buckets;
}