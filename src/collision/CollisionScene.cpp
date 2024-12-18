#include "collision/CollisionScene.hpp"

#include "rootparitycollisiontest.h"

#include "simobject/XPBDMeshObject.hpp"

#include <chrono>

CollisionScene::CollisionScene(const double dt, const double cell_size, const int num_buckets)
    : _dt(dt), _cell_size(cell_size), _num_buckets(num_buckets)
{
    _buckets.resize(_num_buckets);
}

void CollisionScene::addObject(Sim::Object* new_obj)
{
    _objects.push_back(new_obj);
}

void CollisionScene::collideObjects(const double sim_time)
{
    // if (_objects.size() == 0)
    //     return;

    // // clear potential collisions
    // _potential_collisions.clear();

    // int num_tests = 0;
    // auto t1 = std::chrono::steady_clock::now();
    // _addVerticesToBuckets(sim_time);
    // auto t2 = std::chrono::steady_clock::now();
    // std::cout << "Adding vertices to buckets took " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us" << std::endl;

    // for (int oi = 0; oi < _objects.size(); oi++)
    // {
    //     // check collisions with faces
    //     const MeshObject::VerticesMat& obj_vertices = _objects[oi].obj_ptr->vertices();
    //     const MeshObject::VerticesMat& obj_prev_vertices = _objects[oi].prev_vertices;
    //     const MeshObject::VerticesMat& obj_velocities = _objects[oi].obj_ptr->velocities();
    //     const MeshObject::FacesMat& obj_faces = _objects[oi].obj_ptr->faces();
    //     for (int fi = 0; fi < obj_faces.rows(); fi++)
    //     {
    //         const int v0 = obj_faces(fi,0);
    //         const int v1 = obj_faces(fi,1);
    //         const int v2 = obj_faces(fi,2);
    //         const Eigen::Vector3d& cur_pos0 = obj_vertices.row(v0) + obj_velocities.row(v0)*_dt;
    //         const Eigen::Vector3d& cur_pos1 = obj_vertices.row(v1) + obj_velocities.row(v1)*_dt;
    //         const Eigen::Vector3d& cur_pos2 = obj_vertices.row(v2) + obj_velocities.row(v2)*_dt;
    //         // const Eigen::Vector3d next_pos0 = cur_pos0 + obj_velocities.row(v0).transpose()*_dt;
    //         // const Eigen::Vector3d next_pos1 = cur_pos1 + obj_velocities.row(v1).transpose()*_dt;
    //         // const Eigen::Vector3d next_pos2 = cur_pos2 + obj_velocities.row(v2).transpose()*_dt;

    //         // find cells that this face is a part of
    //         int i1 = std::floor(cur_pos0[0]/_cell_size);
    //         int i2 = std::floor(cur_pos1[0]/_cell_size);
    //         int i3 = std::floor(cur_pos2[0]/_cell_size);
    //         int min_i = std::min(std::min(i1, i2), i3);
    //         int max_i = std::max(std::max(i1, i2), i3);

    //         int j1 = std::floor(cur_pos0[1]/_cell_size);
    //         int j2 = std::floor(cur_pos1[1]/_cell_size);
    //         int j3 = std::floor(cur_pos2[1]/_cell_size);
    //         int min_j = std::min(std::min(j1, j2), j3);
    //         int max_j = std::max(std::max(j1, j2), j3);

    //         int k1 = std::floor(cur_pos0[2]/_cell_size);
    //         int k2 = std::floor(cur_pos1[2]/_cell_size);
    //         int k3 = std::floor(cur_pos2[2]/_cell_size);
    //         int min_k = std::min(std::min(k1, k2), k3);
    //         int max_k = std::max(std::max(k1, k2), k3);
            
    //         // add this vertex to "box" of cells between the current cell and the next cell
    //         for (int i = min_i; i <= max_i; i++)
    //         {
    //             for (int j = min_j; j <= max_j; j++)
    //             {
    //                 for (int k = min_k; k <= max_k; k++)
    //                 {
    //                     // find the bucket index for this cell
    //                     int bucket_index = _hash(i, j, k);
                        
    //                     // std::cout << "Num vertices in bucket: " << _buckets[bucket_index].vertices.size() << std::endl;

    //                     if (_buckets[bucket_index].time != sim_time)
    //                     {
    //                         continue;
    //                     }

    //                     const Eigen::Vector3d& last_pos0 = obj_prev_vertices.row(v0);
    //                     const Eigen::Vector3d& last_pos1 = obj_prev_vertices.row(v1);
    //                     const Eigen::Vector3d& last_pos2 = obj_prev_vertices.row(v2);

    //                     // do collision detection for any other vertices (not part of the triangle)
    //                     for (int vi = 0; vi < _buckets[bucket_index].vertices.size(); vi++)
    //                     {
                            

    //                         int vertex_obj_ind = _buckets[bucket_index].vertices[vi].first;
    //                         int vertex_ind = _buckets[bucket_index].vertices[vi].second;

    //                         const Eigen::Vector<int, 3>& face = obj_faces.row(fi);
    //                         // skip vertices that are part of this triangle
    //                         if (vertex_obj_ind == oi && (vertex_ind == face[0] || vertex_ind == face[1] || vertex_ind == face[2]) )
    //                         {
    //                             continue;
    //                         }

    //                         const Eigen::Vector3d& q_vel = _objects[vertex_obj_ind].obj_ptr->velocities().row(vertex_ind);
    //                         const Eigen::Vector3d& cur_q = _objects[vertex_obj_ind].obj_ptr->vertices().row(vertex_ind).transpose() + q_vel*_dt;
    //                         const Eigen::Vector3d& last_q = _objects[vertex_obj_ind].prev_vertices.row(vertex_ind);
    //                         // const Eigen::Vector3d next_q = cur_q + q_vel*_dt;
    //                         bool collision = false;
                            
                            

    //                         if (_distanceToFace(cur_q, cur_pos0, cur_pos1, cur_pos2) <= 1e-4)
    //                         {
    //                             std::cout << "DISTANCE TO FACE < 1e-6! " << std::endl;
    //                             collision = true;
    //                         }
    //                         else
    //                         {
    //                             Vec3d q(last_q.data());
    //                             Vec3d q_new(cur_q.data());

    //                             Vec3d p0(last_pos0.data());
    //                             Vec3d p0_new(cur_pos0.data());

    //                             Vec3d p1(last_pos1.data());
    //                             Vec3d p1_new(cur_pos1.data());

    //                             Vec3d p2(last_pos2.data());
    //                             Vec3d p2_new(cur_pos2.data());

    //                             rootparity::RootParityCollisionTest test(q, p0, p1, p2, q_new, p0_new, p1_new, p2_new, false);
    //                             collision = test.run_test();
    //                         }
                            
                            

    //                         num_tests+=collision;

    //                         if (collision)
    //                         {
                                
    //                             const Eigen::Vector3d last_n = (last_pos1 - last_pos0).cross(last_pos2 - last_pos0);
    //                             const double last_d = (last_q - last_pos0).dot(last_n);
                                

    //                             XPBDMeshObject* xpbd_obj1 = dynamic_cast<XPBDMeshObject*>(_objects[vertex_obj_ind].obj_ptr);
    //                             XPBDMeshObject* xpbd_obj2 = dynamic_cast<XPBDMeshObject*>(_objects[oi].obj_ptr);
    //                             if (xpbd_obj1 && xpbd_obj2)
    //                             {
    //                                 if (last_d >= 0)
    //                                 {
    //                                     xpbd_obj1->addCollisionConstraint(xpbd_obj1, vertex_ind, xpbd_obj2, v0, v1, v2);
    //                                     // xpbd_obj2->addCollisionConstraint(xpbd_obj1, vertex_ind, xpbd_obj2, v0, v1, v2);
    //                                 }
                                        

    //                             }
                                
    //                         }

                            

    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }
    
    // auto t3 = std::chrono::steady_clock::now();
    // std::cout << "Adding faces to buckets took " << std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() << " us" << std::endl;
    // std::cout << "Num collision tests: " << num_tests << std::endl;
}

inline void CollisionScene::_addVerticesToBuckets(const double sim_time)
{
    // for (int oi = 0; oi < _objects.size(); oi++)
    // {
    //     // add vertices for this object to their corresponding buckets
    //     const MeshObject::VerticesMat& obj_vertices = _objects[oi]->mesh().vertices();

    //     for (int vi = 0; vi < obj_vertices.rows(); vi++)
    //     {
    //         // only consider surface vertices
    //         if (!_objects[oi]->vertexOnSurface(vi))
    //             continue;

    //         const Eigen::Vector3d& pos = obj_vertices.row(vi);// + obj_velocities.row(vi)*_dt;
    //         int i = std::floor(pos[0]/_cell_size);
    //         int j = std::floor(pos[1]/_cell_size);
    //         int k = std::floor(pos[2]/_cell_size);

    //         int bucket_index = _hash(i, j, k);

    //         // clear the bucket if it's stale
    //         if (_buckets[bucket_index].time != sim_time)
    //         {
    //             _buckets[bucket_index].vertices.clear();
    //             _buckets[bucket_index].time = sim_time;
    //         }
    //         // add it the appropriate bucket
    //         _buckets[bucket_index].vertices.push_back(std::make_pair(oi, vi));
    //     }
    // }
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