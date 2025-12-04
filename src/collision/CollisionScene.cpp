#include "collision/CollisionScene.hpp"
#include "simulation/Simulation.hpp"
#include "simobject/RigidPrimitives.hpp"
#include "simobject/MeshObject.hpp"
#include "simobject/RigidMeshObject.hpp"
#include "simobject/XPBDMeshObject.hpp"
#include "simobject/VirtuosoArm.hpp"
#include "simobject/VirtuosoRobot.hpp"
#include "geometry/SphereSDF.hpp"
#include "geometry/BoxSDF.hpp"
#include "geometry/CylinderSDF.hpp"
#include "geometry/MeshSDF.hpp"
#include "geometry/Mesh.hpp"
#include "geometry/VirtuosoArmSDF.hpp"
#include "utils/GeometryUtils.hpp"

#include <chrono>   // For performance monitoring
#include <iostream> // For debug output
#include <iomanip>  // For std::setprecision

#ifdef HAVE_CUDA
#include "gpu/resource/GPUResource.hpp"
#include "gpu/resource/MeshGPUResource.hpp"
#include "gpu/Collision.cuh"
#endif

// namespace Collision
// {

CollisionScene::CollisionScene(const Sim::Simulation* sim, Geometry::EmbreeScene* embree_scene)
    : _sim(sim), _embree_scene(embree_scene)
{

}

void CollisionScene::collideObjects()
{
    // CRITICAL: Update Embree scene with current vertex positions BEFORE collision detection!
    // Without this, Embree BVH still has stale data from initialization
    // Note: We could optimize this by only updating objects that have moved, but for now update all
    // std::cerr << "[CRITICAL] Updating Embree scene before collision detection..." << std::endl;
    _embree_scene->update();
    // std::cerr << "[CRITICAL] Embree scene updated." << std::endl;
    
    // collide object pairs
    _objects.for_each_element([this](auto obj1){
        _objects.for_each_element([this, obj1](auto obj2){
            // skip when objects are the same
            if ((void*)obj1 == (void*)obj2)
                return;

            _collideObjectPair(obj1, obj2);
        });
    });

    // run self-collision tests for any objects with self-collisions enabled
    for (auto& xpbd_obj : _self_collision_objects)
    {
        _embree_scene->updateObject(xpbd_obj.getAsTetMeshObject());
        xpbd_obj.selfCollisionCheck();
    }
}

void CollisionScene::_collideObjectPair(Sim::Object* /*obj1*/, Sim::Object* /*obj2*/)
{
    // do nothing in the general case
}

template <bool IsFirstOrder>
void CollisionScene::_collideObjectPair(Sim::VirtuosoArm* virtuoso_arm, Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj)
{
    return;
    const typename Sim::XPBDMeshObject_Base::SDFType* mesh_sdf = xpbd_mesh_obj->SDF();

    // sample points along backbone to check against the Deformable SDF
    const Vec3r& inner_tube_start = virtuoso_arm->innerTubeStartFrame().origin();
    const Vec3r& inner_tube_end = virtuoso_arm->innerTubeEndFrame().origin();
    
    const Real it_dia = virtuoso_arm->innerTubeOuterDiameter();

    const Vec3r& dir = (inner_tube_end - inner_tube_start).normalized();
    const Real dist_thresh = 0.5*it_dia;

    const int num_samples = (inner_tube_end - inner_tube_start).norm() / dist_thresh;
    // std::cout << "Num samples: " << num_samples << std::endl;
    for (int i = 0; i < num_samples; i++)
    {
        const Vec3r& pos = inner_tube_start + dir*dist_thresh*i;
        const Real sdf_dist = mesh_sdf->evaluate(pos);

        if (sdf_dist <= dist_thresh)
        {
            // there is penetration
            std::cout << "DeformableSDF collision!" << std::endl;

            // find the index of closest face and the closest surface point on the deformable mesh
            const auto [face_ind, closest_point] = mesh_sdf->closestSurfacePoint(pos);

            // calculate barycentric coordinates of closest surface point
            const Eigen::Vector3i& f = xpbd_mesh_obj->mesh()->face(face_ind);
            const Vec3r& p1 = xpbd_mesh_obj->mesh()->vertex(f[0]);
            const Vec3r& p2 = xpbd_mesh_obj->mesh()->vertex(f[1]);
            const Vec3r& p3 = xpbd_mesh_obj->mesh()->vertex(f[2]);
            const auto [u, v, w] = GeometryUtils::barycentricCoords(pos, p1, p2, p3);


            // calculate collision normal
            const Vec3r collision_normal = (closest_point - pos).normalized();

            // find appropriate point on surface of Virtuoso arm (a.k.a a cylinder)
            // Hacky way: move some distance towards the surface in the opposite direction from the collision normal and query the arm's SDF
            const typename Sim::VirtuosoArm::SDFType* arm_sdf = virtuoso_arm->SDF();
            const Vec3r arm_sdf_query_point = pos - collision_normal*it_dia;
            const Real arm_sdf_dist = arm_sdf->evaluate(arm_sdf_query_point);
            const Vec3r arm_sdf_grad = arm_sdf->gradient(arm_sdf_query_point);
            const Vec3r arm_surface_point = arm_sdf_query_point - arm_sdf_dist * arm_sdf_grad;
            xpbd_mesh_obj->addStaticCollisionConstraint(arm_sdf, arm_surface_point, collision_normal, face_ind, u, v, w);
        }
    }

    // std::cout << "DeformableSDF distance: " << dist << std::endl;
}

template <bool IsFirstOrder>
void CollisionScene::_collideObjectPair(Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj1, Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj2)
{
    // Performance monitoring (optional - can remove after testing)
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Check if inter-object collision detection is enabled for BOTH objects
    if (!xpbd_mesh_obj1->interObjectCollisionsEnabled() || !xpbd_mesh_obj2->interObjectCollisionsEnabled())
    {
        return; // Skip collision detection if either object has it disabled
    }

    // Inter-object deformable-deformable collision detection
    // Strategy: Check vertices of obj1 against faces of obj2, and vice versa
    
    const Geometry::Mesh* mesh1 = xpbd_mesh_obj1->mesh();
    const Geometry::Mesh* mesh2 = xpbd_mesh_obj2->mesh();
    
    // ========== OPTIMIZATION: AABB Broad-Phase Culling ==========
    // Compute axis-aligned bounding boxes for both meshes
    const Geometry::Mesh::VerticesMat& verts1 = mesh1->vertices();
    const Geometry::Mesh::VerticesMat& verts2 = mesh2->vertices();
    
    Vec3r bbox1_min = verts1.rowwise().minCoeff();
    Vec3r bbox1_max = verts1.rowwise().maxCoeff();
    Vec3r bbox2_min = verts2.rowwise().minCoeff();
    Vec3r bbox2_max = verts2.rowwise().maxCoeff();
    
    // Add safety margin equal to collision threshold
    const Real collision_threshold = 1e-3; // 1mm
    const Real margin = collision_threshold;
    
    // Check if bounding boxes overlap on all three axes
    bool overlap_x = (bbox1_min[0] - margin) <= bbox2_max[0] && 
                     (bbox1_max[0] + margin) >= bbox2_min[0];
    bool overlap_y = (bbox1_min[1] - margin) <= bbox2_max[1] && 
                     (bbox1_max[1] + margin) >= bbox2_min[1];
    bool overlap_z = (bbox1_min[2] - margin) <= bbox2_max[2] && 
                     (bbox1_max[2] + margin) >= bbox2_min[2];
    
    if (!overlap_x || !overlap_y || !overlap_z)
    {
        // Bounding boxes don't overlap - objects are far apart!
        // Skip all expensive vertex-face checks
        
        // Performance logging (print every 1000th call to avoid spam)
        static int skip_count = 0;
        if (++skip_count % 1000 == 0)
        {
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            std::cout << "[AABB Culling] Objects separated - skipped collision check in " 
                      << duration.count() << " μs (count: " << skip_count << ")\n";
        }
        
        return;
    }
    // ========== End AABB Culling ==========
    
    const Geometry::Mesh::FacesMat& faces2 = mesh2->faces();
    
    // Part 1: Check vertices of object1 against faces of object2
    // Using Embree BVH for efficient spatial queries - O(n log m) complexity!
    int checks_performed = 0;  // Performance counter
    int checks_skipped = 0;    // Performance counter
    int embree_hits = 0;       // Triangles returned by BVH query
    int total_constraints_part1 = 0;  // Count constraints created in Part 1
    
    // Set search radius for Embree query (how far from vertex to search for triangles)
    const Real embree_search_radius = 0.2;  // 200mm - generous to not miss collisions
    
    for (int v_idx = 0; v_idx < mesh1->numVertices(); v_idx++)
    {
        // Only check surface vertices for efficiency
        if (!mesh1->vertexOnSurface(v_idx))
            continue;
            
        const Vec3r& vertex1 = mesh1->vertex(v_idx);
        
        // Use Embree BVH to find all triangles near this vertex!
        // This replaces the O(m) loop with O(log m) BVH query
        std::set<Geometry::EmbreeHit> nearby_triangles = 
            _embree_scene->interObjectCollisionQuery(vertex1, xpbd_mesh_obj2, embree_search_radius);
        
        embree_hits += nearby_triangles.size();
        checks_skipped += (faces2.cols() - nearby_triangles.size());  // Triangles culled by BVH
        
        int triangles_checked_for_this_vertex = 0;
        
        // Now check ONLY the nearby triangles found by Embree BVH (much smaller set!)
        for (const auto& hit : nearby_triangles)
        {
            const int face_idx = hit.prim_index;  // Get triangle index from Embree hit
            
            // SAFETY CHECK: Validate face index is within bounds
            if (face_idx < 0 || face_idx >= faces2.cols())
            {
                std::cerr << "[ERROR] Invalid face_idx from Embree: " << face_idx 
                          << " (max is " << faces2.cols()-1 << ")" << std::endl;
                continue;  // Skip invalid face
            }
            
            const Eigen::Vector3i& face = faces2.col(face_idx);
            
            // SAFETY CHECK: Validate vertex indices are within bounds
            if (face[0] < 0 || face[0] >= mesh2->numVertices() ||
                face[1] < 0 || face[1] >= mesh2->numVertices() ||
                face[2] < 0 || face[2] >= mesh2->numVertices())
            {
                std::cerr << "[ERROR] Invalid vertex indices in face " << face_idx 
                          << ": [" << face[0] << ", " << face[1] << ", " << face[2] << "]"
                          << " (max vertex is " << mesh2->numVertices()-1 << ")" << std::endl;
                continue;  // Skip invalid face
            }
            const Vec3r& p1 = mesh2->vertex(face[0]);
            const Vec3r& p2 = mesh2->vertex(face[1]);
            const Vec3r& p3 = mesh2->vertex(face[2]);
            
            triangles_checked_for_this_vertex++;
            checks_performed++;
            
            // Compute triangle normal
            const Vec3r edge1 = p2 - p1;
            const Vec3r edge2 = p3 - p1;
            const Vec3r normal = edge1.cross(edge2);
            const Real normal_length = normal.norm();
            
            if (normal_length < 1e-10)
                continue; // Degenerate triangle
            
            const Vec3r normal_normalized = normal / normal_length;
        
            // Compute signed distance from vertex to triangle plane
            const Vec3r to_vertex = vertex1 - p1;
            const Real signed_distance = to_vertex.dot(normal_normalized);
        
            if (std::abs(signed_distance) > collision_threshold)
                continue;
        
            // Project vertex onto triangle plane
        const Vec3r projected_point = vertex1 - signed_distance * normal_normalized;
        
        // Compute barycentric coordinates to check if point is inside triangle
        const auto [u, v, w] = GeometryUtils::barycentricCoords(projected_point, p1, p2, p3);
        
        // Debug output
        static int debug_count = 0;
        if (++debug_count % 10000 == 0)  // Only print every 10000th check to reduce spam
        {
            std::cout << "[Collision Debug] signed_dist=" << signed_distance
                      << ", bary=(" << u << "," << v << "," << w << ")\n";
        }
        
        // Check if point is inside triangle (all barycentric coords should be >= 0 and sum to 1)
        // Use same epsilon as old brute-force code for consistency
        const Real bary_epsilon = -0.1; // Allow vertices near triangle edges/corners
        if (u >= bary_epsilon && v >= bary_epsilon && w >= bary_epsilon)
        {
            total_constraints_part1++;  // Count this constraint
            
            // Collision detected! Create inter-object collision constraint
            // Get pointers and masses from object2's face vertices
            Real* p1_ptr = mesh2->vertexPointer(face[0]);
            Real* p2_ptr = mesh2->vertexPointer(face[1]);
            Real* p3_ptr = mesh2->vertexPointer(face[2]);
            
            Real m1 = xpbd_mesh_obj2->vertexConstraintInertia(face[0]);
            Real m2 = xpbd_mesh_obj2->vertexConstraintInertia(face[1]);
            Real m3 = xpbd_mesh_obj2->vertexConstraintInertia(face[2]);
            
            // Add constraint: vertex from obj1 colliding with face from obj2
            xpbd_mesh_obj1->addInterObjectCollisionConstraint(
                v_idx,                          // Vertex index from THIS object (obj1)
                face[0], p1_ptr, m1,           // Face vertex 1 from OTHER object (obj2)
                face[1], p2_ptr, m2,           // Face vertex 2 from OTHER object (obj2)
                face[2], p3_ptr, m3            // Face vertex 3 from OTHER object (obj2)
            );
            }
        }  // End of inner for loop (nearby triangles from Embree BVH)
        
        // Debug: Log triangle checking stats occasionally
        static int vertex_check_debug_count = 0;
        if (triangles_checked_for_this_vertex > 0 && ++vertex_check_debug_count % 200 == 0)
        {
            std::cout << "[DEBUG] Vertex " << v_idx << " checked " << triangles_checked_for_this_vertex 
                      << " triangles (Embree BVH returned " << nearby_triangles.size() 
                      << " out of " << faces2.cols() << " total)\n";
        }
        
    }  // End of outer for loop (all vertices of obj1)
    
    // Summary output (only print occasionally to reduce spam)
    static int summary_count = 0;
    if (++summary_count % 100 == 0)
    {
        std::cout << "[DEBUG] Part 1 complete: Created " << total_constraints_part1 << " constraints\n";
    }
    
    // Part 2: Check vertices of object2 against faces of object1 (symmetric)
    const Geometry::Mesh::FacesMat& faces1 = mesh1->faces();
    int total_constraints_part2 = 0;  // Count constraints created in Part 2
    
    for (int v_idx = 0; v_idx < mesh2->numVertices(); v_idx++)
    {
        // Only check surface vertices for efficiency
        if (!mesh2->vertexOnSurface(v_idx))
            continue;
            
        const Vec3r& vertex2 = mesh2->vertex(v_idx);
        
        // Use Embree BVH to find all triangles near this vertex (same as Part 1!)
        std::set<Geometry::EmbreeHit> nearby_triangles = 
            _embree_scene->interObjectCollisionQuery(vertex2, xpbd_mesh_obj1, embree_search_radius);
        
        embree_hits += nearby_triangles.size();
        checks_skipped += (faces1.cols() - nearby_triangles.size());  // Triangles culled by BVH
        
        // Now check ONLY the nearby triangles found by Embree BVH
        for (const auto& hit : nearby_triangles)
        {
            const int face_idx = hit.prim_index;  // Get triangle index from Embree hit
            
            // SAFETY CHECK: Validate face index is within bounds
            if (face_idx < 0 || face_idx >= faces1.cols())
            {
                std::cerr << "[ERROR] Invalid face_idx from Embree: " << face_idx 
                          << " (max is " << faces1.cols()-1 << ")" << std::endl;
                continue;  // Skip invalid face
            }
            
            const Eigen::Vector3i& face = faces1.col(face_idx);
            
            // SAFETY CHECK: Validate vertex indices are within bounds
            if (face[0] < 0 || face[0] >= mesh1->numVertices() ||
                face[1] < 0 || face[1] >= mesh1->numVertices() ||
                face[2] < 0 || face[2] >= mesh1->numVertices())
            {
                std::cerr << "[ERROR] Invalid vertex indices in face " << face_idx 
                          << ": [" << face[0] << ", " << face[1] << ", " << face[2] << "]"
                          << " (max vertex is " << mesh1->numVertices()-1 << ")" << std::endl;
                continue;  // Skip invalid face
            }
            const Vec3r& p1 = mesh1->vertex(face[0]);
            const Vec3r& p2 = mesh1->vertex(face[1]);
            const Vec3r& p3 = mesh1->vertex(face[2]);
            
            checks_performed++;
            
            // Compute triangle normal
            const Vec3r edge1 = p2 - p1;
            const Vec3r edge2 = p3 - p1;
            const Vec3r normal = edge1.cross(edge2);
            const Real normal_length = normal.norm();
            
            if (normal_length < 1e-10)
                continue; // Degenerate triangle
                
            const Vec3r normal_normalized = normal / normal_length;
        
            // Compute signed distance from vertex to triangle plane
            const Vec3r to_vertex = vertex2 - p1;
        const Real signed_distance = to_vertex.dot(normal_normalized);
        
        if (std::abs(signed_distance) > collision_threshold)
            continue;
        
        // Project vertex onto triangle plane
        const Vec3r projected_point = vertex2 - signed_distance * normal_normalized;
        
        // Compute barycentric coordinates
        const auto [u, v, w] = GeometryUtils::barycentricCoords(projected_point, p1, p2, p3);
        
        // Check if point is inside triangle
        // Use same epsilon as old brute-force code for consistency
        const Real bary_epsilon = -0.1; // Allow vertices near triangle edges/corners
        if (u >= bary_epsilon && v >= bary_epsilon && w >= bary_epsilon)
        {
            total_constraints_part2++;  // Count this constraint
            
            // Collision detected! Create inter-object collision constraint
            // Get pointers and masses from object1's face vertices
            Real* p1_ptr = mesh1->vertexPointer(face[0]);
            Real* p2_ptr = mesh1->vertexPointer(face[1]);
            Real* p3_ptr = mesh1->vertexPointer(face[2]);
            
            Real m1 = xpbd_mesh_obj1->vertexConstraintInertia(face[0]);
            Real m2 = xpbd_mesh_obj1->vertexConstraintInertia(face[1]);
            Real m3 = xpbd_mesh_obj1->vertexConstraintInertia(face[2]);
            
            // Add constraint: vertex from obj2 colliding with face from obj1
            xpbd_mesh_obj2->addInterObjectCollisionConstraint(
                v_idx,                          // Vertex index from THIS object (obj2)
                face[0], p1_ptr, m1,           // Face vertex 1 from OTHER object (obj1)
                face[1], p2_ptr, m2,           // Face vertex 2 from OTHER object (obj1)
                face[2], p3_ptr, m3            // Face vertex 3 from OTHER object (obj1)
            );
        }
        }  // End of inner for loop (nearby triangles from Embree BVH)
    }  // End of outer for loop (all vertices of obj2)
    
    // Summary output (only print occasionally to reduce spam)
    static int summary_count2 = 0;
    if (++summary_count2 % 100 == 0)
    {
        std::cout << "[DEBUG] Part 2 complete: Created " << total_constraints_part2 << " constraints\n";
        std::cout << "[DEBUG] TOTAL constraints created: " << (total_constraints_part1 + total_constraints_part2) << "\n";
    }
    
    // ========== Performance Logging ==========
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    // Print performance stats every 1000th collision check
    static int collision_count = 0;
    if (++collision_count % 1000 == 0)
    {
        Real skip_ratio = checks_skipped > 0 ? 
            100.0 * checks_skipped / (checks_skipped + checks_performed) : 0.0;
        
        std::cout << "[Collision Performance - EMBREE BVH] Time: " << duration.count() << " μs, "
                  << "BVH hits: " << embree_hits << ", "
                  << "Checks performed: " << checks_performed << ", "
                  << "Culled by BVH: " << checks_skipped << " ("
                  << std::fixed << std::setprecision(1) << skip_ratio << "%)\n";
    }
    // ========== End Performance Logging ==========
}

template <bool IsFirstOrder>
void CollisionScene::_collideObjectPair(Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj, Sim::VirtuosoArm* virtuoso_arm)
{
    // iterate through faces of mesh
    const typename Sim::VirtuosoArm::SDFType* sdf = virtuoso_arm->SDF();
    const Geometry::Mesh* mesh = xpbd_mesh_obj->mesh();
    const Geometry::Mesh::FacesMat& faces = mesh->faces();
    for (int i = 0; i < faces.cols(); i++)
    {
        const Eigen::Vector3i& f = faces.col(i);
        const Vec3r& p1 = mesh->vertex(f[0]);
        const Vec3r& p2 = mesh->vertex(f[1]);
        const Vec3r& p3 = mesh->vertex(f[2]);

        // check if centroid of face is close
        const Real centroid_dist = sdf->evaluate((p1+p2+p3)/3);
        if (centroid_dist > 2e-3)
            continue;

        const Real p1p2 = (p2-p1).norm();
        const Real p1p3 = (p3-p1).norm();
        const Real p2p3 = (p3-p2).norm();

        const Real max_edge = std::max({p1p2, p1p3, p2p3});
        const int num_samples = (int)(5*max_edge / 0.5e-3);

        // const int num_samples = 4;
        for (int si = 0; si <= num_samples; si++)
        {
            for (int sj = 0; sj <= num_samples - si; sj++)
            {
                const Real u = (Real)(si+1) / (num_samples+2);
                const Real v = (Real)(sj+1) / (num_samples+2);
                const Real w = 1 - u - v;
                const Vec3r x = u*p1 + v*p2 + w*p3;
                const auto result = sdf->evaluateWithGradientAndNodeInfo(x);
                if (result.distance <= virtuoso_arm->innerTubeOuterDiameter())
                {// collision occurred, find barycentric coordinates (u,v,w) of x on triangle face
                    // from https://ceng2.ktu.edu.tr/~cakir/files/grafikler/Texture_Mapping.pdf
                    const auto [u, v, w] = GeometryUtils::barycentricCoords(x, p1, p2, p3);
                    const Vec3r surface_x = x - result.gradient*result.distance;
                    Solver::ConstraintProjectorReferenceWrapper<Solver::StaticDeformableCollisionConstraint> proj_ref = 
                        xpbd_mesh_obj->addStaticCollisionConstraint(sdf, surface_x, result.gradient, i, u, v, w);
                    
                    virtuoso_arm->addCollisionConstraint(std::move(proj_ref), result.node_index, result.interp_factor);
                    
                }
            }
        }
    }
}

template <bool IsFirstOrder>
void CollisionScene::_collideObjectPair(Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj, Sim::RigidObject* rigid_obj)
{
    // iterate through faces of mesh
    const Geometry::SDF* sdf = rigid_obj->SDF();
    const Geometry::Mesh* mesh = xpbd_mesh_obj->mesh();
    const Geometry::Mesh::FacesMat& faces = mesh->faces();
    for (int i = 0; i < faces.cols(); i++)
    {
        const Eigen::Vector3i& f = faces.col(i);
        const Vec3r& p1 = mesh->vertex(f[0]);
        const Vec3r& p2 = mesh->vertex(f[1]);
        const Vec3r& p3 = mesh->vertex(f[2]);

        // check if centroid of face is close
        const Real p1p2 = (p2-p1).squaredNorm();
        const Real p1p3 = (p3-p1).squaredNorm();
        const Real p2p3 = (p3-p2).squaredNorm();
        const Real max_edge = std::max({p1p2, p1p3, p2p3});
        const Real centroid_dist = sdf->evaluate((p1+p2+p3)/3);
        if (centroid_dist*centroid_dist > max_edge)
            continue;

        const Vec3r x = _frankWolfe(sdf, p1, p2, p3);
        const double distance = sdf->evaluate(x);
        if (distance <= 1e-4)
        {// collision occurred, find barycentric coordinates (u,v,w) of x on triangle face
            // from https://ceng2.ktu.edu.tr/~cakir/files/grafikler/Texture_Mapping.pdf
            const auto [u, v, w] = GeometryUtils::barycentricCoords(x, p1, p2, p3);
            const Vec3r grad = sdf->gradient(x);
            const Vec3r surface_x = x - grad*distance;
            
            if (rigid_obj->isFixed())
            {
                xpbd_mesh_obj->addStaticCollisionConstraint(sdf, surface_x, grad, i, u, v, w);
            }
            else
            {
                xpbd_mesh_obj->addRigidDeformableCollisionConstraint(sdf, rigid_obj, surface_x, grad, i, u, v, w);
            }
            
        }
    }
}

template <bool IsFirstOrder>
void CollisionScene::_collideObjectPair(Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj, Sim::Object* obj2)
{
    // iterate through faces of mesh
    const Geometry::SDF* sdf = obj2->SDF();
    const Geometry::Mesh* mesh = xpbd_mesh_obj->mesh();
    const Geometry::Mesh::FacesMat& faces = mesh->faces();
    for (int i = 0; i < faces.cols(); i++)
    {
        const Eigen::Vector3i& f = faces.col(i);
        const Vec3r& p1 = mesh->vertex(f[0]);
        const Vec3r& p2 = mesh->vertex(f[1]);
        const Vec3r& p3 = mesh->vertex(f[2]);

        // check if centroid of face is close
        const Real p1p2 = (p2-p1).squaredNorm();
        const Real p1p3 = (p3-p1).squaredNorm();
        const Real p2p3 = (p3-p2).squaredNorm();
        const Real max_edge = std::max({p1p2, p1p3, p2p3});
        const Real centroid_dist = sdf->evaluate((p1+p2+p3)/3);
        if (centroid_dist*centroid_dist > max_edge)
            continue;

        const Vec3r x = _frankWolfe(sdf, p1, p2, p3);
        const double distance = sdf->evaluate(x);
        if (distance <= 1e-4)
        {// collision occurred, find barycentric coordinates (u,v,w) of x on triangle face
            // from https://ceng2.ktu.edu.tr/~cakir/files/grafikler/Texture_Mapping.pdf
            const auto [u, v, w] = GeometryUtils::barycentricCoords(x, p1, p2, p3);
            const Vec3r grad = sdf->gradient(x);
            const Vec3r surface_x = x - grad*distance;
            xpbd_mesh_obj->addStaticCollisionConstraint(sdf, surface_x, grad, i, u, v, w);
            
        }
    }
}

Vec3r CollisionScene::_frankWolfe(const Geometry::SDF* sdf, const Vec3r& p1, const Vec3r& p2, const Vec3r& p3) const
{
    // find starting iterate - the triangle vertex with the smallest value of SDF
    const Real d_p1 = sdf->evaluate(p1);
    const Real d_p2 = sdf->evaluate(p2);
    const Real d_p3 = sdf->evaluate(p3);

    Vec3r x;
    if (d_p1 <= d_p2 && d_p1 <= d_p3)       x = p1;
    else if (d_p2 <= d_p1 && d_p2 <= d_p3)  x = p2;
    else                                    x = p3;

    Vec3r s;
    for (int i = 0; i < 32; i++)
    {
        const Real alpha = 2.0/(i+3);
        const Vec3r& gradient = sdf->gradient(x);
        const Real sg1 = p1.dot(gradient);
        const Real sg2 = p2.dot(gradient);
        const Real sg3 = p3.dot(gradient);

        if (sg1 < sg2 && sg1 < sg3)       s = p1;
        else if (sg2 < sg1 && sg2 < sg3)  s = p2;
        else                                s = p3;

        x = x + alpha * (s - x);
        
    }

    return x;
}

// } // namespace Collision