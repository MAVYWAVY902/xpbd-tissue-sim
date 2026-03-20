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
    const Real collision_threshold = 3e-3; // 3mm - increased to detect collisions earlier and prevent penetration
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
    
    const Geometry::Mesh::FacesMat& faces1 = mesh1->faces();
    const Geometry::Mesh::FacesMat& faces2 = mesh2->faces();
    
    // ========== SMART ALGORITHM SELECTION ==========
    // Get collision algorithm from config (options: "auto", "brute-force", "embree")
    std::string algorithm_setting = "auto";  // default
    if (_sim && _sim->config())
    {
        algorithm_setting = _sim->config()->collisionAlgorithm();
    }
    
    bool use_brute_force;
    if (algorithm_setting == "brute-force")
    {
        use_brute_force = true;  // Force brute force
    }
    else if (algorithm_setting == "embree")
    {
        use_brute_force = false;  // Force Embree BVH
    }
    else  // "auto" or any other value
    {
        // Auto-select: use brute force for small meshes (< 2000 faces), Embree for large
        const int BRUTE_FORCE_THRESHOLD = 2000;
        use_brute_force = (faces1.cols() < BRUTE_FORCE_THRESHOLD && 
                          faces2.cols() < BRUTE_FORCE_THRESHOLD);
    }
    
    // Performance counters (used for both algorithms)
    int checks_performed = 0;
    int checks_skipped = 0;
    int embree_hits = 0;
    int total_constraints_part1 = 0;
    int total_constraints_part2 = 0;
    
    // Print algorithm selection (only once per program run)
    static bool first_run = true;
    if (first_run)
    {
        first_run = false;
        std::cout << "[Collision Algorithm] Config setting: \"" << algorithm_setting << "\"\n";
        std::cout << "[Collision Algorithm] Mesh sizes: " << faces1.cols() << " and " 
                  << faces2.cols() << " faces\n";
        std::cout << "[Collision Algorithm] Using: " 
                  << (use_brute_force ? "BRUTE FORCE (cache-friendly)" : "EMBREE BVH (spatial acceleration)")
                  << "\n";
    }
    // ========== End Algorithm Selection ==========
    
    // Constants used by both brute force and Embree paths
    const Real embree_search_radius = 0.02;  // 20mm - 2cm search radius (reduced from 0.2m to avoid false positives)
    const Real bary_epsilon = -0.01; // Only 1% tolerance - much tighter than -0.1
    
    // ========== PART 1: Vertices of obj1 vs Faces of obj2 ==========
    // Check ALL surface vertices for collision (no sparse sampling)
    const int VERTEX_SAMPLING_INTERVAL = 1;
    
    if (use_brute_force)
    {
        // BRUTE FORCE PATH: Simple nested loops (fast for small meshes)
        for (int v_idx = 0; v_idx < mesh1->numVertices(); v_idx++)
        {
            if (!mesh1->vertexOnSurface(v_idx))
                continue;
                
            const Vec3r& vertex1 = mesh1->vertex(v_idx);
            
            // Check this vertex against ALL faces of object2
            for (int face_idx = 0; face_idx < faces2.cols(); face_idx++)
            {
                checks_performed++;
                
                const Eigen::Vector3i& face = faces2.col(face_idx);
                const Vec3r& p1 = mesh2->vertex(face[0]);
                const Vec3r& p2 = mesh2->vertex(face[1]);
                const Vec3r& p3 = mesh2->vertex(face[2]);
                
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
                
                // Compute barycentric coordinates
                const auto [u, v, w] = GeometryUtils::barycentricCoords(projected_point, p1, p2, p3);
                
                // Check if point is inside triangle
                if (u >= bary_epsilon && v >= bary_epsilon && w >= bary_epsilon)
                {
                    total_constraints_part1++;
                    
                    // Get pointers and masses from object2's face vertices
                    Real* p1_ptr = mesh2->vertexPointer(face[0]);
                    Real* p2_ptr = mesh2->vertexPointer(face[1]);
                    Real* p3_ptr = mesh2->vertexPointer(face[2]);
                    
                    Real m1 = xpbd_mesh_obj2->vertexConstraintInertia(face[0]);
                    Real m2 = xpbd_mesh_obj2->vertexConstraintInertia(face[1]);
                    Real m3 = xpbd_mesh_obj2->vertexConstraintInertia(face[2]);
                    
                    // Add constraint
                    xpbd_mesh_obj1->addInterObjectCollisionConstraint(
                        v_idx, face[0], p1_ptr, m1, face[1], p2_ptr, m2, face[2], p3_ptr, m3
                    );
                }
            }
        }
    }
    else
    {
        // EMBREE BVH PATH: Spatial acceleration (fast for large meshes)
        // ⚡ SPARSE SAMPLING: Check every Nth vertex to reduce BVH queries
        for (int v_idx = 0; v_idx < mesh1->numVertices(); v_idx += VERTEX_SAMPLING_INTERVAL)
        {
            if (!mesh1->vertexOnSurface(v_idx))
                continue;
            
            const Vec3r& vertex1 = mesh1->vertex(v_idx);
            
            // Use Embree BVH to find all triangles near this vertex!
            std::set<Geometry::EmbreeHit> nearby_triangles = 
                _embree_scene->interObjectCollisionQuery(vertex1, xpbd_mesh_obj2, embree_search_radius);
            
            embree_hits += nearby_triangles.size();
            checks_skipped += (faces2.cols() - nearby_triangles.size());
            
            // Check ONLY the nearby triangles found by Embree BVH
            for (const auto& hit : nearby_triangles)
            {
                const int face_idx = hit.prim_index;
                
                if (face_idx < 0 || face_idx >= faces2.cols())
                {
                    std::cerr << "[ERROR] Invalid face_idx from Embree: " << face_idx << std::endl;
                    continue;
                }
                
                const Eigen::Vector3i& face = faces2.col(face_idx);
                
                if (face[0] < 0 || face[0] >= mesh2->numVertices() ||
                    face[1] < 0 || face[1] >= mesh2->numVertices() ||
                    face[2] < 0 || face[2] >= mesh2->numVertices())
                {
                    std::cerr << "[ERROR] Invalid vertex indices in face " << face_idx << std::endl;
                    continue;
                }
                
                const Vec3r& p1 = mesh2->vertex(face[0]);
                const Vec3r& p2 = mesh2->vertex(face[1]);
                const Vec3r& p3 = mesh2->vertex(face[2]);
                
                checks_performed++;
                
                // Compute triangle normal
                const Vec3r edge1 = p2 - p1;
                const Vec3r edge2 = p3 - p1;
                const Vec3r normal = edge1.cross(edge2);
                const Real normal_length = normal.norm();
                
                if (normal_length < 1e-10)
                    continue;
                
                const Vec3r normal_normalized = normal / normal_length;
                const Vec3r to_vertex = vertex1 - p1;
                const Real signed_distance = to_vertex.dot(normal_normalized);
                
                if (std::abs(signed_distance) > collision_threshold)
                    continue;
                
                const Vec3r projected_point = vertex1 - signed_distance * normal_normalized;
                const auto [u, v, w] = GeometryUtils::barycentricCoords(projected_point, p1, p2, p3);
                
                if (u >= bary_epsilon && v >= bary_epsilon && w >= bary_epsilon)
                {
                    total_constraints_part1++;
                    
                    Real* p1_ptr = mesh2->vertexPointer(face[0]);
                    Real* p2_ptr = mesh2->vertexPointer(face[1]);
                    Real* p3_ptr = mesh2->vertexPointer(face[2]);
                    
                    Real m1 = xpbd_mesh_obj2->vertexConstraintInertia(face[0]);
                    Real m2 = xpbd_mesh_obj2->vertexConstraintInertia(face[1]);
                    Real m3 = xpbd_mesh_obj2->vertexConstraintInertia(face[2]);
                    
                    xpbd_mesh_obj1->addInterObjectCollisionConstraint(
                        v_idx, face[0], p1_ptr, m1, face[1], p2_ptr, m2, face[2], p3_ptr, m3
                    );
                }
            }
        }
    }
    
    // ========== PART 2: Vertices of obj2 vs Faces of obj1 ==========
    if (use_brute_force)
    {
        // BRUTE FORCE PATH
        for (int v_idx = 0; v_idx < mesh2->numVertices(); v_idx++)
        {
            if (!mesh2->vertexOnSurface(v_idx))
                continue;
                
            const Vec3r& vertex2 = mesh2->vertex(v_idx);
            
            // Check this vertex against ALL faces of object1
            for (int face_idx = 0; face_idx < faces1.cols(); face_idx++)
            {
                checks_performed++;
                
                const Eigen::Vector3i& face = faces1.col(face_idx);
                const Vec3r& p1 = mesh1->vertex(face[0]);
                const Vec3r& p2 = mesh1->vertex(face[1]);
                const Vec3r& p3 = mesh1->vertex(face[2]);
                
                // Compute triangle normal
                const Vec3r edge1 = p2 - p1;
                const Vec3r edge2 = p3 - p1;
                const Vec3r normal = edge1.cross(edge2);
                const Real normal_length = normal.norm();
                
                if (normal_length < 1e-10)
                    continue;
                    
                const Vec3r normal_normalized = normal / normal_length;
                const Vec3r to_vertex = vertex2 - p1;
                const Real signed_distance = to_vertex.dot(normal_normalized);
                
                if (std::abs(signed_distance) > collision_threshold)
                    continue;
                
                const Vec3r projected_point = vertex2 - signed_distance * normal_normalized;
                const auto [u, v, w] = GeometryUtils::barycentricCoords(projected_point, p1, p2, p3);
                
                if (u >= bary_epsilon && v >= bary_epsilon && w >= bary_epsilon)
                {
                    total_constraints_part2++;
                    
                    Real* p1_ptr = mesh1->vertexPointer(face[0]);
                    Real* p2_ptr = mesh1->vertexPointer(face[1]);
                    Real* p3_ptr = mesh1->vertexPointer(face[2]);
                    
                    Real m1 = xpbd_mesh_obj1->vertexConstraintInertia(face[0]);
                    Real m2 = xpbd_mesh_obj1->vertexConstraintInertia(face[1]);
                    Real m3 = xpbd_mesh_obj1->vertexConstraintInertia(face[2]);
                    
                    xpbd_mesh_obj2->addInterObjectCollisionConstraint(
                        v_idx, face[0], p1_ptr, m1, face[1], p2_ptr, m2, face[2], p3_ptr, m3
                    );
                }
            }
        }
    }
    else
    {
        // EMBREE BVH PATH
        // ⚡ SPARSE SAMPLING: Check every Nth vertex to reduce BVH queries
        for (int v_idx = 0; v_idx < mesh2->numVertices(); v_idx += VERTEX_SAMPLING_INTERVAL)
        {
            if (!mesh2->vertexOnSurface(v_idx))
                continue;
            
            const Vec3r& vertex2 = mesh2->vertex(v_idx);
            
            // Use Embree BVH to find all triangles near this vertex
            std::set<Geometry::EmbreeHit> nearby_triangles = 
                _embree_scene->interObjectCollisionQuery(vertex2, xpbd_mesh_obj1, embree_search_radius);
            
            embree_hits += nearby_triangles.size();
            checks_skipped += (faces1.cols() - nearby_triangles.size());
            
            for (const auto& hit : nearby_triangles)
            {
                const int face_idx = hit.prim_index;
                
                if (face_idx < 0 || face_idx >= faces1.cols())
                {
                    std::cerr << "[ERROR] Invalid face_idx from Embree: " << face_idx << std::endl;
                    continue;
                }
                
                const Eigen::Vector3i& face = faces1.col(face_idx);
                
                if (face[0] < 0 || face[0] >= mesh1->numVertices() ||
                    face[1] < 0 || face[1] >= mesh1->numVertices() ||
                    face[2] < 0 || face[2] >= mesh1->numVertices())
                {
                    std::cerr << "[ERROR] Invalid vertex indices in face " << face_idx << std::endl;
                    continue;
                }
                
                const Vec3r& p1 = mesh1->vertex(face[0]);
                const Vec3r& p2 = mesh1->vertex(face[1]);
                const Vec3r& p3 = mesh1->vertex(face[2]);
                
                checks_performed++;
                
                const Vec3r edge1 = p2 - p1;
                const Vec3r edge2 = p3 - p1;
                const Vec3r normal = edge1.cross(edge2);
                const Real normal_length = normal.norm();
                
                if (normal_length < 1e-10)
                    continue;
                    
                const Vec3r normal_normalized = normal / normal_length;
                const Vec3r to_vertex = vertex2 - p1;
                const Real signed_distance = to_vertex.dot(normal_normalized);
                
                if (std::abs(signed_distance) > collision_threshold)
                    continue;
                
                const Vec3r projected_point = vertex2 - signed_distance * normal_normalized;
                const auto [u, v, w] = GeometryUtils::barycentricCoords(projected_point, p1, p2, p3);
                
                if (u >= bary_epsilon && v >= bary_epsilon && w >= bary_epsilon)
                {
                    total_constraints_part2++;
                    
                    Real* p1_ptr = mesh1->vertexPointer(face[0]);
                    Real* p2_ptr = mesh1->vertexPointer(face[1]);
                    Real* p3_ptr = mesh1->vertexPointer(face[2]);
                    
                    Real m1 = xpbd_mesh_obj1->vertexConstraintInertia(face[0]);
                    Real m2 = xpbd_mesh_obj1->vertexConstraintInertia(face[1]);
                    Real m3 = xpbd_mesh_obj1->vertexConstraintInertia(face[2]);
                    
                    xpbd_mesh_obj2->addInterObjectCollisionConstraint(
                        v_idx, face[0], p1_ptr, m1, face[1], p2_ptr, m2, face[2], p3_ptr, m3
                    );
                }
            }
        }
    }
    
    // ========== Performance Logging ==========
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    // Print performance stats every 1000th collision check
    static int collision_count = 0;
    if (++collision_count % 1000 == 0)
    {
        if (use_brute_force)
        {
            std::cout << "[Collision Performance - BRUTE FORCE] Time: " << duration.count() << " μs, "
                      << "Checks performed: " << checks_performed << ", "
                      << "Constraints created: " << (total_constraints_part1 + total_constraints_part2) << "\n";
        }
        else
        {
            Real skip_ratio = checks_skipped > 0 ? 
                100.0 * checks_skipped / (checks_skipped + checks_performed) : 0.0;
            
            // std::cout << "[Collision Performance - EMBREE BVH] Time: " << duration.count() << " μs, "
            //           << "BVH hits: " << embree_hits << ", "
            //           << "Checks performed: " << checks_performed << ", "
            //           << "Culled by BVH: " << checks_skipped << " ("
            //           << std::fixed << std::setprecision(1) << skip_ratio << "%), "
            //           << "Constraints created: " << (total_constraints_part1 + total_constraints_part2) << "\n";
        }
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
    // Collision detection diagnostics
    static int collision_check_count = 0;
    static int total_collisions_detected = 0;
    static bool sdf_warning_printed = false;
    static bool position_info_printed = false;
    int collisions_this_check = 0;
    int faces_culled_by_centroid = 0;
    int faces_checked_detailed = 0;
    
    // iterate through faces of mesh
    const Geometry::SDF* sdf = rigid_obj->SDF();
    
    // Critical check: Does the rigid object have an SDF?
    if (!sdf && !sdf_warning_printed) {
        std::cerr << "\n[CRITICAL ERROR] Rigid object '" << rigid_obj->name() 
                  << "' has NO SDF! Collision detection will FAIL.\n"
                  << "  -> You need to generate an SDF file for this mesh.\n"
                  << "  -> Add 'sdf-filename' to the config, or the SDF will be missing.\n\n";
        sdf_warning_printed = true;
        return;
    }
    
    // Print rigid body position info (once)
    if (!position_info_printed) {
        std::cout << "\n[RIGID BODY INFO] " << rigid_obj->name() << ":\n"
                  << "  Position: (" << rigid_obj->position().transpose() << ")\n"
                  << "  Rotation (quat): (" << rigid_obj->orientation().transpose() << ")\n"
                  << "  Fixed: " << (rigid_obj->isFixed() ? "YES" : "NO") << "\n\n";
        position_info_printed = true;
    }
    
    const Geometry::Mesh* mesh = xpbd_mesh_obj->mesh();
    const Geometry::Mesh::FacesMat& faces = mesh->faces();
    
    // Sample a few face centroids to check SDF values
    Real min_sdf_dist = 1e10;
    Real max_sdf_dist = -1e10;
    Vec3r closest_centroid = Vec3r::Zero();
    Vec3r sample_vertex = mesh->vertex(0);  // Sample first vertex position
    Vec3r rigid_body_pos = rigid_obj->position();  // Get rigid body position
    
    for (int i = 0; i < faces.cols(); i++)
    {
        const Eigen::Vector3i& f = faces.col(i);
        const Vec3r& p1 = mesh->vertex(f[0]);
        const Vec3r& p2 = mesh->vertex(f[1]);
        const Vec3r& p3 = mesh->vertex(f[2]);

        const Vec3r centroid = (p1+p2+p3)/3;
        const Real centroid_dist = sdf->evaluate(centroid);
        
        if (centroid_dist < min_sdf_dist) {
            min_sdf_dist = centroid_dist;
            closest_centroid = centroid;
        }
        max_sdf_dist = std::max(max_sdf_dist, centroid_dist);
        
        // check if centroid of face is close
        const Real p1p2 = (p2-p1).squaredNorm();
        const Real p1p3 = (p3-p1).squaredNorm();
        const Real p2p3 = (p3-p2).squaredNorm();
        const Real max_edge = std::max({p1p2, p1p3, p2p3});
        
        // LESS AGGRESSIVE CULLING: Use safety factor to avoid missing collisions
        // Increased from 4.0 to 100.0 to handle cases where SDF distance (0.5m) >> edge length (0.05m)
        const Real safety_factor = 100.0;
        if (centroid_dist*centroid_dist > safety_factor * max_edge) {
            faces_culled_by_centroid++;
            continue;
        }

        faces_checked_detailed++;
        const Vec3r x = _frankWolfe(sdf, p1, p2, p3);
        const double distance = sdf->evaluate(x);
        // Detection threshold: create constraints BEFORE penetration to prevent oscillation.
        // Surface margin: push vertices to margin distance outside surface, not exactly d=0.
        const Real collision_detect_threshold = 2e-3; // 2mm - detect near-collisions early
        const Real surface_margin = 1e-3;             // 1mm - keep vertices this far from surface
        if (distance <= collision_detect_threshold)
        {// collision or near-collision, find barycentric coordinates (u,v,w) of x on triangle face
            // from https://ceng2.ktu.edu.tr/~cakir/files/grafikler/Texture_Mapping.pdf
            const auto [u, v, w] = GeometryUtils::barycentricCoords(x, p1, p2, p3);
            const Vec3r grad = sdf->gradient(x);
            // Offset surface point outward by margin so constraint enforces d >= margin
            const Vec3r surface_x = x - grad*distance + grad * surface_margin;

            collisions_this_check++;

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
    
    // Print collision diagnostics (print every 500 checks to avoid spam)
    collision_check_count++;
    total_collisions_detected += collisions_this_check;
    
    // if (collision_check_count % 5000 == 0 || collisions_this_check > 0)
    // {
    //     Real deform_to_rigid_dist = (sample_vertex - rigid_body_pos).norm();
        
    //     // DEBUG: Show body-frame transformation
    //     Vec3r sample_vertex_body = rigid_obj->globalToBody(sample_vertex);
    //     Vec3r closest_centroid_body = rigid_obj->globalToBody(closest_centroid);
        
    //     std::cout << "[RIGID-DEFORM COLLISION] Check #" << collision_check_count 
    //               << " | Pair: " << xpbd_mesh_obj->name() << " <-> " << rigid_obj->name()
    //               << "\n  Faces total: " << faces.cols()
    //               << " | Culled: " << faces_culled_by_centroid
    //               << " | Checked: " << faces_checked_detailed
    //               << "\n  SDF range: [" << std::setprecision(6) << min_sdf_dist << ", " << max_sdf_dist << "] meters"
    //               << "\n  Deform sample vertex (world): (" << std::setprecision(4) << sample_vertex.transpose() << ")"
    //               << "\n  Deform sample vertex (body):  (" << sample_vertex_body.transpose() << ")"
    //               << "\n  Rigid body position:  (" << rigid_body_pos.transpose() << ")"
    //               << "\n  Direct distance (vertex to rigid center): " << std::setprecision(4) << deform_to_rigid_dist << "m"
    //               << "\n  Closest face centroid (world): (" << closest_centroid.transpose() << ")"
    //               << "\n  Closest face centroid (body):  (" << closest_centroid_body.transpose() << ")"
    //               << "\n  Closest centroid SDF_dist=" << min_sdf_dist << " meters = " << (min_sdf_dist*1000) << "mm"
    //               << "\n  Collisions: " << collisions_this_check << " | Total: " << total_collisions_detected
    //               << " | Time: " << _sim->time() << "s\n";
    // }
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