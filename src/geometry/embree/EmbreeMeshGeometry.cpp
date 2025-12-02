#include <embree4/rtcore.h>

#include "geometry/embree/EmbreeMeshGeometry.hpp"
#include "geometry/embree/EmbreeQueryStructs.hpp"
#include "geometry/embree/EmbreeTetMeshGeometry.hpp"

#include <iostream>

namespace Geometry
{

EmbreeMeshGeometry::EmbreeMeshGeometry(const Geometry::Mesh* mesh)
: _mesh(mesh), _undeformed_scene(nullptr)
{
    // if double precision is being used in the sim, we must allocate space fo the float vertex buffer
    if constexpr (std::is_same_v<Real, double>)
        _vertex_buffer.resize(mesh->numVertices()*3);
    
    _initial_vertex_buffer.resize(mesh->numVertices()*3);
    const Real* mesh_vertices = _mesh->vertices().data();
    for (int i = 0; i < _mesh->numVertices()*3; i++)
    {
        _initial_vertex_buffer[i] = static_cast<float>(mesh_vertices[i]);
    }
}

EmbreeMeshGeometry::~EmbreeMeshGeometry()
{
    if (_undeformed_scene)
        rtcReleaseScene(_undeformed_scene);
}

const float* EmbreeMeshGeometry::vertices() const
{
    // if double precision is being used in the sim, we need to return the float vertex buffer
    if constexpr (std::is_same_v<Real, double>)
    {
        return _vertex_buffer.data();
    }
    // otherwise we can just the mesh vertices themselves
    else
    {
        return reinterpret_cast<const float*>(_mesh->vertices().data()); // use reinterpret_cast to avoid compiler error
    }
}

void EmbreeMeshGeometry::copyVertices()
{
    if constexpr (std::is_same_v<Real, double>)
    {
        const Real* mesh_vertices = _mesh->vertices().data();
        for (int i = 0; i < _mesh->numVertices()*3; i++)
        {
            _vertex_buffer[i] = static_cast<float>(mesh_vertices[i]);
        }
    }
}

void EmbreeMeshGeometry::boundsFuncTriangle(const struct RTCBoundsFunctionArguments *args)
{
    const EmbreeMeshGeometry *geom = static_cast<const EmbreeMeshGeometry *>(args->geometryUserPtr);
    const int *indices = geom->faceIndices() + 3 * args->primID;
    const float *v1 = geom->vertices() + 3 * indices[0];
    const float *v2 = geom->vertices() + 3 * indices[1];
    const float *v3 = geom->vertices() + 3 * indices[2];

    RTCBounds* bounds = args->bounds_o;
    bounds->lower_x = std::min({v1[0], v2[0], v3[0]});
    bounds->lower_y = std::min({v1[1], v2[1], v3[1]});
    bounds->lower_z = std::min({v1[2], v2[2], v3[2]});

    bounds->upper_x = std::max({v1[0], v2[0], v3[0]});
    bounds->upper_y = std::max({v1[1], v2[1], v3[1]});
    bounds->upper_z = std::max({v1[2], v2[2], v3[2]});
}

void EmbreeMeshGeometry::boundsFuncTriangleInitialVertices(const struct RTCBoundsFunctionArguments *args)
{
    const EmbreeMeshGeometry *geom = static_cast<const EmbreeMeshGeometry *>(args->geometryUserPtr);
    const int *indices = geom->faceIndices() + 3 * args->primID;
    const float *v1 = geom->initialVertices() + 3 * indices[0];
    const float *v2 = geom->initialVertices() + 3 * indices[1];
    const float *v3 = geom->initialVertices() + 3 * indices[2];

    RTCBounds* bounds = args->bounds_o;
    bounds->lower_x = std::min({v1[0], v2[0], v3[0]});
    bounds->lower_y = std::min({v1[1], v2[1], v3[1]});
    bounds->lower_z = std::min({v1[2], v2[2], v3[2]});

    bounds->upper_x = std::max({v1[0], v2[0], v3[0]});
    bounds->upper_y = std::max({v1[1], v2[1], v3[1]});
    bounds->upper_z = std::max({v1[2], v2[2], v3[2]});
}

void EmbreeMeshGeometry::intersectFuncTriangle(const RTCIntersectFunctionNArguments *args)
{
    const int* valid = args->valid;
    const EmbreeMeshGeometry* geom = static_cast<EmbreeMeshGeometry*>(args->geometryUserPtr);

    // only consider the geometry we're interested in
    // if args->geomId is "invalid", we are interested in all geometry
    // if (args->geomID != RTC_INVALID_GEOMETRY_ID && args->geomID != geom->meshGeomID())
    //     return;

    assert(args->N == 1);

    if (!valid[0])
        return;

    // get vertices of face
    const int *indices = geom->faceIndices() + 3 * args->primID;
    const float *v1 = geom->vertices() + 3 * indices[0];
    const float *v2 = geom->vertices() + 3 * indices[1];
    const float *v3 = geom->vertices() + 3 * indices[2];

    // loop through cast rays (there may be more than 1 in a batch)
    RTCRayHit* rayhit = (RTCRayHit*)args->rayhit;

    // test intersection between ray and triangle
    // this function will update the ray's tfar and the hit's u and v and geometry normal (if there is an intersection)
    if (_rayTriangleIntersect(&rayhit->ray, &rayhit->hit, v1, v2, v3))
    {
        // if there is an intersection, update the hit's primID and geomID for the intersected face
        rayhit->hit.primID = args->primID;
        rayhit->hit.geomID = args->geomID;
    }
}

void EmbreeMeshGeometry::intersectFuncTriangleInitialVertices(const RTCIntersectFunctionNArguments *args)
{
    const int* valid = args->valid;
    const EmbreeMeshGeometry* geom = static_cast<EmbreeMeshGeometry*>(args->geometryUserPtr);

    // only consider the geometry we're interested in
    // if args->geomId is "invalid", we are interested in all geometry
    // if (args->geomID != RTC_INVALID_GEOMETRY_ID && args->geomID != geom->meshGeomID())
    //     return;

    assert(args->N == 1);

    if (!valid[0])
        return;

    // get vertices of face
    const int *indices = geom->faceIndices() + 3 * args->primID;
    const float *v1 = geom->initialVertices() + 3 * indices[0];
    const float *v2 = geom->initialVertices() + 3 * indices[1];
    const float *v3 = geom->initialVertices() + 3 * indices[2];

    // loop through cast rays (there may be more than 1 in a batch)
    RTCRayHit* rayhit = (RTCRayHit*)args->rayhit;

    // test intersection between ray and triangle
    // this function will update the ray's tfar and the hit's u and v and geometry normal (if there is an intersection)
    if (_rayTriangleIntersect(&rayhit->ray, &rayhit->hit, v1, v2, v3))
    {
        // if there is an intersection, update the hit's primID and geomID for the intersected face
        rayhit->hit.primID = args->primID;
        rayhit->hit.geomID = args->geomID;
    }
}

bool EmbreeMeshGeometry::pointQueryFuncTriangle(RTCPointQueryFunctionArguments *args)
{
    // Get user data containing the query point and results vector
    EmbreeClosestPointQueryUserData *userData = static_cast<EmbreeClosestPointQueryUserData *>(args->userPtr);
    // Get the geometry data
    const EmbreeMeshGeometry *geom = userData->geom;

    // only consider point queries for the geometry we're interested in
    // TODO: should we do point queries for 
    if (args->geomID != geom->meshGeomID())
        return true;

    
    const int *indices = geom->faceIndices() + 3 * args->primID;
    const float *v1 = geom->vertices() + 3 * indices[0];
    const float *v2 = geom->vertices() + 3 * indices[1];
    const float *v3 = geom->vertices() + 3 * indices[2];

    
    const float *point = userData->point;
    float closest_point[3]; closest_point[0] = 0; closest_point[1] = 0; closest_point[2] = 0;

    _closestPointTriangle(point, v1, v2, v3, closest_point);
    const float d = (Eigen::Map<const Eigen::Vector3f>(point) - Eigen::Map<const Eigen::Vector3f>(closest_point)).norm();

    if (d < args->query->radius)
    {
        EmbreeHit& hit = userData->result;
        args->query->radius = d;
        hit.prim_index = args->primID;
        hit.hit_point[0] = Real(closest_point[0]); 
        hit.hit_point[1] = Real(closest_point[1]);
        hit.hit_point[2] = Real(closest_point[2]);

        return true; // return true to indicate that the query radius changed
    }

    return false;
}

bool EmbreeMeshGeometry::pointQueryFuncTriangleInterObject(RTCPointQueryFunctionArguments *args)
{
    // Safety check: validate args and userPtr
    if (!args || !args->userPtr)
        return false;
    
    // Get user data containing the query parameters
    EmbreeInterObjectCollisionQueryUserData *userData = static_cast<EmbreeInterObjectCollisionQueryUserData *>(args->userPtr);
    
    // Safety check: validate userData fields
    if (!userData->geom || !userData->point)
        return false;
    
    // Get the target geometry we're searching for (from our query user data)
    const EmbreeTetMeshGeometry *geom = userData->geom;

    // IMPORTANT: Only consider triangles from the target geometry
    // args->geomID is the ID of the geometry that Embree found in the BVH traversal
    // We only want hits from the target object, not from other objects in the scene
    if (args->geomID != geom->meshGeomID())
        return false;

    // Safety check: validate primID is within bounds
    // Each TetMeshObject has a surface mesh - check primitive index is valid
    if (args->primID < 0)
        return false;
    
    // Now safe to access geometry data since we confirmed geomID matches
    const int *indices = geom->faceIndices() + 3 * args->primID;
    const float *v1 = geom->vertices() + 3 * indices[0];
    const float *v2 = geom->vertices() + 3 * indices[1];
    const float *v3 = geom->vertices() + 3 * indices[2];

    const float *point = userData->point;
    
    // Calculate point-to-triangle distance WITHOUT using Eigen::Map to avoid alignment issues
    // Use simple vector math with raw floats
    
    // Vector from v1 to point
    float v1p[3] = {point[0] - v1[0], point[1] - v1[1], point[2] - v1[2]};
    
    // Triangle edges
    float edge1[3] = {v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2]};
    float edge2[3] = {v3[0] - v1[0], v3[1] - v1[1], v3[2] - v1[2]};
    
    // Triangle normal (edge1 × edge2)
    float normal[3] = {
        edge1[1] * edge2[2] - edge1[2] * edge2[1],
        edge1[2] * edge2[0] - edge1[0] * edge2[2],
        edge1[0] * edge2[1] - edge1[1] * edge2[0]
    };
    
    float normal_len = std::sqrt(normal[0]*normal[0] + normal[1]*normal[1] + normal[2]*normal[2]);
    if (normal_len < 1e-10f)
        return false; // Degenerate triangle
    
    // Normalize
    normal[0] /= normal_len;
    normal[1] /= normal_len;
    normal[2] /= normal_len;
    
    // Distance from point to triangle plane
    float plane_dist = std::abs(v1p[0]*normal[0] + v1p[1]*normal[1] + v1p[2]*normal[2]);
    
    // Quick rejection: if distance to plane is already beyond search radius, skip
    if (plane_dist > userData->search_radius)
        return false;
    
    // For a more accurate check, we should project to triangle and check barycentric coords,
    // but for BVH culling purposes, plane distance is a good conservative estimate.
    // The actual geometric validation will happen in the collision detection code.
    const float d = plane_dist;

    // Add all triangles within search radius (not just the closest one)
    if (d <= userData->search_radius)
    {
        EmbreeHit hit;
        hit.obj = reinterpret_cast<const Sim::MeshObject*>(userData->obj_ptr);
        hit.prim_index = args->primID;
        // For BVH queries, we don't need exact hit point - just the primitive index
        // The collision detection code will recalculate the exact geometry
        hit.hit_point[0] = 0.0;
        hit.hit_point[1] = 0.0;
        hit.hit_point[2] = 0.0;
        userData->result.insert(hit);
    }

    return false; // Continue traversal to find all triangles within radius
}

bool EmbreeMeshGeometry::pointQueryFuncTriangleInitialVertices(RTCPointQueryFunctionArguments *args)
{
    // Get user data containing the query point and results vector
    EmbreeClosestPointQueryUserData *userData = static_cast<EmbreeClosestPointQueryUserData *>(args->userPtr);
    // Get the geometry data
    const EmbreeMeshGeometry *geom = userData->geom;

    // only consider point queries for the geometry we're interested in
    // TODO: should we do point queries for 
    if (args->geomID != geom->meshGeomID())
        return true;

    
    const int *indices = geom->faceIndices() + 3 * args->primID;
    const float *v1 = geom->initialVertices() + 3 * indices[0];
    const float *v2 = geom->initialVertices() + 3 * indices[1];
    const float *v3 = geom->initialVertices() + 3 * indices[2];

    
    const float *point = userData->point;
    float closest_point[3]; closest_point[0] = 0; closest_point[1] = 0; closest_point[2] = 0;

    _closestPointTriangle(point, v1, v2, v3, closest_point);
    const float d = (Eigen::Map<const Eigen::Vector3f>(point) - Eigen::Map<const Eigen::Vector3f>(closest_point)).norm();

    if (d < args->query->radius)
    {
        EmbreeHit& hit = userData->result;
        args->query->radius = d;
        hit.prim_index = args->primID;
        hit.hit_point[0] = Real(closest_point[0]); 
        hit.hit_point[1] = Real(closest_point[1]);
        hit.hit_point[2] = Real(closest_point[2]);

        return true; // return true to indicate that the query radius changed
    }

    return false;
}

// adapted from: https://github.com/RenderKit/embree/blob/master/tutorials/common/math/closest_point.h
void EmbreeMeshGeometry::_closestPointTriangle(const float p_[3], const float a_[3], const float b_[3], const float c_[3], float out_[3])
{
    Eigen::Map<const Eigen::Vector3f> p(p_);
    Eigen::Map<const Eigen::Vector3f> a(a_);
    Eigen::Map<const Eigen::Vector3f> b(b_);
    Eigen::Map<const Eigen::Vector3f> c(c_);
    Eigen::Map<Eigen::Vector3f> out(out_);

    const Eigen::Vector3f ab = b - a;
    const Eigen::Vector3f ac = c - a;
    const Eigen::Vector3f ap = p - a;

    const float d1 = ab.dot(ap);
    const float d2 = ac.dot(ap);
    if (d1 <= 0.f && d2 <= 0.f)
    {
        out = a;
        return;
    }

    const Eigen::Vector3f bp = p - b;
    const float d3 = ab.dot(bp);
    const float d4 = ac.dot(bp);
    if (d3 >= 0.f && d4 <= d3)
    {
        out = b;
        return;
    }

    const Eigen::Vector3f cp = p - c;
    const float d5 = ab.dot(cp);
    const float d6 = ac.dot(cp);
    if (d6 >= 0.f && d5 <= d6)
    {
        out = c;
        return;
    }

    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f)
    {
        const float v = d1 / (d1 - d3);
        out = a + v * ab;
        return;
    }
    
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f)
    {
        const float v = d2 / (d2 - d6);
        out = a + v * ac;
        return;
    }
    
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f)
    {
        const float v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        out = b + v * (c - b);
        return;
    }

    const float denom = 1.f / (va + vb + vc);
    const float v = vb * denom;
    const float w = vc * denom;
    out = a + v * ab + w * ac;
    // out_[0] = out[0]; out_[1] = out[1]; out_[2] = out[2];
    return;
}

bool EmbreeMeshGeometry::_rayTriangleIntersect(RTCRay* ray, RTCHit* hit,
    const float a_[3], const float b_[3], const float c_[3])
{
    // Eigen::Map<const Eigen::Vector3f> ray_origin(ray_origin_);
    // Eigen::Map<const Eigen::Vector3f> ray_dir(ray_dir_);
    const Eigen::Vector3f ray_origin(ray->org_x, ray->org_y, ray->org_z);
    const Eigen::Vector3f ray_dir(ray->dir_x, ray->dir_y, ray->dir_z);

    Eigen::Map<const Eigen::Vector3f> a(a_);
    Eigen::Map<const Eigen::Vector3f> b(b_);
    Eigen::Map<const Eigen::Vector3f> c(c_);

    constexpr float epsilon = std::numeric_limits<float>::epsilon();

    const Eigen::Vector3f edge1 = b - a;
    const Eigen::Vector3f edge2 = c - a;
    const Eigen::Vector3f ray_cross_e2 = ray_dir.cross(edge2);
    const float det = edge1.dot(ray_cross_e2);

    if (det > -epsilon && det < epsilon)
        return false;
    
    const float inv_det = 1.0 / det;
    const Eigen::Vector3f s = ray_origin - a;
    const float v = inv_det * s.dot(ray_cross_e2);

    if ( (v < 0 && std::abs(v) > epsilon) || (v > 1 && std::abs(v - 1) > epsilon) )
        return false;

    const Eigen::Vector3f s_cross_e1 = s.cross(edge1);
    const float w = inv_det * ray_dir.dot(s_cross_e1);

    if ( (w < 0 && std::abs(w) > epsilon) || (w + v > 1 && std::abs(w + v - 1) > epsilon) )
        return false;
    
    const float t = inv_det * edge2.dot(s_cross_e1);

    if (t >= ray->tnear && t <= ray->tfar)
    {
        ray->tfar = t;
        hit->u = (1-v-w);
        hit->v = v;

        // compute normal
        const Eigen::Vector3f n = edge1.cross(edge2);
        hit->Ng_x = n[0];
        hit->Ng_y = n[1];
        hit->Ng_z = n[2];

        return true;
    }
    else
    {
        return false;
    }
}

} // namespace Geometry