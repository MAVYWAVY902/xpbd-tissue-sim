#include "geometry/embree/EmbreeMeshGeometry.hpp"
#include "geometry/embree/EmbreeQueryStructs.hpp"
#include "geometry/embree/EmbreeTetMeshGeometry.hpp"

namespace Geometry
{

EmbreeMeshGeometry::EmbreeMeshGeometry(const Geometry::Mesh* mesh)
: _mesh(mesh)
{
    // if double precision is being used in the sim, we must allocate space fo the float vertex buffer
    if constexpr (std::is_same_v<Real, double>)
        _vertex_buffer.resize(mesh->numVertices()*3);
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

void EmbreeMeshGeometry::intersectFuncTriangle(const RTCIntersectFunctionNArguments *args)
{
    // TODO: triangle-ray intersect function
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

// adapted from: https://github.com/RenderKit/embree/blob/master/tutorials/common/math/closest_point.h
void EmbreeMeshGeometry::_closestPointTriangle(const float p_[3], const float a_[3], const float b_[3], const float c_[3], float out_[3])
{
    const Eigen::Vector3f p = Eigen::Map<const Eigen::Vector3f>(p_);
    const Eigen::Vector3f a = Eigen::Map<const Eigen::Vector3f>(a_);
    const Eigen::Vector3f b = Eigen::Map<const Eigen::Vector3f>(b_);
    const Eigen::Vector3f c = Eigen::Map<const Eigen::Vector3f>(c_);
    Eigen::Vector3f out;// = Eigen::Map<Eigen::Vector3f>(out_);

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
    out_[0] = out[0]; out_[1] = out[1]; out_[2] = out[2];
    return;
}

} // namespace Geometry