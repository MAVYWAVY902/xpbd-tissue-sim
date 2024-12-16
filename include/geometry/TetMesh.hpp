#ifndef __TET_MESH_HPP
#define __TET_MESH_HPP

#include "geometry/Mesh.hpp"

namespace Geometry
{

class TetMesh : public Mesh
{
    TetMesh(const VerticesMat& vertices, const FacesMat& faces, const ElementsMat& elements);

    const ElementsMat& elements() const { return _elements; }
    int numElements() const { return _elements.cols(); }

    const Eigen::Vector4i& element(const int index) const { return _elements.col(index); }

    /** Returns the volume of the specified element. */
    double volume(const int index) const;

    /** Returns the number of edges along with the average edge length in the tetrahedra of the mesh.
     * Note that this is different from averageFaceEdgeLength, which only returns the average edge length in the faces (i.e. the surface) of the mesh.
     */
    std::pair<int, double> averageTetEdgeLength() const;

    protected:
    ElementsMat _elements;
};

} // namespace Geometry

#endif // __TET_MESH_HPP
