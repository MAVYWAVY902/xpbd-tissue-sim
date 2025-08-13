#ifndef __TET_MESH_HPP
#define __TET_MESH_HPP

#include "geometry/Mesh.hpp"

namespace Geometry
{

/** A class for a tetrahedral mesh which consists of a set of vertices, and a set of volumetric tetrahedral elements connecting those vertices.
 * Note that this class extends the Mesh base class, meaning that it also has a matrix for faces.
 *  - These faces are only SURFACE faces - this is useful for things like visualization and collision detection.
 * The elements are specified as 4-vectors of element indices.
 */
class TetMesh : public Mesh
{
    public:
    /** Constructs a tetrahedral mesh from a set of vertices, faces, and elements.
     * This is usually done using the helper methods in the MeshUtils library.
     */
    TetMesh(const VerticesMat& vertices, const FacesMat& faces, const ElementsMat& elements);

    TetMesh(const TetMesh& other);

    TetMesh(TetMesh&& other);

    /** Returns a const-reference to the elements of the mesh. */
    const ElementsMat& elements() const { return _elements; }

    /** Returns a non-const-reference to the elements of the mesh. */
    ElementsMat& elements() { return _elements; }

    /** Returns the number of elements in the mesh. */
    int numElements() const { return _elements.cols(); }

    /** Returns a single element as an Eigen 4-vector, given the element index. */
    Eigen::Vector4i element(int index) const { return _elements.col(index); }

    /** Returns the current volume of the specified element. */
    Real elementVolume(int index) const;

    /** Returns the rest volume of the specified element. */
    Real elementRestVolume(int index) const { return _element_rest_volumes[index]; }

    /** Returns deformation gradient for the specified element.
     * Assumes linear shape functions (deformation gradient is constant throughout the element)
     */
    Mat3r elementDeformationGradient(int index) const;

    /** Returns the number of edges along with the average edge length in the tetrahedra of the mesh.
     * Note that this is different from averageFaceEdgeLength, which only returns the average edge length in the faces (i.e. the surface) of the mesh.
     */
    std::pair<int, Real> averageTetEdgeLength() const;

 #ifdef HAVE_CUDA
    virtual void createGPUResource() override;
 #endif

    const std::vector<int>& attachedElementsToVertex(int vertex_index) const { return _attached_elements_to_vertex[vertex_index]; }

    protected:
    ElementsMat _elements;  // the matrix of tetrahedral elements

    std::vector<Real> _element_rest_volumes;

    /** inverse undeformed basis for each element
     *   - used in calculating the deformation gradient (F = XQ) where X is current deformed basis, Q is inverse undeformed basis
     *  calculated [v1 - v4, v2 - v4, v3 - v4]
   */
    std::vector<Mat3r> _element_inv_undeformed_basis;  

    std::vector<std::vector<int>> _attached_elements_to_vertex; // lists the elements (by index) attached to a vertex
};

} // namespace Geometry

#endif // __TET_MESH_HPP
