#include "geometry/TetMesh.hpp"

#include <set>

namespace Geometry
{

TetMesh::TetMesh(const VerticesMat& vertices, const FacesMat& faces, const ElementsMat& elements)
    : Mesh(vertices, faces), _elements(elements)
{}

Real TetMesh::elementVolume(const int index) const
{
    const Eigen::Vector4i& elem = element(index);
    const Vec3r& v1 = vertex(elem[0]);
    const Vec3r& v2 = vertex(elem[1]);
    const Vec3r& v3 = vertex(elem[2]);
    const Vec3r& v4 = vertex(elem[3]);

    Mat3r X;
    X.col(0) = (v1 - v4);
    X.col(1) = (v2 - v4);
    X.col(2) = (v3 - v4);

    return std::abs(X.determinant() / 6.0);
}

std::pair<int, Real> TetMesh::averageTetEdgeLength() const
{
    std::set<std::pair<int, int>> edges;

    auto make_edge = [] (int v1, int v2)
    {
        if (v1 > v2)
            return std::pair<int, int>(v2, v1);
        else 
            return std::pair<int, int>(v1, v2);
    };

    Real total_length = 0;
    for (const auto& elem : _elements.colwise())
    {
        const Vec3r& v1 = vertex(elem(0));
        const Vec3r& v2 = vertex(elem(1));
        const Vec3r& v3 = vertex(elem(2));
        const Vec3r& v4 = vertex(elem(3));

        std::pair<int, int> e1 = make_edge(elem(0), elem(1));
        std::pair<int, int> e2 = make_edge(elem(0), elem(2));
        std::pair<int, int> e3 = make_edge(elem(0), elem(3));
        std::pair<int, int> e4 = make_edge(elem(1), elem(2));
        std::pair<int, int> e5 = make_edge(elem(1), elem(3));
        std::pair<int, int> e6 = make_edge(elem(2), elem(3));
        

        if (edges.count(e1) == 0)
        {
            total_length += (v1-v2).norm();
            edges.insert(e1);
        }
        if (edges.count(e2) == 0)
        {
            total_length += (v1-v3).norm();
            edges.insert(e2);
        }
        if (edges.count(e3) == 0)
        {
            total_length += (v1-v4).norm();
            edges.insert(e3);
        }
        if (edges.count(e4) == 0)
        {
            total_length += (v2-v3).norm();
            edges.insert(e4);
        }
        if (edges.count(e5) == 0)
        {
            total_length += (v2-v4).norm();
            edges.insert(e5);
        }
        if (edges.count(e6) == 0)
        {
            total_length += (v3-v4).norm();
            edges.insert(e6);
        }
    }

    return std::pair<int,Real>(edges.size(), total_length/edges.size());
}

} // namespace Geometry