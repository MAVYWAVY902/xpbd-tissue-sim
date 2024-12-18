#include "simobject/ElasticMeshObject.hpp"
#include "utils/MeshUtils.hpp"
#include <filesystem>

ElasticMeshObject::ElasticMeshObject(const std::string& name)
    : MeshObject(name), _material(ElasticMaterial::RUBBER())
{

}

// ElasticMeshObject::ElasticMeshObject(const std::string& name, const YAML::Node& config)
//     : MeshObject(name, config), _material(ElasticMaterial::RUBBER())
// {
//     // read in material
//     YAML::Node material_yaml_node = config["material"];
//     if (material_yaml_node.Type() != YAML::NodeType::Null)
//     {
//         const std::string& mat_name = material_yaml_node["name"].as<std::string>();
//         const double mat_density = material_yaml_node["density"].as<double>();
//         const double mat_E = material_yaml_node["E"].as<double>();
//         const double mat_nu = material_yaml_node["nu"].as<double>();
//         ElasticMaterial mat(mat_name, mat_density, mat_E, mat_nu);

//         _material = mat;
//     }

//     // read in filename and load from file if specified
//     YAML::Node filename_yaml_node = config["filename"];
//     if (filename_yaml_node.Type() != YAML::NodeType::Null)
//     {
//         _loadMeshFromFile(filename_yaml_node.as<std::string>());
//     }

//     // read in starting position
//     YAML::Node position_yaml_node = config["position"];
//     if (position_yaml_node.Type() != YAML::NodeType::Null)
//     {
//         Eigen::Vector3d position {position_yaml_node[0].as<double>(), position_yaml_node[1].as<double>(), position_yaml_node[2].as<double>()};
//         moveTo(position, PositionReference::CENTER);
//     }

//     // read in starting max size
//     YAML::Node max_size_yaml_node = config["max-size"];
//     if (max_size_yaml_node.Type() != YAML::NodeType::Null)
//     {
//         resize(max_size_yaml_node.as<double>());
//     }

//     // read in starting velocity
//     YAML::Node velocity_yaml_node = config["velocity"];
//     if (velocity_yaml_node.Type() != YAML::NodeType::Null)
//     {
//         Eigen::Vector3d velocity {velocity_yaml_node[0].as<double>(), velocity_yaml_node[1].as<double>(), velocity_yaml_node[2].as<double>()};
//         _v.rowwise() = velocity.transpose(); 
//     }

//     updateVertexCache();
// }
ElasticMeshObject::ElasticMeshObject(const ElasticMeshObjectConfig* config)
    : MeshObject(config), _material(config->materialConfig())
{
    std::string filename = config->filename().value_or("");
    _loadMeshFromFile(filename);

    // order matters here...
    // first apply scaling before rotating - either through the max-size criteria or a user-specified size
    if (config->maxSize().has_value())
    {
        resize(config->maxSize().value());
    }

    if (config->size().has_value())
    {
        resize(config->size().value());
    }

    // then do rigid transformation - rotation and translation
    if (config->initialRotation().has_value())
    {
        rotate(config->initialRotation().value());
    }

    if (config->initialPosition().has_value())
    {
        moveTo(config->initialPosition().value(), PositionReference::CENTER);
    }

    
    // set initial velocity if specified in config
    if (config->initialVelocity().has_value())
    {
        _v.rowwise() = config->initialVelocity().value().transpose();
    }

    _x_prev = _vertices;

    // calculate volumes for each element and
    // calculate masses for each vertex
    _m = Eigen::VectorXd::Zero(numVertices());
    _v_volume = Eigen::VectorXd::Zero(numVertices());
    for (int i = 0; i < numElements(); i++)
    {
        // compute Q for each element
        const Eigen::Vector3d& X1 = getVertex(_elements(i,0));
        const Eigen::Vector3d& X2 = _vertices.row(_elements(i,1));
        const Eigen::Vector3d& X3 = _vertices.row(_elements(i,2));
        const Eigen::Vector3d& X4 = _vertices.row(_elements(i,3));

        Eigen::Matrix3d X;
        X.col(0) = (X1 - X4);
        X.col(1) = (X2 - X4);
        X.col(2) = (X3 - X4);

        // add it to overall vector of Q's
        // _Q.at(i) = X.inverse();

        // compute volume from X
        double vol = std::abs(X.determinant()/6);
        // _vols(i) = vol;

        // compute mass of element
        double m_element = vol * _material.density();
        // add mass contribution of element to each of its vertices
        _m(_elements(i,0)) += m_element/4;
        _m(_elements(i,1)) += m_element/4;
        _m(_elements(i,2)) += m_element/4;
        _m(_elements(i,3)) += m_element/4;

        // add volume contribution of element to each of its vertices
        _v_volume(_elements(i,0)) += vol/4;
        _v_volume(_elements(i,1)) += vol/4;
        _v_volume(_elements(i,2)) += vol/4;
        _v_volume(_elements(i,3)) += vol/4;
    }

    _v_attached_elements = Eigen::VectorXd::Zero(_vertices.rows());
    for (int i = 0; i < _elements.rows(); i++)
    {
        _v_attached_elements(_elements(i,0))++;
        _v_attached_elements(_elements(i,1))++;
        _v_attached_elements(_elements(i,2))++;
        _v_attached_elements(_elements(i,3))++;
    }

    _inv_m = 1.0/_m.array();

}

ElasticMeshObject::ElasticMeshObject(const std::string& name, const std::string& filename, const ElasticMaterial& material = ElasticMaterial::RUBBER())
    : MeshObject(name), _material(material)
{
    _loadMeshFromFile(filename);
    _v = VerticesMat::Zero(_vertices.rows(), 3);
    _x_prev = _vertices;
}

ElasticMeshObject::ElasticMeshObject(const std::string& name, const VerticesMat& verts, const ElementsMat& elems, const ElasticMaterial& material = ElasticMaterial::RUBBER())
    : MeshObject(name), _elements(elems), _material(material)
{

    // TODO: should we somehow do this before calling MeshObject base constructor?

    setVertices(verts);
    _setFacesFromElements();

    _v = VerticesMat::Zero(_vertices.rows(), 3);
    _x_prev = _vertices;

}

std::string ElasticMeshObject::toString() const
{
    return MeshObject::toString() + "\n\tNum elements: " + std::to_string(_elements.rows()) +
        "\n" + _material.toString(); 
}

void ElasticMeshObject::setVertices(const VerticesMat& verts)
{
    MeshObject::setVertices(verts);
    _fixed_vertices = Eigen::Vector<bool, -1>::Zero(_vertices.rows());
}

double* ElasticMeshObject::getVertexPreviousPositionPointer(const unsigned index) const
{
    assert(index < _x_prev.rows());
    double* p = const_cast<double*>(_x_prev.row(index).data());
    return p;
}


void ElasticMeshObject::fixVertex(unsigned vertex)
{
    _fixed_vertices(vertex) = true;
}

void ElasticMeshObject::fixVerticesWithMinY()
{
    Eigen::Vector3d min_coeff = _vertices.colwise().minCoeff();
    Eigen::Vector3d max_coeff = _vertices.colwise().maxCoeff();
    for (int i = 0; i < _vertices.rows(); i++)
    {
        if (_vertices(i,1) == min_coeff(1))
        {
            _fixed_vertices(i) = true;
        }
    }
}

void ElasticMeshObject::fixVerticesWithMinZ()
{
    Eigen::Vector3d min_coeff = _vertices.colwise().minCoeff();
    Eigen::Vector3d max_coeff = _vertices.colwise().maxCoeff();
    for (int i = 0; i < _vertices.rows(); i++)
    {
        if (_vertices(i,2) == min_coeff(2))
        {
            _fixed_vertices(i) = true;
        }
    }
}

void ElasticMeshObject::addVertexDriver(const std::shared_ptr<VertexDriver>& vd)
{
    _vertex_drivers.push_back(vd);
}

void ElasticMeshObject::removeVertexDriver(const unsigned vertex)
{
    // just do a brute force search
    _vertex_drivers.erase(std::remove_if(_vertex_drivers.begin(), _vertex_drivers.end(), [=](const std::shared_ptr<VertexDriver>& vd){ return vd->vertexIndex() == vertex; }), _vertex_drivers.end());
}

void ElasticMeshObject::clearVertexDrivers()
{
    _vertex_drivers.clear();
}

void ElasticMeshObject::stretch(const double x_stretch, const double y_stretch, const double z_stretch)
{
    const Eigen::Vector3d& min_coords = bboxMinCoords();
    const Eigen::Vector3d& max_coords = bboxMaxCoords(); 
    resize((max_coords(0)-min_coords(0))*x_stretch, (max_coords(1)-min_coords(1))*y_stretch, (max_coords(2)-min_coords(2))*z_stretch);
    _x_prev = _vertices;
}

void ElasticMeshObject::collapseToMinZ()
{
    const double min_z = bboxMinCoords()(2);
    for (int i = 0; i < _vertices.rows(); i++)
    {
        _vertices(i,2) = min_z;
    }

    _x_prev = _vertices;
}

double ElasticMeshObject::smallestEdgeLength()
{
    double min_length = std::numeric_limits<double>::max();
    for (const auto& elem : _elements.rowwise())
    {
        const Eigen::Vector3d& v1 = _vertices.row(elem(0));
        const Eigen::Vector3d& v2 = _vertices.row(elem(1));
        const Eigen::Vector3d& v3 = _vertices.row(elem(2));
        const Eigen::Vector3d& v4 = _vertices.row(elem(3));

        double v1v2 = (v1-v2).norm();
        double v1v3 = (v1-v3).norm();
        double v1v4 = (v1-v4).norm();
        double v2v3 = (v2-v3).norm();
        double v2v4 = (v2-v4).norm();
        double v3v4 = (v3-v4).norm();

        // double v1v2 = (v1-v2).cwiseAbs().minCoeff();
        // double v1v3 = (v1-v3).cwiseAbs().minCoeff();
        // double v1v4 = (v1-v4).cwiseAbs().minCoeff();
        // double v2v3 = (v2-v3).cwiseAbs().minCoeff();
        // double v2v4 = (v2-v4).cwiseAbs().minCoeff();
        // double v3v4 = (v3-v4).cwiseAbs().minCoeff();

        if (v1v2 < min_length) min_length = v1v2;
        if (v1v3 < min_length) min_length = v1v3;
        if (v1v4 < min_length) min_length = v1v4;
        if (v2v3 < min_length) min_length = v2v3;
        if (v2v4 < min_length) min_length = v2v4;
        if (v3v4 < min_length) min_length = v3v4;
    }

    return min_length;
}

double ElasticMeshObject::averageEdgeLength()
{
    std::set<std::pair<unsigned, unsigned>> edges;

    auto make_edge = [] (unsigned v1, unsigned v2)
    {
        if (v1 > v2)
            return std::pair<unsigned, unsigned>(v2, v1);
        else 
            return std::pair<unsigned, unsigned>(v1, v2);
    };

    double total_length = 0;
    for (const auto& elem : _elements.rowwise())
    {
        const Eigen::Vector3d& v1 = _vertices.row(elem(0));
        const Eigen::Vector3d& v2 = _vertices.row(elem(1));
        const Eigen::Vector3d& v3 = _vertices.row(elem(2));
        const Eigen::Vector3d& v4 = _vertices.row(elem(3));

        std::pair<unsigned, unsigned> e1 = make_edge(elem(0), elem(1));
        std::pair<unsigned, unsigned> e2 = make_edge(elem(0), elem(2));
        std::pair<unsigned, unsigned> e3 = make_edge(elem(0), elem(3));
        std::pair<unsigned, unsigned> e4 = make_edge(elem(1), elem(2));
        std::pair<unsigned, unsigned> e5 = make_edge(elem(1), elem(3));
        std::pair<unsigned, unsigned> e6 = make_edge(elem(2), elem(3));
        

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

    return total_length/edges.size();
}

double ElasticMeshObject::smallestVolume()
{
    double min_vol = std::numeric_limits<double>::max();
    for (unsigned i = 0; i < _elements.rows(); i++)
    {
        // compute Q for each element
        const Eigen::Vector3d& X1 = _vertices.row(_elements(i,0));
        const Eigen::Vector3d& X2 = _vertices.row(_elements(i,1));
        const Eigen::Vector3d& X3 = _vertices.row(_elements(i,2));
        const Eigen::Vector3d& X4 = _vertices.row(_elements(i,3));

        Eigen::Matrix3d X;
        X.col(0) = (X1 - X4);
        X.col(1) = (X2 - X4);
        X.col(2) = (X3 - X4);

        // compute volume from X
        double vol = std::abs(X.determinant()/6);
        
        if (vol < min_vol) 
            min_vol = vol;
        
    }

    return min_vol;
}

void ElasticMeshObject::_loadMeshFromFile(const std::string& filename)
{
    std::filesystem::path path(filename);
    // get the extension from the filename to know what type of file it is
    std::string input_file_extension = path.extension();

    // the filenames of the .stl and .msh files that will be generated
    const std::string& stl_filename = path.replace_extension(".stl").string();
    const std::string& msh_filename = path.replace_extension(".msh").string();

    // if it is not a .stl and not a .msh, first convert to .stl and then from .stl to .msh
    if (input_file_extension != ".stl" && input_file_extension != ".msh")
    {
        MeshUtils::convertToSTL(filename);
        MeshUtils::convertSTLtoMSH(stl_filename);
    }
    // if the input file is a .stl, use gmsh to convert to .msh
    else if (input_file_extension == ".stl")
    {
        MeshUtils::convertSTLtoMSH(filename);
    }

    // load the vertices and elements from .msh file
    VerticesMat loaded_verts;
    ElementsMat loaded_elems;
    FacesMat loaded_surface_faces;
    MeshUtils::loadMeshDataFromGmshFile(msh_filename, loaded_verts, loaded_surface_faces, loaded_elems);
    
    std::cout << "Number of loaded vertices: " << loaded_verts.rows() << std::endl;
    std::cout << "Number of loaded surface faces: " << loaded_surface_faces.rows() << std::endl;
    std::cout << "Number of loaded elements: " << loaded_elems.rows() << std::endl;

    std::cout << "Created a new mesh object from filename " << filename << "!" << std::endl;

    // set the new vertices and elements
    setVertices(loaded_verts);
    setElements(loaded_elems);
    setFaces(loaded_surface_faces);

    // set the velocities to all be 0, AFTER vertices have been loaded
    _v = VerticesMat::Zero(_vertices.rows(), 3);
}

void ElasticMeshObject::_setFacesFromElements()
{
    // create a new matrix for the faces
    // each element has 4 faces
    FacesMat faces_from_elems(_elements.rows()*4, 3);

    for (int i = 0; i < _elements.rows(); i++)
    {
        
        // TODO: don't add duplicate faces

        // add the four faces to the faces matrix for this element
        faces_from_elems(i*4,0) = _elements(i,0);
        faces_from_elems(i*4,1) = _elements(i,1);
        faces_from_elems(i*4,2) = _elements(i,3);

        faces_from_elems(i*4+1,0) = _elements(i,1);
        faces_from_elems(i*4+1,1) = _elements(i,2);
        faces_from_elems(i*4+1,2) = _elements(i,3);

        faces_from_elems(i*4+2,0) = _elements(i,0);
        faces_from_elems(i*4+2,1) = _elements(i,2);
        faces_from_elems(i*4+2,2) = _elements(i,1);

        faces_from_elems(i*4+3,0) = _elements(i,0);
        faces_from_elems(i*4+3,1) = _elements(i,3);
        faces_from_elems(i*4+3,2) = _elements(i,2);
    }
    setFaces(faces_from_elems);
}

void ElasticMeshObject::setElements(const ElementsMat& elems)
{
    _elements = elems;
    // set the faces from the new matrix of elements
    // _setFacesFromElements();

}