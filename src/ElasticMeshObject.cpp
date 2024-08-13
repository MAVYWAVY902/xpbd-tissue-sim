#include "ElasticMeshObject.hpp"
#include "MeshUtils.hpp"
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

    if (config->initialPosition().has_value())
    {
        moveTo(config->initialPosition().value(), PositionReference::CENTER);
    }

    if (config->maxSize().has_value())
    {
        resize(config->maxSize().value());
    }

    if (config->size().has_value())
    {
        resize(config->size().value());
    }

    if (config->initialVelocity().has_value())
    {
        _v.rowwise() = config->initialVelocity().value().transpose();
    }

    updateVertexCache();

}

ElasticMeshObject::ElasticMeshObject(const std::string& name, const std::string& filename, const ElasticMaterial& material = ElasticMaterial::RUBBER())
    : MeshObject(name), _material(material)
{
    _loadMeshFromFile(filename);
}

ElasticMeshObject::ElasticMeshObject(const std::string& name, const VerticesMat& verts, const ElementsMat& elems, const ElasticMaterial& material = ElasticMaterial::RUBBER())
    : MeshObject(name), _elements(elems), _material(material)
{

    // TODO: should we somehow do this before calling MeshObject base constructor?

    setVertices(verts);
    _setFacesFromElements();

    _v = VerticesMat::Zero(_vertices.rows(), 3);

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
    MeshUtils::loadMeshDataFromGmshFile(msh_filename, loaded_verts, loaded_elems);
    
    std::cout << "Number of loaded vertices: " << loaded_verts.rows() << std::endl;
    std::cout << "Number of loaded elements: " << loaded_elems.rows() << std::endl;

    std::cout << "Created a new mesh object from filename " << filename << "!" << std::endl;

    // set the new vertices and elements
    setVertices(loaded_verts);
    setElements(loaded_elems);

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
    _setFacesFromElements();

}