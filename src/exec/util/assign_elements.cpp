#include "geometry/Mesh.hpp"
#include "geometry/embree/EmbreeScene.hpp"

#include "utils/MeshUtils.hpp"

#include <vector>
#include <set>
#include <map>
#include <fstream>
#include <iostream>

/** A simple utility for generating an "element classes" file, which is simply a vector of integers.
 * The i'th integer in the vector corresponds to the class of the i'th element.
 * 
 * Classes are determined by finding overlapping elements with "submeshes".
 * E.g. 
 *   If we have a trachea mesh with a tumor, and we want to distinguish between the trachea and the tumor, we can give this script
 *   the tumor as a separate mesh, and the script will find which elements in the combined mesh overlap with those in the separate tumor mesh,
 *   and generate a file with 0s for element indices corresponding to the trachea, and 1s for element indices corresponding to the tumor.
 * 
 * Usage:
 * 
 * ./AssignElements <combined-mesh-filename> <separate-mesh-1> <class-1> <separate-mesh-2> <class-2> ...
 * 
 * where 
 *  - <combined-mesh-filename> is the path to the "combined" mesh used in the simulation that will have different material properties
 *  - <separate-mesh-i> is the i'th separate mesh (usually generated from CT segmentations)
 *  - <class-i> is an integer corresponding to the class to be associated with elements that overlap with separate-mesh-i.
 */

int main(int argc, char* argv[])
{
    // extract command line arguments
    bool invalid_args = false;
    if (argc%2 != 0 || argc < 4)
        invalid_args = true;

    if (invalid_args)
    {
        std::cout << "INVALID ARGUMENTS!\nUsage:\n" << "  ./AssignElements <combined-mesh-filename> <separate-mesh-1> <class-1> <separate-mesh-2> <class-2> ..." << std::endl;
        return EXIT_FAILURE;
    }

    std::string combined_mesh_filename = argv[1];
    std::map<std::string, int> separate_meshes;
    try
    {
        for (int i = 2; i < argc; i+=2)
        {
            std::string filename = argv[i];
            int cl = atoi(argv[i+1]);
            separate_meshes[filename] = cl;
        }
    }
    catch(const std::exception& e)
    {
        invalid_args = true;
    }
    
    

    if (invalid_args)
    {
        std::cout << "INVALID ARGUMENTS!\nUsage:\n" << "./AssignElements <combined-mesh-filename> <separate-mesh-1> <class-1> <separate-mesh-2> <class-2> ..." << std::endl;
        return EXIT_FAILURE;
    }
    

    gmsh::initialize();

    // std::string combined_mesh_filename = "../resource/demos/trachea_virtuoso/cao_04_29_25_model1_decimated3_r.msh";
    // std::vector<std::string> class_mesh_filenames = {
    //     "../resource/demos/trachea_virtuoso/cao_04_29_25_model1_tumor.msh",
    //     "../resource/demos/trachea_virtuoso/cao_04_29_25_model1_trachea.msh"
    // };

    // create Embree scene
    Geometry::EmbreeScene embree_scene;

    Geometry::TetMesh combined_mesh = MeshUtils::loadTetMeshFromGmshFile(combined_mesh_filename);
    Vec3r combined_mesh_cm = combined_mesh.massCenter();

    // load combined mesh from file
    Config::MeshObjectConfig config(combined_mesh_filename, std::nullopt, std::nullopt, false, false, false, Vec4r(0,0,0,0));
    Config::ObjectConfig obj_config("combined", combined_mesh_cm, Vec3r::Zero(), Vec3r::Zero(), false, false, Config::ObjectRenderConfig());

    
    Sim::TetMeshObject combined_mesh_obj(&config, &obj_config);
    combined_mesh_obj.loadAndConfigureMesh();
    std::cout << "bounding box: " << combined_mesh_obj.mesh()->boundingBox().min.transpose() << " to " << combined_mesh_obj.mesh()->boundingBox().max.transpose() << std::endl;

    // add the combined mesh to the Embree scene
    // this will create a scene for the tetrahedra that we can do point queries on
    embree_scene.addObject(&combined_mesh_obj);

    std::vector<int> elem_classes(combined_mesh.numElements(), 0);
    for (const auto& [mesh_filename, class_int] : separate_meshes)
    // for (unsigned i = 0; i < class_mesh_filenames.size(); i++)
    {
        // load the class mesh from file - this will convert any .obj or .stl to .msh
        Geometry::TetMesh class_mesh = MeshUtils::loadTetMeshFromGmshFile(mesh_filename);

        std::cout << "bounding box: " << class_mesh.boundingBox().min.transpose() << " to " << class_mesh.boundingBox().max.transpose() << std::endl;

        // go through each element and query each of its points
        for (int e = 0; e < class_mesh.numElements(); e++)
        {
            const Eigen::Vector4i& element = class_mesh.element(e);

            for (int vi = 0; vi < 4; vi++)
            {
                const Vec3r& v = class_mesh.vertex(element[vi]);
                // perform the query
                std::set<Geometry::EmbreeHit> res = embree_scene.pointInTetrahedraQuery(v, &combined_mesh_obj);
                if (!res.empty())
                {
                    // if there's a hit, label the element that was hit with the current class
                    int elem_index = res.begin()->prim_index;
                    if (elem_classes[elem_index] == 0)
                        elem_classes[elem_index] = class_int;
                    break;
                }
            }
        }
    }

    // int num0=0, num1=0, num2=0;
    // for (int e = 0; e < combined_mesh.numElements(); e++)
    // {
    //     if (elem_classes[e] == 0)   num0++;
    //     if (elem_classes[e] == 1)   num1++;
    //     if (elem_classes[e] == 2)   num2++;
    // }

    // std::cout << "Num class 0: " << num0 << std::endl;
    // std::cout << "Num class 1: " << num1 << std::endl;
    // std::cout << "Num class 2: " << num2 << std::endl;

    std::string out_filename = combined_mesh_filename.substr(0,combined_mesh_filename.length()-4) + "_element_classes.txt";
    std::ofstream out(out_filename);
    for (const auto& elem_class : elem_classes)
    {
        out << elem_class << "\n";
    }
    out.close();
}