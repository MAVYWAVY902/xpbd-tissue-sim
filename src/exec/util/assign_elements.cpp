#include "geometry/Mesh.hpp"
#include "geometry/embree/EmbreeScene.hpp"

#include "utils/MeshUtils.hpp"

#include <vector>
#include <set>

int main()
{
    gmsh::initialize();

    std::string combined_mesh_filename = "../resource/demos/trachea_virtuoso/cao_04_29_25_model1_decimated3_r.msh";
    std::vector<std::string> class_mesh_filenames = {
        "../resource/demos/trachea_virtuoso/cao_04_29_25_model1_tumor.msh",
        "../resource/demos/trachea_virtuoso/cao_04_29_25_model1_trachea.msh"
    };

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
    for (unsigned i = 0; i < class_mesh_filenames.size(); i++)
    {
        // load the class mesh from file - this will convert any .obj or .stl to .msh
        Geometry::TetMesh class_mesh = MeshUtils::loadTetMeshFromGmshFile(class_mesh_filenames[i]);

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
                        elem_classes[elem_index] = i+1;
                    break;
                }
            }
        }
    }

    int num0=0, num1=0, num2=0;
    for (int e = 0; e < combined_mesh.numElements(); e++)
    {
        if (elem_classes[e] == 0)   num0++;
        if (elem_classes[e] == 1)   num1++;
        if (elem_classes[e] == 2)   num2++;
    }

    std::cout << "Num class 0: " << num0 << std::endl;
    std::cout << "Num class 1: " << num1 << std::endl;
    std::cout << "Num class 2: " << num2 << std::endl;
}