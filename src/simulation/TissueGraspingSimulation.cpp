#include "simulation/TissueGraspingSimulation.hpp"
#include "utils/MeshUtils.hpp"

#include "solver/xpbd_solver/XPBDSolver.hpp"

namespace Sim
{

TissueGraspingSimulation::TissueGraspingSimulation(const std::string& config_filename)
    : Simulation()
{
    // create a more specialized config object specifically for BeamStretchSimulations
    _config = std::make_unique<TissueGraspingSimulationConfig>(YAML::LoadFile(config_filename));

    // initialize quantities using config object
    _init();

    _last_mesh_write_time = 10000;
    _write_mesh = false;

    // extract the stretch velocity and time from the config object
    TissueGraspingSimulationConfig* tissue_grasping_simulation_config = dynamic_cast<TissueGraspingSimulationConfig*>(_config.get());
    _grasp_size = tissue_grasping_simulation_config->graspSize().value();
    _z_scaling = tissue_grasping_simulation_config->zScaling().value();
    _input_device = tissue_grasping_simulation_config->inputDevice().value();
    _fixed_faces_filename = tissue_grasping_simulation_config->fixedFacesFilename();

    _grasp_tip_rotation = Mat3r::Identity();

    if (_input_device == SimulationInputDevice::HAPTIC)
    {
        std::cout << BOLD << "Initializing haptic device..." << RST << std::endl;
        _haptic_device_manager = std::make_unique<HapticDeviceManager>();
    }

    // _out_file << "Tissue Grasping Simulation" << std::endl;
}

void TissueGraspingSimulation::setup()
{
    _write_mesh = true;
    _last_mesh_write_time = 0;

    Simulation::setup();

    // _viewer->enableMouseInteraction(false);
    
    // _grasp_tip = std::make_shared<RigidMeshObject>("grasp_tip", "../resource/general/Sphere_Cursor.stl");
    // _grasp_tip->moveTo(Vec3r(0,0,0));
    // _grasp_tip->resize(_grasp_size);
    // addObject(_grasp_tip);

    // _out_file << toString() << std::endl;

    for (auto& obj : _objects)
    {
        if (XPBDMeshObject_Base* xpbd_obj = dynamic_cast<XPBDMeshObject_Base*>(obj.get()))
        {
            _tissue_obj = xpbd_obj;
            break;
        }
    }
    
    assert(_tissue_obj);

    // if (_fixed_faces_filename.has_value())
    // {
    //     std::set<unsigned> vertices = MeshUtils::verticesFromFixedFacesFile(_fixed_faces_filename.value());
    //     for (const auto& v : vertices)
    //     {
    //         _tissue_obj->fixVertex(v);
    //     }
    // }

    /** Hack! Hard-coded. Uncomment for tissue pull. */
    // fix left-most third of bottom face
    // Real smallest_edge_length = _tissue_obj->smallestEdgeLength();
    // const Vec3r& min_coords = _tissue_obj->bboxMinCoords();
    // const Vec3r& max_coords = _tissue_obj->bboxMaxCoords();
    // int x_steps = static_cast<int>((max_coords(0) - min_coords(0)) / smallest_edge_length) + 1;
    // int y_steps = static_cast<int>((max_coords(1) - min_coords(1)) / smallest_edge_length) + 1;

    // // top and bottom surfaces (XY plane)
    // for (int xi = 0; xi < x_steps; xi++)
    // {
    //     for (int yi = 0; yi < y_steps/3; yi++)
    //     {
    //         Real x = xi*smallest_edge_length + min_coords(0);
    //         Real y = yi*smallest_edge_length + min_coords(1);
    //         unsigned v1 = _tissue_obj->getClosestSurfaceVertex(x, y, min_coords(2));
    //         _tissue_obj->fixVertex(v1);
    //     }
    // }
    



    // _out_file << _tissue_obj->toString(0) << std::endl;
    
}

void TissueGraspingSimulation::_updateGraphics()
{   
    // if (_input_device == SimulationInputDevice::HAPTIC)
    // {
    //     if (!_grasping && _haptic_device_manager->button1Pressed())
    //         _toggleTissueGrasping();
    //     else if (_grasping && !_haptic_device_manager->button1Pressed())
    //         _toggleTissueGrasping();

    //     const Vec3r haptic_position = _haptic_device_manager->position()/500;
        
    //     const Vec3r old_tip_position = _grasp_tip_position;
    //     _grasp_tip_position = _transformInputPosition(haptic_position);

    //     // std::cout << "Position: " << _grasp_tip_position(0) << ", " << _grasp_tip_position(1) << ", " << _grasp_tip_position(2) << std::endl;
        
    //     Mat3r camera_rotation;
    //     // easy3d::vec3 view_dir_easy3d = _viewer->camera()->viewDirection();
    //     // easy3d::vec3 up_vec_easy3d = _viewer->camera()->upVector();
    //     // easy3d::vec3 right_vec_easy3d = -_viewer->camera()->rightVector();
    //     // Vec3r view_dir(view_dir_easy3d[0], view_dir_easy3d[1], view_dir_easy3d[2]);
    //     // Vec3r up_vec(up_vec_easy3d[0], up_vec_easy3d[1], up_vec_easy3d[2]);
    //     // Vec3r right_vec(right_vec_easy3d[0], right_vec_easy3d[1], right_vec_easy3d[2]);
    //     Vec3r view_dir = _graphics_scene->cameraViewDirection();
    //     Vec3r up_vec = _graphics_scene->cameraUpDirection();
    //     Vec3r right_vec = _graphics_scene->cameraRightDirection(); 
        
    //     camera_rotation.row(0) = right_vec;
    //     camera_rotation.row(1) = up_vec;
    //     camera_rotation.row(2) = view_dir;

    //     const Mat3r haptic_orientation = camera_rotation*_haptic_device_manager->orientation();

    //     _grasp_tip->moveTo(_grasp_tip_position);
    //     Mat3r rot_transpose = _grasp_tip_rotation.transpose();
    //     // _grasp_tip->rotate(rot_transpose);
    //     // _grasp_tip->rotate(haptic_orientation);

    //     _grasp_tip_rotation = haptic_orientation;

    //     for (const auto& vd : _grasped_vertex_drivers)
    //     {
    //         Vec3r new_position = vd->position() + (_grasp_tip_position-old_tip_position);
    //         const Real dist_from_grasp_tip = (new_position - _grasp_tip_position).norm();
    //         if (dist_from_grasp_tip > _grasp_size*0.25)
    //         {
    //             const Vec3r dir = (_grasp_tip_position - new_position).normalized();
    //             new_position += dir*(dist_from_grasp_tip - _grasp_size*0.25);
    //         }
    //         vd->setPosition(new_position);
    //     }
    // }

    Simulation::_updateGraphics();
}

void TissueGraspingSimulation::_timeStep()
{
    if (_time - _last_mesh_write_time >= 3.33e-2 && _write_mesh)
    {
        // std::ofstream obj_file("../output/mesh_output/mesh_" + std::to_string(_time) + ".obj");
        // if (obj_file.is_open())
        // {
        //     MeshObject::VerticesMat verts = _tissue_obj->vertices();
        //     for (const auto& v : verts.rowwise())
        //     {
        //         obj_file << "v " << v << std::endl;
        //     }
            
        //     MeshObject::FacesMat faces = _tissue_obj->faces();
        //     for (const auto& f : faces.rowwise())
        //     {
        //         obj_file << "f " << f(0)+1 << " " << f(1)+1 << " " << f(2)+1 << std::endl;
        //     }
        // }

        _last_mesh_write_time = _time;
    }

    Simulation::_timeStep();
}

void TissueGraspingSimulation::notifyMouseButtonPressed(int button, int action, int modifiers)
{
    if (_input_device != SimulationInputDevice::MOUSE)
    {
        return;
    }

    // button = 0 ==> left mouse button
    // button = 1 ==> right mouse button
    // action = 0 ==> mouse up
    // action = 1 ==> mouse down
    
    if (button == 0 && action == 1)
    {
        // _toggleTissueGrasping();
    }

    Simulation::notifyMouseButtonPressed(button, action, modifiers);
}

void TissueGraspingSimulation::notifyMouseMoved(Real x, Real y)
{
    
    if (_input_device != SimulationInputDevice::MOUSE)
    {
        return;
    }

    bool found;
    // easy3d::vec3 mouse_pt = _viewer->point_under_pixel(x, y, found);
    Vec3r new_mouse_pos(0,0,0);
    // new_mouse_pos(0) = mouse_pt.x;
    // new_mouse_pos(1) = mouse_pt.y;
    // new_mouse_pos(2) = mouse_pt.z;

    // for (const auto& vd : _grasped_vertex_drivers)
    // {
    //     Vec3r new_position;
    //     new_position(0) = vd->position()(0);
    //     new_position(1) = vd->position()(1);
    //     new_position(2) = vd->position()(2) - _z_scaling*(y - _mouse_pos_2d(1));
    //     vd->setPosition(new_position);
    // }
    // for (const auto& vd : _grasped_vertex_drivers)
    // {
    //     vd->setPosition(new_mouse_pos);
    // }

    _mouse_pos_3d = new_mouse_pos;

    _mouse_pos_2d(0) = x;
    _mouse_pos_2d(1) = y;

    Simulation::notifyMouseMoved(x, y);
}   

void TissueGraspingSimulation::notifyKeyPressed(int key, int action, int modifiers)
{
    if (action > 0)
    {
        /** Prostate */
        // _viewer->camera()->setViewDirection(easy3d::vec3(0.0356192, 0.819275, 0.572293));
        // _viewer->camera()->setPosition(easy3d::vec3(0.00371679, -0.163586, 0.261103));
    
        /** Trachea */
        // _viewer->camera()->setPosition(easy3d::vec3(-0.00324725, -0.0680968, 1.00019));
        // easy3d::vec3 view_dir(0.105808, 0.990158, -0.0916088);
        // easy3d::vec3 pos(-0.00324725, -0.0680968, 1.00019);
        // _viewer->camera()->setPosition(pos-0*view_dir);
        // _viewer->camera()->setViewDirection(easy3d::vec3(0.105808, 0.990158, -0.0916088));
        // if (_graphics_scene)
        // {
        //     Vec3r position(-0.00324725, -0.0680968, 1.00019);
        //     Vec3r view_dir(0.105808, 0.990158, -0.0916088);
        //     _graphics_scene->setCameraPosition(position);
        //     _graphics_scene->setCameraViewDirection(view_dir);
        // }


        /** Tissue block */
        // _viewer->camera()->setPosition(easy3d::vec3(0.660399, 0.0116081, 1.64928));
        // _viewer->camera()->setViewDirection(easy3d::vec3(-0.713113, -0.0129693, -0.70093));

        // save tissue vertices, faces, elements
        std::ofstream vertices_file("vertices_" + std::to_string(_time) + ".txt");
        if (vertices_file.is_open())
        {
            vertices_file << _tissue_obj->mesh()->vertices() << std::endl;
        }

        std::ofstream faces_file("faces_" + std::to_string(_time) + ".txt");
        if (faces_file.is_open())
        {
            faces_file << _tissue_obj->mesh()->faces() << std::endl;
        }

        std::ofstream elements_file("elements_" + std::to_string(_time) + ".txt");
        if (elements_file.is_open())
        {
            elements_file << _tissue_obj->mesh()->faces() << std::endl;
        }

        
    }

    Simulation::notifyKeyPressed(key, action, modifiers);
    
}

void TissueGraspingSimulation::_toggleTissueGrasping()
{
    // if (!_write_mesh)
    // {
    //     _write_mesh = true;
    //     _last_mesh_write_time = ((static_cast<int>(_time*10000 + 333 - 1) / 333) * 333) / 10000.0;
    // }
    // if (_grasping)
    // {
    //     for (const auto& vd : _grasped_vertex_drivers)
    //     {
    //         _tissue_obj->removeVertexDriver(vd->vertexIndex());
    //     }
    //     _grasped_vertex_drivers.clear();

    //     _grasping = false;
    // }
    // else
    // {
    //     std::set<unsigned> vertices_to_grasp;// = _getAllVerticesInGraspingArea();
    //     if (_input_device == SimulationInputDevice::MOUSE)
    //     {
    //         unsigned v = _tissue_obj->getClosestSurfaceVertex(_mouse_pos_3d(0), _mouse_pos_3d(1), _mouse_pos_3d(2));
    //         if (!_tissue_obj->vertexFixed(v))
    //             vertices_to_grasp.insert(v);
    //     }
    //     if (_input_device == SimulationInputDevice::HAPTIC)
    //     {
    //         // unsigned v = _tissue_obj->getClosestSurfaceVertex(_grasp_tip_position(0), _grasp_tip_position(1), _grasp_tip_position(2));
            
    //         for (int theta = 0; theta < 360; theta+=30)
    //         {
    //             for (int phi = 0; phi < 360; phi+=30)
    //             {
    //                 for (Real p = 0; p < _grasp_size; p+=_grasp_size/5.0)
    //                 {
    //                     const Real x = _grasp_tip_position(0) + p*std::sin(phi*M_PI/180)*std::cos(theta*M_PI/180);
    //                     const Real y = _grasp_tip_position(1) + p*std::sin(phi*M_PI/180)*std::sin(theta*M_PI/180);
    //                     const Real z = _grasp_tip_position(2) + p*std::cos(phi*M_PI/180);
    //                     unsigned v = _tissue_obj->getClosestVertex(x, y, z);

    //                     // make sure v is inside sphere
    //                     if ((_grasp_tip_position - _tissue_obj->getVertex(v)).norm() <= _grasp_size)
    //                         if (!_tissue_obj->vertexFixed(v))
    //                             vertices_to_grasp.insert(v);
    //                 }
    //             }
    //         }

            
    //     }
        

    //     for (const auto& v : vertices_to_grasp)
    //     {
    //         // grab middle vertex and lift it up
            
    //         Vec3r vertex_pos = _tissue_obj->getVertex(v);


    //         std::shared_ptr<StaticVertexDriver> vd = std::make_shared<StaticVertexDriver>("tissue grasping", v, vertex_pos);
    //         _tissue_obj->addVertexDriver(vd);

    //         _grasped_vertex_drivers.push_back(vd);
    //     }
        

    //     _grasping = true;
    // }
}

// std::set<unsigned> TissueGraspingSimulation::_getAllVerticesInGraspingArea()
// {
//     std::set<unsigned> vertices_to_grasp;
//     const Real step = 1e-3; // some small value to capture all vertices on the top plane inside the area
//     // const Real grab_size = 0.05;

//     const int num_steps = _grasp_size / step;

//     const Vec3r& min_coords = _tissue_obj->bboxMinCoords();
//     const Vec3r& max_coords = _tissue_obj->bboxMaxCoords();
//     Vec3r vertex_grab_pos({min_coords(0) + (max_coords(0)-min_coords(0))*0.5, 
//                                     min_coords(1) + (max_coords(1)-min_coords(1))*0.5,
//                                     max_coords(2)});
    
//     for (int i = 0; i <= num_steps; i++)
//     {
//         for (int j = 0; j <= num_steps; j++)
//         {
//             // grab middle vertex and lift it up
            
//             unsigned vertex_ind = _tissue_obj->getClosestVertex(vertex_grab_pos(0) - _grasp_size/2 + i*step, vertex_grab_pos(1) - _grasp_size/2 + j*step, vertex_grab_pos(2));

//             vertices_to_grasp.insert(vertex_ind);
//         }
//     }

//     return vertices_to_grasp;
// }

Vec3r TissueGraspingSimulation::_transformInputPosition(const Vec3r& input_position)
{
    Vec4r input_position_h(input_position(0), input_position(1), input_position(2), 1);

    Mat4r T_OC = Mat4r::Identity();
    T_OC(0,0) = -1;
    T_OC(2,2) = -1;
    T_OC(2,3) = 0.2; // distance

    Vec3r view_dir_easy3d = _graphics_scene->cameraViewDirection();
    Vec3r up_vec_easy3d = _graphics_scene->cameraUpDirection();
    Vec3r right_vec_easy3d = -_graphics_scene->cameraRightDirection();
    Vec3r camera_position_easy3d = _graphics_scene->cameraPosition();
    Vec4r view_dir(view_dir_easy3d[0], view_dir_easy3d[1], view_dir_easy3d[2], 0);
    Vec4r up_vec(up_vec_easy3d[0], up_vec_easy3d[1], up_vec_easy3d[2], 0);
    Vec4r right_vec(right_vec_easy3d[0], right_vec_easy3d[1], right_vec_easy3d[2], 0);
    Vec4r camera_position(camera_position_easy3d[0], camera_position_easy3d[1], camera_position_easy3d[2], 1);

    Mat4r T_CW;
    T_CW.col(0) = right_vec;
    T_CW.col(1) = up_vec;
    T_CW.col(2) = view_dir;
    T_CW.col(3) = camera_position;

    Vec4r transformed_position_h = T_CW * T_OC * input_position_h;

    return Vec3r(transformed_position_h[0], transformed_position_h[1], transformed_position_h[2]);

}

} // namespace Sim