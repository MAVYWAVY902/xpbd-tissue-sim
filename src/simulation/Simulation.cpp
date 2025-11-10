// #include "simulation/Simulation.hpp"

// #include "config/simobject/RigidMeshObjectConfig.hpp"
// #include "config/simobject/XPBDMeshObjectConfig.hpp"
// #include "config/simobject/FirstOrderXPBDMeshObjectConfig.hpp"
// #include "config/simobject/RigidPrimitiveConfigs.hpp"
// #include "config/simobject/VirtuosoArmConfig.hpp"
// #include "config/simobject/VirtuosoRobotConfig.hpp"

// #include "graphics/easy3d/Easy3DGraphicsScene.hpp"
// #include "graphics/vtk/VTKGraphicsScene.hpp"

// #include "simobject/RigidMeshObject.hpp"
// #include "simobject/XPBDMeshObject.hpp"
// #include "simobject/RigidPrimitives.hpp"
// #include "simobject/VirtuosoArm.hpp"
// #include "simobject/VirtuosoRobot.hpp"

// #include "simobject/XPBDObjectFactory.hpp"

// #include "solver/constraint/NerveStretchConstraint.hpp"
// #include "utils/MeshUtils.hpp"

// #include <gmsh.h>
// #include <chrono>
// #include <thread>
// #include <iomanip>

// namespace Sim
// {

// Simulation::Simulation(const Config::SimulationConfig* config)
//     : _setup(false), _config(config)
// {
//     // initialize gmsh
//     gmsh::initialize();

//     // set simulation properties based on YAML file
//     _name = _config->name();
//     _description = _config->description();
//     _time_step = _config->timeStep();
//     _end_time = _config->endTime();
//     _time = 0;
//     _g_accel = _config->gAccel();
//     _viewer_refresh_time = 1 / _config->fps() * 1000;
//     _time_between_collision_checks = 1.0 / _config->collisionRate();

//     // set the Simulation mode from the YAML config
//     _sim_mode = _config->simMode();

//     // initialize the graphics scene according to the type specified by the user
//     if (_config->visualization() == Config::Visualization::EASY3D)
//     {
//         _graphics_scene = std::make_unique<Graphics::Easy3DGraphicsScene>("main", config->renderConfig());
//     }

//     if (_config->visualization() == Config::Visualization::VTK)
//     {
//         _graphics_scene = std::make_unique<Graphics::VTKGraphicsScene>("main", config->renderConfig());
//     }

//     // initialize the Embree scene
//     _embree_scene = std::make_unique<Geometry::EmbreeScene>();

//     // initialize the collision scene
//     _collision_scene = std::make_unique<CollisionScene>(this, _embree_scene.get());
//     _last_collision_detection_time = 0;

//     // initialize the logger
//     if (_config->logging())
//     {
//         // get datetime string
//         auto now = std::chrono::system_clock::now();
//         auto time_t = std::chrono::system_clock::to_time_t(now);

//         std::stringstream ss;
//         ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d_%H:%M:%S") << ".txt";
//         std::string filename = ss.str();

//         std::filesystem::path output_dir(config->loggingOutputDir());
//         std::filesystem::path filepath = output_dir / filename;

//         _logger = std::make_unique<SimulationLogger>(filepath.string());
//     }

//     // create materials
//     for (const auto& mat_config : config->materialConfigs())
//     {
//         _materials.emplace_back(&mat_config);
//     }
// }

// std::string Simulation::toString(const int indent) const
// {
//     std::string indent_str(indent, '\t');
//     std::stringstream ss;
//     ss << indent_str << "=====" << type() << " '" << _name << "'=====" << std::endl;
//     ss << indent_str << "Time step: " << _time_step << " s" << std::endl;
//     ss << indent_str << "End time: " << _end_time << " s" << std::endl;
//     ss << indent_str << "Gravity: " << _g_accel << " m/s2" << std::endl;
//     return ss.str();
// }

// void Simulation::setup()
// {
//     assert(!_setup);
//     _setup = true;

//     // graphics
//     if (_graphics_scene)
//     {
//         _graphics_scene->init();
//         _graphics_scene->viewer()->registerSimulation(this);
//         _graphics_scene->viewer()->addText(
//             "time", "Sim Time: 0.000 s",
//             10.0f, 10.0f, 15.0f,
//             Graphics::Viewer::TextAlignment::LEFT,
//             Graphics::Viewer::Font::MAO,
//             std::array<float, 3>({0, 0, 0}),
//             0.5f,
//             false);

//         _graphics_scene->viewer()->enableMouseInteraction(_config->enableMouseInteraction());
//     }

//     // create objects from YAML
//     auto& object_configs = _config->objectConfigs();
//     object_configs.for_each_element([this](const auto& config)
//     {
//         this->_addObjectFromConfig(&config);
//     });

//     // ==================== TEST: add NerveStretchConstraint to first XPBD object ====================
//     {
//         auto& xpbd_objs = _objects.get<std::unique_ptr<XPBDMeshObject_Base>>();
//         if (!xpbd_objs.empty())
//         {
//             XPBDMeshObject_Base* base_ptr = xpbd_objs[0].get();

//             // 这里的几个类型名都跟 XPBDMeshObject.cpp 里的一样，只是搬过来用了
//             using ConstraintCfg = XPBDMeshObjectConstraintConfigurations<false>;
//             using SolverTypes = XPBDObjectSolverTypes<
//                 false,
//                 typename ConstraintCfg::StableNeohookean::projector_type_list
//             >;
//             using TestXPBD = XPBDMeshObject_<
//                 false,
//                 SolverTypes::GaussSeidel,
//                 typename ConstraintCfg::StableNeohookean::constraint_type_list
//             >;

//             if (auto* xpbd = dynamic_cast<TestXPBD*>(base_ptr))
//             {
//                 // 确保有足够的顶点
//                 if (xpbd->mesh()->numVertices() > 10)
//                 {
//                     const auto& V = xpbd->mesh()->vertices();
//                     const int v0 = 0;
//                     const int v1 = 10;
//                     Vec3r p0 = V.col(v0);
//                     Vec3r p1 = V.col(v1);
//                     Real rest_len = (p0 - p1).norm();

//                     xpbd->addNerveStretchConstraint(v0, v1, rest_len, /*alpha=*/0.0);

//                     std::cout << "[Simulation::setup] added NerveStretchConstraint between "
//                               << v0 << " and " << v1
//                               << ", rest_len = " << rest_len << std::endl;
//                 }
//                 else
//                 {
//                     std::cout << "[Simulation::setup] XPBD mesh has < 11 vertices, skip test nerve constraint."
//                               << std::endl;
//                 }
//             }
//             else
//             {
//                 // 说明当前场景用的不是这一条 solver 组合
//                 std::cout << "[Simulation::setup] Found an XPBD object but dynamic_cast to "
//                              "XPBDMeshObject_<false, GaussSeidel, StableNeohookean> failed. "
//                              "If your YAML uses another solver (Jacobi / ParallelJacobi / first-order), "
//                              "you need to add another cast here."
//                           << std::endl;
//             }
//         }
//     }
//     // ================== END TEST ==================

//     // logger
//     if (_logger)
//     {
//         _logger->addOutput("time [s]", &_time);
//     }
// }

// void Simulation::update()
// {
//     if (_logger)
//         _logger->startLogging();

//     auto start = std::chrono::steady_clock::now();
//     _wall_time_start = std::chrono::steady_clock::now();
//     auto last_redraw = std::chrono::steady_clock::now();

//     while (_time < _end_time)
//     {
//         Real wall_time_elapsed_s =
//             std::chrono::duration_cast<std::chrono::nanoseconds>(
//                 std::chrono::steady_clock::now() - _wall_time_start)
//                 .count() /
//             1000000000.0;

//         // callbacks
//         for (auto& cb : _callbacks)
//         {
//             if (wall_time_elapsed_s > cb.next_exec_time)
//             {
//                 cb.callback();
//                 cb.next_exec_time = cb.next_exec_time + cb.interval;
//             }
//         }

//         // real-time block
//         if (_sim_mode == Config::SimulationMode::VISUALIZATION && _time > wall_time_elapsed_s)
//         {
//             continue;
//         }

//         _timeStep();

//         auto time_since_last_redraw_ms =
//             std::chrono::duration_cast<std::chrono::milliseconds>(
//                 std::chrono::steady_clock::now() - last_redraw)
//                 .count();

//         if (time_since_last_redraw_ms > _viewer_refresh_time)
//         {
//             _updateGraphics();
//             last_redraw = std::chrono::steady_clock::now();
//         }
//     }

//     _updateGraphics();

//     auto end = std::chrono::steady_clock::now();
//     std::cout << "Simulating " << _end_time << " seconds took "
//               << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
//               << " ms" << std::endl;
// }

// void Simulation::_timeStep()
// {
//     // ① 原来就有的：按频率清一次碰撞约束
//     if (_time - _last_collision_detection_time > _time_between_collision_checks)
//     {
//         auto& xpbd_mesh_objs = _objects.get<std::unique_ptr<XPBDMeshObject_Base>>();
//         for (auto& obj : xpbd_mesh_objs)
//         {
//             obj->clearCollisionConstraints();
//         }
//         auto& fo_xpbd_mesh_objs = _objects.get<std::unique_ptr<FirstOrderXPBDMeshObject_Base>>();
//         for (auto& obj : fo_xpbd_mesh_objs)
//         {
//             obj->clearCollisionConstraints();
//         }
//         auto& virtuoso_robots = _objects.get<std::unique_ptr<VirtuosoRobot>>();
//         for (auto& obj : virtuoso_robots)
//         {
//             if (obj->hasArm1())
//                 obj->arm1()->clearCollisionConstraints();
//             if (obj->hasArm2())
//                 obj->arm2()->clearCollisionConstraints();
//         }
//         auto& virtuoso_arms = _objects.get<std::unique_ptr<VirtuosoArm>>();
//         for (auto& obj : virtuoso_arms)
//         {
//             obj->clearCollisionConstraints();
//         }

//         _collision_scene->collideObjects();
//     }

//     // ② ====== 这里是我们要加的“假神经”测试 ======
//     // 用 static 保证每一帧都能记住上一次的位置
//     static Vec3r nerveP0(0.0, 0.0, 0.0);
//     static Vec3r nerveP1(0.01, 0.0, 0.0);
//     static Real nerveInvMass0 = 1.0;
//     static Real nerveInvMass1 = 1.0;
//     static Real nerveRestLen  = (nerveP1 - nerveP0).norm();
//     static int  frame_cnt     = 0;

//     frame_cnt++;

//     // 你想偶尔“拽一下尾巴”就这样写
//     if (frame_cnt % 5 == 0) {
//         // 每 20 帧往外拉一点
//         nerveP1 += Vec3r(0.004, 0.002, 0.001);
//     }

//     // (a) 投影前打印
//     {
//         Real pre_len = (nerveP1 - nerveP0).norm();
//         std::cout << "[pre]  nerve len = " << pre_len << std::endl;
//     }

//     // (b) 做一遍“XPBD 距离约束”的最简单手写版本
//     {
//         Vec3r d   = nerveP1 - nerveP0;
//         Real len  = d.norm();
//         if (len > Real(1e-9))
//         {
//             Real C  = len - nerveRestLen;   // 想要 C=0
//             Vec3r n = d / len;
//             Real w0 = nerveInvMass0;
//             Real w1 = nerveInvMass1;
//             Real wsum = w0 + w1;
//             if (wsum > Real(1e-9))
//             {
//                 Vec3r corr = (C / wsum) * n;
//                 nerveP0 += w0 * corr;   // 往外推
//                 nerveP1 -= w1 * corr;   // 往回拉
//             }
//         }
//     }

//     // (c) 投影后打印
//     {
//         Real post_len = (nerveP1 - nerveP0).norm();
//         std::cout << "[post] nerve len = " << post_len << std::endl;
//     }
//     // ② ====== 假神经测试结束 ======


//     // ③ 原来就有的：更新所有 sim object
//     _objects.for_each_element([](auto& obj)
//     {
//         obj->update();
//     });

//     // ④ 原来就有的：速度更新
//     _objects.for_each_element([](auto& obj)
//     {
//         obj->velocityUpdate();
//     });

//     // ⑤ 原来就有的：更新上一次碰撞检测时间
//     if (_time - _last_collision_detection_time > _time_between_collision_checks)
//     {
//         _last_collision_detection_time = _time;
//     }

//     // ⑥ 原来就有的：logger
//     if (_logger)
//     {
//         _logger->logToFile();
//     }

//     // ⑦ 原来就有的：时间往前走
//     _time += _time_step;
// }

// void Simulation::_updateGraphics()
// {
//     if (_graphics_scene)
//     {
//         _graphics_scene->update();
//         _graphics_scene->viewer()->editText("time", "Sim Time: " + std::to_string(_time) + " s");
//     }
// }

// void Simulation::notifyKeyPressed(SimulationInput::Key /* key */, SimulationInput::KeyAction action, int /* modifiers */)
// {
//     if (_sim_mode == Config::SimulationMode::FRAME_BY_FRAME && action == SimulationInput::KeyAction::PRESS)
//     {
//         _timeStep();
//         _updateGraphics();
//     }
// }

// void Simulation::notifyMouseButtonPressed(SimulationInput::MouseButton /* button */, SimulationInput::MouseAction /* action */, int /* modifiers */)
// {
//     // do nothing
// }

// void Simulation::notifyMouseMoved(double /* x */, double /* y */)
// {
//     // do nothing
// }

// void Simulation::notifyMouseScrolled(double /* dx */, double /* dy */)
// {
//     // do nothing
// }

// int Simulation::run()
// {
//     if (!_setup)
//         setup();

//     std::thread update_thread;
//     if (_sim_mode != Config::SimulationMode::FRAME_BY_FRAME)
//     {
//         update_thread = std::thread(&Simulation::update, this);
//     }

//     if (_graphics_scene)
//     {
//         _graphics_scene->run();
//         return 0;
//     }
//     else
//     {
//         update_thread.join();
//         return 0;
//     }
// }

// } // namespace Sim



// #include "simulation/Simulation.hpp"

// #include "config/simobject/RigidMeshObjectConfig.hpp"
// #include "config/simobject/XPBDMeshObjectConfig.hpp"
// #include "config/simobject/FirstOrderXPBDMeshObjectConfig.hpp"
// #include "config/simobject/RigidPrimitiveConfigs.hpp"
// #include "config/simobject/VirtuosoArmConfig.hpp"
// #include "config/simobject/VirtuosoRobotConfig.hpp"

// #include "graphics/easy3d/Easy3DGraphicsScene.hpp"
// #include "graphics/vtk/VTKGraphicsScene.hpp"

// #include "simobject/RigidMeshObject.hpp"
// #include "simobject/XPBDMeshObject.hpp"
// #include "simobject/RigidPrimitives.hpp"
// #include "simobject/VirtuosoArm.hpp"
// #include "simobject/VirtuosoRobot.hpp"

// #include "simobject/XPBDObjectFactory.hpp"

// #include "solver/constraint/NerveStretchConstraint.hpp"
// #include "utils/MeshUtils.hpp"

// #include <gmsh.h>
// #include <chrono>
// #include <thread>
// #include <iomanip>
// #include <filesystem>
// #include <sstream>
// #include <iostream>
// #include <limits>   // ★ 新增：为挑“底面三角形”用

// namespace Sim
// {

// // === Static cache for the picked edge we’ll monitor each frame ===
// static bool  s_edge_initialized = false;
// static int   s_edge_i = -1;
// static int   s_edge_j = -1;
// static Real  s_edge_rest_len = 0.0;

// Simulation::Simulation(const Config::SimulationConfig* config)
//     : _setup(false), _config(config)
// {
//     // initialize gmsh
//     gmsh::initialize();

//     // set simulation properties based on YAML file
//     _name = _config->name();
//     _description = _config->description();
//     _time_step = _config->timeStep();
//     _end_time = _config->endTime();
//     _time = 0;
//     _g_accel = _config->gAccel();
//     _viewer_refresh_time = 1 / _config->fps() * 1000;
//     _time_between_collision_checks = 1.0 / _config->collisionRate();

//     // set the Simulation mode from the YAML config
//     _sim_mode = _config->simMode();

//     // initialize the graphics scene according to the type specified by the user
//     if (_config->visualization() == Config::Visualization::EASY3D)
//     {
//         _graphics_scene = std::make_unique<Graphics::Easy3DGraphicsScene>("main", config->renderConfig());
//     }
//     if (_config->visualization() == Config::Visualization::VTK)
//     {
//         _graphics_scene = std::make_unique<Graphics::VTKGraphicsScene>("main", config->renderConfig());
//     }

//     // initialize the Embree scene
//     _embree_scene = std::make_unique<Geometry::EmbreeScene>();

//     // initialize the collision scene
//     _collision_scene = std::make_unique<CollisionScene>(this, _embree_scene.get());
//     _last_collision_detection_time = 0;

//     // initialize the logger
//     if (_config->logging())
//     {
//         // get datetime string
//         auto now = std::chrono::system_clock::now();
//         auto time_t = std::chrono::system_clock::to_time_t(now);

//         std::stringstream ss;
//         ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d_%H:%M:%S") << ".txt";
//         std::string filename = ss.str();

//         std::filesystem::path output_dir(config->loggingOutputDir());
//         std::filesystem::path filepath = output_dir / filename;

//         _logger = std::make_unique<SimulationLogger>(filepath.string());
//     }

//     // create materials
//     for (const auto& mat_config : config->materialConfigs())
//     {
//         _materials.emplace_back(&mat_config);
//     }
// }

// std::string Simulation::toString(const int indent) const
// {
//     std::string indent_str(indent, '\t');
//     std::stringstream ss;
//     ss << indent_str << "=====" << type() << " '" << _name << "'=====" << std::endl;
//     ss << indent_str << "Time step: " << _time_step << " s" << std::endl;
//     ss << indent_str << "End time: " << _end_time << " s" << std::endl;
//     ss << indent_str << "Gravity: " << _g_accel << " m/s2" << std::endl;
//     return ss.str();
// }

// void Simulation::setup()
// {
//     assert(!_setup);
//     _setup = true;

//     // graphics
//     if (_graphics_scene)
//     {
//         _graphics_scene->init();
//         _graphics_scene->viewer()->registerSimulation(this);
//         _graphics_scene->viewer()->addText(
//             "time", "Sim Time: 0.000 s",
//             10.0f, 10.0f, 15.0f,
//             Graphics::Viewer::TextAlignment::LEFT,
//             Graphics::Viewer::Font::MAO,
//             std::array<float, 3>({0, 0, 0}),
//             0.5f,
//             false);
//         _graphics_scene->viewer()->enableMouseInteraction(_config->enableMouseInteraction());
//     }

//     // create objects from YAML
//     auto& object_configs = _config->objectConfigs();
//     object_configs.for_each_element([this](const auto& config)
//     {
//         this->_addObjectFromConfig(&config);
//     });

//     // ==================== Pick an existing mesh edge and add NerveStretchConstraint ====================
//     {
//         auto& xpbd_objs = _objects.get<std::unique_ptr<XPBDMeshObject_Base>>();
//         std::cout << "[setup] XPBD objects count = " << xpbd_objs.size() << std::endl;
//         if (xpbd_objs.empty()) {
//             std::cout << "[setup] No XPBD objects found. Check YAML 'objects'." << std::endl;
//         }

//         bool added = false;

//         // Helper: pick the lowest (z-avg) surface triangle and take its longest edge
//         auto add_edge_constraint = [&](auto* xpbd, const char* tag){
//             if (!xpbd) return false;
//             std::cout << "[setup] cast hit: " << tag << std::endl;

//             const int nF = xpbd->mesh()->numFaces();
//             const int nV = xpbd->mesh()->numVertices();
//             if (nF <= 0 || nV <= 0) {
//                 std::cout << "[setup] Mesh has no faces/vertices; cannot pick edge." << std::endl;
//                 return false;
//             }

//             const auto& V = xpbd->mesh()->vertices();

//             // 1) find the bottom face (min average z)
//             int best_face = -1;
//             Real best_z = std::numeric_limits<Real>::infinity();
//             for (int fi = 0; fi < nF; ++fi) {
//                 Eigen::Vector3i f = xpbd->mesh()->face(fi);
//                 if (f[0] < 0 || f[1] < 0 || f[2] < 0 ||
//                     f[0] >= nV || f[1] >= nV || f[2] >= nV) continue;



//                 Real zavg = (V(2, f[0]) + V(2, f[1]) + V(2, f[2])) / Real(3);
//                 if (zavg > best_z) { best_z = zavg; best_face = fi; }  // 注意 > 号
//                 // 同时把 best_z 的初值从 +∞ 改成 -∞
//                 Real best_z = -std::numeric_limits<Real>::infinity();
//             }
//             if (best_face < 0) {
//                 std::cout << "[setup] Failed to find a valid surface face." << std::endl;
//                 return false;
//             }

//             // 2) take the longest edge of that triangle
//             Eigen::Vector3i f = xpbd->mesh()->face(best_face);
//             Real L01 = (V.col(f[0]) - V.col(f[1])).norm();
//             Real L12 = (V.col(f[1]) - V.col(f[2])).norm();
//             Real L20 = (V.col(f[2]) - V.col(f[0])).norm();

//             int i = f[0], j = f[1];
//             Real L = L01;
//             if (L12 > L) { i = f[1]; j = f[2]; L = L12; }
//             if (L20 > L) { i = f[2]; j = f[0]; L = L20; }

//             // 3) add constraint and cache indices
//             xpbd->addNerveStretchConstraint(i, j, L, /*alpha=*/0.0);

//             s_edge_initialized = true;
//             s_edge_i = i; s_edge_j = j; s_edge_rest_len = L;

//             std::cout << "[setup] chose face " << best_face
//                       << " (zavg=" << best_z << "), longest edge ("
//                       << i << "," << j << "), rest_len=" << L << std::endl;
//             return true;
//         };

//         for (auto& uptr : xpbd_objs) {
//             XPBDMeshObject_Base* base_ptr = uptr.get();
//             std::cout << "[setup] trying on object @" << base_ptr << std::endl;

//             // ===== 2nd-order + Stable-Neohookean (Non-Combined) =====
//             {
//                 using Cfg = XPBDMeshObjectConstraintConfigurations<false>;
//                 using Sol = XPBDObjectSolverTypes<false, typename Cfg::StableNeohookean::projector_type_list>;
//                 using T_GS = XPBDMeshObject_<false, Sol::GaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
//                 using T_J  = XPBDMeshObject_<false, Sol::Jacobi,       typename Cfg::StableNeohookean::constraint_type_list>;
//                 using T_PJ = XPBDMeshObject_<false, Sol::ParallelJacobi,typename Cfg::StableNeohookean::constraint_type_list>;
//                 if (!added) added = add_edge_constraint(dynamic_cast<T_GS*>(base_ptr), "2nd + NonCombined + GS");
//                 if (!added) added = add_edge_constraint(dynamic_cast<T_J *>(base_ptr), "2nd + NonCombined + Jacobi");
//                 if (!added) added = add_edge_constraint(dynamic_cast<T_PJ*>(base_ptr), "2nd + NonCombined + ParallelJacobi");
//             }

//             // ===== 2nd-order + Stable-Neohookean-Combined =====
//             {
//                 using Cfg = XPBDMeshObjectConstraintConfigurations<false>;
//                 using Sol = XPBDObjectSolverTypes<false, typename Cfg::StableNeohookeanCombined::projector_type_list>;
//                 using T_GS = XPBDMeshObject_<false, Sol::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
//                 using T_J  = XPBDMeshObject_<false, Sol::Jacobi,       typename Cfg::StableNeohookeanCombined::constraint_type_list>;
//                 using T_PJ = XPBDMeshObject_<false, Sol::ParallelJacobi,typename Cfg::StableNeohookeanCombined::constraint_type_list>;
//                 if (!added) added = add_edge_constraint(dynamic_cast<T_GS*>(base_ptr), "2nd + Combined + GS");
//                 if (!added) added = add_edge_constraint(dynamic_cast<T_J *>(base_ptr), "2nd + Combined + Jacobi");
//                 if (!added) added = add_edge_constraint(dynamic_cast<T_PJ*>(base_ptr), "2nd + Combined + ParallelJacobi");
//             }

//             // ===== 1st-order + Stable-Neohookean (Non-Combined) =====
//             {
//                 using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
//                 using Sol1 = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookean::projector_type_list>;
//                 using A_GS = XPBDMeshObject_<true, Sol1::GaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
//                 using A_J  = XPBDMeshObject_<true, Sol1::Jacobi,       typename Cfg::StableNeohookean::constraint_type_list>;
//                 using A_PJ = XPBDMeshObject_<true, Sol1::ParallelJacobi,typename Cfg::StableNeohookean::constraint_type_list>;
//                 if (!added) added = add_edge_constraint(dynamic_cast<A_GS*>(base_ptr), "1st + NonCombined + GS");
//                 if (!added) added = add_edge_constraint(dynamic_cast<A_J *>(base_ptr), "1st + NonCombined + Jacobi");
//                 if (!added) added = add_edge_constraint(dynamic_cast<A_PJ*>(base_ptr), "1st + NonCombined + ParallelJacobi");
//             }

//             // ===== 1st-order + Stable-Neohookean-Combined =====
//             {
//                 using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
//                 using Sol2 = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookeanCombined::projector_type_list>;
//                 using B_GS = XPBDMeshObject_<true, Sol2::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
//                 using B_J  = XPBDMeshObject_<true, Sol2::Jacobi,       typename Cfg::StableNeohookeanCombined::constraint_type_list>;
//                 using B_PJ = XPBDMeshObject_<true, Sol2::ParallelJacobi,typename Cfg::StableNeohookeanCombined::constraint_type_list>;
//                 if (!added) added = add_edge_constraint(dynamic_cast<B_GS*>(base_ptr), "1st + Combined + GS");
//                 if (!added) added = add_edge_constraint(dynamic_cast<B_J *>(base_ptr), "1st + Combined + Jacobi");
//                 if (!added) added = add_edge_constraint(dynamic_cast<B_PJ*>(base_ptr), "1st + Combined + ParallelJacobi");
//             }

//             if (added) break; // one is enough
//         }

//         if (!added) {
//             std::cout << "[setup] WARNING: no XPBD template combination matched; no edge constraint added.\n"
//                          "         Check your YAML (type, solver-type, constraint-type) vs. these branches.\n";
//         }
//     }
//     // ================== END ==================

//     // logger
//     if (_logger)
//     {
//         _logger->addOutput("time [s]", &_time);
//     }
// }

// void Simulation::update()
// {
//     if (_logger)
//         _logger->startLogging();

//     auto start = std::chrono::steady_clock::now();
//     _wall_time_start = std::chrono::steady_clock::now();
//     auto last_redraw = std::chrono::steady_clock::now();

//     while (_time < _end_time)
//     {
//         Real wall_time_elapsed_s =
//             std::chrono::duration_cast<std::chrono::nanoseconds>(
//                 std::chrono::steady_clock::now() - _wall_time_start)
//                 .count() /
//             1000000000.0;

//         // callbacks
//         for (auto& cb : _callbacks)
//         {
//             if (wall_time_elapsed_s > cb.next_exec_time)
//             {
//                 cb.callback();
//                 cb.next_exec_time = cb.next_exec_time + cb.interval;
//             }
//         }

//         // real-time block
//         if (_sim_mode == Config::SimulationMode::VISUALIZATION && _time > wall_time_elapsed_s)
//         {
//             continue;
//         }

//         _timeStep();

//         auto time_since_last_redraw_ms =
//             std::chrono::duration_cast<std::chrono::milliseconds>(
//                 std::chrono::steady_clock::now() - last_redraw)
//                 .count();

//         if (time_since_last_redraw_ms > _viewer_refresh_time)
//         {
//             _updateGraphics();
//             last_redraw = std::chrono::steady_clock::now();
//         }
//     }

//     _updateGraphics();

//     auto end = std::chrono::steady_clock::now();
//     std::cout << "Simulating " << _end_time << " seconds took "
//               << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
//               << " ms" << std::endl;
// }

// void Simulation::_timeStep()
// {
//     // —— refresh collision constraints —— //
//     if (_time - _last_collision_detection_time > _time_between_collision_checks)
//     {
//         auto& xpbd_mesh_objs = _objects.get<std::unique_ptr<XPBDMeshObject_Base>>();
//         for (auto& obj : xpbd_mesh_objs) obj->clearCollisionConstraints();

//         auto& fo_xpbd_mesh_objs = _objects.get<std::unique_ptr<FirstOrderXPBDMeshObject_Base>>();
//         for (auto& obj : fo_xpbd_mesh_objs) obj->clearCollisionConstraints();

//         auto& virtuoso_robots = _objects.get<std::unique_ptr<VirtuosoRobot>>();
//         for (auto& obj : virtuoso_robots)
//         {
//             if (obj->hasArm1()) obj->arm1()->clearCollisionConstraints();
//             if (obj->hasArm2()) obj->arm2()->clearCollisionConstraints();
//         }

//         auto& virtuoso_arms = _objects.get<std::unique_ptr<VirtuosoArm>>();
//         for (auto& obj : virtuoso_arms) obj->clearCollisionConstraints();

//         _collision_scene->collideObjects();
//     }

//     // —— PRE: read current length of the picked edge —— //
//     if (s_edge_initialized)
//     {
//         auto read_and_print = [&](auto* xpbd, const char* tag, const char* phase){
//             if (!xpbd) return false;
//             const auto& V = xpbd->mesh()->vertices();
//             if (s_edge_i < 0 || s_edge_j < 0 ||
//                 s_edge_i >= xpbd->mesh()->numVertices() ||
//                 s_edge_j >= xpbd->mesh()->numVertices()) return false;
//             const Real len = (V.col(s_edge_i) - V.col(s_edge_j)).norm();
//             std::cout << "[" << phase << "](" << tag << ") edge("
//                       << s_edge_i << "," << s_edge_j << ") len = "
//                       << len << " (rest = " << s_edge_rest_len << ")\n";
//             return true;
//         };

//         auto& xpbd_objs = _objects.get<std::unique_ptr<XPBDMeshObject_Base>>();
//         bool printed = false;
//         for (auto& uptr : xpbd_objs) {
//             auto* base_ptr = uptr.get();

//             // 2nd + NonCombined
//             {
//                 using Cfg = XPBDMeshObjectConstraintConfigurations<false>;
//                 using Sol = XPBDObjectSolverTypes<false, typename Cfg::StableNeohookean::projector_type_list>;
//                 using T_GS = XPBDMeshObject_<false, Sol::GaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
//                 using T_J  = XPBDMeshObject_<false, Sol::Jacobi,       typename Cfg::StableNeohookean::constraint_type_list>;
//                 using T_PJ = XPBDMeshObject_<false, Sol::ParallelJacobi,typename Cfg::StableNeohookean::constraint_type_list>;
//                 if (!printed) printed = read_and_print(dynamic_cast<T_GS*>(base_ptr), "2nd+NonCombined+GS", "pre");
//                 if (!printed) printed = read_and_print(dynamic_cast<T_J *>(base_ptr), "2nd+NonCombined+Jacobi", "pre");
//                 if (!printed) printed = read_and_print(dynamic_cast<T_PJ*>(base_ptr), "2nd+NonCombined+PJacobi", "pre");
//             }
//             // 2nd + Combined
//             {
//                 using Cfg = XPBDMeshObjectConstraintConfigurations<false>;
//                 using Sol = XPBDObjectSolverTypes<false, typename Cfg::StableNeohookeanCombined::projector_type_list>;
//                 using T_GS = XPBDMeshObject_<false, Sol::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
//                 using T_J  = XPBDMeshObject_<false, Sol::Jacobi,       typename Cfg::StableNeohookeanCombined::constraint_type_list>;
//                 using T_PJ = XPBDMeshObject_<false, Sol::ParallelJacobi,typename Cfg::StableNeohookeanCombined::constraint_type_list>;
//                 if (!printed) printed = read_and_print(dynamic_cast<T_GS*>(base_ptr), "2nd+Combined+GS", "pre");
//                 if (!printed) printed = read_and_print(dynamic_cast<T_J *>(base_ptr), "2nd+Combined+Jacobi", "pre");
//                 if (!printed) printed = read_and_print(dynamic_cast<T_PJ*>(base_ptr), "2nd+Combined+PJacobi", "pre");
//             }
//             // 1st + NonCombined
//             {
//                 using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
//                 using Sol = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookean::projector_type_list>;
//                 using A_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
//                 using A_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::StableNeohookean::constraint_type_list>;
//                 using A_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::StableNeohookean::constraint_type_list>;
//                 if (!printed) printed = read_and_print(dynamic_cast<A_GS*>(base_ptr), "1st+NonCombined+GS", "pre");
//                 if (!printed) printed = read_and_print(dynamic_cast<A_J *>(base_ptr), "1st+NonCombined+Jacobi", "pre");
//                 if (!printed) printed = read_and_print(dynamic_cast<A_PJ*>(base_ptr), "1st+NonCombined+PJacobi", "pre");
//             }
//             // 1st + Combined
//             {
//                 using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
//                 using Sol = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookeanCombined::projector_type_list>;
//                 using B_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
//                 using B_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::StableNeohookeanCombined::constraint_type_list>;
//                 using B_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::StableNeohookeanCombined::constraint_type_list>;
//                 if (!printed) printed = read_and_print(dynamic_cast<B_GS*>(base_ptr), "1st+Combined+GS", "pre");
//                 if (!printed) printed = read_and_print(dynamic_cast<B_J *>(base_ptr), "1st+Combined+Jacobi", "pre");
//                 if (!printed) printed = read_and_print(dynamic_cast<B_PJ*>(base_ptr), "1st+Combined+PJacobi", "pre");
//             }

//             if (printed) break;
//         }

//         static bool warned_pre = false;
//         if (!printed && !warned_pre) {
//             std::cout << "[pre] WARNING: s_edge_initialized=true but couldn't read vertices; "
//                          "template combo at runtime didn't match. Check setup prints."
//                       << std::endl;
//             warned_pre = true;
//         }
//     }

//     // —— Run one XPBD step (objects do elasticity + collisions + your stretch) —— //
//     _objects.for_each_element([](auto& obj) { obj->update(); });

//     // —— POST: read again and print error —— //
//     if (s_edge_initialized)
//     {
//         auto read_and_print_post = [&](auto* xpbd, const char* tag){
//             if (!xpbd) return false;
//             const auto& V = xpbd->mesh()->vertices();
//             if (s_edge_i < 0 || s_edge_j < 0 ||
//                 s_edge_i >= xpbd->mesh()->numVertices() ||
//                 s_edge_j >= xpbd->mesh()->numVertices()) return false;
//             const Real len = (V.col(s_edge_i) - V.col(s_edge_j)).norm();
//             const Real err = std::abs(len - s_edge_rest_len);
//             std::cout << "[post](" << tag << ") edge("
//                       << s_edge_i << "," << s_edge_j << ") len = "
//                       << len << "  |len-rest| = " << err << "\n";
//             return true;
//         };

//         auto& xpbd_objs = _objects.get<std::unique_ptr<XPBDMeshObject_Base>>();
//         bool printed = false;
//         for (auto& uptr : xpbd_objs) {
//             auto* base_ptr = uptr.get();

//             // 2nd + NonCombined
//             {
//                 using Cfg = XPBDMeshObjectConstraintConfigurations<false>;
//                 using Sol = XPBDObjectSolverTypes<false, typename Cfg::StableNeohookean::projector_type_list>;
//                 using T_GS = XPBDMeshObject_<false, Sol::GaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
//                 using T_J  = XPBDMeshObject_<false, Sol::Jacobi,       typename Cfg::StableNeohookean::constraint_type_list>;
//                 using T_PJ = XPBDMeshObject_<false, Sol::ParallelJacobi,typename Cfg::StableNeohookean::constraint_type_list>;
//                 if (!printed) printed = read_and_print_post(dynamic_cast<T_GS*>(base_ptr), "2nd+NonCombined+GS");
//                 if (!printed) printed = read_and_print_post(dynamic_cast<T_J *>(base_ptr), "2nd+NonCombined+Jacobi");
//                 if (!printed) printed = read_and_print_post(dynamic_cast<T_PJ*>(base_ptr), "2nd+NonCombined+PJacobi");
//             }
//             // 2nd + Combined
//             {
//                 using Cfg = XPBDMeshObjectConstraintConfigurations<false>;
//                 using Sol = XPBDObjectSolverTypes<false, typename Cfg::StableNeohookeanCombined::projector_type_list>;
//                 using T_GS = XPBDMeshObject_<false, Sol::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
//                 using T_J  = XPBDMeshObject_<false, Sol::Jacobi,       typename Cfg::StableNeohookeanCombined::constraint_type_list>;
//                 using T_PJ = XPBDMeshObject_<false, Sol::ParallelJacobi,typename Cfg::StableNeohookeanCombined::constraint_type_list>;
//                 if (!printed) printed = read_and_print_post(dynamic_cast<T_GS*>(base_ptr), "2nd+Combined+GS");
//                 if (!printed) printed = read_and_print_post(dynamic_cast<T_J *>(base_ptr), "2nd+Combined+Jacobi");
//                 if (!printed) printed = read_and_print_post(dynamic_cast<T_PJ*>(base_ptr), "2nd+Combined+PJacobi");
//             }
//             // 1st + NonCombined
//             {
//                 using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
//                 using Sol = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookean::projector_type_list>;
//                 using A_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
//                 using A_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::StableNeohookean::constraint_type_list>;
//                 using A_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::StableNeohookean::constraint_type_list>;
//                 if (!printed) printed = read_and_print_post(dynamic_cast<A_GS*>(base_ptr), "1st+NonCombined+GS");
//                 if (!printed) printed = read_and_print_post(dynamic_cast<A_J *>(base_ptr), "1st+NonCombined+Jacobi");
//                 if (!printed) printed = read_and_print_post(dynamic_cast<A_PJ*>(base_ptr), "1st+NonCombined+PJacobi");
//             }
//             // 1st + Combined
//             {
//                 using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
//                 using Sol = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookeanCombined::projector_type_list>;
//                 using B_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
//                 using B_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::StableNeohookeanCombined::constraint_type_list>;
//                 using B_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::StableNeohookeanCombined::constraint_type_list>;
//                 if (!printed) printed = read_and_print_post(dynamic_cast<B_GS*>(base_ptr), "1st+Combined+GS");
//                 if (!printed) printed = read_and_print_post(dynamic_cast<B_J *>(base_ptr), "1st+Combined+Jacobi");
//                 if (!printed) printed = read_and_print_post(dynamic_cast<B_PJ*>(base_ptr), "1st+Combined+PJacobi");
//             }

//             if (printed) break;
//         }

//         static bool warned_post = false;
//         if (!printed && !warned_post) {
//             std::cout << "[post] WARNING: s_edge_initialized=true but couldn't read vertices; "
//                          "template combo at runtime didn't match. Check setup prints."
//                       << std::endl;
//             warned_post = true;
//         }
//     }

//     // —— velocity update —— //
//     _objects.for_each_element([](auto& obj) { obj->velocityUpdate(); });

//     // —— collision timestamp —— //
//     if (_time - _last_collision_detection_time > _time_between_collision_checks)
//     {
//         _last_collision_detection_time = _time;
//     }

//     // —— logging —— //
//     if (_logger) _logger->logToFile();

//     // —— advance time —— //
//     _time += _time_step;
// }

// void Simulation::_updateGraphics()
// {
//     if (_graphics_scene)
//     {
//         _graphics_scene->update();
//         _graphics_scene->viewer()->editText("time", "Sim Time: " + std::to_string(_time) + " s");
//     }
// }

// void Simulation::notifyKeyPressed(SimulationInput::Key /* key */, SimulationInput::KeyAction action, int /* modifiers */)
// {
//     if (_sim_mode == Config::SimulationMode::FRAME_BY_FRAME && action == SimulationInput::KeyAction::PRESS)
//     {
//         _timeStep();
//         _updateGraphics();
//     }
// }

// void Simulation::notifyMouseButtonPressed(SimulationInput::MouseButton /* button */, SimulationInput::MouseAction /* action */, int /* modifiers */)
// {
//     // do nothing
// }

// void Simulation::notifyMouseMoved(double /* x */, double /* y */)
// {
//     // do nothing
// }

// void Simulation::notifyMouseScrolled(double /* dx */, double /* dy */)
// {
//     // do nothing
// }

// int Simulation::run()
// {
//     if (!_setup)
//         setup();

//     std::thread update_thread;
//     if (_sim_mode != Config::SimulationMode::FRAME_BY_FRAME)
//     {
//         update_thread = std::thread(&Simulation::update, this);
//     }

//     if (_graphics_scene)
//     {
//         _graphics_scene->run();
//         return 0;
//     }
//     else
//     {
//         update_thread.join();
//         return 0;
//     }
// }

// } // namespace Sim


#include "simulation/Simulation.hpp"

#include "config/simobject/RigidMeshObjectConfig.hpp"
#include "config/simobject/XPBDMeshObjectConfig.hpp"
#include "config/simobject/FirstOrderXPBDMeshObjectConfig.hpp"
#include "config/simobject/RigidPrimitiveConfigs.hpp"
#include "config/simobject/VirtuosoArmConfig.hpp"
#include "config/simobject/VirtuosoRobotConfig.hpp"

#include "graphics/easy3d/Easy3DGraphicsScene.hpp"
#include "graphics/vtk/VTKGraphicsScene.hpp"

#include "simobject/RigidMeshObject.hpp"
#include "simobject/XPBDMeshObject.hpp"
#include "simobject/RigidPrimitives.hpp"
#include "simobject/VirtuosoArm.hpp"
#include "simobject/VirtuosoRobot.hpp"

#include "simobject/XPBDObjectFactory.hpp"

#include "solver/constraint/NerveStretchConstraint.hpp"
#include "solver/constraint/NerveBendingConstraint.hpp"
#include "utils/MeshUtils.hpp"

#include <gmsh.h>
#include <chrono>
#include <thread>
#include <iomanip>
#include <filesystem>
#include <sstream>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <cmath>

namespace Sim
{

// === Static cache for the picked edge we’ll monitor each frame ===
static bool  s_edge_initialized = false;
static int   s_edge_i = -1;
static int   s_edge_j = -1;
static Real  s_edge_rest_len = 0.0;
static int   s_print_counter = 0;  // Counter for controlling print frequency

// ===== helper: map gmsh node position -> nearest vertex index in internal mesh (with adaptive tolerance)
static int mapNodeToVertex(
    const Eigen::Matrix<Real, 3, Eigen::Dynamic>& V,
    const Vec3r& p,
    const Real bbox_diag)
{
    const Real tol = std::max(Real(1e-9), bbox_diag * Real(1e-6));
    int best = -1;
    Real best_d2 = std::numeric_limits<Real>::infinity();
    const int n = (int)V.cols();
    for (int i = 0; i < n; ++i) {
        const Real d2 = (V.col(i) - p).squaredNorm();
        if (d2 < best_d2) { best_d2 = d2; best = i; }
    }
    if (best >= 0 && std::sqrt(best_d2) <= tol) return best;
    return -1;
}

Simulation::Simulation(const Config::SimulationConfig* config)
    : _setup(false), _config(config)
{
    // initialize gmsh
    gmsh::initialize();

    // set simulation properties based on YAML file
    _name = _config->name();
    _description = _config->description();
    _time_step = _config->timeStep();
    _end_time = _config->endTime();
    _time = 0;
    _g_accel = _config->gAccel();
    _viewer_refresh_time = 1 / _config->fps() * 1000;
    _time_between_collision_checks = 1.0 / _config->collisionRate();

    // set the Simulation mode from the YAML config
    _sim_mode = _config->simMode();

    // initialize the graphics scene according to the type specified by the user
    if (_config->visualization() == Config::Visualization::EASY3D)
    {
        _graphics_scene = std::make_unique<Graphics::Easy3DGraphicsScene>("main", config->renderConfig());
    }
    if (_config->visualization() == Config::Visualization::VTK)
    {
        _graphics_scene = std::make_unique<Graphics::VTKGraphicsScene>("main", config->renderConfig());
    }

    // initialize the Embree scene
    _embree_scene = std::make_unique<Geometry::EmbreeScene>();

    // initialize the collision scene
    _collision_scene = std::make_unique<CollisionScene>(this, _embree_scene.get());
    _last_collision_detection_time = 0;

    // initialize the logger
    if (_config->logging())
    {
        // get datetime string
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);

        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d_%H:%M:%S") << ".txt";
        std::string filename = ss.str();

        std::filesystem::path output_dir(config->loggingOutputDir());
        std::filesystem::path filepath = output_dir / filename;

        _logger = std::make_unique<SimulationLogger>(filepath.string());
    }

    // create materials
    for (const auto& mat_config : config->materialConfigs())
    {
        _materials.emplace_back(&mat_config);
    }
}

std::string Simulation::toString(const int indent) const
{
    std::string indent_str(indent, '\t');
    std::stringstream ss;
    ss << indent_str << "=====" << type() << " '" << _name << "'=====" << std::endl;
    ss << indent_str << "Time step: " << _time_step << " s" << std::endl;
    ss << indent_str << "End time: " << _end_time << " s" << std::endl;
    ss << indent_str << "Gravity: " << _g_accel << " m/s2" << std::endl;
    return ss.str();
}

void Simulation::setup()
{
    assert(!_setup);
    _setup = true;

    // graphics
    if (_graphics_scene)
    {
        _graphics_scene->init();
        _graphics_scene->viewer()->registerSimulation(this);
        _graphics_scene->viewer()->addText(
            "time", "Sim Time: 0.000 s",
            10.0f, 10.0f, 15.0f,
            Graphics::Viewer::TextAlignment::LEFT,
            Graphics::Viewer::Font::MAO,
            std::array<float, 3>({0, 0, 0}),
            0.5f,
            false);
        _graphics_scene->viewer()->enableMouseInteraction(_config->enableMouseInteraction());
    }

    // create objects from YAML
    auto& object_configs = _config->objectConfigs();
    object_configs.for_each_element([this](const auto& config)
    {
        this->_addObjectFromConfig(&config);
    });

    // ==================== Read Physical Line("nerve_edge") from .msh and add NerveStretchConstraint ====================
    {
        // Read nerve configuration from YAML config instead of environment variables
        const bool nerve_enabled = _config->nerveEnable();
        const bool nerve_stretch_enabled = _config->nerveStretchEnable();
        const bool nerve_bending_enabled = _config->nerveBendingEnable();
        const std::string msh_path = _config->nerveMeshFile();
        const std::string phys_name = _config->nervePhysicalGroup();

        // Also check environment variables for backward compatibility (but YAML takes precedence)
        const char* env_msh  = std::getenv("NERVE_MSH");
        const char* env_name = std::getenv("NERVE_PHYS");
        const char* env_enable = std::getenv("NERVE_ENABLE");
        
        // YAML configuration takes precedence, environment variables used as fallback
        // Check if YAML explicitly set nerve-enable (not default)
        const bool yaml_explicitly_set_enable = (_config->nerveEnable() != true) || 
                                               (!_config->nerveMeshFile().empty()) ||
                                               (_config->nervePhysicalGroup() != "nerve_edge");
        
        const std::string final_msh_path = (!msh_path.empty()) ? msh_path : (env_msh ? std::string(env_msh) : std::string());
        const std::string final_phys_name = (phys_name != "nerve_edge") ? phys_name : (env_name ? std::string(env_name) : "nerve_edge");
        
        // Use YAML value if explicitly set, otherwise fall back to environment variable logic
        const bool final_nerve_enabled = yaml_explicitly_set_enable ? 
            nerve_enabled : 
            (env_enable ? (std::string(env_enable) == "1" || std::string(env_enable) == "true") : true);
            
        // For individual constraint types, use YAML values (with nerve_enabled as master switch)
        const bool final_stretch_enabled = final_nerve_enabled && nerve_stretch_enabled;
        const bool final_bending_enabled = final_nerve_enabled && nerve_bending_enabled;

        auto& xpbd_objs = _objects.get<std::unique_ptr<XPBDMeshObject_Base>>();
        auto& fo_xpbd_objs = _objects.get<std::unique_ptr<FirstOrderXPBDMeshObject_Base>>();
        std::cout << "[setup] XPBD objects count = " << xpbd_objs.size() << std::endl;
        std::cout << "[setup] FirstOrder XPBD objects count = " << fo_xpbd_objs.size() << std::endl;

        std::cout << "[nerve] Configuration: enable=" << final_nerve_enabled 
                  << ", stretch=" << final_stretch_enabled << ", bending=" << final_bending_enabled
                  << ", mesh='" << final_msh_path << "', physical-group='" << final_phys_name << "'" << std::endl;

        if (!final_nerve_enabled) {
            std::cout << "[nerve] Nerve constraints DISABLED (nerve-enable=false or NERVE_ENABLE=0).\n";
            // BUT still set up monitoring edge for length comparison
            if (!final_msh_path.empty() && (!xpbd_objs.empty() || !fo_xpbd_objs.empty())) {
                std::cout << "[monitor] Setting up edge monitoring without constraints for comparison...\n";
                
                try {
                    gmsh::model::add("monitor_tag_reader");
                    gmsh::open(final_msh_path);

                    // Find the physical line for monitoring
                    std::vector<std::pair<int,int>> phys_groups;
                    gmsh::model::getPhysicalGroups(phys_groups);
                    int target_phys_tag = -1;
                    for (auto [dim, tag] : phys_groups) {
                        if (dim != 1) continue;
                        std::string nm;
                        gmsh::model::getPhysicalName(dim, tag, nm);
                        if (nm == final_phys_name) { target_phys_tag = tag; break; }
                    }
                    
                    if (target_phys_tag >= 0) {
                        std::vector<int> curve_tags;
                        gmsh::model::getEntitiesForPhysicalGroup(1, target_phys_tag, curve_tags);
                        
                        if (!curve_tags.empty()) {
                            // Get node coordinates
                            std::vector<std::size_t> nodeTags;
                            std::vector<double> nodeCoords, nodeParams;
                            gmsh::model::mesh::getNodes(nodeTags, nodeCoords, nodeParams);
                            
                            // Collect line segments  
                            std::vector<std::pair<std::size_t, std::size_t>> line_pairs;
                            for (int ctag : curve_tags) {
                                std::vector<int> types;
                                std::vector<std::vector<std::size_t>> elemTags, elemNodeTags;
                                gmsh::model::mesh::getElements(types, elemTags, elemNodeTags, 1, ctag);
                                for (std::size_t k = 0; k < types.size(); ++k) {
                                    if (types[k] != 1) continue;
                                    const auto& nodes = elemNodeTags[k];
                                    for (std::size_t i = 0; i + 1 < nodes.size(); i += 2) {
                                        std::size_t n0 = nodes[i], n1 = nodes[i+1];
                                        if (n0 != n1) line_pairs.emplace_back(n0, n1);
                                    }
                                }
                            }
                            
                            // Set up monitoring on first available edge (no constraints added)
                            auto setup_monitor = [&](auto* xpbd, const char* tag)->bool {
                                if (!xpbd) return false;
                                const auto& tag2idx = xpbd->mesh()->tagMap();
                                if (tag2idx.empty()) return false;
                                
                                for (const auto& seg : line_pairs) {
                                    auto it0 = tag2idx.find(seg.first);
                                    auto it1 = tag2idx.find(seg.second);
                                    if (it0 != tag2idx.end() && it1 != tag2idx.end()) {
                                        const int i = it0->second;
                                        const int j = it1->second;
                                        if (i >= 0 && j >= 0 && i != j) {
                                            const auto& V = xpbd->mesh()->vertices();
                                            const Real rest_len = (V.col(i) - V.col(j)).norm();
                                            s_edge_initialized = true;
                                            s_edge_i = i; s_edge_j = j; s_edge_rest_len = rest_len;
                                            std::cout << "[monitor] NO CONSTRAINTS, but monitoring edge (" 
                                                      << i << "," << j << "), rest_len=" << rest_len << "\n";
                                            return true;
                                        }
                                    }
                                }
                                return false;
                            };
                            
                            // Try to set up monitoring on any available object
                            bool monitor_set = false;
                            for (auto& uptr : xpbd_objs) {
                                XPBDMeshObject_Base* base_ptr = uptr.get();
                                
                                // Try all common template combinations for monitoring
                                // 2nd + NonCombined
                                {
                                    using Cfg = XPBDMeshObjectConstraintConfigurations<false>;
                                    using Sol = XPBDObjectSolverTypes<false, typename Cfg::StableNeohookean::projector_type_list>;
                                    using T_GS = XPBDMeshObject_<false, Sol::GaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
                                    using T_J  = XPBDMeshObject_<false, Sol::Jacobi,       typename Cfg::StableNeohookean::constraint_type_list>;
                                    using T_PJ = XPBDMeshObject_<false, Sol::ParallelJacobi,typename Cfg::StableNeohookean::constraint_type_list>;
                                    if (!monitor_set) monitor_set = setup_monitor(dynamic_cast<T_GS*>(base_ptr), "2nd+NonCombined+GS");
                                    if (!monitor_set) monitor_set = setup_monitor(dynamic_cast<T_J *>(base_ptr), "2nd+NonCombined+Jacobi");
                                    if (!monitor_set) monitor_set = setup_monitor(dynamic_cast<T_PJ*>(base_ptr), "2nd+NonCombined+PJ");
                                }
                                // 2nd + Combined
                                {
                                    using Cfg = XPBDMeshObjectConstraintConfigurations<false>;
                                    using Sol = XPBDObjectSolverTypes<false, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                                    using T_GS = XPBDMeshObject_<false, Sol::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                    using T_J  = XPBDMeshObject_<false, Sol::Jacobi,       typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                    using T_PJ = XPBDMeshObject_<false, Sol::ParallelJacobi,typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                    if (!monitor_set) monitor_set = setup_monitor(dynamic_cast<T_GS*>(base_ptr), "2nd+Combined+GS");
                                    if (!monitor_set) monitor_set = setup_monitor(dynamic_cast<T_J *>(base_ptr), "2nd+Combined+Jacobi");
                                    if (!monitor_set) monitor_set = setup_monitor(dynamic_cast<T_PJ*>(base_ptr), "2nd+Combined+PJ");
                                }
                                if (monitor_set) break;
                            }
                            
                            if (!monitor_set) {
                                for (auto& fo_uptr : fo_xpbd_objs) {
                                    FirstOrderXPBDMeshObject_Base* fo_base_ptr = fo_uptr.get();
                                    
                                    // 1st + NonCombined
                                    {
                                        using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                                        using Sol = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookean::projector_type_list>;
                                        using A_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
                                        using A_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::StableNeohookean::constraint_type_list>;
                                        using A_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::StableNeohookean::constraint_type_list>;
                                        if (!monitor_set) monitor_set = setup_monitor(dynamic_cast<A_GS*>(fo_base_ptr), "1st+NonCombined+GS");
                                        if (!monitor_set) monitor_set = setup_monitor(dynamic_cast<A_J *>(fo_base_ptr), "1st+NonCombined+Jacobi");
                                        if (!monitor_set) monitor_set = setup_monitor(dynamic_cast<A_PJ*>(fo_base_ptr), "1st+NonCombined+PJ");
                                    }
                                    // 1st + Combined
                                    {
                                        using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                                        using Sol = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                                        using B_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                        using B_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                        using B_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                        if (!monitor_set) monitor_set = setup_monitor(dynamic_cast<B_GS*>(fo_base_ptr), "1st+Combined+GS");
                                        if (!monitor_set) monitor_set = setup_monitor(dynamic_cast<B_J *>(fo_base_ptr), "1st+Combined+Jacobi");
                                        if (!monitor_set) monitor_set = setup_monitor(dynamic_cast<B_PJ*>(fo_base_ptr), "1st+Combined+PJ");
                                    }
                                    if (monitor_set) break;
                                }
                            }
                            
                            if (!monitor_set) {
                                std::cout << "[monitor] WARNING: Could not set up monitoring for NERVE_ENABLE=0 case.\n";
                                std::cout << "[monitor] Template combination at runtime didn't match. Check YAML config.\n";
                            }
                        }
                    }
                    
                    gmsh::clear();
                } catch (std::exception& e) {
                    std::cout << "[monitor] Exception while setting up monitoring: " << e.what() << "\n";
                    try { gmsh::clear(); } catch (...) {}
                }
            }
        } else if (final_msh_path.empty()) {
            std::cout << "[nerve] nerve-mesh-file not set; skip .msh-driven nerve constraints.\n";
        } else if (xpbd_objs.empty() && fo_xpbd_objs.empty()) {
            std::cout << "[nerve] No XPBD objects; skip.\n";
        } else {
            std::cout << "[nerve] Loading physical line from '" << final_msh_path
                      << "'  name='" << final_phys_name << "'\n";

            // Build a temporary gmsh model to read just the nerve line
            try {
                gmsh::model::add("nerve_tag_reader");
                gmsh::open(final_msh_path);

                // 1) locate dim=1 physical group with given name
                std::vector<std::pair<int,int>> phys_groups;
                gmsh::model::getPhysicalGroups(phys_groups);
                int target_phys_tag = -1;
                for (auto [dim, tag] : phys_groups) {
                    if (dim != 1) continue;
                    std::string nm;
                    gmsh::model::getPhysicalName(dim, tag, nm);
                    if (nm == final_phys_name) { target_phys_tag = tag; break; }
                }
                if (target_phys_tag < 0) {
                    std::cout << "[nerve] Physical Line '" << final_phys_name << "' not found. Skip.\n";
                } else {
                    // 2) curves under this physical
                    std::vector<int> curve_tags;
                    gmsh::model::getEntitiesForPhysicalGroup(1, target_phys_tag, curve_tags);
                    if (curve_tags.empty()) {
                        std::cout << "[nerve] Physical '" << phys_name << "' has no curve entities. Skip.\n";
                    } else {
                        // 3) global node coords
                        std::vector<std::size_t> nodeTags;
                        std::vector<double> nodeCoords, nodeParams;
                        gmsh::model::mesh::getNodes(nodeTags, nodeCoords, nodeParams);
                        std::unordered_map<std::size_t, Vec3r> tag2pos;
                        tag2pos.reserve(nodeTags.size());
                        for (std::size_t i = 0; i < nodeTags.size(); ++i) {
                            tag2pos.emplace(nodeTags[i],
                                            Vec3r(nodeCoords[3*i+0], nodeCoords[3*i+1], nodeCoords[3*i+2]));
                        }

                        // 4) collect all 2-node line segments (n0,n1)
                        std::vector<std::pair<std::size_t, std::size_t>> line_pairs;
                        for (int ctag : curve_tags) {
                            std::vector<int> types;
                            std::vector<std::vector<std::size_t>> elemTags, elemNodeTags;
                            gmsh::model::mesh::getElements(types, elemTags, elemNodeTags, 1, ctag);
                            for (std::size_t k = 0; k < types.size(); ++k) {
                                if (types[k] != 1) continue; // only 2-node line
                                const auto& nodes = elemNodeTags[k];
                                const std::size_t m = (nodes.size() / 2) * 2;
                                for (std::size_t i = 0; i + 1 < m; i += 2) {
                                    std::size_t n0 = nodes[i], n1 = nodes[i+1];
                                    if (n0 != n1) line_pairs.emplace_back(n0, n1);
                                }
                            }
                        }

                        if (line_pairs.empty()) {
                            std::cout << "[nerve] No type=1 elements under '" << phys_name << "'. Skip.\n";
                        } else {
                            std::cout << "[nerve] Found " << line_pairs.size()
                                      << " segments in '" << phys_name << "'. Mapping to internal mesh...\n";

                            bool added_any = false;
                            bool monitor_set = false;

                            // Define lambda function that can be used by both loops
                            auto try_add_for = [&](auto* xpbd, const char* tag)->bool {
                                    if (!xpbd) return false;
                                    std::cout << "[nerve] cast hit: " << tag << "\n";

                                    const auto& V = xpbd->mesh()->vertices();
                                    const int nV = xpbd->mesh()->numVertices();    // Get gmsh node tag -> internal vertex index map
                                    const auto& tag2idx = xpbd->mesh()->tagMap();
                                    // if (nV <= 1) { std::cout << "[nerve] mesh has <=1 vertex.\n"; return false; }
                                    if (tag2idx.empty()) {
                                        std::cout << "[nerve] WARNING: tagMap() is empty; did loader fill gmshTag2Index?\n";
                                        return false;
                                    }
                                    // bbox diag for tolerance
                                    // Vec3r vmin = V.rowwise().minCoeff();
                                    // Vec3r vmax = V.rowwise().maxCoeff();
                                    // const Real bbox_diag = (vmax - vmin).norm();

                                    int add_ok = 0, add_fail = 0;
                                    int bend_ok = 0, bend_fail = 0;
                                    
                                    // First pass: Add stretch constraints (if enabled)
                                    if (final_stretch_enabled) {
                                        for (const auto& seg : line_pairs) {
                                            auto it0 = tag2idx.find(seg.first);
                                            auto it1 = tag2idx.find(seg.second);
                                            if (it0 == tag2idx.end() || it1 == tag2idx.end()) { ++add_fail; continue; }

                                            const int i = it0->second;
                                            const int j = it1->second;
                                            if (i < 0 || j < 0 || i == j) { ++add_fail; continue; }

                                            const Real rest_len = (V.col(i) - V.col(j)).norm();
                                            xpbd->addNerveStretchConstraint(i, j, rest_len, /*alpha=*/0.0);
                                            ++add_ok;

                                            if (!monitor_set) {
                                                s_edge_initialized = true;
                                                s_edge_i = i; s_edge_j = j; s_edge_rest_len = rest_len;
                                                monitor_set = true;
                                            }
                                        }
                                    } else {
                                        std::cout << "[nerve] Stretch constraints DISABLED (nerve-stretch-enable=false)\n";
                                    }

                                    // Second pass: Add bending constraints for consecutive triplets (if enabled)
                                    if (final_bending_enabled) {
                                        // Build adjacency to find consecutive vertices along the nerve
                                        std::unordered_map<int, std::vector<int>> adjacency;
                                        for (const auto& seg : line_pairs) {
                                            auto it0 = tag2idx.find(seg.first);
                                            auto it1 = tag2idx.find(seg.second);
                                            if (it0 == tag2idx.end() || it1 == tag2idx.end()) continue;
                                            
                                            const int i = it0->second;
                                            const int j = it1->second;
                                            if (i < 0 || j < 0 || i == j) continue;
                                            
                                            adjacency[i].push_back(j);
                                            adjacency[j].push_back(i);
                                        }
                                        
                                        // Find triplets for bending constraints
                                        std::set<std::array<int, 3>> triplets;
                                        for (const auto& [center, neighbors] : adjacency) {
                                            if (neighbors.size() == 2) {
                                                // This vertex has exactly 2 neighbors - good for bending constraint
                                                int v0 = neighbors[0];
                                                int v1 = center;
                                                int v2 = neighbors[1];
                                                
                                                // Ensure consistent ordering to avoid duplicates
                                                if (v0 > v2) std::swap(v0, v2);
                                                triplets.insert({v0, v1, v2});
                                            }
                                        }
                                        
                                        // Add bending constraints for all valid triplets
                                        for (const auto& triplet : triplets) {
                                            try {
                                                // Rest curvature = 0 (straight nerve)
                                                xpbd->addNerveBendingConstraint(triplet[0], triplet[1], triplet[2], 
                                                                              /*rest_curvature=*/0.0, /*alpha=*/0.0);
                                                ++bend_ok;
                                            } catch (...) {
                                                ++bend_fail;
                                            }
                                        }
                                    } else {
                                        std::cout << "[nerve] Bending constraints DISABLED (nerve-bending-enable=false)\n";
                                    }

                                    std::cout << "[nerve] addNerveStretchConstraint: ok=" << add_ok
                                              << "  fail=" << add_fail << "\n";
                                    std::cout << "[nerve] addNerveBendingConstraint: ok=" << bend_ok
                                              << "  fail=" << bend_fail << "\n";
                                    return add_ok > 0;
                                };

                            // try on each XPBD object; the first matching template gets the constraints
                            for (auto& uptr : xpbd_objs) {
                                XPBDMeshObject_Base* base_ptr = uptr.get();

                                bool added = false;

                                // ===== 2nd-order + Stable-Neohookean (Non-Combined) =====
                                {
                                    using Cfg = XPBDMeshObjectConstraintConfigurations<false>;
                                    using Sol = XPBDObjectSolverTypes<false, typename Cfg::StableNeohookean::projector_type_list>;
                                    using T_GS = XPBDMeshObject_<false, Sol::GaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
                                    using T_J  = XPBDMeshObject_<false, Sol::Jacobi,       typename Cfg::StableNeohookean::constraint_type_list>;
                                    using T_PJ = XPBDMeshObject_<false, Sol::ParallelJacobi,typename Cfg::StableNeohookean::constraint_type_list>;
                                    if (!added) added = try_add_for(dynamic_cast<T_GS*>(base_ptr), "2nd + NonCombined + GS");
                                    if (!added) added = try_add_for(dynamic_cast<T_J *>(base_ptr), "2nd + NonCombined + Jacobi");
                                    if (!added) added = try_add_for(dynamic_cast<T_PJ*>(base_ptr), "2nd + NonCombined + ParallelJacobi");
                                }

                                // ===== 2nd-order + Stable-Neohookean-Combined =====
                                {
                                    using Cfg = XPBDMeshObjectConstraintConfigurations<false>;
                                    using Sol = XPBDObjectSolverTypes<false, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                                    using T_GS = XPBDMeshObject_<false, Sol::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                    using T_J  = XPBDMeshObject_<false, Sol::Jacobi,       typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                    using T_PJ = XPBDMeshObject_<false, Sol::ParallelJacobi,typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                    if (!added) added = try_add_for(dynamic_cast<T_GS*>(base_ptr), "2nd + Combined + GS");
                                    if (!added) added = try_add_for(dynamic_cast<T_J *>(base_ptr), "2nd + Combined + Jacobi");
                                    if (!added) added = try_add_for(dynamic_cast<T_PJ*>(base_ptr), "2nd + Combined + ParallelJacobi");
                                }

                                // ===== 1st-order + Stable-Neohookean (Non-Combined) =====
                                {
                                    using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                                    using Sol1 = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookean::projector_type_list>;
                                    using A_GS = XPBDMeshObject_<true, Sol1::GaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
                                    using A_J  = XPBDMeshObject_<true, Sol1::Jacobi,       typename Cfg::StableNeohookean::constraint_type_list>;
                                    using A_PJ = XPBDMeshObject_<true, Sol1::ParallelJacobi,typename Cfg::StableNeohookean::constraint_type_list>;
                                    if (!added) added = try_add_for(dynamic_cast<A_GS*>(base_ptr), "1st + NonCombined + GS");
                                    if (!added) added = try_add_for(dynamic_cast<A_J *>(base_ptr), "1st + NonCombined + Jacobi");
                                    if (!added) added = try_add_for(dynamic_cast<A_PJ*>(base_ptr), "1st + NonCombined + ParallelJacobi");
                                }

                                // ===== 1st-order + Stable-Neohookean-Combined =====
                                {
                                    using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                                    using Sol2 = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                                    using B_GS = XPBDMeshObject_<true, Sol2::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                    using B_J  = XPBDMeshObject_<true, Sol2::Jacobi,       typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                    using B_PJ = XPBDMeshObject_<true, Sol2::ParallelJacobi,typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                    if (!added) added = try_add_for(dynamic_cast<B_GS*>(base_ptr), "1st + Combined + GS");
                                    if (!added) added = try_add_for(dynamic_cast<B_J *>(base_ptr), "1st + Combined + Jacobi");
                                    if (!added) added = try_add_for(dynamic_cast<B_PJ*>(base_ptr), "1st + Combined + ParallelJacobi");
                                }

                                if (added) { added_any = true; break; }
                            }

                            // If no 2nd-order objects worked, try first-order objects
                            if (!added_any) {
                                for (auto& fo_uptr : fo_xpbd_objs) {
                                    FirstOrderXPBDMeshObject_Base* fo_base_ptr = fo_uptr.get();

                                    bool added = false;

                                    // ===== 1st-order + Stable-Neohookean (Non-Combined) =====
                                    {
                                        using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                                        using Sol = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookean::projector_type_list>;
                                        using A_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
                                        using A_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::StableNeohookean::constraint_type_list>;
                                        using A_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::StableNeohookean::constraint_type_list>;
                                        if (!added) added = try_add_for(dynamic_cast<A_GS*>(fo_base_ptr), "1st + NonCombined + GS");
                                        if (!added) added = try_add_for(dynamic_cast<A_J *>(fo_base_ptr), "1st + NonCombined + Jacobi");
                                        if (!added) added = try_add_for(dynamic_cast<A_PJ*>(fo_base_ptr), "1st + NonCombined + ParallelJacobi");
                                    }

                                    // ===== 1st-order + Stable-Neohookean-Combined =====
                                    {
                                        using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                                        using Sol = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                                        using B_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                        using B_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                        using B_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                        if (!added) added = try_add_for(dynamic_cast<B_GS*>(fo_base_ptr), "1st + Combined + GS");
                                        if (!added) added = try_add_for(dynamic_cast<B_J *>(fo_base_ptr), "1st + Combined + Jacobi");
                                        if (!added) added = try_add_for(dynamic_cast<B_PJ*>(fo_base_ptr), "1st + Combined + ParallelJacobi");
                                    }

                                    if (added) { added_any = true; break; }
                                }
                            }

                            if (!added_any) {
                                std::cout << "[nerve] WARNING: no XPBD template combination matched; constraints not added.\n"
                                             "         Check your YAML (type, solver-type, constraint-type).\n";
                            } else if (s_edge_initialized) {
                                std::cout << "[nerve] Monitor edge set to (" << s_edge_i << "," << s_edge_j
                                          << "), rest_len=" << s_edge_rest_len << "\n";
                            }
                        }
                    }
                }

                // clear the temporary model
                gmsh::clear();
            } catch (std::exception& e) {
                std::cout << "[nerve] Exception while reading '" << msh_path << "': " << e.what() << "\n";
                // try to leave gmsh in a clean state
                try { gmsh::clear(); } catch (...) {}
            }
        }
    }
    // ================== END ==================

    // logger
    if (_logger)
    {
        _logger->addOutput("time [s]", &_time);
    }
}

void Simulation::update()
{
    if (_logger)
        _logger->startLogging();

    auto start = std::chrono::steady_clock::now();
    _wall_time_start = std::chrono::steady_clock::now();
    auto last_redraw = std::chrono::steady_clock::now();

    while (_time < _end_time)
    {
        Real wall_time_elapsed_s =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - _wall_time_start)
                .count() /
            1000000000.0;

        // callbacks
        for (auto& cb : _callbacks)
        {
            if (wall_time_elapsed_s > cb.next_exec_time)
            {
                cb.callback();
                cb.next_exec_time = cb.next_exec_time + cb.interval;
            }
        }

        // real-time block
        if (_sim_mode == Config::SimulationMode::VISUALIZATION && _time > wall_time_elapsed_s)
        {
            continue;
        }

        _timeStep();

        auto time_since_last_redraw_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - last_redraw)
                .count();

        if (time_since_last_redraw_ms > _viewer_refresh_time)
        {
            _updateGraphics();
            last_redraw = std::chrono::steady_clock::now();
        }
    }

    _updateGraphics();

    auto end = std::chrono::steady_clock::now();
    std::cout << "Simulating " << _end_time << " seconds took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << " ms" << std::endl;
}

void Simulation::_timeStep()
{
    // —— refresh collision constraints —— //
    if (_time - _last_collision_detection_time > _time_between_collision_checks)
    {
        auto& xpbd_mesh_objs = _objects.get<std::unique_ptr<XPBDMeshObject_Base>>();
        for (auto& obj : xpbd_mesh_objs) obj->clearCollisionConstraints();

        auto& fo_xpbd_mesh_objs = _objects.get<std::unique_ptr<FirstOrderXPBDMeshObject_Base>>();
        for (auto& obj : fo_xpbd_mesh_objs) obj->clearCollisionConstraints();

        auto& virtuoso_robots = _objects.get<std::unique_ptr<VirtuosoRobot>>();
        for (auto& obj : virtuoso_robots)
        {
            if (obj->hasArm1()) obj->arm1()->clearCollisionConstraints();
            if (obj->hasArm2()) obj->arm2()->clearCollisionConstraints();
        }

        auto& virtuoso_arms = _objects.get<std::unique_ptr<VirtuosoArm>>();
        for (auto& obj : virtuoso_arms) obj->clearCollisionConstraints();

        _collision_scene->collideObjects();
    }

    // —— PRE: read current length of the picked edge —— //
    if (s_edge_initialized)
    {
        auto read_and_print = [&](auto* xpbd, const char* tag, const char* phase){
            if (!xpbd) return false;
            const auto& V = xpbd->mesh()->vertices();
            if (s_edge_i < 0 || s_edge_j < 0 ||
                s_edge_i >= xpbd->mesh()->numVertices() ||
                s_edge_j >= xpbd->mesh()->numVertices()) return false;
            const Real len = (V.col(s_edge_i) - V.col(s_edge_j)).norm();
            
            // Only print every 900 steps to avoid flooding the terminal
            if (s_print_counter % 900 == 0) {
                std::cout << "[" << phase << "](" << tag << ") step=" << s_print_counter 
                          << " edge(" << s_edge_i << "," << s_edge_j << ") len = "
                          << len << " (rest = " << s_edge_rest_len << ")\n";
            }
            return true;
        };

        auto& xpbd_objs = _objects.get<std::unique_ptr<XPBDMeshObject_Base>>();
        bool printed = false;
        for (auto& uptr : xpbd_objs) {
            auto* base_ptr = uptr.get();

            // 2nd + NonCombined
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<false>;
                using Sol = XPBDObjectSolverTypes<false, typename Cfg::StableNeohookean::projector_type_list>;
                using T_GS = XPBDMeshObject_<false, Sol::GaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
                using T_J  = XPBDMeshObject_<false, Sol::Jacobi,       typename Cfg::StableNeohookean::constraint_type_list>;
                using T_PJ = XPBDMeshObject_<false, Sol::ParallelJacobi,typename Cfg::StableNeohookean::constraint_type_list>;
                if (!printed) printed = read_and_print(dynamic_cast<T_GS*>(base_ptr), "2nd+NonCombined+GS", "pre");
                if (!printed) printed = read_and_print(dynamic_cast<T_J *>(base_ptr), "2nd+NonCombined+Jacobi", "pre");
                if (!printed) printed = read_and_print(dynamic_cast<T_PJ*>(base_ptr), "2nd+NonCombined+PJacobi", "pre");
            }
            // 2nd + Combined
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<false>;
                using Sol = XPBDObjectSolverTypes<false, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                using T_GS = XPBDMeshObject_<false, Sol::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                using T_J  = XPBDMeshObject_<false, Sol::Jacobi,       typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                using T_PJ = XPBDMeshObject_<false, Sol::ParallelJacobi,typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                if (!printed) printed = read_and_print(dynamic_cast<T_GS*>(base_ptr), "2nd+Combined+GS", "pre");
                if (!printed) printed = read_and_print(dynamic_cast<T_J *>(base_ptr), "2nd+Combined+Jacobi", "pre");
                if (!printed) printed = read_and_print(dynamic_cast<T_PJ*>(base_ptr), "2nd+Combined+PJacobi", "pre");
            }
            // 1st + NonCombined
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookean::projector_type_list>;
                using A_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
                using A_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::StableNeohookean::constraint_type_list>;
                using A_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::StableNeohookean::constraint_type_list>;
                if (!printed) printed = read_and_print(dynamic_cast<A_GS*>(base_ptr), "1st+NonCombined+GS", "pre");
                if (!printed) printed = read_and_print(dynamic_cast<A_J *>(base_ptr), "1st+NonCombined+Jacobi", "pre");
                if (!printed) printed = read_and_print(dynamic_cast<A_PJ*>(base_ptr), "1st+NonCombined+PJacobi", "pre");
            }
            // 1st + Combined
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                using B_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                using B_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                using B_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                if (!printed) printed = read_and_print(dynamic_cast<B_GS*>(base_ptr), "1st+Combined+GS", "pre");
                if (!printed) printed = read_and_print(dynamic_cast<B_J *>(base_ptr), "1st+Combined+Jacobi", "pre");
                if (!printed) printed = read_and_print(dynamic_cast<B_PJ *>(base_ptr), "1st+Combined+PJacobi", "pre");
            }

            if (printed) break;
        }

        // If no 2nd-order objects printed, try first-order objects
        if (!printed) {
            auto& fo_xpbd_objs = _objects.get<std::unique_ptr<FirstOrderXPBDMeshObject_Base>>();
            for (auto& fo_uptr : fo_xpbd_objs) {
                auto* fo_base_ptr = fo_uptr.get();

                // 1st + NonCombined
                {
                    using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                    using Sol = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookean::projector_type_list>;
                    using A_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
                    using A_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::StableNeohookean::constraint_type_list>;
                    using A_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::StableNeohookean::constraint_type_list>;
                    if (!printed) printed = read_and_print(dynamic_cast<A_GS*>(fo_base_ptr), "1st+NonCombined+GS", "pre");
                    if (!printed) printed = read_and_print(dynamic_cast<A_J *>(fo_base_ptr), "1st+NonCombined+Jacobi", "pre");
                    if (!printed) printed = read_and_print(dynamic_cast<A_PJ*>(fo_base_ptr), "1st+NonCombined+PJacobi", "pre");
                }
                // 1st + Combined
                {
                    using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                    using Sol = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                    using B_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                    using B_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                    using B_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                    if (!printed) printed = read_and_print(dynamic_cast<B_GS*>(fo_base_ptr), "1st+Combined+GS", "pre");
                    if (!printed) printed = read_and_print(dynamic_cast<B_J *>(fo_base_ptr), "1st+Combined+Jacobi", "pre");
                    if (!printed) printed = read_and_print(dynamic_cast<B_PJ*>(fo_base_ptr), "1st+Combined+PJacobi", "pre");
                }

                if (printed) break;
            }
        }

        static bool warned_pre = false;
        if (!printed && !warned_pre) {
            std::cout << "[pre] WARNING: s_edge_initialized=true but couldn't read vertices; "
                         "template combo at runtime didn't match. Check setup prints."
                      << std::endl;
            warned_pre = true;
        }
    }

    // —— Run one XPBD step (objects do elasticity + collisions + your stretch) —— //
    _objects.for_each_element([](auto& obj) { obj->update(); });

    // —— POST: read again and print error —— //
    if (s_edge_initialized)
    {
        auto read_and_print_post = [&](auto* xpbd, const char* tag){
            if (!xpbd) return false;
            const auto& V = xpbd->mesh()->vertices();
            if (s_edge_i < 0 || s_edge_j < 0 ||
                s_edge_i >= xpbd->mesh()->numVertices() ||
                s_edge_j >= xpbd->mesh()->numVertices()) return false;
            const Real len = (V.col(s_edge_i) - V.col(s_edge_j)).norm();
            const Real err = std::abs(len - s_edge_rest_len);
            
            // Only print every 300 steps to avoid flooding the terminal
            if (s_print_counter % 900 == 0) {
                std::cout << "[post](" << tag << ") step=" << s_print_counter 
                          << " edge(" << s_edge_i << "," << s_edge_j << ") len = "
                          << len << "  |len-rest| = " << err << "\n";
            }
            return true;
        };

        auto& xpbd_objs = _objects.get<std::unique_ptr<XPBDMeshObject_Base>>();
        bool printed = false;
        for (auto& uptr : xpbd_objs) {
            auto* base_ptr = uptr.get();

            // 2nd + NonCombined
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<false>;
                using Sol = XPBDObjectSolverTypes<false, typename Cfg::StableNeohookean::projector_type_list>;
                using T_GS = XPBDMeshObject_<false, Sol::GaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
                using T_J  = XPBDMeshObject_<false, Sol::Jacobi,       typename Cfg::StableNeohookean::constraint_type_list>;
                using T_PJ = XPBDMeshObject_<false, Sol::ParallelJacobi,typename Cfg::StableNeohookean::constraint_type_list>;
                if (!printed) printed = read_and_print_post(dynamic_cast<T_GS*>(base_ptr), "2nd+NonCombined+GS");
                if (!printed) printed = read_and_print_post(dynamic_cast<T_J *>(base_ptr), "2nd+NonCombined+Jacobi");
                if (!printed) printed = read_and_print_post(dynamic_cast<T_PJ*>(base_ptr), "2nd+NonCombined+PJacobi");
            }
            // 2nd + Combined
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<false>;
                using Sol = XPBDObjectSolverTypes<false, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                using T_GS = XPBDMeshObject_<false, Sol::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                using T_J  = XPBDMeshObject_<false, Sol::Jacobi,       typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                using T_PJ = XPBDMeshObject_<false, Sol::ParallelJacobi,typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                if (!printed) printed = read_and_print_post(dynamic_cast<T_GS*>(base_ptr), "2nd+Combined+GS");
                if (!printed) printed = read_and_print_post(dynamic_cast<T_J *>(base_ptr), "2nd+Combined+Jacobi");
                if (!printed) printed = read_and_print_post(dynamic_cast<T_PJ*>(base_ptr), "2nd+Combined+PJacobi");
            }
            // 1st + NonCombined
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookean::projector_type_list>;
                using A_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
                using A_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::StableNeohookean::constraint_type_list>;
                using A_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::StableNeohookean::constraint_type_list>;
                if (!printed) printed = read_and_print_post(dynamic_cast<A_GS*>(base_ptr), "1st+NonCombined+GS");
                if (!printed) printed = read_and_print_post(dynamic_cast<A_J *>(base_ptr), "1st+NonCombined+Jacobi");
                if (!printed) printed = read_and_print_post(dynamic_cast<A_PJ*>(base_ptr), "1st+NonCombined+PJacobi");
            }
            // 1st + Combined
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                using B_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                using B_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                using B_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                if (!printed) printed = read_and_print_post(dynamic_cast<B_GS*>(base_ptr), "1st+Combined+GS");
                if (!printed) printed = read_and_print_post(dynamic_cast<B_J *>(base_ptr), "1st+Combined+Jacobi");
                if (!printed) printed = read_and_print_post(dynamic_cast<B_PJ *>(base_ptr), "1st+Combined+PJacobi");
            }

            if (printed) break;
        }

        // If no 2nd-order objects printed, try first-order objects
        if (!printed) {
            auto& fo_xpbd_objs = _objects.get<std::unique_ptr<FirstOrderXPBDMeshObject_Base>>();
            for (auto& fo_uptr : fo_xpbd_objs) {
                auto* fo_base_ptr = fo_uptr.get();

                // 1st + NonCombined
                {
                    using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                    using Sol = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookean::projector_type_list>;
                    using A_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
                    using A_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::StableNeohookean::constraint_type_list>;
                    using A_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::StableNeohookean::constraint_type_list>;
                    if (!printed) printed = read_and_print_post(dynamic_cast<A_GS*>(fo_base_ptr), "1st+NonCombined+GS");
                    if (!printed) printed = read_and_print_post(dynamic_cast<A_J *>(fo_base_ptr), "1st+NonCombined+Jacobi");
                    if (!printed) printed = read_and_print_post(dynamic_cast<A_PJ*>(fo_base_ptr), "1st+NonCombined+PJacobi");
                }
                // 1st + Combined
                {
                    using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                    using Sol = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                    using B_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                    using B_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                    using B_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                    if (!printed) printed = read_and_print_post(dynamic_cast<B_GS*>(fo_base_ptr), "1st+Combined+GS");
                    if (!printed) printed = read_and_print_post(dynamic_cast<B_J *>(fo_base_ptr), "1st+Combined+Jacobi");
                    if (!printed) printed = read_and_print_post(dynamic_cast<B_PJ*>(fo_base_ptr), "1st+Combined+PJacobi");
                }

                if (printed) break;
            }
        }

        static bool warned_post = false;
        if (!printed && !warned_post) {
            std::cout << "[post] WARNING: s_edge_initialized=true but couldn't read vertices; "
                         "template combo at runtime didn't match. Check setup prints."
                      << std::endl;
            warned_post = true;
        }
    }

    // —— velocity update —— //
    _objects.for_each_element([](auto& obj) { obj->velocityUpdate(); });

    // —— collision timestamp —— //
    if (_time - _last_collision_detection_time > _time_between_collision_checks)
    {
        _last_collision_detection_time = _time;
    }

    // —— logging —— //
    if (_logger) _logger->logToFile();

    // —— advance time —— //
    _time += _time_step;
    
    // —— increment print counter —— //
    s_print_counter++;
}

void Simulation::_updateGraphics()
{
    if (_graphics_scene)
    {
        _graphics_scene->update();
        _graphics_scene->viewer()->editText("time", "Sim Time: " + std::to_string(_time) + " s");
    }
}

void Simulation::notifyKeyPressed(SimulationInput::Key /* key */, SimulationInput::KeyAction action, int /* modifiers */)
{
    if (_sim_mode == Config::SimulationMode::FRAME_BY_FRAME && action == SimulationInput::KeyAction::PRESS)
    {
        _timeStep();
        _updateGraphics();
    }
}

void Simulation::notifyMouseButtonPressed(SimulationInput::MouseButton /* button */, SimulationInput::MouseAction /* action */, int /* modifiers */)
{
    // do nothing
}

void Simulation::notifyMouseMoved(double /* x */, double /* y */)
{
    // do nothing
}

void Simulation::notifyMouseScrolled(double /* dx */, double /* dy */)
{
    // do nothing
}

int Simulation::run()
{
    if (!_setup)
        setup();

    std::thread update_thread;
    if (_sim_mode != Config::SimulationMode::FRAME_BY_FRAME)
    {
        update_thread = std::thread(&Simulation::update, this);
    }

    if (_graphics_scene)
    {
        _graphics_scene->run();
        return 0;
    }
    else
    {
        update_thread.join();
        return 0;
    }
}

} // namespace Sim
