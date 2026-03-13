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
#include "common/XPBDEnumTypes.hpp"
#include "geometry/TetMesh.hpp"

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

// Helper function: Compute point-to-triangle distance for adhesion constraint creation
// Simplified version of NerveTumorAdhesionConstraint::computePointTriangleDistance
static Real computePointTriangleDistance(const Vec3r& nerve_pos,
                                        const Vec3r& tri_p1, 
                                        const Vec3r& tri_p2,
                                        const Vec3r& tri_p3,
                                        Vec3r& closest_point,
                                        Vec3r& normal,
                                        Vec3r& bary_coords)
{
    // Compute triangle normal and area
    const Vec3r edge1 = tri_p2 - tri_p1;
    const Vec3r edge2 = tri_p3 - tri_p1;
    const Vec3r triangle_normal = edge1.cross(edge2);
    const Real area = triangle_normal.norm();
    
    if (area < 1e-12) {
        // Degenerate triangle - return large distance
        normal = Vec3r::UnitZ(); // arbitrary normal
        closest_point = tri_p1; // arbitrary point on triangle
        bary_coords = Vec3r(1.0, 0.0, 0.0); // all weight on first vertex
        return 1e6; // large distance to indicate invalid
    }

    normal = triangle_normal / area;
    
    // Compute plane signed distance  
    Real signed_distance = (nerve_pos - tri_p1).dot(normal);
    
    // Flip normal to point toward nerve if needed (orientation-invariant)
    if (signed_distance < 0) {
        normal = -normal;
        signed_distance = -signed_distance;
    }
    
    // Project point onto triangle plane (with corrected normal)
    const Vec3r projected_point = nerve_pos - signed_distance * normal;
    
    // Compute barycentric coordinates of projected point
    const Vec3r v0 = edge2;
    const Vec3r v1 = edge1;  
    const Vec3r v2 = projected_point - tri_p1;
    
    const Real dot00 = v0.dot(v0);
    const Real dot01 = v0.dot(v1);
    const Real dot02 = v0.dot(v2);
    const Real dot11 = v1.dot(v1);
    const Real dot12 = v1.dot(v2);
    
    const Real inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    const Real u = (dot11 * dot02 - dot01 * dot12) * inv_denom; // weight for tri_p3
    const Real v = (dot00 * dot12 - dot01 * dot02) * inv_denom; // weight for tri_p2
    const Real w = 1.0 - u - v; // weight for tri_p1
    
    bary_coords = Vec3r(w, v, u);
    
    // Check if point is inside triangle
    if (u >= 0.0 && v >= 0.0 && (u + v) <= 1.0) {
        // Point projects inside triangle
        closest_point = projected_point;
        return signed_distance; // Always non-negative due to normal flip
    } else {
        // Point projects outside triangle - clamp to triangle boundary
        Real u_clamp = std::max(0.0, std::min(1.0, u));
        Real v_clamp = std::max(0.0, std::min(1.0, v));
        if (u_clamp + v_clamp > 1.0) {
            const Real scale = 1.0 / (u_clamp + v_clamp);
            u_clamp *= scale;
            v_clamp *= scale;
        }
        const Real w_clamp = 1.0 - u_clamp - v_clamp;
        
        bary_coords = Vec3r(w_clamp, v_clamp, u_clamp);
        closest_point = w_clamp * tri_p1 + v_clamp * tri_p2 + u_clamp * tri_p3;
        
        // For outside points: return actual distance to closest point on triangle
        return (nerve_pos - closest_point).norm();
    }
}

// === Static cache for the picked edge we’ll monitor each frame ===
static bool  s_edge_initialized = false;
static int   s_edge_i = -1;
static int   s_edge_j = -1;
static Real  s_edge_rest_len = 0.0;
static int   s_print_counter = 0;  // Counter for controlling print frequency

// === Static cache for the picked triplet (bending) we'll monitor each frame ===
static bool  s_triplet_initialized = false;
static int   s_triplet_i = -1;  // first vertex
static int   s_triplet_j = -1;  // middle vertex
static int   s_triplet_k = -1;  // third vertex
static Real  s_triplet_rest_curvature = 0.0;

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

    // initialize the state recorder
    if (_config->stateRecordingEnable())
    {
        _state_recorder = std::make_unique<SimulationStateRecorder>(
            _config->stateRecordingOutputFolder(),
            _config->stateRecordingSnapshotInterval()
        );
        std::cout << "[Simulation] State recording enabled - snapshots will be saved to: "
                  << _config->stateRecordingOutputFolder() << std::endl;
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
    ss << indent_str << "Gravity: [" << _g_accel[0] << ", " << _g_accel[1] << ", " << _g_accel[2] << "] m/s2" << std::endl;
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
        // std::cout << "[DEBUG] *** ENTERING NERVE SECTION *** (line 1263)\n" << std::flush;
        
        // Read nerve configuration from YAML config instead of environment variables
        // std::cout << "[DEBUG] About to read nerve config from YAML...\n" << std::flush;
        const bool nerve_enabled = _config->nerveEnable();
        const bool nerve_stretch_enabled = _config->nerveStretchEnable();
        const bool nerve_bending_enabled = _config->nerveBendingEnable();
        const std::string msh_path = _config->nerveMeshFile();
        const std::string phys_name = _config->nervePhysicalGroup();
        
        std::cout << "[DEBUG] Nerve config: enabled=" << nerve_enabled 
                  << " stretch=" << nerve_stretch_enabled << " bending=" << nerve_bending_enabled 
                  << " mesh='" << msh_path << "' group='" << phys_name << "'\n";

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

                        // —— Setup triplet monitoring (even if no bending constraints) —— //
                        if (!final_bending_enabled && !s_triplet_initialized) {
                            try {
                                // Set up monitoring on first available triplet (no bending constraints added)
                                auto setup_triplet_monitor = [&](auto* xpbd, const char* tag)->bool {
                                    if (!xpbd) return false;
                                    const auto& mesh = xpbd->mesh();
                                    if (mesh->numVertices() < 3) return false;
                                    
                                    // Find first valid triplet (3 consecutive vertices)
                                    for (int i = 0; i < mesh->numVertices() - 2; ++i) {
                                        int j = i + 1;
                                        int k = i + 2;
                                        
                                        const auto& V = mesh->vertices();
                                        const auto p0 = V.col(i);
                                        const auto p1 = V.col(j);
                                        const auto p2 = V.col(k);
                                        
                                        const auto e1 = p1 - p0;
                                        const auto e2 = p2 - p1;
                                        
                                        const auto e1_norm = e1.norm();
                                        const auto e2_norm = e2.norm();
                                        
                                        // Only use non-degenerate triplets
                                        if (e1_norm > 1e-12 && e2_norm > 1e-12) {
                                            const auto cross = e1.cross(e2);
                                            const auto rest_curvature = 2.0 * cross.norm() / (e1_norm * e2_norm * (e1_norm + e2_norm));
                                            
                                            s_triplet_initialized = true;
                                            s_triplet_i = i; s_triplet_j = j; s_triplet_k = k;
                                            s_triplet_rest_curvature = rest_curvature;
                                            std::cout << "[monitor] NO BENDING CONSTRAINTS, but monitoring triplet (" 
                                                      << i << "," << j << "," << k << "), rest_curvature=" << rest_curvature << "\n";
                                            return true;
                                        }
                                    }
                                    return false;
                                };
                                
                                // Try to set up triplet monitoring on any available object
                                bool triplet_monitor_set = false;
                                for (auto& uptr : xpbd_objs) {
                                    XPBDMeshObject_Base* base_ptr = uptr.get();
                                    
                                    // Try all common template combinations for triplet monitoring
                                    // 2nd + NonCombined
                                    {
                                        using Cfg = XPBDMeshObjectConstraintConfigurations<false>;
                                        using Sol = XPBDObjectSolverTypes<false, typename Cfg::StableNeohookean::projector_type_list>;
                                        using T_GS = XPBDMeshObject_<false, Sol::GaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
                                        using T_J  = XPBDMeshObject_<false, Sol::Jacobi,       typename Cfg::StableNeohookean::constraint_type_list>;
                                        using T_PJ = XPBDMeshObject_<false, Sol::ParallelJacobi,typename Cfg::StableNeohookean::constraint_type_list>;
                                        if (!triplet_monitor_set) triplet_monitor_set = setup_triplet_monitor(dynamic_cast<T_GS*>(base_ptr), "2nd+NonCombined+GS");
                                        if (!triplet_monitor_set) triplet_monitor_set = setup_triplet_monitor(dynamic_cast<T_J *>(base_ptr), "2nd+NonCombined+Jacobi");
                                        if (!triplet_monitor_set) triplet_monitor_set = setup_triplet_monitor(dynamic_cast<T_PJ*>(base_ptr), "2nd+NonCombined+PJ");
                                    }
                                    // 2nd + Combined
                                    {
                                        using Cfg = XPBDMeshObjectConstraintConfigurations<false>;
                                        using Sol = XPBDObjectSolverTypes<false, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                                        using T_GS = XPBDMeshObject_<false, Sol::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                        using T_J  = XPBDMeshObject_<false, Sol::Jacobi,       typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                        using T_PJ = XPBDMeshObject_<false, Sol::ParallelJacobi,typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                        if (!triplet_monitor_set) triplet_monitor_set = setup_triplet_monitor(dynamic_cast<T_GS*>(base_ptr), "2nd+Combined+GS");
                                        if (!triplet_monitor_set) triplet_monitor_set = setup_triplet_monitor(dynamic_cast<T_J *>(base_ptr), "2nd+Combined+Jacobi");
                                        if (!triplet_monitor_set) triplet_monitor_set = setup_triplet_monitor(dynamic_cast<T_PJ*>(base_ptr), "2nd+Combined+PJ");
                                    }
                                    if (triplet_monitor_set) break;
                                }

                                // If no 2nd-order objects worked, try first-order objects  
                                if (!triplet_monitor_set) {
                                    auto& fo_xpbd_objs = _objects.get<std::unique_ptr<FirstOrderXPBDMeshObject_Base>>();
                                    for (auto& fo_uptr : fo_xpbd_objs) {
                                        FirstOrderXPBDMeshObject_Base* fo_base_ptr = fo_uptr.get();
                                        
                                        // 1st + NonCombined
                                        {
                                            using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                                            using Sol = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookean::projector_type_list>;
                                            using A_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
                                            using A_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::StableNeohookean::constraint_type_list>;
                                            using A_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::StableNeohookean::constraint_type_list>;
                                            if (!triplet_monitor_set) triplet_monitor_set = setup_triplet_monitor(dynamic_cast<A_GS*>(fo_base_ptr), "1st+NonCombined+GS");
                                            if (!triplet_monitor_set) triplet_monitor_set = setup_triplet_monitor(dynamic_cast<A_J *>(fo_base_ptr), "1st+NonCombined+Jacobi");
                                            if (!triplet_monitor_set) triplet_monitor_set = setup_triplet_monitor(dynamic_cast<A_PJ*>(fo_base_ptr), "1st+NonCombined+PJ");
                                        }
                                        // 1st + Combined
                                        {
                                            using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                                            using Sol = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                                            using B_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                            using B_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                            using B_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                            if (!triplet_monitor_set) triplet_monitor_set = setup_triplet_monitor(dynamic_cast<B_GS*>(fo_base_ptr), "1st+Combined+GS");
                                            if (!triplet_monitor_set) triplet_monitor_set = setup_triplet_monitor(dynamic_cast<B_J *>(fo_base_ptr), "1st+Combined+Jacobi");
                                            if (!triplet_monitor_set) triplet_monitor_set = setup_triplet_monitor(dynamic_cast<B_PJ*>(fo_base_ptr), "1st+Combined+PJ");
                                        }
                                        if (triplet_monitor_set) break;
                                    }
                                }
                                
                                if (!triplet_monitor_set) {
                                    std::cout << "[monitor] WARNING: Could not set up triplet monitoring for NERVE_BENDING_ENABLE=0 case.\n";
                                    std::cout << "[monitor] Template combination at runtime didn't match. Check YAML config.\n";
                                }
                            } catch (std::exception& e) {
                                std::cout << "[monitor] Exception while setting up triplet monitoring: " << e.what() << "\n";
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

                                    // Check constraint-type-specific nerve control flags
                                    const auto constraint_type = xpbd->constraintType();
                                    bool object_stretch_enabled = final_stretch_enabled;
                                    bool object_bending_enabled = final_bending_enabled;
                                    
                                    // Apply constraint-type-specific overrides
                                    if (constraint_type == XPBDMeshObjectConstraintConfigurationEnum::STABLE_NEOHOOKEAN) {
                                        object_stretch_enabled = object_stretch_enabled && _config->stableNeohookeanNerveStretchEnable();
                                        object_bending_enabled = object_bending_enabled && _config->stableNeohookeanNerveBendingEnable();
                                    } else if (constraint_type == XPBDMeshObjectConstraintConfigurationEnum::STABLE_NEOHOOKEAN_COMBINED) {
                                        object_stretch_enabled = object_stretch_enabled && _config->stableNeohookeanCombinedNerveStretchEnable();
                                        object_bending_enabled = object_bending_enabled && _config->stableNeohookeanCombinedNerveBendingEnable();
                                    }
                                    // For NERVE_ONLY, use the global flags (no additional constraints)

                                    const auto& V = xpbd->mesh()->vertices();
                                    const int nV = xpbd->mesh()->numVertices();    // Get gmsh node tag -> internal vertex index map
                                    
                                    // DEBUG: Check if vertices are valid after getting them from xpbd mesh
                                    std::cout << "[nerve] DEBUG: XPBD mesh has nV=" << nV << ", V.cols()=" << V.cols() << ", V.rows()=" << V.rows() << std::endl;
                                    if (V.cols() > 0) {
                                        std::cout << "[nerve] DEBUG: V.col(0)=" << V.col(0).transpose() << std::endl;
                                        if (V.cols() > 1) {
                                            std::cout << "[nerve] DEBUG: V.col(1)=" << V.col(1).transpose() << std::endl;
                                        }
                                    }
                                    
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
                                    if (object_stretch_enabled) {
                                        for (const auto& seg : line_pairs) {
                                            auto it0 = tag2idx.find(seg.first);
                                            auto it1 = tag2idx.find(seg.second);
                                            if (it0 == tag2idx.end() || it1 == tag2idx.end()) { ++add_fail; continue; }

                                            const int i = it0->second;
                                            const int j = it1->second;
                                            if (i < 0 || j < 0 || i == j) { ++add_fail; continue; }

                                            const Real rest_len = (V.col(i) - V.col(j)).norm();
                                            // std::cerr << "[nerve] DEBUG: monitor edge (" << i << "," << j << "), V[" << i << "]=" << V.col(i).transpose() << ", V[" << j << "]=" << V.col(j).transpose() << ", rest_len=" << rest_len << "\n";
                                            
                                            // Read stretch alpha from config (smaller alpha = stiffer constraint)
                                            Real stretch_alpha = _config->nerveStretchAlpha();
                                            xpbd->addNerveStretchConstraint(i, j, rest_len, stretch_alpha);
                                            ++add_ok;

                                            if (!monitor_set) {
                                                s_edge_initialized = true;
                                                s_edge_i = i; s_edge_j = j; s_edge_rest_len = rest_len;
                                                std::cerr << "[nerve] DEBUG: setting monitor globals: s_edge_rest_len=" << s_edge_rest_len << "\n";
                                                monitor_set = true;
                                            }
                                        }
                                    } else {
                                        std::cout << "[nerve] Stretch constraints DISABLED for " << tag 
                                                  << " (object-specific or global nerve-stretch-enable=false)\n";
                                    }

                                    // Second pass: Add bending constraints for consecutive triplets (if enabled)
                                    if (object_bending_enabled) {
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
                                                // Read bending alpha from config (smaller alpha = stiffer constraint)
                                                Real bend_alpha = _config->nerveBendingAlpha();
                                                xpbd->addNerveBendingConstraint(triplet[0], triplet[1], triplet[2], 
                                                                              /*rest_curvature=*/0.0, bend_alpha);
                                                ++bend_ok;
                                                
                                                // Set up triplet monitoring (use first valid triplet)
                                                if (!s_triplet_initialized) {
                                                    s_triplet_initialized = true;
                                                    s_triplet_i = triplet[0];
                                                    s_triplet_j = triplet[1]; 
                                                    s_triplet_k = triplet[2];
                                                    s_triplet_rest_curvature = 0.0;
                                                }
                                            } catch (...) {
                                                ++bend_fail;
                                            }
                                        }
                                    } else {
                                        std::cout << "[nerve] Bending constraints DISABLED for " << tag 
                                                  << " (object-specific or global nerve-bending-enable=false)\n";
                                    }

                                    std::cout << "[nerve] addNerveStretchConstraint: ok=" << add_ok
                                              << "  fail=" << add_fail << "\n";
                                    std::cout << "[nerve] addNerveBendingConstraint: ok=" << bend_ok
                                              << "  fail=" << bend_fail << "\n";
                                    return (add_ok > 0) || (bend_ok > 0);
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

                                // ===== 2nd-order + Nerve-Only =====
                                {
                                    using Cfg = XPBDMeshObjectConstraintConfigurations<false>;
                                    using Sol = XPBDObjectSolverTypes<false, typename Cfg::NerveOnly::projector_type_list>;
                                    using N_GS = XPBDMeshObject_<false, Sol::GaussSeidel, typename Cfg::NerveOnly::constraint_type_list>;
                                    using N_J  = XPBDMeshObject_<false, Sol::Jacobi,       typename Cfg::NerveOnly::constraint_type_list>;
                                    using N_PJ = XPBDMeshObject_<false, Sol::ParallelJacobi,typename Cfg::NerveOnly::constraint_type_list>;
                                    if (!added) added = try_add_for(dynamic_cast<N_GS*>(base_ptr), "2nd + NerveOnly + GS");
                                    if (!added) added = try_add_for(dynamic_cast<N_J *>(base_ptr), "2nd + NerveOnly + Jacobi");
                                    if (!added) added = try_add_for(dynamic_cast<N_PJ*>(base_ptr), "2nd + NerveOnly + ParallelJacobi");
                                }

                                // ===== 1st-order + Stable-Neohookean (Non-Combined) =====
                                {
                                    using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                                    using Sol1 = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookean::projector_type_list>;
                                    using A_GS = XPBDMeshObject_<true, Sol1::GaussSeidel,        typename Cfg::StableNeohookean::constraint_type_list>;
                                    using A_J  = XPBDMeshObject_<true, Sol1::Jacobi,              typename Cfg::StableNeohookean::constraint_type_list>;
                                    using A_PJ = XPBDMeshObject_<true, Sol1::ParallelJacobi,      typename Cfg::StableNeohookean::constraint_type_list>;
                                    using A_CG = XPBDMeshObject_<true, Sol1::ColoredGaussSeidel,  typename Cfg::StableNeohookean::constraint_type_list>;
                                    if (!added) added = try_add_for(dynamic_cast<A_GS*>(base_ptr), "1st + NonCombined + GS");
                                    if (!added) added = try_add_for(dynamic_cast<A_J *>(base_ptr), "1st + NonCombined + Jacobi");
                                    if (!added) added = try_add_for(dynamic_cast<A_PJ*>(base_ptr), "1st + NonCombined + ParallelJacobi");
                                    if (!added) added = try_add_for(dynamic_cast<A_CG*>(base_ptr), "1st + NonCombined + ColoredGS");
                                }

                                // ===== 1st-order + Stable-Neohookean-Combined =====
                                {
                                    using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                                    using Sol2 = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                                    using B_GS = XPBDMeshObject_<true, Sol2::GaussSeidel,        typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                    using B_J  = XPBDMeshObject_<true, Sol2::Jacobi,              typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                    using B_PJ = XPBDMeshObject_<true, Sol2::ParallelJacobi,      typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                    using B_CG = XPBDMeshObject_<true, Sol2::ColoredGaussSeidel,  typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                    if (!added) added = try_add_for(dynamic_cast<B_GS*>(base_ptr), "1st + Combined + GS");
                                    if (!added) added = try_add_for(dynamic_cast<B_J *>(base_ptr), "1st + Combined + Jacobi");
                                    if (!added) added = try_add_for(dynamic_cast<B_PJ*>(base_ptr), "1st + Combined + ParallelJacobi");
                                    if (!added) added = try_add_for(dynamic_cast<B_CG*>(base_ptr), "1st + Combined + ColoredGS");
                                }

                                // ===== 1st-order + Nerve-Only =====
                                {
                                    using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                                    using Sol3 = XPBDObjectSolverTypes<true, typename Cfg::NerveOnly::projector_type_list>;
                                    using C_GS = XPBDMeshObject_<true, Sol3::GaussSeidel,        typename Cfg::NerveOnly::constraint_type_list>;
                                    using C_J  = XPBDMeshObject_<true, Sol3::Jacobi,              typename Cfg::NerveOnly::constraint_type_list>;
                                    using C_PJ = XPBDMeshObject_<true, Sol3::ParallelJacobi,      typename Cfg::NerveOnly::constraint_type_list>;
                                    using C_CG = XPBDMeshObject_<true, Sol3::ColoredGaussSeidel,  typename Cfg::NerveOnly::constraint_type_list>;
                                    if (!added) added = try_add_for(dynamic_cast<C_GS*>(base_ptr), "1st + NerveOnly + GS");
                                    if (!added) added = try_add_for(dynamic_cast<C_J *>(base_ptr), "1st + NerveOnly + Jacobi");
                                    if (!added) added = try_add_for(dynamic_cast<C_PJ*>(base_ptr), "1st + NerveOnly + ParallelJacobi");
                                    if (!added) added = try_add_for(dynamic_cast<C_CG*>(base_ptr), "1st + NerveOnly + ColoredGS");
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

                                    // ===== 1st-order + Nerve-Only (fallback) =====
                                    {
                                        using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                                        using Sol = XPBDObjectSolverTypes<true, typename Cfg::NerveOnly::projector_type_list>;
                                        using C_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::NerveOnly::constraint_type_list>;
                                        using C_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::NerveOnly::constraint_type_list>;
                                        using C_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::NerveOnly::constraint_type_list>;
                                        if (!added) added = try_add_for(dynamic_cast<C_GS*>(fo_base_ptr), "1st + NerveOnly + GS");
                                        if (!added) added = try_add_for(dynamic_cast<C_J *>(fo_base_ptr), "1st + NerveOnly + Jacobi");
                                        if (!added) added = try_add_for(dynamic_cast<C_PJ*>(fo_base_ptr), "1st + NerveOnly + ParallelJacobi");
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
                            if (s_triplet_initialized) {
                                std::cout << "[nerve] Monitor triplet set to (" << s_triplet_i << "," << s_triplet_j 
                                          << "," << s_triplet_k << "), rest_curvature=" << s_triplet_rest_curvature << "\n";
                            }
                            
                            std::cout << "[DEBUG] *** NERVE SECTION COMPLETED *** About to start adhesion section...\n";
                        }
                    }
                }

                // ==================== NERVE-TUMOR ADHESION CONSTRAINTS ====================
                
                std::cout << "[adhesion DEBUG] Checking if adhesion is enabled...\n";
                std::cout << "[adhesion DEBUG] nerveTumorAdhesionEnable() = " << _config->nerveTumorAdhesionEnable() << "\n";
                std::cout << "[adhesion DEBUG] Config object pointer: " << _config << "\n";
                
                // Check if nerve-tumor adhesion is enabled
                if (_config->nerveTumorAdhesionEnable()) {
                    std::cout << "[adhesion] *** NERVE-TUMOR ADHESION ENABLED *** Creating constraints...\n";
                    
                    // Get adhesion parameters from config
                    const Real target_gap = _config->nerveTumorAdhesionTargetGap();
                    const Real alpha = _config->nerveTumorAdhesionAlpha();
                    const Real distance_window = _config->nerveTumorAdhesionDistanceWindow();
                    
                    std::cout << "[adhesion] Parameters: target_gap=" << target_gap 
                              << ", alpha=" << alpha << ", distance_window=" << distance_window << "\n";
                    
                    // Cross-object approach: collect nerve (vertex-only) and tumor (face-containing) objects
                    int total_constraints = 0;
                    std::vector<FirstOrderXPBDMeshObject_Base*> nerve_objs;
                    std::vector<FirstOrderXPBDMeshObject_Base*> tumor_objs;

                    for (auto& fo_uptr : fo_xpbd_objs) {
                        FirstOrderXPBDMeshObject_Base* fo_base_ptr = fo_uptr.get();
                        if (!fo_base_ptr) continue;
                        const auto* mesh = fo_base_ptr->mesh();
                        const int num_vertices = mesh->numVertices();
                        const int num_faces = mesh->numFaces();
                        std::cout << "[adhesion] Processing object: vertices=" << num_vertices 
                                  << ", faces=" << num_faces << "\n";

                        if (num_vertices > 0 && num_faces == 0) {
                            nerve_objs.push_back(fo_base_ptr);
                        } else if (num_faces > 0) {
                            tumor_objs.push_back(fo_base_ptr);
                        }
                    }

                    // Try pairing each nerve object with each tumor object and create adhesion constraints
                    for (auto* nerve_ptr : nerve_objs) {
                        const auto* nerve_mesh = nerve_ptr->mesh();
                        const int nerve_nv = nerve_mesh->numVertices();
                        
                        // DEBUG: Print nerve actual positions
                        if (nerve_nv > 0) {
                            Vec3r nerve_first = nerve_mesh->vertex(0);
                            Vec3r nerve_last = nerve_mesh->vertex(nerve_nv - 1);
                            std::cout << "[adhesion DEBUG] Nerve mesh (" << nerve_nv << " vertices):\n";
                            std::cout << "  First vertex: " << nerve_first.transpose() << "\n";
                            std::cout << "  Last vertex:  " << nerve_last.transpose() << "\n";
                        }

                        for (auto* tumor_ptr : tumor_objs) {
                            const auto* tumor_mesh = tumor_ptr->mesh();
                            const int tumor_nf = tumor_mesh->numFaces();
                            
                            // DEBUG: Print tumor actual positions
                            if (tumor_mesh->numVertices() > 0) {
                                Vec3r tumor_vertex = tumor_mesh->vertex(0);
                                std::cout << "[adhesion DEBUG] Tumor mesh (" << tumor_nf << " faces, " 
                                          << tumor_mesh->numVertices() << " vertices):\n";
                                std::cout << "  First vertex: " << tumor_vertex.transpose() << "\n";
                            }

                            int constraints_added = 0;
                            Real min_distance = std::numeric_limits<Real>::max();
                            Real max_checked_distance = 0.0;
                            int distances_checked = 0;
                            
                            try {
                                // For tumor object, try several possible XPBD instantiations and call addNerveTumorAdhesionConstraint
                                bool tumor_cast_handled = false;

                                // Helper lambda: attempt to cast tumor_ptr to a concrete XPBD type and add constraints
                                auto try_add_on_tumor = [&](auto* typed_tumor_ptr)->bool {
                                    if (!typed_tumor_ptr) return false;
                                    
                                    // Performance optimization: Track culled triangles
                                    int triangles_culled_by_aabb = 0;
                                    
                                    // FIXED: One constraint per nerve vertex approach
                                    // Check ALL nerve vertices and ALL tumor faces for proper distance calculation
                                    for (int v = 0; v < nerve_nv; ++v) {
                                        const Vec3r nerve_pos = nerve_mesh->vertex(v);
                                        
                                        // Find the closest triangle to this nerve vertex using proper distance calculation
                                        Real closest_distance = std::numeric_limits<Real>::max();
                                        int closest_face = -1;
                                        int closest_v1 = -1, closest_v2 = -1, closest_v3 = -1;
                                        
                                        // Check ALL tumor faces to find the globally closest triangle
                                        for (int f = 0; f < tumor_nf; ++f) {
                                            const auto face = tumor_mesh->face(f);
                                            const int v1 = face[0], v2 = face[1], v3 = face[2];
                                            // Skip if nerve vertex equals a face vertex (unlikely across different objects, but safe)
                                            if (v == v1 || v == v2 || v == v3) continue;
                                            
                                            // PERFORMANCE: Get triangle vertices
                                            const Vec3r tri_p1 = tumor_mesh->vertex(v1);
                                            const Vec3r tri_p2 = tumor_mesh->vertex(v2);
                                            const Vec3r tri_p3 = tumor_mesh->vertex(v3);
                                            
                                            // PERFORMANCE OPTIMIZATION: Early rejection using triangle AABB
                                            // Compute triangle bounding box
                                            const Vec3r tri_min = tri_p1.cwiseMin(tri_p2).cwiseMin(tri_p3);
                                            const Vec3r tri_max = tri_p1.cwiseMax(tri_p2).cwiseMax(tri_p3);
                                            
                                            // Compute distance from nerve point to AABB
                                            Vec3r closest_aabb_point = nerve_pos.cwiseMax(tri_min).cwiseMin(tri_max);
                                            Real aabb_distance = (nerve_pos - closest_aabb_point).norm();
                                            
                                            // Skip this triangle if AABB is too far (conservative early rejection)
                                            if (aabb_distance > distance_window) {
                                                ++triangles_culled_by_aabb;
                                                continue;
                                            }
                                            
                                            // Compute actual point-to-triangle distance (simplified version)
                                            Vec3r closest_point, normal, bary_coords;
                                            Real distance = computePointTriangleDistance(nerve_pos, tri_p1, tri_p2, tri_p3, 
                                                                                       closest_point, normal, bary_coords);
                                            
                                            // Track distance statistics using ACTUAL point-to-triangle distance
                                            min_distance = std::min(min_distance, distance);
                                            max_checked_distance = std::max(max_checked_distance, distance);
                                            distances_checked++;
                                            
                                            // Check if this is the closest triangle so far (use actual distance for constraint selection)
                                            if (distance < closest_distance && distance <= distance_window) {
                                                closest_distance = distance;
                                                closest_face = f;
                                                closest_v1 = v1;
                                                closest_v2 = v2;
                                                closest_v3 = v3;
                                            }
                                        }
                                        
                                        // Only create constraint for the closest triangle (if within window)
                                        if (closest_face >= 0) {
                                            try {
                                                // ✅ NEW APPROACH: Use initial distance d_0 as rest_gap for this constraint
                                                // Instead of using a global target_gap, each constraint stores its own
                                                // initial separation distance as the rest state.
                                                Real rest_gap = closest_distance; // d_0 = initial distance
                                                Real break_ratio = _config->nerveTumorAdhesionBreakRatio(); // e.g., 1.5 = 50% strain
                                                
                                                // ✅ FIXED: Use distance_window as the threshold (from YAML config)
                                                // This allows you to control the adhesion formation distance via YAML
                                                // No need for hardcoded threshold - distance_window already filters in outer loop
                                                
                                                if (constraints_added == 0) {
                                                    std::cout << "[adhesion] Creating ONE constraint per nerve vertex:"
                                                              << " alpha=" << alpha 
                                                              << ", distance_window=" << distance_window << "m"
                                                              << ", break_ratio=" << break_ratio << " (strain-based)\n";
                                                    std::cout << "[adhesion] Each constraint stores its own rest_gap (d_0 = initial distance)\n";
                                                }
                                                
                                                // ✅ CRITICAL FIX: Pass nerve_ptr so tumor can get position pointer from NERVE mesh!
                                                // Previously was passing nerve vertex index, causing tumor to look up wrong mesh
                                                typed_tumor_ptr->addNerveTumorAdhesionConstraint(nerve_ptr, v, 
                                                                                                closest_v1, closest_v2, closest_v3, 
                                                                                                rest_gap, break_ratio, alpha);
                                                
                                                // Also mark the nerve vertex on the nerve mesh for visualization
                                                if (!nerve_ptr->mesh()->template hasVertexProperty<bool>("has_adhesion_constraint")) {
                                                    nerve_ptr->mesh()->template addVertexProperty<bool>("has_adhesion_constraint", false);
                                                    std::cout << "[viz] Created adhesion constraint property for nerve mesh " << nerve_ptr->mesh() << "\n";
                                                }
                                                auto& nerve_adhesion_prop = nerve_ptr->mesh()->template getVertexProperty<bool>("has_adhesion_constraint");
                                                nerve_adhesion_prop.set(v, true);
                                                // std::cout << "[viz] Marked nerve vertex " << v << " as having adhesion constraint on nerve mesh " << nerve_ptr->mesh() << "\n";
                                                
                                                ++constraints_added; ++total_constraints;
                                                if (constraints_added <= 10) { // Show more examples since we have fewer constraints now
                                                    std::cout << "[adhesion] Added constraint (nerve->tumor): nerve_v=" << v 
                                                              << " closest_face=[" << closest_v1 << "," << closest_v2 << "," << closest_v3 
                                                              << "] d_0=" << rest_gap << "m (rest_gap)\n";
                                                }
                                            } catch (const std::exception& e) {
                                                std::cout << "[adhesion] Failed to add constraint on tumor instance: " << e.what() << "\n";
                                            }
                                        } else {
                                            // DEBUG: Show why this nerve vertex didn't get a constraint
                                            std::cout << "[adhesion] No constraint for nerve_v=" << v 
                                                      << " (no triangle within " << distance_window << "m window)\n";
                                        }
                                    }
                                    
                                    // Performance report
                                    if (constraints_added > 0) {
                                        std::cout << "[adhesion PERFORMANCE] AABB culling rejected " << triangles_culled_by_aabb 
                                                  << " / " << distances_checked + triangles_culled_by_aabb 
                                                  << " triangle checks (" 
                                                  << (100.0 * triangles_culled_by_aabb / (distances_checked + triangles_culled_by_aabb)) 
                                                  << "% speedup)\n";
                                    }
                                    
                                    return constraints_added > 0;
                                };

                                // Try known constraint configurations for first-order tumor objects
                                {
                                    using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                                    using Sol1 = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookean::projector_type_list>;
                                    using TumorType1 = XPBDMeshObject_<true, Sol1::GaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
                                    if (!tumor_cast_handled) tumor_cast_handled = try_add_on_tumor(dynamic_cast<TumorType1*>(tumor_ptr));
                                }
                                {
                                    using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                                    using Sol2 = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                                    using TumorType2 = XPBDMeshObject_<true, Sol2::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                    if (!tumor_cast_handled) tumor_cast_handled = try_add_on_tumor(dynamic_cast<TumorType2*>(tumor_ptr));
                                }
                                {
                                    using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                                    using Sol3 = XPBDObjectSolverTypes<true, typename Cfg::NerveOnly::projector_type_list>;
                                    using TumorType3 = XPBDMeshObject_<true, Sol3::GaussSeidel, typename Cfg::NerveOnly::constraint_type_list>;
                                    if (!tumor_cast_handled) tumor_cast_handled = try_add_on_tumor(dynamic_cast<TumorType3*>(tumor_ptr));
                                }
                                // Try Colored-Gauss-Seidel variants
                                {
                                    using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                                    using Sol1 = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookean::projector_type_list>;
                                    using TumorType1 = XPBDMeshObject_<true, Sol1::ColoredGaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
                                    if (!tumor_cast_handled) tumor_cast_handled = try_add_on_tumor(dynamic_cast<TumorType1*>(tumor_ptr));
                                }
                                {
                                    using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                                    using Sol2 = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                                    using TumorType2 = XPBDMeshObject_<true, Sol2::ColoredGaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                                    if (!tumor_cast_handled) tumor_cast_handled = try_add_on_tumor(dynamic_cast<TumorType2*>(tumor_ptr));
                                }
                                {
                                    using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                                    using Sol3 = XPBDObjectSolverTypes<true, typename Cfg::NerveOnly::projector_type_list>;
                                    using TumorType3 = XPBDMeshObject_<true, Sol3::ColoredGaussSeidel, typename Cfg::NerveOnly::constraint_type_list>;
                                    if (!tumor_cast_handled) tumor_cast_handled = try_add_on_tumor(dynamic_cast<TumorType3*>(tumor_ptr));
                                }
                            } catch (const std::exception& e) {
                                std::cout << "[adhesion] Error pairing nerve and tumor objects: " << e.what() << "\n";
                            }
                            
                            // Print distance statistics
                            if (distances_checked > 0) {
                                std::cout << "[adhesion] Distance stats: min=" << min_distance << "m, max=" << max_checked_distance 
                                          << "m, window=" << distance_window << "m, checked=" << distances_checked << " pairs\n";
                            }
                            
                            std::cout << "[adhesion] Added " << constraints_added << " constraints for this nerve/tumor pair\n";
                        }
                    }
                    
                    // Summary report
                    int total_nerve_vertices = 0;
                    for (auto* nerve_ptr : nerve_objs) {
                        total_nerve_vertices += nerve_ptr->mesh()->numVertices();
                    }
                    
                    if (total_constraints > 0) {
                        std::cout << "[adhesion] =========================\n";
                        std::cout << "[adhesion] SUMMARY REPORT:\n";
                        std::cout << "[adhesion] Total nerve vertices: " << total_nerve_vertices << "\n";
                        std::cout << "[adhesion] Constraints created: " << total_constraints << "\n";
                        std::cout << "[adhesion] Success rate: " << (total_constraints * 100 / total_nerve_vertices) << "%\n";
                        std::cout << "[adhesion] Target: One constraint per nerve vertex (1:1 mapping)\n";
                        if (total_constraints == total_nerve_vertices) {
                            std::cout << "[adhesion] ✅ SUCCESS: Perfect 1:1 mapping achieved!\n";
                        } else {
                            std::cout << "[adhesion] ⚠️  PARTIAL: " << (total_nerve_vertices - total_constraints) 
                                      << " nerve vertices have no constraints (outside distance window)\n";
                        }
                        std::cout << "[adhesion] =========================\n";
                    } else {
                        std::cout << "[adhesion] Warning: No adhesion constraints were created\n";
                    }
                } else {
                    std::cout << "[adhesion] Nerve-tumor adhesion disabled\n";
                }
                
                // ==================== END NERVE-TUMOR ADHESION CONSTRAINTS ====================

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

    // ==================== INTER-DEFORM ADHESION CONSTRAINTS ====================
    // This section runs independently of nerve constraints
    // std::cout << "[inter-deform adhesion DEBUG] Checking if inter-deform adhesion is enabled...\n";
    
    auto& fo_xpbd_objs = _objects.get<std::unique_ptr<FirstOrderXPBDMeshObject_Base>>();
    
    if (_config->interDeformAdhesionEnable()) {
        std::cout << "[inter-deform adhesion] *** INTER-DEFORM ADHESION ENABLED *** Creating constraints...\n";
        
        // Get interaction type
        const std::string interaction_type = _config->interDeformAdhesionInteractionType();
        std::cout << "[inter-deform adhesion] Interaction type: " << interaction_type << "\n";
        
        // Get adhesion parameters from config
        // CRITICAL FIX: Use correct bond_distance parameter based on interaction type
        Real bond_distance;
        if (interaction_type == "unified-distance") {
            bond_distance = _config->interDeformUnifiedBondDistance();  // Use unified parameter (10mm)
        } else {
            bond_distance = _config->interDeformAdhesionBondDistance();  // Use standard parameter (3mm)
        }
        
        // Get parameters based on interaction type
        Real alpha, break_ratio, rest_gap;
        Real d_contact, d_rest, d_neutral_start, d_neutral_end, d_bond, stretch_abs_min;
        
        if (interaction_type == "unified-distance") {
            // Use unified-distance parameters
            alpha = _config->interDeformUnifiedAlpha();
            break_ratio = _config->interDeformUnifiedBreakRatio();
            d_contact = _config->interDeformUnifiedDContact();
            d_rest = _config->interDeformUnifiedDRest();
            d_neutral_start = _config->interDeformUnifiedDNeutralStart();
            d_neutral_end = _config->interDeformUnifiedDNeutralEnd();
            d_bond = _config->interDeformUnifiedDBond();
            stretch_abs_min = _config->interDeformUnifiedStretchAbsMin();
            
            std::cout << "[inter-deform adhesion] Unified-distance parameters:\n"
                      << "  alpha=" << alpha << ", break_ratio=" << break_ratio 
                      << ", bond_distance=" << bond_distance << "\n"
                      << "  d_contact=" << d_contact*1000 << "mm, d_rest=" << d_rest*1000 << "mm\n"
                      << "  d_neutral_start=" << d_neutral_start*1000 << "mm, d_neutral_end=" << d_neutral_end*1000 << "mm\n"
                      << "  d_bond=" << d_bond*1000 << "mm, stretch_abs_min=" << stretch_abs_min*1000 << "mm\n";
        } else {
            // Use regular adhesion parameters
            alpha = _config->interDeformAdhesionAlpha();
            break_ratio = _config->interDeformAdhesionBreakRatio();
            rest_gap = _config->interDeformAdhesionRestGap();
            
            std::cout << "[inter-deform adhesion] Standard adhesion parameters:\n"
                      << "  rest_gap=" << rest_gap << ", break_ratio=" << break_ratio 
                      << ", alpha=" << alpha << ", bond_distance=" << bond_distance << "\n";
        }
        
        // Find objects named "Tumor" and "Brain"
        FirstOrderXPBDMeshObject_Base* cube1_ptr = nullptr;
        FirstOrderXPBDMeshObject_Base* cube2_ptr = nullptr;
        
        for (auto& fo_uptr : fo_xpbd_objs) {
            FirstOrderXPBDMeshObject_Base* fo_base_ptr = fo_uptr.get();
            if (!fo_base_ptr) continue;
            
            std::cout << "[inter-deform adhesion] Found object: " << fo_base_ptr->name() << "\n";
            
            if (fo_base_ptr->name() == "Tumor") {
                cube1_ptr = fo_base_ptr;
                std::cout << "[inter-deform adhesion] ✅ Found Tumor\n";
            } else if (fo_base_ptr->name() == "Brain") {
                cube2_ptr = fo_base_ptr;
                std::cout << "[inter-deform adhesion] ✅ Found Brain\n";
            }
        }
        
        if (cube1_ptr && cube2_ptr) {
            const auto* cube1_mesh = cube1_ptr->mesh();
            const auto* cube2_mesh = cube2_ptr->mesh();
            const int cube1_nv = cube1_mesh->numVertices();
            const int cube2_nf = cube2_mesh->numFaces();
            
            std::cout << "[inter-deform adhesion] Tumor: " << cube1_nv << " vertices\n";
            std::cout << "[inter-deform adhesion] Brain: " << cube2_nf << " faces\n";
            
            int constraints_added = 0;
            int vertices_checked = 0;
            int vertices_within_range = 0;
            Real min_distance_found = std::numeric_limits<Real>::max();
            Real max_distance_found = 0.0;
            
            // Helper lambda to attempt casting and adding constraints
            auto try_add_inter_deform = [&](auto* typed_cube2_ptr) -> bool {
                if (!typed_cube2_ptr) return false;
                
                std::cout << "[inter-deform adhesion] Successfully cast Brain to typed pointer\n";
                std::cout << "[inter-deform adhesion] Using EMBREE BVH for spatial acceleration\n";
                
                // For each vertex in Tumor, find closest face in Brain using Embree BVH
                for (int v = 0; v < cube1_nv; ++v) {
                    vertices_checked++;
                    const Vec3r cube1_vertex = cube1_mesh->vertex(v);
                    
                    Real closest_distance = std::numeric_limits<Real>::max();
                    int closest_face = -1;
                    int closest_v1 = -1, closest_v2 = -1, closest_v3 = -1;
                    
                    // ✅ USE EMBREE BVH: Query nearby triangles within bond_distance
                    std::set<Geometry::EmbreeHit> nearby_triangles = 
                        _embree_scene->interObjectCollisionQuery(cube1_vertex, cube2_ptr, bond_distance);
                    
                    // Check ONLY the nearby triangles found by Embree (much faster than brute force!)
                    for (const auto& hit : nearby_triangles) {
                        const int f = hit.prim_index;
                        
                        if (f < 0 || f >= cube2_nf) {
                            std::cerr << "[ERROR] Invalid face index from Embree: " << f << std::endl;
                            continue;
                        }
                        
                        const auto face = cube2_mesh->face(f);
                        const int v1 = face[0], v2 = face[1], v3 = face[2];
                        
                        const Vec3r tri_p1 = cube2_mesh->vertex(v1);
                        const Vec3r tri_p2 = cube2_mesh->vertex(v2);
                        const Vec3r tri_p3 = cube2_mesh->vertex(v3);
                        
                        // Compute point-to-triangle distance
                        Vec3r closest_point, normal, bary_coords;
                        Real distance = computePointTriangleDistance(cube1_vertex, tri_p1, tri_p2, tri_p3,
                                                                    closest_point, normal, bary_coords);
                        
                        // Track closest triangle within bond distance
                        if (distance <= bond_distance && distance < closest_distance) {
                            closest_distance = distance;
                            closest_face = f;
                            closest_v1 = v1;
                            closest_v2 = v2;
                            closest_v3 = v3;
                        }
                    }
                    
                    // Track statistics
                    if (closest_distance < std::numeric_limits<Real>::max()) {
                        min_distance_found = std::min(min_distance_found, closest_distance);
                        max_distance_found = std::max(max_distance_found, closest_distance);
                        
                        if (closest_distance <= bond_distance) {
                            vertices_within_range++;
                        }
                    }
                    
                    // Create constraint if a close triangle was found
                    if (closest_face >= 0) {
                        try {
                            if (interaction_type == "unified-distance") {
                                // Use unified-distance constraint
                                typed_cube2_ptr->addInterDeformUnifiedDistanceConstraint(
                                    cube1_ptr, v, closest_v1, closest_v2, closest_v3,
                                    alpha, break_ratio, closest_distance,  // initial_distance
                                    d_contact, d_rest, d_neutral_start, d_neutral_end, d_bond, stretch_abs_min
                                );
                            } else {
                                // Use standard adhesion constraint
                                typed_cube2_ptr->addInterDeformDeformAdhesionConstraint(
                                    cube1_ptr, v, closest_v1, closest_v2, closest_v3, rest_gap, break_ratio, alpha
                                );
                            }
                            
                            ++constraints_added;
                            // if (constraints_added <= 10) {
                            //     std::cout << "[inter-deform adhesion] Added constraint: Cube1_v" << v 
                            //               << " -> Cube2_face[" << closest_v1 << "," << closest_v2 << "," << closest_v3 
                            //               << "] distance=" << closest_distance << "m\n";
                            // }
                        } catch (const std::exception& e) {
                            std::cout << "[inter-deform adhesion] Failed to add constraint: " << e.what() << "\n";
                        }
                    }
                }
                
                return constraints_added > 0;
            };
            
            // Try different constraint configurations for Cube2
            // Need to try both Gauss-Seidel AND Jacobi solvers
            bool handled = false;
            
            // Try Gauss-Seidel variants
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol1 = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookean::projector_type_list>;
                using Cube2Type1 = XPBDMeshObject_<true, Sol1::GaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
                if (!handled) handled = try_add_inter_deform(dynamic_cast<Cube2Type1*>(cube2_ptr));
            }
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol2 = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                using Cube2Type2 = XPBDMeshObject_<true, Sol2::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                if (!handled) handled = try_add_inter_deform(dynamic_cast<Cube2Type2*>(cube2_ptr));
            }
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol3 = XPBDObjectSolverTypes<true, typename Cfg::NerveOnly::projector_type_list>;
                using Cube2Type3 = XPBDMeshObject_<true, Sol3::GaussSeidel, typename Cfg::NerveOnly::constraint_type_list>;
                if (!handled) handled = try_add_inter_deform(dynamic_cast<Cube2Type3*>(cube2_ptr));
            }

            // Try Colored-Gauss-Seidel variants
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol1 = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookean::projector_type_list>;
                using Cube2Type1 = XPBDMeshObject_<true, Sol1::ColoredGaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
                if (!handled) handled = try_add_inter_deform(dynamic_cast<Cube2Type1*>(cube2_ptr));
            }
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol2 = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                using Cube2Type2 = XPBDMeshObject_<true, Sol2::ColoredGaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                if (!handled) handled = try_add_inter_deform(dynamic_cast<Cube2Type2*>(cube2_ptr));
            }
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol3 = XPBDObjectSolverTypes<true, typename Cfg::NerveOnly::projector_type_list>;
                using Cube2Type3 = XPBDMeshObject_<true, Sol3::ColoredGaussSeidel, typename Cfg::NerveOnly::constraint_type_list>;
                if (!handled) handled = try_add_inter_deform(dynamic_cast<Cube2Type3*>(cube2_ptr));
            }

            // Try Jacobi variants
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol1 = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookean::projector_type_list>;
                using Cube2Type1 = XPBDMeshObject_<true, Sol1::Jacobi, typename Cfg::StableNeohookean::constraint_type_list>;
                if (!handled) handled = try_add_inter_deform(dynamic_cast<Cube2Type1*>(cube2_ptr));
            }
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol2 = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                using Cube2Type2 = XPBDMeshObject_<true, Sol2::Jacobi, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                if (!handled) handled = try_add_inter_deform(dynamic_cast<Cube2Type2*>(cube2_ptr));
            }
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol3 = XPBDObjectSolverTypes<true, typename Cfg::NerveOnly::projector_type_list>;
                using Cube2Type3 = XPBDMeshObject_<true, Sol3::Jacobi, typename Cfg::NerveOnly::constraint_type_list>;
                if (!handled) handled = try_add_inter_deform(dynamic_cast<Cube2Type3*>(cube2_ptr));
            }
            
            // Summary
            if (constraints_added > 0) {
                std::cout << "[inter-deform adhesion] =========================\n";
                std::cout << "[inter-deform adhesion] ✅ SUCCESSFULLY CREATED ADHESION CONSTRAINTS\n";
                std::cout << "[inter-deform adhesion] =========================\n";
                std::cout << "[inter-deform adhesion] Total constraints created: " << constraints_added << "\n";
                std::cout << "[inter-deform adhesion] Between Tumor (" << cube1_nv << " vertices) and Brain (" << cube2_nf << " faces)\n";
                std::cout << "[inter-deform adhesion] \n";
                std::cout << "[inter-deform adhesion] Distance statistics:\n";
                std::cout << "[inter-deform adhesion]   Vertices checked: " << vertices_checked << "\n";
                std::cout << "[inter-deform adhesion]   Vertices within bond_distance: " << vertices_within_range << "\n";
                std::cout << "[inter-deform adhesion]   Min distance found: " << min_distance_found << " m\n";
                std::cout << "[inter-deform adhesion]   Max distance found: " << max_distance_found << " m\n";
                std::cout << "[inter-deform adhesion] \n";
                std::cout << "[inter-deform adhesion] Parameters used:\n";
                std::cout << "[inter-deform adhesion]   rest_gap = " << rest_gap << " m (constraint rest distance)\n";
                std::cout << "[inter-deform adhesion]   break_ratio = " << break_ratio << " (breaks at " << (break_ratio-1.0)*100 << "% strain)\n";
                std::cout << "[inter-deform adhesion]   alpha = " << alpha << " (compliance)\n";
                std::cout << "[inter-deform adhesion]   bond_distance = " << bond_distance << " m (creation threshold)\n";
                std::cout << "[inter-deform adhesion] =========================\n";
            } else {
                std::cout << "[inter-deform adhesion] ❌ WARNING: NO CONSTRAINTS CREATED\n";
                std::cout << "[inter-deform adhesion] =========================\n";
                std::cout << "[inter-deform adhesion] Diagnostic information:\n";
                std::cout << "[inter-deform adhesion]   Vertices checked: " << vertices_checked << "\n";
                std::cout << "[inter-deform adhesion]   Vertices within bond_distance: " << vertices_within_range << "\n";
                std::cout << "[inter-deform adhesion]   Min distance found: " << min_distance_found << " m\n";
                std::cout << "[inter-deform adhesion]   Max distance found: " << max_distance_found << " m\n";
                std::cout << "[inter-deform adhesion]   Bond distance threshold: " << bond_distance << " m\n";
                std::cout << "[inter-deform adhesion] \n";
                std::cout << "[inter-deform adhesion] Possible reasons:\n";
                std::cout << "[inter-deform adhesion]   - Objects too far apart (min_distance > bond_distance)\n";
                std::cout << "[inter-deform adhesion]   - Failed to cast Brain to correct XPBD type\n";
                std::cout << "[inter-deform adhesion]   - Tumor vertices: " << cube1_nv << ", Brain faces: " << cube2_nf << "\n";
                std::cout << "[inter-deform adhesion] =========================\n";
            }
        } else {
            std::cout << "[inter-deform adhesion] ❌ Could not find Tumor and/or Brain objects\n";
            std::cout << "[inter-deform adhesion] Available objects:\n";
            for (auto& fo_uptr : fo_xpbd_objs) {
                if (fo_uptr) std::cout << "[inter-deform adhesion]   - " << fo_uptr->name() << "\n";
            }
        }
    } else {
        std::cout << "[inter-deform adhesion] Inter-deform adhesion disabled in config\n";
    }
    // ==================== END INTER-DEFORM ADHESION CONSTRAINTS ====================

    // ==================== RIGID-DEFORM ADHESION CONSTRAINTS ====================
    // Create adhesion constraints between rigid objects and deformable meshes
    // This allows rigid bodies to stick to soft tissue with breakable bonds
    
    if (_config->rigidDeformAdhesionEnable()) {
        std::cout << "[rigid-deform adhesion] *** RIGID-DEFORM ADHESION ENABLED *** Creating constraints...\n";
        
        // Get adhesion parameters from config
        const std::string interaction_type = _config->rigidDeformAdhesionInteractionType();
        const Real rest_gap = _config->rigidDeformAdhesionRestGap();
        const Real break_ratio = _config->rigidDeformAdhesionBreakRatio();
        const Real alpha = _config->rigidDeformAdhesionAlpha();
        const Real bond_distance = _config->rigidDeformAdhesionBondDistance();
        
        // Get unified-distance curve parameters (only used if interaction_type == "unified-distance")
        const Real d_contact = _config->rigidDeformAdhesionDContact();
        const Real d_rest = _config->rigidDeformAdhesionDRest();
        const Real d_neutral_start = _config->rigidDeformAdhesionDNeutralStart();
        const Real d_neutral_end = _config->rigidDeformAdhesionDNeutralEnd();
        const Real d_bond = _config->rigidDeformAdhesionDBond();
        const Real stretch_abs_min = _config->rigidDeformAdhesionStretchAbsMin();
        
        std::cout << "[rigid-deform adhesion] Interaction type: " << interaction_type << "\n";
        std::cout << "[rigid-deform adhesion] Parameters: rest_gap=" << rest_gap 
                  << ", break_ratio=" << break_ratio << ", alpha=" << alpha 
                  << ", bond_distance=" << bond_distance << "\n";
        
        if (interaction_type == "unified-distance") {
            std::cout << "[rigid-deform adhesion] Curve parameters: d_contact=" << (d_contact*1000) << "mm"
                      << ", d_rest=" << (d_rest*1000) << "mm"
                      << ", d_bond=" << (d_bond*1000) << "mm"
                      << ", stretch_abs_min=" << (stretch_abs_min*1000) << "mm\n";
        }
        
        // Find rigid and deformable objects to pair
        // Example: Find object named "RigidBone" and "Tissue"
        Sim::RigidObject* rigid_obj_ptr = nullptr;
        FirstOrderXPBDMeshObject_Base* tissue_ptr = nullptr;
        
        // Search for rigid mesh objects specifically
        auto& rigid_mesh_objs = _objects.get<std::unique_ptr<Sim::RigidMeshObject>>();
        for (auto& rigid_uptr : rigid_mesh_objs) {
            if (!rigid_uptr) continue;
            std::cout << "[rigid-deform adhesion] Found rigid mesh object: " << rigid_uptr->name() << "\n";
            if (rigid_uptr->name() == "RigidBone" || rigid_uptr->name() == "Bone" || rigid_uptr->name() == "sphere") {
                rigid_obj_ptr = rigid_uptr.get();
                std::cout << "[rigid-deform adhesion] ✅ Found rigid object for adhesion: " << rigid_uptr->name() << "\n";
                break;
            }
        }
        
        // If not found in RigidMeshObject, check other rigid types
        if (!rigid_obj_ptr) {
            auto& rigid_spheres = _objects.get<std::unique_ptr<Sim::RigidSphere>>();
            for (auto& rigid_uptr : rigid_spheres) {
                if (!rigid_uptr) continue;
                std::cout << "[rigid-deform adhesion] Found rigid sphere: " << rigid_uptr->name() << "\n";
                if (rigid_uptr->name() == "RigidBone" || rigid_uptr->name() == "Bone" || rigid_uptr->name() == "sphere") {
                    rigid_obj_ptr = rigid_uptr.get();
                    std::cout << "[rigid-deform adhesion] ✅ Found rigid object for adhesion: " << rigid_uptr->name() << "\n";
                    break;
                }
            }
        }
        
        if (!rigid_obj_ptr) {
            auto& rigid_boxes = _objects.get<std::unique_ptr<Sim::RigidBox>>();
            for (auto& rigid_uptr : rigid_boxes) {
                if (!rigid_uptr) continue;
                std::cout << "[rigid-deform adhesion] Found rigid box: " << rigid_uptr->name() << "\n";
                if (rigid_uptr->name() == "RigidBone" || rigid_uptr->name() == "Bone" || rigid_uptr->name() == "sphere") {
                    rigid_obj_ptr = rigid_uptr.get();
                    std::cout << "[rigid-deform adhesion] ✅ Found rigid object for adhesion: " << rigid_uptr->name() << "\n";
                    break;
                }
            }
        }
        
        if (!rigid_obj_ptr) {
            auto& rigid_cylinders = _objects.get<std::unique_ptr<Sim::RigidCylinder>>();
            for (auto& rigid_uptr : rigid_cylinders) {
                if (!rigid_uptr) continue;
                std::cout << "[rigid-deform adhesion] Found rigid cylinder: " << rigid_uptr->name() << "\n";
                if (rigid_uptr->name() == "RigidBone" || rigid_uptr->name() == "Bone" || rigid_uptr->name() == "sphere") {
                    rigid_obj_ptr = rigid_uptr.get();
                    std::cout << "[rigid-deform adhesion] ✅ Found rigid object for adhesion: " << rigid_uptr->name() << "\n";
                    break;
                }
            }
        }
        
        // Search for deformable objects
        for (auto& fo_uptr : fo_xpbd_objs) {
            if (!fo_uptr) continue;
            std::cout << "[rigid-deform adhesion] Found deformable object: " << fo_uptr->name() << "\n";
            if (fo_uptr->name() == "Tumor" || fo_uptr->name() == "Brain" || 
                fo_uptr->name() == "Tissue" || fo_uptr->name() == "DeformableMesh") {
                tissue_ptr = fo_uptr.get();
                std::cout << "[rigid-deform adhesion] ✅ Found deformable object for adhesion\n";
            }
        }
        
        if (rigid_obj_ptr && tissue_ptr) {
            const auto* tissue_mesh = tissue_ptr->mesh();
            const int tissue_nf = tissue_mesh->numFaces();
            
            std::cout << "[rigid-deform adhesion] Rigid object: " << rigid_obj_ptr->name() << "\n";
            std::cout << "[rigid-deform adhesion] Tissue mesh: " << tissue_ptr->name() << " with " << tissue_nf << " faces\n";
            
            int constraints_added = 0;
            int faces_checked = 0;
            int faces_within_range = 0;
            Real min_distance_found = std::numeric_limits<Real>::max();
            Real max_distance_found = 0.0;
            
            // Get rigid object SDF for distance queries
            // First, ensure SDF is created for primitives
            if (auto* rigid_sphere = dynamic_cast<Sim::RigidSphere*>(rigid_obj_ptr)) {
                rigid_sphere->createSDF();
            } else if (auto* rigid_box = dynamic_cast<Sim::RigidBox*>(rigid_obj_ptr)) {
                rigid_box->createSDF();
            } else if (auto* rigid_cylinder = dynamic_cast<Sim::RigidCylinder*>(rigid_obj_ptr)) {
                rigid_cylinder->createSDF();
            }
            
            // Now get the SDF
            const Geometry::SDF* sdf = nullptr;
            if (auto* rigid_mesh_obj = dynamic_cast<Sim::RigidMeshObject*>(rigid_obj_ptr)) {
                sdf = rigid_mesh_obj->SDF();
            } else if (auto* rigid_sphere = dynamic_cast<Sim::RigidSphere*>(rigid_obj_ptr)) {
                sdf = rigid_sphere->SDF();
            } else if (auto* rigid_box = dynamic_cast<Sim::RigidBox*>(rigid_obj_ptr)) {
                sdf = rigid_box->SDF();
            } else if (auto* rigid_cylinder = dynamic_cast<Sim::RigidCylinder*>(rigid_obj_ptr)) {
                sdf = rigid_cylinder->SDF();
            }
            
            if (!sdf) {
                std::cout << "[rigid-deform adhesion] ❌ ERROR: Could not get SDF for rigid object. Skipping constraint creation.\n";
            } else {
                std::cout << "[rigid-deform adhesion] ✅ Got SDF for rigid object\n";
            }
            
            // Skip if no SDF available
            if (!sdf) {
                std::cout << "[rigid-deform adhesion] ❌ Cannot create constraints without SDF\n";
            } else {
            
            // Helper lambda to attempt casting and adding constraints
            auto try_add_rigid_deform = [&](auto* typed_tissue_ptr) -> bool {
                if (!typed_tissue_ptr) return false;
                
                std::cout << "[rigid-deform adhesion] Successfully cast tissue to typed pointer\n";
                
                // For each face in the tissue, find if it's close to the rigid body surface
                // Use the SDF to compute actual surface-to-point distance
                
                for (int f = 0; f < tissue_nf; ++f) {
                    faces_checked++;
                    
                    const auto face = tissue_mesh->face(f);
                    const int v1 = face[0], v2 = face[1], v3 = face[2];
                    
                    const Vec3r tri_p1 = tissue_mesh->vertex(v1);
                    const Vec3r tri_p2 = tissue_mesh->vertex(v2);
                    const Vec3r tri_p3 = tissue_mesh->vertex(v3);
                    
                    // ⚠️ CRITICAL FIX FOR BUG #3: Use RIGID POINT projection, not triangle centroid!
                    // 
                    // THE PROBLEM:
                    // - Old code: Used triangle centroid to compute distance → d_centroid
                    // - Constraint constructor: Uses rigid point projection to triangle → d_projection
                    // - These can be VERY different! (e.g., centroid 5mm, projection 3mm)
                    // - Breaking logic uses _initial_distance (from projection), but bonding uses d_centroid
                    // - Result: Constraint created when far away, breaks too early!
                    //
                    // THE FIX:
                    // Project the rigid point onto the triangle BEFORE checking bond_distance.
                    // This ensures the distance used for "should I create?" matches "_initial_distance".
                    
                    // Transform rigid object position to global space for distance calculation
                    const Vec3r rigid_pos_global = rigid_obj_ptr->position();
                    
                    // Project rigid position onto triangle to get ACTUAL constraint distance
                    // (This matches what the constraint constructor will compute as _initial_distance)
                    const Vec3r edge1 = tri_p2 - tri_p1;
                    const Vec3r edge2 = tri_p3 - tri_p1;
                    const Vec3r triangle_normal = edge1.cross(edge2);
                    const Real area = triangle_normal.norm();
                    
                    if (area < 1e-12) continue;  // Skip degenerate triangles
                    
                    const Vec3r normal = triangle_normal / area;
                    Real signed_distance = (rigid_pos_global - tri_p1).dot(normal);
                    
                    // Flip normal to point toward rigid if needed
                    Vec3r corrected_normal = normal;
                    if (signed_distance < 0) {
                        corrected_normal = -normal;
                        signed_distance = -signed_distance;
                    }
                    
                    // Project point onto plane
                    const Vec3r projected_point = rigid_pos_global - signed_distance * corrected_normal;
                    
                    // Compute barycentric coordinates
                    const Vec3r v0 = edge2;
                    const Vec3r v1_bary = edge1;  
                    const Vec3r v2_bary = projected_point - tri_p1;  // Renamed to avoid conflict with v2 vertex index
                    
                    const Real dot00 = v0.dot(v0);
                    const Real dot01 = v0.dot(v1_bary);
                    const Real dot02 = v0.dot(v2_bary);
                    const Real dot11 = v1_bary.dot(v1_bary);
                    const Real dot12 = v1_bary.dot(v2_bary);
                    
                    const Real inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
                    const Real u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
                    const Real v_coord = (dot00 * dot12 - dot01 * dot02) * inv_denom;
                    
                    // Determine closest point (inside or clamped to boundary)
                    Vec3r closest_point;
                    Real distance;
                    if (u >= 0.0 && v_coord >= 0.0 && (u + v_coord) <= 1.0) {
                        // Inside triangle
                        closest_point = projected_point;
                        distance = signed_distance;
                    } else {
                        // Outside - clamp to boundary
                        Real u_clamp = std::max(0.0, std::min(1.0, u));
                        Real v_clamp = std::max(0.0, std::min(1.0, v_coord));
                        if (u_clamp + v_clamp > 1.0) {
                            const Real scale = 1.0 / (u_clamp + v_clamp);
                            u_clamp *= scale;
                            v_clamp *= scale;
                        }
                        const Real w_clamp = 1.0 - u_clamp - v_clamp;
                        closest_point = w_clamp * tri_p1 + v_clamp * tri_p2 + u_clamp * tri_p3;
                        distance = (rigid_pos_global - closest_point).norm();
                    }
                    
                    // Now use the ACTUAL distance (projection-based) for bonding decision
                    // This matches _initial_distance that will be computed in constraint constructor!
                    
                    // Now use the ACTUAL distance (projection-based) for bonding decision
                    // This matches _initial_distance that will be computed in constraint constructor!
                    
                    // NOTE: Do NOT track statistics here - this 'distance' is object-to-triangle,
                    // not the surface-to-surface distance used by constraints.
                    // Statistics will be tracked after computing actual initial_distance below.
                    
                    if (distance <= bond_distance) {
                        faces_within_range++;
                        
                        // 🔧 CRITICAL FIX: Use rigid surface point, not triangle point!
                        // OLD (WRONG): rigid_body_point = SDF query from tri_centroid (moves with tissue!)
                        // NEW (CORRECT): rigid_body_point = SDF query from closest_point on triangle
                        //
                        // WHY THIS MATTERS:
                        // - closest_point is on the TISSUE triangle (deformable, moves)
                        // - We need to find the corresponding point on RIGID bone surface
                        // - Query SDF with closest_point (initial tissue position)
                        // - Project onto rigid surface → gives fixed point on bone
                        // - Store in body coordinates → stays fixed as bone rotates/translates
                        
                        // Find closest point on rigid surface using SDF
                        // ✅ Use closest_point (on tissue) as query, not tri_centroid!
                        // This ensures we get the rigid point that's actually closest to the tissue
                        const Real signed_dist = sdf->evaluate(closest_point);
                        const Vec3r grad = sdf->gradient(closest_point);  // unit normal (points outward from bone)
                        
                        // Closest point on rigid surface: move from query point along negative gradient
                        // (gradient points outward from bone, so -gradient points toward bone surface)
                        const Vec3r closest_on_rigid_surface = closest_point - signed_dist * grad;
                        
                        // Convert to body coordinates (THIS IS THE FIXED ATTACHMENT POINT ON BONE!)
                        const Vec3r rigid_body_point = rigid_obj_ptr->globalToBody(closest_on_rigid_surface);
                        
                        // Recompute initial distance using correct point pair
                        // (rigid surface point → triangle surface point)
                        const Real initial_distance = (closest_on_rigid_surface - closest_point).norm();
                        
                        // ✅ Track ACTUAL constraint distance statistics (surface-to-surface)
                        min_distance_found = std::min(min_distance_found, initial_distance);
                        max_distance_found = std::max(max_distance_found, initial_distance);
                        
                        // 🔍 DEBUG: Verify distance calculation consistency (first 3 constraints)
                        static int creation_debug_count = 0;
                        if (creation_debug_count < 3) {
                            std::cout << "\n🔍📍 [CREATION DEBUG #" << creation_debug_count << "] FIXED VERSION" << std::endl;
                            std::cout << "  Query: closest_point (on triangle) = " << closest_point.transpose() << std::endl;
                            std::cout << "  SDF: signed_dist = " << signed_dist*1000 << " mm, grad = " << grad.transpose() << std::endl;
                            std::cout << "  Rigid surface: closest_on_rigid_surface = " << closest_on_rigid_surface.transpose() << std::endl;
                            std::cout << "  Triangle: closest_point = " << closest_point.transpose() << std::endl;
                            std::cout << "  initial_distance (rigid_surf → tri_surf) = " << initial_distance*1000 << " mm" << std::endl;
                            std::cout << "  ✅ This should match what evaluate() computes!" << std::endl;
                            
                            // Verify by transforming back
                            const Vec3r rigid_attach_global = rigid_obj_ptr->bodyToGlobal(rigid_body_point);
                            std::cout << "  Verification:" << std::endl;
                            std::cout << "    rigid_body_point (body) = " << rigid_body_point.transpose() << std::endl;
                            std::cout << "    rigid_body_point (global) = " << rigid_attach_global.transpose() << std::endl;
                            std::cout << "    distance to triangle = " << (rigid_attach_global - closest_point).norm()*1000 << " mm" << std::endl;
                            creation_debug_count++;
                        }
                        
                        // CRITICAL FIX: Use config rest_gap as the SLACK LENGTH for breaking!
                        // This means: adhesion can stretch by (rest_gap * break_ratio) from initial position
                        // before breaking, regardless of initial distance.
                        
                        try {
                            if (interaction_type == "unified-distance") {
                                // Use new unified distance constraint (smooth signed-distance curve)
                                typed_tissue_ptr->addUnifiedDistanceConstraint(
                                    sdf, rigid_obj_ptr, rigid_body_point,
                                    v1, v2, v3,
                                    alpha,
                                    break_ratio,  // Pass break_ratio from config
                                    initial_distance,  // Pass precomputed initial distance (CRITICAL!)
                                    d_contact,         // Pass curve parameters from YAML
                                    d_rest,
                                    d_neutral_start,
                                    d_neutral_end,
                                    d_bond,
                                    stretch_abs_min    // Pass stretch absolute minimum from YAML
                                );
                                
                                ++constraints_added;
                                if (constraints_added <= 10) {
                                    std::cout << "[rigid-deform adhesion] Added UNIFIED-DISTANCE constraint: RigidBody -> Tissue_face[" 
                                              << v1 << "," << v2 << "," << v3 
                                              << "] initial=" << (initial_distance*1000) << "mm";
                                    std::cout << " (curve: " << (d_contact*1000) << "mm->" << (d_rest*1000) 
                                              << "mm->" << (d_bond*1000) << "mm, breaks at " 
                                              << ((initial_distance * break_ratio)*1000) << "mm = " 
                                              << (initial_distance * break_ratio) << "m)\n";
                                }
                            } else {
                                // Use traditional adhesion constraint (old behavior)
                                Real effective_rest_gap = rest_gap;  // Use config value for consistent behavior
                                
                                // Optional: enforce minimum gap for numerical stability
                                const Real min_numerical_gap = 0.0001;  // 0.1mm
                                if (effective_rest_gap < min_numerical_gap) {
                                    effective_rest_gap = min_numerical_gap;
                                }
                                
                                typed_tissue_ptr->addRigidDeformAdhesionConstraint(
                                    sdf, rigid_obj_ptr, rigid_body_point,
                                    v1, v2, v3,
                                    effective_rest_gap, break_ratio, alpha
                                );
                                
                                ++constraints_added;
                                if (constraints_added <= 10) {
                                    std::cout << "[rigid-deform adhesion] Added ADHESION constraint: RigidBody -> Tissue_face[" 
                                              << v1 << "," << v2 << "," << v3 
                                              << "] initial_distance=" << initial_distance << "m, rest_gap=" << effective_rest_gap 
                                              << "m, will_break_at=" << (initial_distance + effective_rest_gap * break_ratio) << "m\n";
                                }
                            }
                        } catch (const std::exception& e) {
                            std::cout << "[rigid-deform adhesion] Failed to add constraint: " << e.what() << "\n";
                        }
                    }
                }
                
                return constraints_added > 0;
            };
            
            // Try different constraint configurations for tissue
            bool handled = false;
            
            // Try Gauss-Seidel variants
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol1 = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookean::projector_type_list>;
                using TissueType1 = XPBDMeshObject_<true, Sol1::GaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
                if (!handled) handled = try_add_rigid_deform(dynamic_cast<TissueType1*>(tissue_ptr));
            }
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol2 = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                using TissueType2 = XPBDMeshObject_<true, Sol2::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                if (!handled) handled = try_add_rigid_deform(dynamic_cast<TissueType2*>(tissue_ptr));
            }
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol3 = XPBDObjectSolverTypes<true, typename Cfg::NerveOnly::projector_type_list>;
                using TissueType3 = XPBDMeshObject_<true, Sol3::GaussSeidel, typename Cfg::NerveOnly::constraint_type_list>;
                if (!handled) handled = try_add_rigid_deform(dynamic_cast<TissueType3*>(tissue_ptr));
            }

            // Try Colored-Gauss-Seidel variants
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol1 = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookean::projector_type_list>;
                using TissueType1 = XPBDMeshObject_<true, Sol1::ColoredGaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
                if (!handled) handled = try_add_rigid_deform(dynamic_cast<TissueType1*>(tissue_ptr));
            }
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol2 = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                using TissueType2 = XPBDMeshObject_<true, Sol2::ColoredGaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                if (!handled) handled = try_add_rigid_deform(dynamic_cast<TissueType2*>(tissue_ptr));
            }
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol3 = XPBDObjectSolverTypes<true, typename Cfg::NerveOnly::projector_type_list>;
                using TissueType3 = XPBDMeshObject_<true, Sol3::ColoredGaussSeidel, typename Cfg::NerveOnly::constraint_type_list>;
                if (!handled) handled = try_add_rigid_deform(dynamic_cast<TissueType3*>(tissue_ptr));
            }

            // Try Jacobi variants
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol1 = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookean::projector_type_list>;
                using TissueType1 = XPBDMeshObject_<true, Sol1::Jacobi, typename Cfg::StableNeohookean::constraint_type_list>;
                if (!handled) handled = try_add_rigid_deform(dynamic_cast<TissueType1*>(tissue_ptr));
            }
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol2 = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                using TissueType2 = XPBDMeshObject_<true, Sol2::Jacobi, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                if (!handled) handled = try_add_rigid_deform(dynamic_cast<TissueType2*>(tissue_ptr));
            }
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol3 = XPBDObjectSolverTypes<true, typename Cfg::NerveOnly::projector_type_list>;
                using TissueType3 = XPBDMeshObject_<true, Sol3::Jacobi, typename Cfg::NerveOnly::constraint_type_list>;
                if (!handled) handled = try_add_rigid_deform(dynamic_cast<TissueType3*>(tissue_ptr));
            }
            
            // Summary - always show statistics to help debug
            std::cout << "[rigid-deform adhesion] =========================\n";
            std::cout << "[rigid-deform adhesion] CONSTRAINT CREATION SUMMARY\n";
            std::cout << "[rigid-deform adhesion] =========================\n";
            std::cout << "[rigid-deform adhesion] Total constraints created: " << constraints_added << "\n";
            std::cout << "[rigid-deform adhesion] Between " << rigid_obj_ptr->name() << " and " << tissue_ptr->name() << " (" << tissue_nf << " faces)\n";
            std::cout << "[rigid-deform adhesion] \n";
            std::cout << "[rigid-deform adhesion] Distance statistics (SURFACE-TO-SURFACE):\n";
            std::cout << "[rigid-deform adhesion]   Faces checked: " << faces_checked << "\n";
            std::cout << "[rigid-deform adhesion]   Faces within bond_distance: " << faces_within_range << "\n";
            if (min_distance_found < std::numeric_limits<Real>::max()) {
                std::cout << "[rigid-deform adhesion]   Min initial_distance: " << min_distance_found << " m (" << (min_distance_found*1000) << " mm)\n";
                std::cout << "[rigid-deform adhesion]   Max initial_distance: " << max_distance_found << " m (" << (max_distance_found*1000) << " mm)\n";
                std::cout << "[rigid-deform adhesion]   NOTE: These are the ACTUAL constraint distances (rigid surface -> triangle surface)\n";
            }
            std::cout << "[rigid-deform adhesion] \n";
            std::cout << "[rigid-deform adhesion] Parameters used:\n";
            std::cout << "[rigid-deform adhesion]   initial_distance = PRECOMPUTED (surface-to-surface per constraint)\n";
            std::cout << "[rigid-deform adhesion]   initial_distance range: " << (min_distance_found*1000) << " mm to " << (max_distance_found*1000) << " mm\n";
            std::cout << "[rigid-deform adhesion]   break_ratio = " << break_ratio << "\n";
            std::cout << "[rigid-deform adhesion]   alpha = " << alpha << "\n";
            std::cout << "[rigid-deform adhesion]   bond_distance = " << bond_distance << " m (" << (bond_distance*1000) << " mm)\n";
            std::cout << "[rigid-deform adhesion] =========================\n";
            
            if (constraints_added > 0) {
                std::cout << "[rigid-deform adhesion] ✅ SUCCESSFULLY CREATED RIGID-DEFORM ADHESION CONSTRAINTS\n";
            } else {
                std::cout << "[rigid-deform adhesion] ❌ WARNING: NO CONSTRAINTS CREATED\n";
                std::cout << "[rigid-deform adhesion] 💡 Reason: No tissue faces within bond_distance (" << (bond_distance*1000) << " mm)\n";
                if (min_distance_found < std::numeric_limits<Real>::max()) {
                    std::cout << "[rigid-deform adhesion] 💡 Closest face is " << (min_distance_found*1000) << " mm away\n";
                    std::cout << "[rigid-deform adhesion] 💡 Try: Increase bond-distance to at least " << (min_distance_found*1.1) << " m, or move objects closer\n";
                }
            }
            
            } // End of if(sdf) block
        } else {
            std::cout << "[rigid-deform adhesion] ❌ Could not find required objects\n";
            if (!rigid_obj_ptr) std::cout << "[rigid-deform adhesion]   Missing: Rigid object\n";
            if (!tissue_ptr) std::cout << "[rigid-deform adhesion]   Missing: Deformable tissue\n";
        }
    } else {
        std::cout << "[rigid-deform adhesion] Rigid-deform adhesion disabled in config\n";
    }
    // ==================== END RIGID-DEFORM ADHESION CONSTRAINTS ====================

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
#if 0  // DEBUG MONITORING — disabled for performance (120+ dynamic_casts per timestep)
    if (s_edge_initialized)
    {
        auto read_and_print = [&](auto* xpbd, const char* tag, const char* phase){
            if (!xpbd) return false;
            const auto& V = xpbd->mesh()->vertices();
            if (s_edge_i < 0 || s_edge_j < 0 ||
                s_edge_i >= xpbd->mesh()->numVertices() ||
                s_edge_j >= xpbd->mesh()->numVertices()) return false;
            const Real len = (V.col(s_edge_i) - V.col(s_edge_j)).norm();
            
            // Only print every 3000 steps to avoid flooding the terminal
            if (s_print_counter % 3000 == 0) {
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
            // 2nd + NerveOnly
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<false>;
                using Sol = XPBDObjectSolverTypes<false, typename Cfg::NerveOnly::projector_type_list>;
                using N_GS = XPBDMeshObject_<false, Sol::GaussSeidel, typename Cfg::NerveOnly::constraint_type_list>;
                using N_J  = XPBDMeshObject_<false, Sol::Jacobi,       typename Cfg::NerveOnly::constraint_type_list>;
                using N_PJ = XPBDMeshObject_<false, Sol::ParallelJacobi,typename Cfg::NerveOnly::constraint_type_list>;
                if (!printed) printed = read_and_print(dynamic_cast<N_GS*>(base_ptr), "2nd+NerveOnly+GS", "pre");
                if (!printed) printed = read_and_print(dynamic_cast<N_J *>(base_ptr), "2nd+NerveOnly+Jacobi", "pre");
                if (!printed) printed = read_and_print(dynamic_cast<N_PJ*>(base_ptr), "2nd+NerveOnly+PJacobi", "pre");
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
                // 1st + NerveOnly
                {
                    using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                    using Sol = XPBDObjectSolverTypes<true, typename Cfg::NerveOnly::projector_type_list>;
                    using C_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::NerveOnly::constraint_type_list>;
                    using C_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::NerveOnly::constraint_type_list>;
                    using C_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::NerveOnly::constraint_type_list>;
                    if (!printed) printed = read_and_print(dynamic_cast<C_GS*>(fo_base_ptr), "1st+NerveOnly+GS", "pre");
                    if (!printed) printed = read_and_print(dynamic_cast<C_J *>(fo_base_ptr), "1st+NerveOnly+Jacobi", "pre");
                    if (!printed) printed = read_and_print(dynamic_cast<C_PJ*>(fo_base_ptr), "1st+NerveOnly+PJacobi", "pre");
                }

                if (printed) break;
            }
        }

        // Note: warned_pre removed as it was unused (debug code commented out)
    }

    // —— PRE: read current curvature of the picked triplet —— //
    if (s_triplet_initialized)
    {
        auto read_and_print_triplet = [&](auto* xpbd, const char* tag, const char* phase){
            if (!xpbd) return false;
            const auto& V = xpbd->mesh()->vertices();
            if (s_triplet_i < 0 || s_triplet_j < 0 || s_triplet_k < 0 ||
                s_triplet_i >= xpbd->mesh()->numVertices() ||
                s_triplet_j >= xpbd->mesh()->numVertices() ||
                s_triplet_k >= xpbd->mesh()->numVertices()) return false;
            
            // Compute discrete curvature manually
            Vec3r p0 = V.col(s_triplet_i);
            Vec3r p1 = V.col(s_triplet_j);
            Vec3r p2 = V.col(s_triplet_k);
            
            Vec3r e1 = p1 - p0;
            Vec3r e2 = p2 - p1;
            Real norm_e1 = e1.norm();
            Real norm_e2 = e2.norm();
            
            Real curvature = 0.0;
            if (norm_e1 > 1e-12 && norm_e2 > 1e-12) {
                Vec3r cross_product = e1.cross(e2);
                Real denominator = norm_e1 * norm_e2 * (norm_e1 + norm_e2);
                if (denominator > 1e-12) {
                    curvature = 2.0 * cross_product.norm() / denominator;
                }
            }
            
            // Only print every 3000 steps to avoid flooding the terminal
            // if (s_print_counter % 3000 == 0) {
            //     std::cout << "[" << phase << "](" << tag << ") step=" << s_print_counter 
            //               << " triplet(" << s_triplet_i << "," << s_triplet_j << "," << s_triplet_k 
            //               << ") curvature = " << curvature << " (rest = " << s_triplet_rest_curvature << ")\n";
            // }
            return true;
        };

        auto& xpbd_objs = _objects.get<std::unique_ptr<XPBDMeshObject_Base>>();
        auto& fo_xpbd_objs = _objects.get<std::unique_ptr<FirstOrderXPBDMeshObject_Base>>();
        
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
                if (!printed) printed = read_and_print_triplet(dynamic_cast<T_GS*>(base_ptr), "2nd+NonCombined+GS", "pre");
                if (!printed) printed = read_and_print_triplet(dynamic_cast<T_J *>(base_ptr), "2nd+NonCombined+Jacobi", "pre");
                if (!printed) printed = read_and_print_triplet(dynamic_cast<T_PJ*>(base_ptr), "2nd+NonCombined+PJacobi", "pre");
            }
            // 2nd + Combined
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<false>;
                using Sol = XPBDObjectSolverTypes<false, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                using T_GS = XPBDMeshObject_<false, Sol::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                using T_J  = XPBDMeshObject_<false, Sol::Jacobi,       typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                using T_PJ = XPBDMeshObject_<false, Sol::ParallelJacobi,typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                if (!printed) printed = read_and_print_triplet(dynamic_cast<T_GS*>(base_ptr), "2nd+Combined+GS", "pre");
                if (!printed) printed = read_and_print_triplet(dynamic_cast<T_J *>(base_ptr), "2nd+Combined+Jacobi", "pre");
                if (!printed) printed = read_and_print_triplet(dynamic_cast<T_PJ*>(base_ptr), "2nd+Combined+PJacobi", "pre");
            }
            // 2nd + NerveOnly
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<false>;
                using Sol = XPBDObjectSolverTypes<false, typename Cfg::NerveOnly::projector_type_list>;
                using N_GS = XPBDMeshObject_<false, Sol::GaussSeidel, typename Cfg::NerveOnly::constraint_type_list>;
                using N_J  = XPBDMeshObject_<false, Sol::Jacobi,       typename Cfg::NerveOnly::constraint_type_list>;
                using N_PJ = XPBDMeshObject_<false, Sol::ParallelJacobi,typename Cfg::NerveOnly::constraint_type_list>;
                if (!printed) printed = read_and_print_triplet(dynamic_cast<N_GS*>(base_ptr), "2nd+NerveOnly+GS", "pre");
                if (!printed) printed = read_and_print_triplet(dynamic_cast<N_J *>(base_ptr), "2nd+NerveOnly+Jacobi", "pre");
                if (!printed) printed = read_and_print_triplet(dynamic_cast<N_PJ*>(base_ptr), "2nd+NerveOnly+PJacobi", "pre");
            }

            if (printed) break;
        }

        for (auto& fo_uptr : fo_xpbd_objs) {
            auto* fo_base_ptr = fo_uptr.get();
            
            // 1st + NonCombined
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookean::projector_type_list>;
                using A_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
                using A_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::StableNeohookean::constraint_type_list>;
                using A_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::StableNeohookean::constraint_type_list>;
                if (!printed) printed = read_and_print_triplet(dynamic_cast<A_GS*>(fo_base_ptr), "1st+NonCombined+GS", "pre");
                if (!printed) printed = read_and_print_triplet(dynamic_cast<A_J *>(fo_base_ptr), "1st+NonCombined+Jacobi", "pre");
                if (!printed) printed = read_and_print_triplet(dynamic_cast<A_PJ*>(fo_base_ptr), "1st+NonCombined+PJacobi", "pre");
            }
            // 1st + Combined
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                using B_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                using B_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                using B_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                if (!printed) printed = read_and_print_triplet(dynamic_cast<B_GS*>(fo_base_ptr), "1st+Combined+GS", "pre");
                if (!printed) printed = read_and_print_triplet(dynamic_cast<B_J *>(fo_base_ptr), "1st+Combined+Jacobi", "pre");
                if (!printed) printed = read_and_print_triplet(dynamic_cast<B_PJ*>(fo_base_ptr), "1st+Combined+PJacobi", "pre");
            }
            // 1st + NerveOnly
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                using Sol = XPBDObjectSolverTypes<true, typename Cfg::NerveOnly::projector_type_list>;
                using C_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::NerveOnly::constraint_type_list>;
                using C_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::NerveOnly::constraint_type_list>;
                using C_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::NerveOnly::constraint_type_list>;
                if (!printed) printed = read_and_print_triplet(dynamic_cast<C_GS*>(fo_base_ptr), "1st+NerveOnly+GS", "pre");
                if (!printed) printed = read_and_print_triplet(dynamic_cast<C_J *>(fo_base_ptr), "1st+NerveOnly+Jacobi", "pre");
                if (!printed) printed = read_and_print_triplet(dynamic_cast<C_PJ*>(fo_base_ptr), "1st+NerveOnly+PJacobi", "pre");
            }

            if (printed) break;
        }

        static bool warned_pre_triplet = false;
        if (!printed && !warned_pre_triplet) {
            std::cout << "[pre] WARNING: s_triplet_initialized=true but couldn't read vertices; "
                         "template combo at runtime didn't match. Check setup prints."
                      << std::endl;
            warned_pre_triplet = true;
        }
    }
#endif  // DEBUG MONITORING disabled

    // —— Run one XPBD step (objects do elasticity + collisions + your stretch) —— //
    _objects.for_each_element([](auto& obj) { obj->update(); });

    // —— check and break adhesion constraints AFTER physics update —— //
    // ⚡ PERFORMANCE OPTIMIZATION: Check breaking every N steps instead of every step
    // This significantly reduces overhead when there are many adhesion constraints
    static int adhesion_check_counter = 0;
    const int ADHESION_CHECK_INTERVAL = 50;  // Check every 50 steps (with dt=5e-4, this is 25ms)
    adhesion_check_counter++;
    
    const bool should_check_breaking = (adhesion_check_counter % ADHESION_CHECK_INTERVAL == 0);
    
    if (should_check_breaking) {
        // This ensures we check distances AFTER constraint projection and fixed vertex enforcement
        if (_config->nerveTumorAdhesionEnable()) {
            const Real break_distance = _config->nerveTumorAdhesionBreakDistance();
            
            auto& xpbd_mesh_objs = _objects.get<std::unique_ptr<XPBDMeshObject_Base>>();
            for (auto& obj : xpbd_mesh_objs) obj->checkAndBreakAdhesionConstraints(break_distance);

            auto& fo_xpbd_mesh_objs = _objects.get<std::unique_ptr<FirstOrderXPBDMeshObject_Base>>();
            for (auto& obj : fo_xpbd_mesh_objs) obj->checkAndBreakAdhesionConstraints(break_distance);
        }
        
        // Check and break inter-deform adhesion constraints (uses strain-based breaking, no break_distance param)
        if (_config->interDeformAdhesionEnable()) {
            // Note: Inter-deform constraints use strain-based breaking (via shouldBreak()), 
            // not distance-based like nerve-tumor, so we pass 0.0 as dummy parameter
            auto& xpbd_mesh_objs = _objects.get<std::unique_ptr<XPBDMeshObject_Base>>();
            for (auto& obj : xpbd_mesh_objs) {
                obj->checkAndBreakAdhesionConstraints(0.0);
            }

            auto& fo_xpbd_mesh_objs = _objects.get<std::unique_ptr<FirstOrderXPBDMeshObject_Base>>();
            for (auto& obj : fo_xpbd_mesh_objs) {
                obj->checkAndBreakAdhesionConstraints(0.0);
            }
        }
        
        // Check and break rigid-deform adhesion constraints (uses strain-based breaking)
        if (_config->rigidDeformAdhesionEnable()) {
            auto& xpbd_mesh_objs = _objects.get<std::unique_ptr<XPBDMeshObject_Base>>();
            for (auto& obj : xpbd_mesh_objs) {
                // Note: Rigid-deform constraints use strain-based breaking, pass 0.0 as dummy
                obj->checkAndBreakAdhesionConstraints(0.0);
            }

            auto& fo_xpbd_mesh_objs = _objects.get<std::unique_ptr<FirstOrderXPBDMeshObject_Base>>();
            for (auto& obj : fo_xpbd_mesh_objs) {
                obj->checkAndBreakAdhesionConstraints(0.0);
            }
        }
    }
    
    // Update visualization markers for active adhesion constraints
    if (_config->rigidDeformAdhesionEnable()) {
        auto& xpbd_mesh_objs = _objects.get<std::unique_ptr<XPBDMeshObject_Base>>();
        for (auto& obj : xpbd_mesh_objs) {
            obj->updateAdhesionVisualizationMarkers();
        }

        auto& fo_xpbd_mesh_objs = _objects.get<std::unique_ptr<FirstOrderXPBDMeshObject_Base>>();
        for (auto& obj : fo_xpbd_mesh_objs) {
            obj->updateAdhesionVisualizationMarkers();
        }
    }

    // —— POST: read again and print error —— //
#if 0  // DEBUG MONITORING — disabled for performance
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
            // if (s_print_counter % 900 == 0) {
            //     std::cout << "[post](" << tag << ") step=" << s_print_counter 
            //               << " edge(" << s_edge_i << "," << s_edge_j << ") len = "
            //               << len << "  |len-rest| = " << err << "\n";
            // }
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
            // 2nd + NerveOnly
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<false>;
                using Sol = XPBDObjectSolverTypes<false, typename Cfg::NerveOnly::projector_type_list>;
                using N_GS = XPBDMeshObject_<false, Sol::GaussSeidel, typename Cfg::NerveOnly::constraint_type_list>;
                using N_J  = XPBDMeshObject_<false, Sol::Jacobi,       typename Cfg::NerveOnly::constraint_type_list>;
                using N_PJ = XPBDMeshObject_<false, Sol::ParallelJacobi,typename Cfg::NerveOnly::constraint_type_list>;
                if (!printed) printed = read_and_print_post(dynamic_cast<N_GS*>(base_ptr), "2nd+NerveOnly+GS");
                if (!printed) printed = read_and_print_post(dynamic_cast<N_J *>(base_ptr), "2nd+NerveOnly+Jacobi");
                if (!printed) printed = read_and_print_post(dynamic_cast<N_PJ*>(base_ptr), "2nd+NerveOnly+PJacobi");
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
                // 1st + NerveOnly
                {
                    using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                    using Sol = XPBDObjectSolverTypes<true, typename Cfg::NerveOnly::projector_type_list>;
                    using C_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::NerveOnly::constraint_type_list>;
                    using C_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::NerveOnly::constraint_type_list>;
                    using C_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::NerveOnly::constraint_type_list>;
                    if (!printed) printed = read_and_print_post(dynamic_cast<C_GS*>(fo_base_ptr), "1st+NerveOnly+GS");
                    if (!printed) printed = read_and_print_post(dynamic_cast<C_J *>(fo_base_ptr), "1st+NerveOnly+Jacobi");
                    if (!printed) printed = read_and_print_post(dynamic_cast<C_PJ*>(fo_base_ptr), "1st+NerveOnly+PJacobi");
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

    // —— POST: read bending constraint and print curvature —— //
    if (s_triplet_initialized)
    {
        auto read_and_print_triplet_post = [&](auto* xpbd, const char* tag){
            if (!xpbd) return false;
            const auto& V = xpbd->mesh()->vertices();
            if (s_triplet_i < 0 || s_triplet_j < 0 || s_triplet_k < 0 ||
                s_triplet_i >= xpbd->mesh()->numVertices() ||
                s_triplet_j >= xpbd->mesh()->numVertices() ||
                s_triplet_k >= xpbd->mesh()->numVertices()) return false;
            
            const auto p0 = V.col(s_triplet_i);
            const auto p1 = V.col(s_triplet_j);
            const auto p2 = V.col(s_triplet_k);
            
            const auto e1 = p1 - p0;
            const auto e2 = p2 - p1;
            
            const Real e1_norm = e1.norm();
            const Real e2_norm = e2.norm();
            
            Real current_curvature = 0.0;
            if (e1_norm > 1e-12 && e2_norm > 1e-12) {
                const auto cross = e1.cross(e2);
                current_curvature = 2.0 * cross.norm() / (e1_norm * e2_norm * (e1_norm + e2_norm));
            }
            
            const Real curvature_error = std::abs(current_curvature - s_triplet_rest_curvature);
            
            // // Only print every 300 steps to avoid flooding the terminal
            // if (s_print_counter % 900 == 0) {
            //     std::cout << "[post](" << tag << ") step=" << s_print_counter 
            //               << " triplet(" << s_triplet_i << "," << s_triplet_j << "," << s_triplet_k 
            //               << ") curvature = " << current_curvature 
            //               << " |curvature-rest| = " << curvature_error << "\n";
            // }
            return true;
        };

        auto& xpbd_objs = _objects.get<std::unique_ptr<XPBDMeshObject_Base>>();
        bool triplet_printed = false;
        for (auto& uptr : xpbd_objs) {
            auto* base_ptr = uptr.get();

            // 2nd + NonCombined
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<false>;
                using Sol = XPBDObjectSolverTypes<false, typename Cfg::StableNeohookean::projector_type_list>;
                using T_GS = XPBDMeshObject_<false, Sol::GaussSeidel, typename Cfg::StableNeohookean::constraint_type_list>;
                using T_J  = XPBDMeshObject_<false, Sol::Jacobi,       typename Cfg::StableNeohookean::constraint_type_list>;
                using T_PJ = XPBDMeshObject_<false, Sol::ParallelJacobi,typename Cfg::StableNeohookean::constraint_type_list>;
                if (!triplet_printed) triplet_printed = read_and_print_triplet_post(dynamic_cast<T_GS*>(base_ptr), "2nd+NonCombined+GS");
                if (!triplet_printed) triplet_printed = read_and_print_triplet_post(dynamic_cast<T_J *>(base_ptr), "2nd+NonCombined+Jacobi");
                if (!triplet_printed) triplet_printed = read_and_print_triplet_post(dynamic_cast<T_PJ*>(base_ptr), "2nd+NonCombined+PJacobi");
            }
            // 2nd + Combined
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<false>;
                using Sol = XPBDObjectSolverTypes<false, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                using T_GS = XPBDMeshObject_<false, Sol::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                using T_J  = XPBDMeshObject_<false, Sol::Jacobi,       typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                using T_PJ = XPBDMeshObject_<false, Sol::ParallelJacobi,typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                if (!triplet_printed) triplet_printed = read_and_print_triplet_post(dynamic_cast<T_GS*>(base_ptr), "2nd+Combined+GS");
                if (!triplet_printed) triplet_printed = read_and_print_triplet_post(dynamic_cast<T_J *>(base_ptr), "2nd+Combined+Jacobi");
                if (!triplet_printed) triplet_printed = read_and_print_triplet_post(dynamic_cast<T_PJ*>(base_ptr), "2nd+Combined+PJacobi");
            }
            // 2nd + NerveOnly
            {
                using Cfg = XPBDMeshObjectConstraintConfigurations<false>;
                using Sol = XPBDObjectSolverTypes<false, typename Cfg::NerveOnly::projector_type_list>;
                using N_GS = XPBDMeshObject_<false, Sol::GaussSeidel, typename Cfg::NerveOnly::constraint_type_list>;
                using N_J  = XPBDMeshObject_<false, Sol::Jacobi,       typename Cfg::NerveOnly::constraint_type_list>;
                using N_PJ = XPBDMeshObject_<false, Sol::ParallelJacobi,typename Cfg::NerveOnly::constraint_type_list>;
                if (!triplet_printed) triplet_printed = read_and_print_triplet_post(dynamic_cast<N_GS*>(base_ptr), "2nd+NerveOnly+GS");
                if (!triplet_printed) triplet_printed = read_and_print_triplet_post(dynamic_cast<N_J *>(base_ptr), "2nd+NerveOnly+Jacobi");
                if (!triplet_printed) triplet_printed = read_and_print_triplet_post(dynamic_cast<N_PJ*>(base_ptr), "2nd+NerveOnly+PJacobi");
            }

            if (triplet_printed) break;
        }

        // If no 2nd-order objects printed, try first-order objects
        if (!triplet_printed) {
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
                    if (!triplet_printed) triplet_printed = read_and_print_triplet_post(dynamic_cast<A_GS*>(fo_base_ptr), "1st+NonCombined+GS");
                    if (!triplet_printed) triplet_printed = read_and_print_triplet_post(dynamic_cast<A_J *>(fo_base_ptr), "1st+NonCombined+Jacobi");
                    if (!triplet_printed) triplet_printed = read_and_print_triplet_post(dynamic_cast<A_PJ*>(fo_base_ptr), "1st+NonCombined+PJacobi");
                }
                // 1st + Combined
                {
                    using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                    using Sol = XPBDObjectSolverTypes<true, typename Cfg::StableNeohookeanCombined::projector_type_list>;
                    using B_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                    using B_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                    using B_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::StableNeohookeanCombined::constraint_type_list>;
                    if (!triplet_printed) triplet_printed = read_and_print_triplet_post(dynamic_cast<B_GS*>(fo_base_ptr), "1st+Combined+GS");
                    if (!triplet_printed) triplet_printed = read_and_print_triplet_post(dynamic_cast<B_J *>(fo_base_ptr), "1st+Combined+Jacobi");
                    if (!triplet_printed) triplet_printed = read_and_print_triplet_post(dynamic_cast<B_PJ*>(fo_base_ptr), "1st+Combined+PJacobi");
                }
                // 1st + NerveOnly
                {
                    using Cfg = XPBDMeshObjectConstraintConfigurations<true>;
                    using Sol = XPBDObjectSolverTypes<true, typename Cfg::NerveOnly::projector_type_list>;
                    using C_GS = XPBDMeshObject_<true, Sol::GaussSeidel, typename Cfg::NerveOnly::constraint_type_list>;
                    using C_J  = XPBDMeshObject_<true, Sol::Jacobi,       typename Cfg::NerveOnly::constraint_type_list>;
                    using C_PJ = XPBDMeshObject_<true, Sol::ParallelJacobi,typename Cfg::NerveOnly::constraint_type_list>;
                    if (!triplet_printed) triplet_printed = read_and_print_triplet_post(dynamic_cast<C_GS*>(fo_base_ptr), "1st+NerveOnly+GS");
                    if (!triplet_printed) triplet_printed = read_and_print_triplet_post(dynamic_cast<C_J *>(fo_base_ptr), "1st+NerveOnly+Jacobi");
                    if (!triplet_printed) triplet_printed = read_and_print_triplet_post(dynamic_cast<C_PJ*>(fo_base_ptr), "1st+NerveOnly+PJacobi");
                }

                if (triplet_printed) break;
            }
        }

        static bool warned_triplet_post = false;
        if (!triplet_printed && !warned_triplet_post) {
            std::cout << "[post] WARNING: s_triplet_initialized=true but couldn't read vertices; "
                         "template combo at runtime didn't match. Check setup prints."
                      << std::endl;
            warned_triplet_post = true;
        }
    }
#endif  // DEBUG MONITORING disabled

    // —— velocity update —— //
    _objects.for_each_element([](auto& obj) { obj->velocityUpdate(); });

    // —— collision timestamp —— //
    if (_time - _last_collision_detection_time > _time_between_collision_checks)
    {
        _last_collision_detection_time = _time;
    }

    // —— logging —— //
    if (_logger) _logger->logToFile();

    // —— state recording —— //
    if (_state_recorder && _state_recorder->shouldRecord(_time))
    {
        SimulationStateRecorder::FrameSnapshot snapshot;
        snapshot.time = _time;
        snapshot.frame_number = static_cast<int>(_steps_taken);
        
        int vertex_offset = 0;
        
        // Collect vertex data from all XPBD mesh objects
        auto& xpbd_objs = _objects.get<std::unique_ptr<XPBDMeshObject_Base>>();
        for (const auto& obj : xpbd_objs)
        {
            const auto* mesh = obj->mesh();
            if (!mesh) continue;
            
            const auto& vertices = mesh->vertices();
            const auto& faces = mesh->faces();
            const int num_verts = mesh->numVertices();
            const int num_faces = mesh->numFaces();
            
            // Collect positions
            for (int i = 0; i < num_verts; ++i)
            {
                snapshot.vertex_positions.push_back(vertices.col(i));
            }
            
            // Collect velocities
            for (int i = 0; i < num_verts; ++i)
            {
                snapshot.vertex_velocities.push_back(obj->vertexVelocity(i));
            }
            
            // Collect mesh topology
            SimulationStateRecorder::MeshTopology topo;
            topo.vertex_offset = vertex_offset;
            topo.num_vertices = num_verts;
            
            // Get surface triangles
            for (int i = 0; i < num_faces; ++i)
            {
                topo.surface_triangles.push_back(faces.col(i));
            }
            
            // Try to get tetrahedral elements if this is a TetMesh
            const auto* tet_mesh = dynamic_cast<const Geometry::TetMesh*>(mesh);
            if (tet_mesh)
            {
                topo.has_tets = true;
                const auto& elements = tet_mesh->elements();
                const int num_tets = tet_mesh->numElements();
                for (int i = 0; i < num_tets; ++i)
                {
                    topo.tetrahedra.push_back(elements.col(i));
                }
            }
            else
            {
                topo.has_tets = false;
            }
            
            snapshot.mesh_topologies.push_back(topo);
            vertex_offset += num_verts;
        }
        
        // Also collect from first-order objects
        auto& fo_xpbd_objs = _objects.get<std::unique_ptr<FirstOrderXPBDMeshObject_Base>>();
        for (const auto& obj : fo_xpbd_objs)
        {
            const auto* mesh = obj->mesh();
            if (!mesh) continue;
            
            const auto& vertices = mesh->vertices();
            const auto& faces = mesh->faces();
            const int num_verts = mesh->numVertices();
            const int num_faces = mesh->numFaces();
            
            for (int i = 0; i < num_verts; ++i)
            {
                snapshot.vertex_positions.push_back(vertices.col(i));
                snapshot.vertex_velocities.push_back(obj->vertexVelocity(i));
            }
            
            // Collect mesh topology
            SimulationStateRecorder::MeshTopology topo;
            topo.vertex_offset = vertex_offset;
            topo.num_vertices = num_verts;
            
            // Get surface triangles
            for (int i = 0; i < num_faces; ++i)
            {
                topo.surface_triangles.push_back(faces.col(i));
            }
            
            // Try to get tetrahedral elements
            const auto* tet_mesh = dynamic_cast<const Geometry::TetMesh*>(mesh);
            if (tet_mesh)
            {
                topo.has_tets = true;
                const auto& elements = tet_mesh->elements();
                const int num_tets = tet_mesh->numElements();
                for (int i = 0; i < num_tets; ++i)
                {
                    topo.tetrahedra.push_back(elements.col(i));
                }
            }
            else
            {
                topo.has_tets = false;
            }
            
            snapshot.mesh_topologies.push_back(topo);
            vertex_offset += num_verts;
        }
        
        // Collect adhesion constraint forces per vertex
        // Initialize force accumulator for all vertices
        int total_vertices = snapshot.vertex_positions.size();
        snapshot.vertex_adhesion_force_magnitude.resize(total_vertices, 0.0);
        std::vector<Vec3r> vertex_adhesion_forces(total_vertices, Vec3r::Zero());
        
        // Collect forces from all XPBD mesh objects
        // Note: This collects forces from NerveTumorAdhesionConstraint and InterDeformDeformAdhesionConstraint
        // Physical meaning: F = ∇C^T · λ / dt (1st-order) or ∇C^T · λ / dt² (2nd-order)
        // These represent the constraint forces applied by adhesion constraints
        
        int current_vertex_offset = 0;
        
        // For second-order objects
        auto& xpbd_objs_for_forces = _objects.get<std::unique_ptr<XPBDMeshObject_Base>>();
        for (const auto& obj : xpbd_objs_for_forces)
        {
            if (!obj || !obj->mesh()) continue;
            
            // Collect adhesion forces from this object
            obj->collectAdhesionForces(vertex_adhesion_forces, current_vertex_offset);
            current_vertex_offset += obj->mesh()->numVertices();
        }
        
        // For first-order objects  
        auto& fo_xpbd_objs_for_forces = _objects.get<std::unique_ptr<FirstOrderXPBDMeshObject_Base>>();
        for (const auto& obj : fo_xpbd_objs_for_forces)
        {
            if (!obj || !obj->mesh()) continue;
            
            // Collect adhesion forces from this object
            obj->collectAdhesionForces(vertex_adhesion_forces, current_vertex_offset);
            current_vertex_offset += obj->mesh()->numVertices();
        }
        
        // Convert accumulated force vectors to magnitudes
        for (int i = 0; i < total_vertices; ++i)
        {
            snapshot.vertex_adhesion_force_magnitude[i] = vertex_adhesion_forces[i].norm();
        }
        
        // Collect inter-deformable adhesion constraint states
        for (const auto& obj : xpbd_objs_for_forces)
        {
            if (!obj || !obj->mesh()) continue;
            obj->collectInterDeformAdhesionStates(snapshot.inter_deform_adhesion_states);
        }
        
        for (const auto& obj : fo_xpbd_objs_for_forces)
        {
            if (!obj || !obj->mesh()) continue;
            obj->collectInterDeformAdhesionStates(snapshot.inter_deform_adhesion_states);
        }
        
        // Collect rigid-deformable adhesion constraint states
        for (const auto& obj : xpbd_objs_for_forces)
        {
            if (!obj || !obj->mesh()) continue;
            obj->collectRigidDeformAdhesionStates(snapshot.rigid_deform_adhesion_states);
        }
        
        for (const auto& obj : fo_xpbd_objs_for_forces)
        {
            if (!obj || !obj->mesh()) continue;
            obj->collectRigidDeformAdhesionStates(snapshot.rigid_deform_adhesion_states);
        }
        
        // Note: Deformation data collection will be added in next step when we add accessor methods
        
        _state_recorder->recordSnapshot(_time, static_cast<int>(_steps_taken), snapshot);
    }

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

void Simulation::notifyKeyPressed(SimulationInput::Key key, SimulationInput::KeyAction action, int /* modifiers */)
{
    if (_sim_mode == Config::SimulationMode::FRAME_BY_FRAME && action == SimulationInput::KeyAction::PRESS)
    {
        _timeStep();
        _updateGraphics();
    }
    
    // Manual save trigger: Press 'P' to save state recording
    if (key == SimulationInput::Key::P && action == SimulationInput::KeyAction::PRESS)
    {
        if (_state_recorder && !_state_recorder->getSnapshots().empty())
        {
            std::cout << "\n[Simulation] Manual save triggered by user (P key)\n";
            _state_recorder->saveToFile();
            std::cout << "[Simulation] Save complete! Safe to close now.\n\n";
        }
        else if (_state_recorder)
        {
            std::cout << "\n[Simulation] No snapshots to save yet.\n\n";
        }
        else
        {
            std::cout << "\n[Simulation] State recording not enabled.\n\n";
        }
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
        if (update_thread.joinable())
            update_thread.join();
        return 0;
    }
    else
    {
        update_thread.join();
        return 0;
    }
}

} // namespace Sim
