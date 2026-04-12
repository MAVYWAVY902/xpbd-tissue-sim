// Compiled as part of physics_core static library
// to check sizeof from the library's perspective
#include "simulation/Simulation.hpp"
#include "simulation/PhysicsContext.hpp"
#include "simobject/Object.hpp"
#include "simobject/XPBDMeshObjectBase.hpp"
#include "simobject/MeshObject.hpp"
#include <iostream>

extern "C" void print_lib_sizeof() {
    std::cout << "[LIB] sizeof(PhysicsContext) = " << sizeof(Sim::PhysicsContext) << std::endl;
    std::cout << "[LIB] sizeof(Simulation) = " << sizeof(Sim::Simulation) << std::endl;
    std::cout << "[LIB] sizeof(Object) = " << sizeof(Sim::Object) << std::endl;
    std::cout << "[LIB] sizeof(MeshObject) = " << sizeof(Sim::MeshObject) << std::endl;
    std::cout << "[LIB] sizeof(TetMeshObject) = " << sizeof(Sim::TetMeshObject) << std::endl;
    std::cout << "[LIB] sizeof(XPBDMeshObject_Base_<false>) = " << sizeof(Sim::XPBDMeshObject_Base_<false>) << std::endl;
}
