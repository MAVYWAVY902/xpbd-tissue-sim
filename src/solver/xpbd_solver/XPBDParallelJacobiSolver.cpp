// #ifdef HAVE_CUDA

// #include "common/types.hpp"
// #include "solver/ConstraintProjector.hpp"
// #include "solver/XPBDParallelJacobiSolver.hpp"
// #include "simulation/Simulation.hpp"

// #include "gpu/XPBDSolver.cuh"

// namespace Solver
// {

// XPBDParallelJacobiSolver::XPBDParallelJacobiSolver(Sim::XPBDMeshObject* obj, int num_iter, XPBDSolverResidualPolicyEnum residual_policy)
//     : XPBDSolver(obj, num_iter, residual_policy),
//     _element_Qs(obj->tetMesh()->numElements()*9),
//     _element_volumes(obj->tetMesh()->numElements()),
//     _temp_vertices(obj->mesh()->numVertices()*3),
//     _Qs_resource(_element_Qs.data(), _element_Qs.size()),
//     _volumes_resource(_element_volumes.data(), _element_volumes.size()),
//     _temp_vertices_resource(_temp_vertices.data(), _temp_vertices.size())
// {
//     // createGPUResource() will create the GPU resource for the XPBDMeshObject AND allocate memory for it
//     obj->createGPUResource();
//     _xpbd_obj_resource = dynamic_cast<Sim::XPBDMeshObjectGPUResource*>(obj->gpuResource());

//     // allocate memory on GPU for array resources
//     _Qs_resource.allocate();
//     _volumes_resource.allocate();
//     _temp_vertices_resource.allocate();

//     // precompute quantities for the mesh
//     for (int i = 0; i < obj->tetMesh()->numElements(); i++)
//     {
//         const Eigen::Vector4i& elem = obj->tetMesh()->element(i);
//         Mat3r X;
//         X.col(0) = obj->mesh()->vertex(elem[0]) - obj->mesh()->vertex(elem[3]);
//         X.col(1) = obj->mesh()->vertex(elem[1]) - obj->mesh()->vertex(elem[3]);
//         X.col(2) = obj->mesh()->vertex(elem[2]) - obj->mesh()->vertex(elem[3]);

//         _element_volumes[i] = std::abs(X.determinant()/6);
        
//         Eigen::Matrix3f Q = X.inverse();
//         for (int k = 0; k < 9; k++) { _element_Qs[9*i+k] = Q.data()[k]; }
//     }

//     // copy quantities to GPU
//     _xpbd_obj_resource->fullCopyToDevice();
//     _Qs_resource.fullCopyToDevice();
//     _volumes_resource.fullCopyToDevice();
    
//     LaunchCopyVertices(_xpbd_obj_resource->meshGpuResource().gpuVertices(), _temp_vertices_resource.gpuArr(), obj->mesh()->numVertices());
// }

// void XPBDParallelJacobiSolver::solve()
// {
//     _xpbd_obj_resource->partialCopyToDevice();
//     CHECK_CUDA_ERROR(cudaDeviceSynchronize());
//     LaunchCopyVertices(_xpbd_obj_resource->meshGpuResource().gpuVertices(), _temp_vertices_resource.gpuArr(), _obj->mesh()->numVertices());

//     const Sim::TetMeshGPUResource& mesh_gpu_resource = _xpbd_obj_resource->meshGpuResource();
//     const Sim::ArrayGPUResource<float>& masses_resource = _xpbd_obj_resource->massesGpuResource();
//     for(int i = 0; i < _num_iter; i++)
//     {
//         LaunchXPBDJacobiSolve(mesh_gpu_resource.gpuElements(), _obj->tetMesh()->numElements(), mesh_gpu_resource.gpuVertices(),
//                                                     masses_resource.gpuArr(), _volumes_resource.gpuArr(), _Qs_resource.gpuArr(),
//                                                     _obj->material().lambda(), _obj->material().mu(), _obj->sim()->dt(),
//                                                     _temp_vertices_resource.gpuArr());

//         LaunchCopyVertices(_temp_vertices_resource.gpuArr(), mesh_gpu_resource.gpuVertices(), _obj->mesh()->numVertices());
//     }
//     CHECK_CUDA_ERROR(cudaPeekAtLastError());

//     _xpbd_obj_resource->copyFromDevice();

//     CHECK_CUDA_ERROR(cudaDeviceSynchronize());
// }


// } // namespace Solver

// #endif