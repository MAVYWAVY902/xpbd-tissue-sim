
#define WP_TILE_BLOCK_DIM 256
#define WP_NO_CRT
#include "builtin.h"

// Map wp.breakpoint() to a device brkpt at the call site so cuda-gdb attributes the stop to the generated .cu line
#if defined(__CUDACC__) && !defined(_MSC_VER)
#define __debugbreak() __brkpt()
#endif

// avoid namespacing of float type for casting to float type, this is to avoid wp::float(x), which is not valid in C++
#define float(x) cast_float(x)
#define adj_float(x, adj_x, adj_ret) adj_cast_float(x, adj_x, adj_ret)

#define int(x) cast_int(x)
#define adj_int(x, adj_x, adj_ret) adj_cast_int(x, adj_x, adj_ret)

#define builtin_tid1d() wp::tid(_idx, dim)
#define builtin_tid2d(x, y) wp::tid(x, y, _idx, dim)
#define builtin_tid3d(x, y, z) wp::tid(x, y, z, _idx, dim)
#define builtin_tid4d(x, y, z, w) wp::tid(x, y, z, w, _idx, dim)

#define builtin_block_dim() wp::block_dim()



extern "C" __global__ void finite_difference_ee291aa8_cuda_kernel_forward(
    wp::launch_bounds_t dim,
    wp::float32 var_dx,
    wp::array_t<wp::float32> var_u,
    wp::array_t<wp::float32> var_u_out)
{
    wp::tile_shared_storage_t tile_mem;

    for (size_t _idx = static_cast<size_t>(blockDim.x) * static_cast<size_t>(blockIdx.x) + static_cast<size_t>(threadIdx.x);
         _idx < dim.size;
         _idx += static_cast<size_t>(blockDim.x) * static_cast<size_t>(gridDim.x))
    {
            // reset shared memory allocator
        wp::tile_shared_storage_t::init();

        //---------
        // primal vars
        wp::int32 var_0;
        wp::shape_t* var_1;
        const wp::int32 var_2 = 0;
        wp::int32 var_3;
        wp::shape_t var_4;
        const wp::int32 var_5 = 1;
        wp::int32 var_6;
        wp::int32 var_7;
        wp::float32* var_8;
        const wp::float32 var_9 = 2.0;
        wp::float32* var_10;
        wp::float32 var_11;
        wp::float32 var_12;
        wp::float32 var_13;
        wp::float32 var_14;
        const wp::int32 var_15 = 1;
        wp::int32 var_16;
        wp::int32 var_17;
        wp::int32 var_18;
        wp::float32* var_19;
        wp::float32 var_20;
        wp::float32 var_21;
        wp::float32 var_22;
        wp::float32 var_23;
        //---------
        // forward
        // def finite_difference(dx: float, u: wp.array(dtype=float), u_out: wp.array(dtype=float)):       <L 12>
        // i = wp.tid()                                                                           <L 13>
        var_0 = builtin_tid1d();
        // total_points = u.shape[0]                                                              <L 14>
        var_1 = &(var_u.shape);
        var_4 = wp::load(var_1);
        var_3 = wp::extract(var_4, var_2);
        // u_out[i] = (                                                                           <L 15>
        // u[(i + 1) % total_points] - 2.0 * u[i] + u[(i - 1 + total_points) % total_points]       <L 16>
        var_6 = wp::add(var_0, var_5);
        var_7 = wp::mod(var_6, var_3);
        var_8 = wp::address(var_u, var_7);
        var_10 = wp::address(var_u, var_0);
        var_12 = wp::load(var_10);
        var_11 = wp::mul(var_9, var_12);
        var_14 = wp::load(var_8);
        var_13 = wp::sub(var_14, var_11);
        var_16 = wp::sub(var_0, var_15);
        var_17 = wp::add(var_16, var_3);
        var_18 = wp::mod(var_17, var_3);
        var_19 = wp::address(var_u, var_18);
        var_21 = wp::load(var_19);
        var_20 = wp::add(var_13, var_21);
        // ) / (dx * dx)                                                                          <L 17>
        var_22 = wp::mul(var_dx, var_dx);
        var_23 = wp::div(var_20, var_22);
        // u_out[i] = (                                                                           <L 15>
        wp::array_store(var_u_out, var_0, var_23);
    }
}



extern "C" __global__ void finite_difference_ee291aa8_cuda_kernel_backward(
    wp::launch_bounds_t dim,
    wp::float32 var_dx,
    wp::array_t<wp::float32> var_u,
    wp::array_t<wp::float32> var_u_out,
    wp::float32 adj_dx,
    wp::array_t<wp::float32> adj_u,
    wp::array_t<wp::float32> adj_u_out)
{
    wp::tile_shared_storage_t tile_mem;

    for (size_t _idx = static_cast<size_t>(blockDim.x) * static_cast<size_t>(blockIdx.x) + static_cast<size_t>(threadIdx.x);
         _idx < dim.size;
         _idx += static_cast<size_t>(blockDim.x) * static_cast<size_t>(gridDim.x))
    {
            // reset shared memory allocator
        wp::tile_shared_storage_t::init();

        //---------
        // primal vars
        wp::int32 var_0;
        wp::shape_t* var_1;
        const wp::int32 var_2 = 0;
        wp::int32 var_3;
        wp::shape_t var_4;
        const wp::int32 var_5 = 1;
        wp::int32 var_6;
        wp::int32 var_7;
        wp::float32* var_8;
        const wp::float32 var_9 = 2.0;
        wp::float32* var_10;
        wp::float32 var_11;
        wp::float32 var_12;
        wp::float32 var_13;
        wp::float32 var_14;
        const wp::int32 var_15 = 1;
        wp::int32 var_16;
        wp::int32 var_17;
        wp::int32 var_18;
        wp::float32* var_19;
        wp::float32 var_20;
        wp::float32 var_21;
        wp::float32 var_22;
        wp::float32 var_23;
        //---------
        // dual vars
        wp::int32 adj_0 = {};
        wp::shape_t adj_1 = {};
        wp::int32 adj_2 = {};
        wp::int32 adj_3 = {};
        wp::shape_t adj_4 = {};
        wp::int32 adj_5 = {};
        wp::int32 adj_6 = {};
        wp::int32 adj_7 = {};
        wp::float32 adj_8 = {};
        wp::float32 adj_9 = {};
        wp::float32 adj_10 = {};
        wp::float32 adj_11 = {};
        wp::float32 adj_12 = {};
        wp::float32 adj_13 = {};
        wp::float32 adj_14 = {};
        wp::int32 adj_15 = {};
        wp::int32 adj_16 = {};
        wp::int32 adj_17 = {};
        wp::int32 adj_18 = {};
        wp::float32 adj_19 = {};
        wp::float32 adj_20 = {};
        wp::float32 adj_21 = {};
        wp::float32 adj_22 = {};
        wp::float32 adj_23 = {};
        //---------
        // forward
        // def finite_difference(dx: float, u: wp.array(dtype=float), u_out: wp.array(dtype=float)):       <L 12>
        // i = wp.tid()                                                                           <L 13>
        var_0 = builtin_tid1d();
        // total_points = u.shape[0]                                                              <L 14>
        var_1 = &(var_u.shape);
        var_4 = wp::load(var_1);
        var_3 = wp::extract(var_4, var_2);
        // u_out[i] = (                                                                           <L 15>
        // u[(i + 1) % total_points] - 2.0 * u[i] + u[(i - 1 + total_points) % total_points]       <L 16>
        var_6 = wp::add(var_0, var_5);
        var_7 = wp::mod(var_6, var_3);
        var_8 = wp::address(var_u, var_7);
        var_10 = wp::address(var_u, var_0);
        var_12 = wp::load(var_10);
        var_11 = wp::mul(var_9, var_12);
        var_14 = wp::load(var_8);
        var_13 = wp::sub(var_14, var_11);
        var_16 = wp::sub(var_0, var_15);
        var_17 = wp::add(var_16, var_3);
        var_18 = wp::mod(var_17, var_3);
        var_19 = wp::address(var_u, var_18);
        var_21 = wp::load(var_19);
        var_20 = wp::add(var_13, var_21);
        // ) / (dx * dx)                                                                          <L 17>
        var_22 = wp::mul(var_dx, var_dx);
        var_23 = wp::div(var_20, var_22);
        // u_out[i] = (                                                                           <L 15>
        // wp::array_store(var_u_out, var_0, var_23);
        //---------
        // reverse
        wp::adj_array_store(var_u_out, var_0, var_23, adj_u_out, adj_0, adj_23);
        // adj: u_out[i] = (                                                                      <L 15>
        wp::adj_div(var_20, var_22, var_23, adj_20, adj_22, adj_23);
        wp::adj_mul(var_dx, var_dx, adj_dx, adj_dx, adj_22);
        // adj: ) / (dx * dx)                                                                     <L 17>
        wp::adj_add(var_13, var_21, adj_13, adj_19, adj_20);
        wp::adj_address(var_u, var_18, adj_u, adj_18, adj_19);
        wp::adj_mod(var_17, var_3, adj_17, adj_3, adj_18);
        wp::adj_add(var_16, var_3, adj_16, adj_3, adj_17);
        wp::adj_sub(var_0, var_15, adj_0, adj_15, adj_16);
        wp::adj_sub(var_14, var_11, adj_8, adj_11, adj_13);
        wp::adj_mul(var_9, var_12, adj_9, adj_10, adj_11);
        wp::adj_address(var_u, var_0, adj_u, adj_0, adj_10);
        wp::adj_address(var_u, var_7, adj_u, adj_7, adj_8);
        wp::adj_mod(var_6, var_3, adj_6, adj_3, adj_7);
        wp::adj_add(var_0, var_5, adj_0, adj_5, adj_6);
        // adj: u[(i + 1) % total_points] - 2.0 * u[i] + u[(i - 1 + total_points) % total_points]  <L 16>
        // adj: u_out[i] = (                                                                      <L 15>
        wp::adj_extract(var_4, var_2, adj_1, adj_2, adj_3);
        adj_u.shape = adj_1;
        // adj: total_points = u.shape[0]                                                         <L 14>
        // adj: i = wp.tid()                                                                      <L 13>
        // adj: def finite_difference(dx: float, u: wp.array(dtype=float), u_out: wp.array(dtype=float)):  <L 12>
        continue;
    }
}

