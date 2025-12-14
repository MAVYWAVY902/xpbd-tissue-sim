
import warp as wp
import numpy as np
import os

wp.config.kernel_cache_dir = os.path.join(
    os.path.dirname(os.path.realpath(__file__)), "Chapter_12_finite_difference_example_cache"
)


@wp.kernel
def finite_difference(dx: float, u: wp.array(dtype=float), u_out: wp.array(dtype=float)):
    i = wp.tid()
    total_points = u.shape[0]
    u_out[i] = (
        u[(i + 1) % total_points] - 2.0 * u[i] + u[(i - 1 + total_points) % total_points]
    ) / (dx * dx)


sin_array_np = np.sin(np.linspace(0, 2 * np.pi, 1024))
sin_array_wp = wp.array(sin_array_np, dtype=float)

u_out_wp = wp.empty_like(sin_array_wp)

dx = 2 * np.pi / 1024

wp.launch(finite_difference, sin_array_wp.shape, inputs=[dx, sin_array_wp], outputs=[u_out_wp])

print(f"{sin_array_wp.numpy()=}")
print(f"{u_out_wp.numpy()=}")
