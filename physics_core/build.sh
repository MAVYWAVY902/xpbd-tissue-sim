#!/bin/bash
# Build XPBDPhysicsCore static library inside Docker container.
# Usage: run this script inside the Docker container with /workspace mounted.
#
# From the host:
#   docker compose -f docker-compose-cpu.yml run dev bash /workspace/physics_core/build.sh

set -ex

cd /workspace/physics_core
rm -rf build && mkdir build && cd build

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/workspace/physics_core/install

make -j$(nproc)
make install

echo ""
echo "========================================"
echo " Build complete!"
echo " Library:  /workspace/physics_core/install/lib/libxpbd_physics.a"
echo " Headers:  /workspace/physics_core/install/include/"
echo "========================================"
