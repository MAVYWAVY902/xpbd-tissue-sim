FROM ubuntu:noble AS build
# install necessary packages
RUN apt-get update && apt-get install -y \
    g++ \
    git \
    cmake \
    xorg-dev \
    libassimp-dev

# create directory for third party software
WORKDIR /thirdparty

# install yaml-cpp
RUN git clone https://github.com/jbeder/yaml-cpp.git
WORKDIR yaml-cpp/build
RUN cmake ..
RUN make -j 8
RUN make install

#install Eigen
WORKDIR /thirdparty
RUN git clone https://gitlab.com/libeigen/eigen.git

# install Easy3D
RUN git clone https://github.com/LiangliangNan/Easy3D.git
WORKDIR Easy3D/build
RUN cmake ..
RUN make -j 8
RUN make install

# install gmsh from source as a dynamic library for access to C++ API
WORKDIR /thirdparty
RUN git clone https://gitlab.onelab.info/gmsh/gmsh.git
WORKDIR gmsh/build
RUN cmake -DENABLE_BUILD_DYNAMIC=1 ..
RUN make -j 8
RUN make install

WORKDIR /workspace

CMD "/bin/bash"