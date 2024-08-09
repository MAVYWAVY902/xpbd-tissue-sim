FROM ubuntu:noble AS build
RUN apt-get update && apt-get install -y \
    g++ \
    git \
    cmake \
    xorg-dev \
    libassimp-dev

WORKDIR /thirdparty

RUN git clone https://github.com/jbeder/yaml-cpp.git
WORKDIR yaml-cpp/build
# RUN pwd
# RUN mkdir build
# RUN cd build
RUN cmake ..
RUN make -j 8
RUN make install

WORKDIR /thirdparty
# RUN cd ..
RUN git clone https://gitlab.com/libeigen/eigen.git

RUN git clone https://github.com/LiangliangNan/Easy3D.git
WORKDIR Easy3D/build
RUN cmake ..
RUN make -j 8
RUN make install

WORKDIR /thirdparty
RUN git clone https://gitlab.onelab.info/gmsh/gmsh.git
WORKDIR gmsh/build
RUN cmake -DENABLE_BUILD_DYNAMIC=1 ..
RUN make -j 8
RUN make install


WORKDIR /workspace
# COPY . .

CMD "/bin/bash"