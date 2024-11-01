FROM ubuntu:noble AS build
# install necessary packages
RUN apt-get update && apt-get install -y \
    g++ \
    git \
    cmake \
    xorg-dev \
    libassimp-dev \
    wget \
    # needed by OpenHaptics API
    libncurses-dev \
    freeglut3-dev \
    build-essential \
    libusb-1.0-0-dev

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

# install OpenHaptics drivers
WORKDIR /thirdparty
RUN wget https://s3.amazonaws.com/dl.3dsystems.com/binaries/support/downloads/KB+Files/Open+Haptics/openhaptics_3.4-0-developer-edition-amd64.tar.gz
RUN tar -xvf openhaptics_3.4-0-developer-edition-amd64.tar.gz
RUN wget https://s3.amazonaws.com/dl.3dsystems.com/binaries/Sensable/Linux/TouchDriver_2023_11_15.tgz
RUN tar -xvf TouchDriver_2023_11_15.tgz

# copied from OpenHaptics ./install script
WORKDIR /thirdparty/openhaptics_3.4-0-developer-edition-amd64
RUN cp -R opt/* /opt                                                                                                                                             
RUN cp -R usr/lib/* /usr/lib                                                                                                                                     
RUN cp -R usr/include/* /usr/include                                                                                                                                                                        
RUN ln -sfn /usr/lib/libHD.so.3.4.0 /usr/lib/libHD.so.3.4                                                                                                                  
RUN ln -sfn /usr/lib/libHD.so.3.4.0 /usr/lib/libHD.so                                                                                                                                                                                                                                                                                          
RUN ln -sfn /usr/lib/libHL.so.3.4.0 /usr/lib/libHL.so.3.4                                                                                                                  
RUN ln -sfn /usr/lib/libHL.so.3.4.0 /usr/lib/libHL.so                                                                                                                                                                                                                                                                                              
RUN ln -sfn /usr/lib/libQH.so.3.4.0 /usr/lib/libQH.so.3.4                                                                                                                  
RUN ln -sfn /usr/lib/libQH.so.3.4.0 /usr/lib/libQH.so                                                                                                                    
RUN ln -sfn /usr/lib/libQHGLUTWrapper.so.3.4.0 /usr/lib/libQHGLUTWrapper.so.3.4                                                                                            
RUN ln -sfn /usr/lib/libQHGLUTWrapper.so.3.4.0 /usr/lib/libQHGLUTWrapper.so                                                                                                                                                                                                                                               
RUN chmod -R 777 /opt/OpenHaptics
RUN export OH_SDK_BASE=/opt/OpenHaptics/Developer/3.4-0

# follow driver installation instructions
WORKDIR /thirdparty/TouchDriver_2023_11_15
RUN cp usr/lib/libPhantomIOLib42.so /usr/lib/

# make symbolic links so API examples build
RUN ln -s /usr/lib/x86_64-linux-gnu/libglut.so /usr/lib/libglut.so.3
RUN ln -s /usr/lib/x86_64-linux-gnu/libncurses.so.6 /usr/lib/libncurses.so.5
RUN ln -s /usr/lib/x86_64-linux-gnu/libtinfo.so.6 /usr/lib/libtinfo.so.5

####### Install NVidia stuff
# for getting libtinfo5 which is needed by CUDA 12.4
RUN echo "Types: deb" >> /etc/apt/sources.list.d/ubuntu.sources
RUN echo "URIs: http://archive.ubuntu.com/ubuntu/" >> /etc/apt/sources.list.d/ubuntu.sources
RUN echo "Suites: lunar" >> /etc/apt/sources.list.d/ubuntu.sources
RUN echo "Components: universe" >> /etc/apt/sources.list.d/ubuntu.sources
RUN echo "Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg" >> /etc/apt/sources.list.d/ubuntu.sources

# install CUDA 12.4
WORKDIR /thirdparty
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
RUN mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
RUN wget https://developer.download.nvidia.com/compute/cuda/12.4.0/local_installers/cuda-repo-ubuntu2204-12-4-local_12.4.0-550.54.14-1_amd64.deb
RUN dpkg -i cuda-repo-ubuntu2204-12-4-local_12.4.0-550.54.14-1_amd64.deb
RUN cp /var/cuda-repo-ubuntu2204-12-4-local/cuda-*-keyring.gpg /usr/share/keyrings/
RUN apt-get update
RUN apt-get -y install cuda-toolkit-12-4

# finish installation by updating path
RUN export PATH=/usr/local/cuda-12.4/bin${PATH:+:${PATH}}
RUN export LD_LIBRARY_PATH=/usr/local/cuda-12.4/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

# clone cuda samples
RUN git clone https://github.com/NVIDIA/cuda-samples.git


WORKDIR /workspace

CMD "/bin/bash"