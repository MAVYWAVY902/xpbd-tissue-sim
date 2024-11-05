FROM ubuntu:noble AS build

#########################
# Package install
#########################

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



########################
# YAML-cpp
########################
RUN git clone https://github.com/jbeder/yaml-cpp.git
WORKDIR yaml-cpp/build
RUN cmake ..
RUN make -j 8
RUN make install


#########################
# Eigen install
#########################
WORKDIR /thirdparty
RUN git clone https://gitlab.com/libeigen/eigen.git



###########################
# Easy3D install
###########################
RUN git clone https://github.com/LiangliangNan/Easy3D.git
WORKDIR Easy3D/build
RUN cmake ..
RUN make -j 8
RUN make install



########################
# GMSH install
########################
WORKDIR /thirdparty
RUN git clone https://gitlab.onelab.info/gmsh/gmsh.git
WORKDIR gmsh/build
# install from source as a dynamic library for access to C++ API
RUN cmake -DENABLE_BUILD_DYNAMIC=1 ..
RUN make -j 8
RUN make install




################################## 
# Geomagic Touch device setup
##################################

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


WORKDIR /workspace

CMD "/bin/bash"