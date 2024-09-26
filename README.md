# XPBD_Sandbox
#### A place to prototype and test algorithms and approaches for simulation of highly deformable elastic materials.

## Running the code
A `Dockerfile` and `docker-compose.yml` have been provided that will set up an interactive Docker container in which to run the code. It will handle the download and installation of the 3rd party libraries required to build and run the code ([Eigen](https://gitlab.com/libeigen/eigen), [Easy3D](https://github.com/LiangliangNan/Easy3D), [yaml-cpp](https://github.com/jbeder/yaml-cpp), [gmsh](https://gitlab.onelab.info/gmsh/gmsh)).

### Linux
First, install the Docker engine (instructions for Ubuntu [here](https://docs.docker.com/engine/install/ubuntu/)).

Then, install the [Docker compose plugin](https://docs.docker.com/compose/install/linux/#install-using-the-repository).

Then, run 

```
xhost +Local:*
```
in your terminal. This will allow Docker to display graphics natively using X11 forwarding.

Finally, we are ready to spin up the Docker container. Navigate to the root directory of the repo (where the `Dockerfile` and `docker-compose.yml` files are). To build the container using Docker compose, run:

```
docker compose up -d --build
```

This may take a while as it downloads and installs the necessary packages and pieces of code. When it has finished building, run:

```
docker ps
```

to ensure that the container is up and running. The output should be something like the following:
![image](https://github.com/user-attachments/assets/dd54e341-ec57-4f8a-a0b6-a82e6ecb475b)
Note the name of the container in the right-most column -- we'll need this to attach to the container to run the code.

To attach to the container, run:

```
docker exec -it sim-dev-1 /bin/bash
```

Where `sim-dev-1` is the name of the container. Your terminal should switch to a `/workspace` directory, and you're inside the Docker container!

To build the code, navigate to the `/workspace/build` directory. Then run:

```
cmake ..
make -j 8
```

This will create a `Test` binary that we can run with:

```
./Test
```

If all goes well, an Easy3D graphics window should pop up!

To exit out of the container, simply use

```
exit
```
However, the container will still be running at this point. You can `docker ps` to verify this fact. To kill the container, you can use:
```
docker kill sim-dev-1
```

When spinning up and attaching to the container again, you **do not need to rebuild the container** (unless dependencies or the `Dockerfile` have changed). Simply use:
```
docker compose up -d
docker exec -it sim-dev-1 /bin/bash
```

### Windows
Using WSL, the process should be similar, though there might be some extra stuff to do with the X11 forwarding.

## Modifying the code
The Docker container is set up to share the repo files outside of the container. That means that you can make edits to files **outside** the container and have those changes be reflected **inside** the container! This is nice because then your code editor does not need to be launched from inside the Docker container.

