# XPBD_Sandbox
#### A place to prototype and test algorithms and approaches for simulation of highly deformable elastic materials.

## Running the code
A `Dockerfile` and `docker-compose.yml` have been provided that will set up an interactive Docker container in which to run the code. It will handle the download and installation of the 3rd party libraries required to build and run the code ([Eigen](https://gitlab.com/libeigen/eigen), [Easy3D](https://github.com/LiangliangNan/Easy3D), [yaml-cpp](https://github.com/jbeder/yaml-cpp), [gmsh](https://gitlab.onelab.info/gmsh/gmsh)).

### Linux
First, install the Docker engine (instructions for Ubuntu [here](https://docs.docker.com/engine/install/ubuntu/)).

Then, install the [Docker compose plugin](https://docs.docker.com/compose/install/linux/#install-using-the-repository).

If using a NVIDIA GPU, install the NVIDIA Container Toolkit (instructions for Linux [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)). Be sure to restart the Docker daemon after installing.
If not using a GPU, comment out the following lines in `docker-compose.yml`: 

https://github.com/smtobin/XPBD_Sandbox/blob/b773619f3a5930c899f1003c76c69ad102699581/docker-compose.yml#L16C5-L22C34. 

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

## Code Structure
This section will briefly go over how the repository is organized, and generally how the code is structured.

### Simulation
The `Simulation` class (found in the `simulation/` folder) is the owner/manager of everything in a simulation. It is responsible for creating and updating the objects in the sim and the visualization (if enabled). Executable files (e.g., `exec/main.cpp`) use the `run()` method to start the simulation, which will first call the `setup()` method, then spawn a thread that will perform the updates (i.e. it repeatedly calls the `update()` method which steps forward in time), and lastly launch the visualization.

Various simulation functionalities are delegated to specific "scenes" to abstract their functionality. For example, the child `GraphicsScene` object handles the visualization aspects of the simulation, and the child `CollisionScene` object handles the collisions between objects. Note that these do not have to be used in every simulation (i.e. graphics/collisions can be turned off), and not all objects in the simulation have to be added to each scene (i.e. certain things may not be used in collision or visualization).

There are multiple derived `Simulation` classes that augment the functionality to aid in simulating specific scenarios. The `OutputSimulation` provides a way to print simulation data to file at a regular interval, and `ResidualSimulation` prints the residuals of the objects in the scene. Further derived classes perform specific simulations that are accessed through their own executables. For example, `BeamSimulation` cantilevers an object and measures its deflection.

### SimObject
The `simobject/` folder contains all the simulation objects which move around and interact with one another, all inheriting from the abstract base `MeshObject` class. The `MeshObject` class is really just a mesh class that stores vertices and faces (triangles), with some added functionality on top. Part of the `MeshObject` class is the `update()` method, which steps the object forward one time step.

Derived from this is `ElasticMeshObject`, which is still abstract, and extends `MeshObject` for continuous hyperelastic materials represented by volumetric tetrahedral meshes. The vertices in the mesh can move independently from one another (i.e. they have their own velocity), and the mesh has elastic material properties (defined by the `ElasticMaterial` class).

Derived from `ElasticMeshObject` is `XPBDMeshObject`, which simulates hyperelastic objects using the XPBD algorithm. It has an array of `Constraint`s that are defined on elements in the mesh and an `XPBDSolver` which solves these constraints and updates the mesh's positions.

`FirstOrderXPBDMeshObject` is derived from `XPBDMeshObject`, and overrides specific behavior to implement the first-order XPBD algorithm.

### Solver
The `solver/` folder contains everything to do with projecting constraints using the XPBD algorithm.

The `Constraint` abstract base class implements a generic interface for a constraint (which does not have to necessarily be used with the XPBD algorithm). This interface basically revolves around two functions: `evaluate()` which evaluates the constraint function `C(x)`, and `gradient()` which evaluates the constraint gradient with respect to the positions that make up that constraint. All classes derived from `Constraint` (e.g. `HydrostaticConstraint`, `DeviatoricConstraint`) implement these functions enabling the generic evaluation of a constraint and its gradient.

The `ConstraintProjector` base class implements the base XPBD projection algorithm (i.e. solving Equation (16) in the XPBD paper for $\Delta\lambda$), and outputs the resulting position updates ($\Delta \mathbf{x}$). This is implemented in the `project()` class method, which is split into numerous helper functions that can be overridden by derived `ConstraintProjector` classes to augment or change the projection algorithm. Also, the `ConstraintProjector` class supports simultaneously solving an arbitrary number of constraints at a single time.

The decorator design pattern is used to dynamically augment the projection formulas used by `ConstraintProjector`. The base decorator is the class `ConstraintProjectorDecorator`. For example, the `WithDamping` decorator overrides some of the helper functions used in `project()` such that the update formula includes constraint damping (i.e. Equation (25) in the XPBD paper). By using the dcorator pattern, **any** `ConstraintProjector` can be converted to use constraint damping, without having to explicitly write a derived class that implements it for each constraint.

The `XPBDSolver` base class implements an interface for solving the constraints with its `solve()` class method. It owns a list of `ConstraintProjector`s that it will use to iteratively solve the constraints - i.e. by calling the `project()` method for each `ConstraintProjector` and applying the position updates. How the `XPBDSolver` applies the position updates is up to the derived classes, one solver iteration is contained in the protected method `_solveConstraints()`. For example, `XPBDGaussSeidelSolver` uses a Gauss-Seidel update strategy (as described in the XPBD paper) which updates the positions immediately after projecting each constraint.

### Config
The `include/config/` folder contains the `Config` class and its derived classes that parse YAML config files and make the parameters available for object instantiation. The base `Config` class provides the machinery needed (with the help of the [yaml-cpp](https://github.com/jbeder/yaml-cpp) library) to parse a YAML node/subnode and extract expected parameters from it using the `_extractParameter()` templated function. Static default values can be set and used if the YAML parameter is not found. If the extracted parameter is limited to a set of options (i.e. it should be a choice from an enum), `_extractParameterWithOptions()` can be used instead, and we pass in a static map that maps user-specified text to the appropriate value (e.g. from "Gauss-Seidel" to `XPBDSolverType::GAUSS_SEIDEL`).

Classes derived from `Config` correspond to specific objects and have extract additional parameters that correspond specifically to the options of that type of object. For example, `MeshObjectConfig` extract information from the YAML file that is used to set up a `MeshObject`, such as the size, initial position, initial velocity, color, etc.






