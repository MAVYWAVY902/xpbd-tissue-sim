// Simple ellipsoid geometry for tumor
SetFactory("OpenCASCADE");

// Create ellipsoid
Sphere(1) = {0.5, 0.5, 0.5, 0.3};  // center (0.5,0.5,0.5), radius 0.3

// Scale to make it ellipsoidal
Dilate {{0.5, 0.5, 0.5}, {1.2, 0.8, 1.0}} {
  Volume{1};
}

// Set mesh size
Mesh.CharacteristicLengthMin = 0.05;
Mesh.CharacteristicLengthMax = 0.1;

// Physical groups for material assignment
Physical Volume("ellipsoid_tumor") = {1};

// IMPORTANT: Also create physical surface for visualization
Physical Surface("ellipsoid_surface") = {1};

// Generate 2D surface mesh first, then 3D
Mesh 2;
Mesh 3;