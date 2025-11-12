// rod_thin_3d.geo
// Create a very thin 3D rod that works with tetrahedral constraints
// but is essentially 1D in behavior

// Rod parameters
rod_length = 1.0;
rod_radius = 0.01; // Very small radius
mesh_size = 0.05;

// Create points along the rod axis
Point(1) = {0, 0, 0, mesh_size};        // Bottom
Point(2) = {0, 0, rod_length, mesh_size}; // Top

// Create a line and extrude to make a thin cylinder
Line(1) = {1, 2};

// Create a very thin circular cross-section
Point(3) = {rod_radius, 0, 0, mesh_size};
Point(4) = {-rod_radius, 0, 0, mesh_size};
Point(5) = {0, rod_radius, 0, mesh_size};
Point(6) = {0, -rod_radius, 0, mesh_size};

Circle(2) = {3, 1, 5};
Circle(3) = {5, 1, 4};
Circle(4) = {4, 1, 6};
Circle(5) = {6, 1, 3};

// Create surface
Line Loop(1) = {2, 3, 4, 5};
Plane Surface(1) = {1};

// Extrude along the rod to create volume
Extrude {0, 0, rod_length} {
  Surface{1};
  Layers{20}; // 20 layers along length
  Recombine;
}

// Physical groups
Physical Volume("rod_volume") = {1};
Physical Surface("rod_bottom") = {1};    // Bottom surface
Physical Surface("rod_top") = {26};      // Top surface (check this number)

// Create nerve edge along the central axis for nerve constraints
Physical Line("nerve_edge") = {1};