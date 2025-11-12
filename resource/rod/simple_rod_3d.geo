// Simple thin 3D rod that generates tetrahedra
// Parameters
lc = 0.05;  // Mesh size
radius = 0.01;  // Very thin radius
height = 1.0;   // Rod height

// Bottom circle points
Point(1) = {0, 0, 0, lc};         // Center
Point(2) = {radius, 0, 0, lc};    // Right
Point(3) = {0, radius, 0, lc};    // Top
Point(4) = {-radius, 0, 0, lc};   // Left
Point(5) = {0, -radius, 0, lc};   // Bottom

// Circle arcs
Circle(1) = {2, 1, 3};
Circle(2) = {3, 1, 4};
Circle(3) = {4, 1, 5};
Circle(4) = {5, 1, 2};

// Surface
Line Loop(1) = {1, 2, 3, 4};
Plane Surface(1) = {1};

// Extrude to create 3D volume
Extrude {0, 0, height} {
  Surface{1}; Layers{20};
}

// Central line for nerve constraints (embedded in volume)
Point(10) = {0, 0, 0, lc};
Point(11) = {0, 0, height, lc};
Line(10) = {10, 11};

// Physical entities
Physical Volume("rod") = {1};
Physical Line("nerve_edge") = {10};