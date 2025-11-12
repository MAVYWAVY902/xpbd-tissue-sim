// Create a working mesh for Nerve-Only constraints
// This mesh has proper vertices and minimal geometry but no tetrahedra
lc = 0.1;

// Create vertices along Z-axis (vertical rod)
Point(1) = {0, 0, 0, lc};     // Bottom
Point(2) = {0, 0, 0.2, lc};  
Point(3) = {0, 0, 0.4, lc};   
Point(4) = {0, 0, 0.6, lc};  
Point(5) = {0, 0, 0.8, lc};   
Point(6) = {0, 0, 1.0, lc};   // Top

// Create a very small offset for minimal 3D structure
Point(11) = {0.001, 0, 0, lc};     // Offset bottom
Point(16) = {0.001, 0, 1.0, lc};   // Offset top

// Create nerve line segments
Line(1) = {1, 2};
Line(2) = {2, 3};
Line(3) = {3, 4};
Line(4) = {4, 5};
Line(5) = {5, 6};

// Create minimal surface elements for proper bounding box
Line(10) = {1, 11};
Line(11) = {6, 16};
Line(12) = {11, 16};

Line Loop(1) = {10, 12, -11, -1, -2, -3, -4, -5};
Plane Surface(1) = {1};

// Physical groups
Physical Line("nerve_edge") = {1, 2, 3, 4, 5};  // Nerve constraints
Physical Surface("rod_surface") = {1};           // Minimal geometry

// Force 2D mesh generation (no 3D tetrahedra)
Mesh 2;