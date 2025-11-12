// Create a mesh with vertices but NO tetrahedral elements
// This will work with Nerve-Only constraint configuration

lc = 0.05;  // Mesh size

// Create vertices along a line (like your original 1D rod)
// But also create a minimal surface for bounding box calculation

// Rod vertices along Z-axis
Point(1) = {0, 0, 0, lc};     // Bottom
Point(2) = {0, 0, 0.25, lc};  
Point(3) = {0, 0, 0.5, lc};   
Point(4) = {0, 0, 0.75, lc};  
Point(5) = {0, 0, 1.0, lc};   // Top

// Add a few more vertices for a minimal "fake" 3D structure
// These give proper bounding box but no volume
Point(6) = {0.001, 0, 0, lc};      // Slightly offset bottom
Point(7) = {0, 0.001, 0, lc};      // Slightly offset bottom
Point(8) = {0.001, 0, 1.0, lc};    // Slightly offset top
Point(9) = {0, 0.001, 1.0, lc};    // Slightly offset top

// Create lines for nerve constraints
Line(1) = {1, 2};
Line(2) = {2, 3};
Line(3) = {3, 4};
Line(4) = {4, 5};

// Create minimal surface for bounding box (but no volume!)
Line(5) = {1, 6};
Line(6) = {1, 7};
Line(7) = {5, 8};
Line(8) = {5, 9};
Line(9) = {6, 7};
Line(10) = {8, 9};

// Create surface loops but NO volume extrusion
Line Loop(1) = {5, 9, -6};   // Bottom triangle
Line Loop(2) = {7, 10, -8};  // Top triangle
Plane Surface(1) = {1};
Plane Surface(2) = {2};

// Physical groups
Physical Line("nerve_edge") = {1, 2, 3, 4};  // Main nerve line
Physical Surface("rod_surface") = {1, 2};    // Minimal surfaces for geometry

// IMPORTANT: NO Physical Volume - this ensures no tetrahedra are generated