// GMSH script to create a rectangular tissue bar with nerve fibers
// Save as tissue_bar_nerve.geo and run: gmsh -3 tissue_bar_nerve.geo

// Define tissue bar dimensions
bar_length = 0.3;   // 30cm long
bar_width = 0.05;   // 5cm wide  
bar_height = 0.05;  // 5cm tall

// Mesh density
lc = 0.01;

// Create main tissue bar geometry
Point(1) = {0, 0, 0, lc};
Point(2) = {bar_length, 0, 0, lc};
Point(3) = {bar_length, bar_width, 0, lc};
Point(4) = {0, bar_width, 0, lc};
Point(5) = {0, 0, bar_height, lc};
Point(6) = {bar_length, 0, bar_height, lc};
Point(7) = {bar_length, bar_width, bar_height, lc};
Point(8) = {0, bar_width, bar_height, lc};

// Create lines for the box
Line(1) = {1, 2};
Line(2) = {2, 3};
Line(3) = {3, 4};
Line(4) = {4, 1};
Line(5) = {5, 6};
Line(6) = {6, 7};
Line(7) = {7, 8};
Line(8) = {8, 5};
Line(9) = {1, 5};
Line(10) = {2, 6};
Line(11) = {3, 7};
Line(12) = {4, 8};

// Create surfaces
Line Loop(1) = {1, 2, 3, 4};
Plane Surface(1) = {1};
Line Loop(2) = {5, 6, 7, 8};
Plane Surface(2) = {2};
Line Loop(3) = {1, 10, -5, -9};
Plane Surface(3) = {3};
Line Loop(4) = {2, 11, -6, -10};
Plane Surface(4) = {4};
Line Loop(5) = {3, 12, -7, -11};
Plane Surface(5) = {5};
Line Loop(6) = {4, 9, -8, -12};
Plane Surface(6) = {6};

// Create volume
Surface Loop(1) = {1, 2, 3, 4, 5, 6};
Volume(1) = {1};

// Add nerve fiber lines (longitudinal fibers along the bar)
nerve_y1 = bar_width * 0.3;  // First nerve at 30% width
nerve_y2 = bar_width * 0.7;  // Second nerve at 70% width
nerve_z = bar_height * 0.5;  // Middle height

// Nerve fiber 1
Point(101) = {0, nerve_y1, nerve_z, lc};
Point(102) = {bar_length, nerve_y1, nerve_z, lc};
Line(101) = {101, 102};

// Nerve fiber 2  
Point(103) = {0, nerve_y2, nerve_z, lc};
Point(104) = {bar_length, nerve_y2, nerve_z, lc};
Line(102) = {103, 104};

// Define physical groups
Physical Volume("tissue") = {1};
Physical Line("nerve_fiber_1") = {101};
Physical Line("nerve_fiber_2") = {102};

// Embed nerve lines in the tissue volume
Line{101, 102} In Volume{1};