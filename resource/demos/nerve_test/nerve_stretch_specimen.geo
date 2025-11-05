// GMSH script for nerve stretch test - designed for 1D rod validation
// Creates a tissue sample with embedded linear nerve fiber
// This geometry is perfect for testing current constraints and future 1D rods

// Tissue specimen dimensions (rectangular bar like tendon/muscle fiber)
tissue_length = 0.2;    // 20cm long specimen
tissue_width = 0.04;    // 4cm wide
tissue_height = 0.04;   // 4cm thick

// Mesh density
tissue_lc = 0.008;      // Tissue mesh size
nerve_lc = 0.004;       // Finer mesh along nerve

// Create tissue geometry
Point(1) = {0.0, 0.0, 0.0, tissue_lc};
Point(2) = {tissue_length, 0.0, 0.0, tissue_lc};
Point(3) = {tissue_length, tissue_width, 0.0, tissue_lc};
Point(4) = {0.0, tissue_width, 0.0, tissue_lc};
Point(5) = {0.0, 0.0, tissue_height, tissue_lc};
Point(6) = {tissue_length, 0.0, tissue_height, tissue_lc};
Point(7) = {tissue_length, tissue_width, tissue_height, tissue_lc};
Point(8) = {0.0, tissue_width, tissue_height, tissue_lc};

// Create tissue box edges
Line(1) = {1, 2}; Line(2) = {2, 3}; Line(3) = {3, 4}; Line(4) = {4, 1};
Line(5) = {5, 6}; Line(6) = {6, 7}; Line(7) = {7, 8}; Line(8) = {8, 5};
Line(9) = {1, 5}; Line(10) = {2, 6}; Line(11) = {3, 7}; Line(12) = {4, 8};

// Create tissue surfaces
Line Loop(1) = {1, 2, 3, 4}; Plane Surface(1) = {1};  // bottom
Line Loop(2) = {5, 6, 7, 8}; Plane Surface(2) = {2};  // top
Line Loop(3) = {1, 10, -5, -9}; Plane Surface(3) = {3}; // front
Line Loop(4) = {2, 11, -6, -10}; Plane Surface(4) = {4}; // right
Line Loop(5) = {3, 12, -7, -11}; Plane Surface(5) = {5}; // back
Line Loop(6) = {4, 9, -8, -12}; Plane Surface(6) = {6}; // left

// Create tissue volume
Surface Loop(1) = {1, 2, 3, 4, 5, 6};
Volume(1) = {1};

// ========== NERVE FIBER DEFINITION ==========
// Create nerve as a series of connected points (future 1D rod path)
// Position nerve fiber along the central axis of the tissue

nerve_y = tissue_width / 2.0;   // Center of tissue width
nerve_z = tissue_height / 2.0;  // Center of tissue height

// Number of nerve segments (important for future 1D rod discretization)
n_segments = 20;
segment_length = tissue_length / n_segments;

// Create nerve points along the central axis
For i In {0:n_segments}
  x_pos = i * segment_length;
  Point(100 + i) = {x_pos, nerve_y, nerve_z, nerve_lc};
EndFor

// Create nerve segments (these will become constraint edges)
For i In {0:n_segments-1}
  Line(100 + i) = {100 + i, 100 + i + 1};
EndFor

// ========== PHYSICAL GROUPS ==========
// Define physical regions for material assignment and constraint definition

// Tissue volume
Physical Volume("tissue") = {1};

// Individual nerve segments (for current point-to-point constraints)
nerve_segments[] = {};
For i In {0:n_segments-1}
  nerve_segments[] += {100 + i};
EndFor
Physical Line("nerve_segments") = {nerve_segments[]};

// Complete nerve fiber (for future 1D rod element)
Physical Line("nerve_fiber") = {nerve_segments[]};

// Boundary regions for applying loads/constraints
Physical Surface("left_end") = {6};   // For fixed boundary condition
Physical Surface("right_end") = {4};  // For applied stretching load

// ========== EMBEDDING ==========
// Embed nerve lines in tissue volume (ensures nerve nodes are part of tissue mesh)
Line{nerve_segments[]} In Volume{1};

// ========== MESH REFINEMENT ==========
// Ensure good mesh quality around nerve fiber
Field[1] = Distance;
Field[1].LinesList = {nerve_segments[]};

Field[2] = Threshold;
Field[2].IField = 1;
Field[2].LcMin = nerve_lc;     // Fine mesh near nerve
Field[2].LcMax = tissue_lc;    // Coarse mesh away from nerve
Field[2].DistMin = 0.005;      // 5mm refinement zone
Field[2].DistMax = 0.015;      // 15mm transition zone

Background Field = 2;