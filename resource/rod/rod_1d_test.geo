// rod_1d_test.geo
Point(1) = {0, 0, 1.0, 1e-3};
Point(2) = {0, 0, 0.75, 1e-3};
Point(3) = {0, 0, 0.5, 1e-3};
Point(4) = {0, 0, 0.25, 1e-3};

Line(1) = {1, 2};
Line(2) = {2, 3};
Line(3) = {3, 4};

Physical Line("nerve_edge") = {1, 2, 3};