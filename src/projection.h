#ifndef PROJECTION_H_
#define PROJECTION_H_

#include "msckf.h"

/* Functions for doing camera projections and related things */


Matrix<double, 2, 3> JacobianH(Vector3d v, Calibration &calib);

Vector3d triangulate(vector<Quaterniond> &qs, vector<Vector3d> &ps, vector<Vector2d> &pts, Calibration &calib);

Vector2d camera_project(Vector3d vec, Calibration &calib);


#endif
