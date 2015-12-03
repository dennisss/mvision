/* Functions for doing camera projections and point triangulation */

#include "msckf.h"


MatrixXd pinv(MatrixXd &X){
	return (X.transpose()*X).inverse()*X.transpose();
}

/*
Using JacobiSVD: http://eigen.tuxfamily.org/index.php?title=FAQ

void pinv( MatrixType& pinvmat) const
   {
     eigen_assert(m_isInitialized && "SVD is not initialized.");
     double  pinvtoler=1.e-6; // choose your tolerance wisely!
     SingularValuesType singularValues_inv=m_singularValues;
     for ( long i=0; i<m_workMatrix.cols(); ++i) {
        if ( m_singularValues(i) > pinvtoler )
           singularValues_inv(i)=1.0/m_singularValues(i);
       else singularValues_inv(i)=0;
     }
     pinvmat= (m_matrixV*singularValues_inv.asDiagonal()*m_matrixU.transpose());
   }


*/


/*
	The filter holds the position of the body in the global frame, and the orientation in the global frame


	Position of camera in global =

*/


/* The h() function takes a 3d x,y,z point (expressed in the camera frame) and projects it to a 2d image point   */
// Note: this assumes a camera at the origin
// This parallels Shelley 5.1.2
Vector2d camera_project(Vector3d vec, Calibration &calib){


	double X = vec.x();
	double Y = vec.y();
	double Z = vec.z();

	// Project
	double u = X / Z, v = Y / Z;

	// Compute distortion
	double r = u*u + v*v;
	double dr = 1 + calib.k1 * r + calib.k2 * (r*r); //+ calib.k3 * (r*r*r);
	Vector2d dt(
		2*u*v*calib.t1 + (r + 2*(u*u))*calib.t2,
		2*u*v*calib.t2 + (r + 2*(v*v))*calib.t1
	);

	Matrix2d F;
	F << calib.f_x, 0,
		 0, calib.f_y;

	return Vector2d(calib.o_x, calib.o_y) + F*(dr*Vector2d(u, v) + dt);

}



// Computes the jacobian of the projection function at a 3d point
Matrix<double, 2, 3> JacobianH(Vector3d vec, Calibration &calib){


	double x = vec.x();
	double y = vec.y();
	double z = vec.z();

	double u = x / z, v = y / z;
	double r = u*u + v*v;

	Matrix<double, 1, 3> du_dx; du_dx << 1.0 / z, 0, -x/(z*z);
	Matrix<double, 1, 3> dv_dx; dv_dx << 0, 1.0 / z, -y/(z*z);

	Matrix<double, 1, 3> dr_dx = 2*u*du_dx + 2*v*dv_dx;


	// These lines are the same as in camera_project
	double dr = 1 + calib.k1 * r + calib.k2 * (r*r) ; //+ calib.k3 * (r*r*r);
	Vector2d dt(
		2*u*v*calib.t1 + (r + 2*(u*u))*calib.t2,
		2*u*v*calib.t2 + (r + 2*(v*v))*calib.t1
	);


	Matrix<double, 1, 3> ddr_dx = calib.k1*dr_dx + 2*calib.k2*r*dr_dx; //+ 3*calib.k3*(r*r);
	Matrix<double, 2, 3> ddt_dx;
	ddt_dx << 2*calib.t1*(u*dv_dx + du_dx*v) + (dr_dx + 4*u*du_dx)*calib.t2,
			  2*calib.t2*(u*dv_dx + du_dx*v) + (dr_dx + 4*v*dv_dx)*calib.t1;



	Matrix2d F;
	F << calib.f_x, 0,
		 0, calib.f_y;

	Matrix<double, 2, 3> M;
	M << (dr/z) + ddr_dx(0)*u, ddr_dx(1)*u, -(dr/z)*u + ddr_dx(2)*u,
		 ddr_dx(0)*v, (dr/z) + ddr_dx(1)*v, -(dr/z)*v + ddr_dx(2)*v;

	return F * (M + ddt_dx);
}


/* Pairwise least-squares  */
Vector3d triangulate(Quaterniond &qC1, Vector3d &pC1, Quaterniond &qC2, Vector3d &pC2, Vector2d &z1, Vector2d &z2, Calibration &calib){


	Vector3d v1(z1(0), z1(1), 1);
	Vector3d v2(z2(0), z2(1), 1);
	v1.normalize();
	v2.normalize();

	Matrix2d A;
	A << v1, v2;

//	Vector2d b =

	/////A << RowVector3d((z1.x() - calib.o_x) / calib.f_x, (z1.y() - calib.o_y) / calib.f_y, 1),

	Vector4d b;
	b << pC1,
		 pC2;


	Vector2d x; ///= pinv(A) * b;

	return Vector3d(0,0,0); //pC1 + x(0)*A.row(0).transpose();

	// Ray1 = pC1 + qC1.conjugate._transformVector(z1 - oCenter   ) k


}



#include <iostream>

/* Gauss-Newton triangulation of a point given many measurements */
Vector3d triangulate(vector<Quaterniond> &qC, vector<Vector3d> &pC, vector<Vector2d> &pts, Calibration &calib){

	// TODO: Model the first and last

	// There should be one observation per image
	//assert(qCs.size() == pts.size());





	// TODO: Normalize input measurements (undistort and apply (u - c_u) / f_u )


	// TODO: Initialize guess at the feature position from pair-wise camera triangulation of first and last poses
	//Vector3d initial = triangulate(R[0], p[0], R[R.size() - 1], p[p.size() - 1], pts[0], pts[pts.size() - 1]);
	// OpenCV has the triangulatePoints function for the pair-wise case


	// Compute poses relative to first camera
	// TODO: Using rotation matrices would probably be more efficient due to the number of operations
	vector<Quaterniond> C0qCi(pts.size());
	vector<Vector3d> CipC0(pts.size());
	for(int i = 0; i < pts.size(); i++){
		C0qCi[i] = qC[i] * qC[0].inverse();
		CipC0[i] = qC[i]._transformVector( pC[0] - pC[i] );
	}


	Vector3d theta(0, 0, 0.0001); // Initial inverse-depth parameter estimate (1 unit ahead of first camera

	/* Minimizing sum of square reprojection errors */

	int n = pts.size();
	MatrixXd Jf(2*n, 3); // Jacobian of reprojection errors
	VectorXd f(2*n); // Evaluation of reprojection error
	for(int it = 0; it < 20; it++){

		for(int i = 0; i < n; i++){

			// Evaluate it here
			// Convert inverse depth to x,y,z in current camera
			Vector3d v = C0qCi[i]._transformVector( Vector3d(theta(0), theta(1), 1) ) + theta(2)*CipC0[i];
			Vector2d x = camera_project(v, calib);

			Matrix3d Jg; // Jacobian of the inversedepth-to-3d function
			// R and p are the relative rotations and translations from the first camera
			Jg << C0qCi[i]._transformVector( Vector3d(1, 0, 0) ),
				  C0qCi[i]._transformVector( Vector3d(0, 1, 0) ),
				  CipC0[i];

			Jf.block<2,3>(2*i, 0) = JacobianH(v, calib)*Jg;

			f.segment<2>(2*i) = pts[i] - x;
		}




//		cout << "E: " << sqrt(f.squaredNorm() / pts.size()) << endl;

		// TODO: Check early termination based on f
		//if(f.squaredNorm() < 4*n)
		//	break;

//		cout << (pinv(Jf)*f) << endl << endl;

		// TODO: Should this be += or -=
		theta += pinv(Jf)*f;
	}

	// TODO: Make sure that no observations are triangulated behind the camera


	//cout << theta(0) << " " << theta(1) << " " << theta(2) << endl;


	if(theta(2) <= 0){
		return Vector3d(NAN, NAN, NAN);
	}


	return (1.0 / theta(2))*qC[0].conjugate()._transformVector(Vector3d(theta(0), theta(1), 1)) + pC[0];

}


/* Perform pair-wise camera triangulation */
//Vector3f triangulate(Matrix3f R1, Matrix3f p1, Matrix3f R2, Matrix3f p2, Vector2f pt1, Vector2f pt2){
//
//
//}

//Vector3d triangulate(vector<Matrix3f> R, vector<Vector3f> p, vector<Vector2f> pts){
//
//	return from_inversedepth(theta);
//}
