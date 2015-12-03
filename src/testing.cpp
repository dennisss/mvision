#include <msckf.h>

#include "projection.h"

#include <iostream>
#include <cmath>
#include <random>

using namespace std;


#define M_PI 3.14159265358979323846

void test_triangulate(){

	// Tests for the triangulation procedure

	Calibration calib;
	calib.o_x = 300; calib.o_y = 200;
	calib.f_x = 100; calib.f_y = 100;
	calib.k1 = 0; calib.k2 = 0; calib.t1 = 0; calib.t2 = 0;


	vector<Quaterniond> qs;
	vector<Vector3d> ps;


	qs.push_back(Quaterniond(cos( (3.14159 / 6) / 2  ), sin( (3.14159 / 6) / 2), 0, 0));
	qs.back().normalize();
	ps.push_back(Vector3d(1.5, 0, 0));


	qs.push_back(Quaterniond(cos( (3.14159 / 6) / 2  ), 0, sin( (3.14159 / 6) / 2), 0));
	qs.back().normalize();
	ps.push_back(Vector3d(-1.5, 0, 0));


	qs.push_back(Quaterniond(1, 0, 0, 0));
	ps.push_back(Vector3d(0, 0, 0));


	Vector3d v(0, 0, 10);

	Vector2d noise[] = {Vector2d(0.5, -0.2), Vector2d(-0.1, 0.1), Vector2d(-0.4, -0.1)};

	vector<Vector2d> z;
	for(int i = 0; i < qs.size(); i++){
		z.push_back(camera_project( qs[i]._transformVector( v - ps[i] ), calib) + noise[i]);
		cout << i << ": " << z.back()[0] << " " << z.back()[1] << endl;
	}

	Vector3d t = triangulate(qs, ps, z, calib);

	cout << t << endl;

}


//default_random_engine generator;
mt19937 generator;
normal_distribution<double> distribution(0.0, 1.0); // noise

double noise(){
	return distribution(generator);
}


void test_msckf(){

	for(int i = 0; i < 15; i++){
		cout << "noise: " << noise() << endl;
	}


	MSCKF model;


	model.calib.sigma_ac = 0.01;
	model.calib.sigma_gc = 0.001;
	model.calib.sigma_wac = 0.001;
	model.calib.sigma_wgc = 0.0001;

	model.calib.sigma_img = 0.1;

	model.calib.Rbc << 1, 0, 0,
					   0, 1, 0,
					   0, 0, 1;

//	model.calib.Rbc << 0, -1, 0, // Body to camera
//				 -1, 0, 0,
//				 0, 0, -1;

	model.calib.CpB << 0.0, 0, 0; // 0.06, 0, 0
	//model.calib.BpC << 0, 0.06, 0;

	model.calib.f_x = 100;
	model.calib.f_y = 100;
	model.calib.o_x = 300;
	model.calib.o_y = 200;
	model.calib.k1 = 0;
	model.calib.k2 = 0;
	model.calib.t1 = 0;
	model.calib.t2 = 0;

	model.aB_last = Vector3d(1, 0, 9.81);
	model.wB_last = Vector3d(0, 0, 0);


	//double theta = - M_PI / 2.0;
	//model.state.segment<4>(0) = Vector4d(0, -sin(theta / 2), 0, cos(theta / 2));

	//Matrix3d R;
	//R << 0, 0, 1,
	//	 0, 1, 0,
	//	 1, 0, 0;
	//model.state.segment<4>(0) = Quaterniond(R).coeffs();

	model.state.segment<4>(0) = Vector4d(0,0,0,1);


	// Test steady state
	model.time = 0;
	for(int i = 0; i < 100; i++){

		// Compute ground truth
		double t = i*0.01;
		Vector3d p = 0.5*Vector3d(1,0,0)*t*t;
		Vector3d v = Vector3d(1,0,0)*t;

		model.propagate_inertial(model.aB_last, model.wB_last, i*(NANOPERSEC / 100));



		// Generate synthetic image measurements
		if(i % 10 == 0){
			vector<Vector3d> points = {
				Vector3d(0, 0, 10),
				Vector3d(2, 2, 5),
				Vector3d(-2, -2, 10),
				Vector3d(-2, 2, 5),
				Vector3d(2, -2, 10)
			};


			vector<Vector2d> measurements(points.size());
			vector<int> matches(points.size(), -1);


			for(int j = 0; j < points.size(); j++){

				measurements[j] = camera_project(points[j] - p, model.calib);

				measurements[j] += Vector2d(model.calib.sigma_img*noise(), model.calib.sigma_img*noise());

				if(i != 50  && i != 90){
					matches[j] = j; // Match each to the same one in the previous frame
				}


			}

			model.update_image(measurements, matches);

		}




		cout << i << ": " << model.state.segment<7>(0).transpose() << endl;

		if(i == 50){
			//model.covar = MatrixXd::Identity(model.covar.rows(), model.covar.cols());

			model.state.segment<4>(0) = Vector4d(0,0,0,1);
			model.state.segment<3>(4) = p;
			model.state.segment<3>(7) = v;
			model.state.segment<3>(10) = Vector3d(0,0,0);
			model.state.segment<3>(13) = Vector3d(0,0,0);
		}


	}







}

