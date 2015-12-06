#include "msckf.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>

using namespace Eigen;


static Matrix3d Ta, Tg, Ts;


MSCKF::MSCKF(){

	//Matrix3f Ta; // Shape matrix: Scale and misalignment for accelerometer
	Ta << 1, 0, 0,
		  0, 1, 0,
		  0, 0, 1;



	//Matrix3f Tg; // gyro shape matrix
	Tg << 1, 0, 0,
		  0, 1, 0,
		  0, 0, 1;

	// Gravity is positive in the +z direction (normal force)

	//Matrix3f Ts; // g-sensitivity of gyroscrope. This is multiplied by the acceleration
	Ts << 0, 0, 0,
		  0, 0, 0,
		  0, 0, 0;


	// TODO: Initialize state estimate





	// Each frame is of size 10 (9 in covariance)

    state = VectorXd::Zero(4+3+3+3+3);

	// TODO: initialize rotation and initialize aB_last and wB_last and time


	int m = 3+3+3+3+3;
	covar = MatrixXd::Identity(m, m);





	/*
	int nfeatures = 500;
	float scaleFactor = 1.2f;
	int nlevels = 8; // 8
	int edgeThreshold = 31;
	int firstLevel = 0;
	int WTA_K = 2;
	int scoreType = ORB::HARRIS_SCORE;
	int patchSize = 31;
	int fastThreshold = 20;

	featureType = new ORB(
			nfeatures,
			scaleFactor,
			nlevels,
			edgeThreshold,
			firstLevel,
			WTA_K,
			scoreType,
			patchSize
			//fastThreshold
	);
	*/

	featureType = new SIFT();
	//featureType = new SURF();

	matcher = new FlannBasedMatcher();


	this->wB_last = Vector3d(0,0,0);
	this->aB_last = Vector3d(0,0,0);


}


MSCKF::~MSCKF(){



}


Matrix3d CrossMat(Vector3d w){
	Matrix3d wCross;
	wCross << 0, -w.z(), w.y(),
			  w.z(), 0, -w.x(),
			  -w.y(), w.x(), 0;
	return wCross;
}

Matrix4d OmegaMat(Vector3d w){

	Matrix4d Om = Matrix4d::Zero();

	Om.block<3,3>(0,0) = -CrossMat(w);
	Om.block<1,3>(3,0) = -w.transpose();
	Om.block<3,1>(0,3) = w;

	return Om;
}


// file:///home/dennis/papers/mono-vision/shelley14msc.pdf
/*
	This needs to buffer them as I will need to integrate up to each camera read time (then also up to every column of the image)

	The buffer should have the time of the measurement and the measurement data

	Additionally, I need to know the current time of the evolving body pose and of each frame in the sliding window

*/
#include <iostream>
void MSCKF::propagate_inertial(Vector3d aM /* Acceleration measured w.r.t body */, Vector3d wM /* Acceleration measured w.r.t. body */, uint64_t time){

	// TODO: The time needs to be updated every time through and also needs a good initialization point

	double dt = ((uint64_t)(time - this->time)) / 1.0e9;


	this->time = time;

	// Get values prior to integration
	Quaterniond q_last(state.segment<4>(0));
	Vector3d p_last(state.segment<3>(4));
	Vector3d v_last(state.segment<3>(7));
	Vector3d bG = state.segment<3>(10);
	Vector3d bA = state.segment<3>(13);



	Vector3d aB = Ta.inverse() * (aM - bA); // Estimated acceleration w.r.t. body

	// TODO: First get estimated from measured omega from 6.8
	Vector3d wB = Tg.inverse() * (wM - Ts*aB - bG);


	// Integrate gyroscope
	// 4th order Runge-Kutta integration of angular velocity
	Vector4d q0(0, 0, 0, 1);
	Vector4d k1 = 0.5*OmegaMat(wB_last)*q0;
	Vector4d k2 = 0.5*OmegaMat((wB_last+wB)/2.0)*(q0 + (dt/2.0)*k1);
	Vector4d k3 = 0.5*OmegaMat((wB_last+wB)/2.0)*(q0 + (dt/2.0)*k2);
	Vector4d k4 = 0.5*OmegaMat(wB)*(q0 + dt*k3);

	Quaterniond dq( q0 + (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4) );
	dq.normalize();

	Quaterniond q = dq * q_last;



	Vector3d g(0, 0, -calib.g); // Acceleration of gravity in the global frame

	// Normalize acceleration: This is the acceleration in global frame
	Vector3d a = q.conjugate()._transformVector(aB) + g; // Estimated accleration w.r.t. ground


	// Integrate acceleration and velocity
	// Done by trapezoidal integration
	Vector3d s = (dt / 2.0) * (dq.conjugate()._transformVector(aB) + aB_last);
	Vector3d y = (dt / 2.0) * s;
	Vector3d v = v_last + q_last.conjugate()._transformVector( s ) + g*dt;
	Vector3d p = p_last + v_last*dt + q_last.conjugate()._transformVector(y) + 0.5*g*dt*dt;

	// Update state
	state.segment<4>(0) = q.coeffs();
	state.segment<3>(4) = p;
	state.segment<3>(7) = v;
	// Biases don't change
	// Update last
	this->wB_last = wB;
	this->aB_last = aB;



	// Propagate error

	Matrix3d R_lastT = q_last.toRotationMatrix().transpose();
	Matrix3d R_T = q.toRotationMatrix().transpose();
	Matrix3d Rsum = R_lastT + R_T;

	Matrix3d Pqq = Matrix3d::Identity();
	Matrix3d Ppq = -CrossMat(R_lastT*y);
	Matrix3d Pvq = -CrossMat(R_lastT*s);

	Matrix3d Pqbg = -0.5*dt*(Rsum); // *Tg.inverse();
	Matrix3d Pvbg = 0.25*dt*dt*CrossMat(a - g)*(Rsum); // *Tg.inverse();
	Matrix3d Ppbg = 0.5*dt*Pvbg;

	Matrix3d Pqba = Matrix3d::Zero(); //0.5*dt*Rsum*Tg.inverse()*Ts*Ta.inverse(); // Goes to zero usually
	Matrix3d Pvba = -0.5*dt*Rsum; // *Ta.inverse() + 0.25*dt*dt*CrossMat(a - g)*Rsum*Tg.inverse()*Ts*Ta.inverse();
	Matrix3d Ppba = 0.5*dt*Pvba;





	Matrix<double, 15, 15> P;
	Matrix3d Zero = Matrix3d::Zero();
	Matrix3d I = Matrix3d::Identity();
	P << Pqq, Zero, Zero, Pqbg, Pqba,
		 Ppq, I,    I*dt, Ppbg, Ppba,
		 Pvq, Zero, I,    Pvbg, Pvba,
		 Zero,Zero, Zero, I,    Zero,
		 Zero,Zero, Zero, Zero, I;


	Matrix<double, 15, 15> Nc;
	Matrix<double, 15, 1> Nc_diag;
	Nc_diag <<
		calib.sigma_gc*calib.sigma_gc * Vector3d(1, 1, 1),
		Vector3d(0, 0, 0),
		calib.sigma_ac*calib.sigma_ac * Vector3d(1, 1, 1),
		calib.sigma_wgc*calib.sigma_wgc * Vector3d(1, 1, 1),
		calib.sigma_wac*calib.sigma_wac * Vector3d(1, 1, 1);
	Nc = Nc_diag.asDiagonal();
	Matrix<double, 15, 15> Qd = 0.5*dt*P*Nc*P.transpose() + Nc;



	covar.block<15,15>(0, 0) = P * covar.block<15,15>(0, 0) * P.transpose() + Qd;
	covar.block(0, 15, 15, covar.cols() - 15) = P * covar.block(0, 15, 15, covar.cols() - 15);
	covar.block(15, 0, covar.rows() - 15, 15) = covar.block(0, 15, 15, covar.cols() - 15).transpose();


}


