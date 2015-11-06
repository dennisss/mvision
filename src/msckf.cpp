#include "msckf.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;


//MSCKF::MSCKF(){
//
//    Vector3f vec(1,2,3);
//
//}


MSCKF::~MSCKF(){



}


static Matrix4f Omega(Vector3f w){

	Matrix3d wCross;
	wCross << 0, -w.z(), w.y(),
			  w.z(), 0, -w.x(),
			  -w.y(), w.x(), 0;

	Matrix4f Om = Matrix4f::Zeros();

	Om.block<3,3>(0,0) = -wCross;
	Om.block<1,3>(3,0) = -w.transpose();
	Om.block<3,1>(0,3) = w;

	return Om;
}



// file:///home/dennis/papers/mono-vision/shelley14msc.pdf
void MSCKF::propagate_inertial(float *acc, float *gyro, float dt){


	// TODO: First get estimated from measured omega from 6.8


	// Integrate gyroscope


	// 4th order Runge-Kutta integration of angular velocity
	Quaternionf q0(1, 0, 0, 0);
	Quaternionf k1 = 0.5*Omega(w_last)*q0;
	Quaternionf k2 = 0.5*Omega((w_last+w)/2.0)*(q0 + (dt/2.0)*k1);
	Quaternionf k3 = 0.5*Omega((w_last+w)/2.0)*(q0 + (dt/2.0)*k2);
	Quaternionf k4 = 0.5*Omega(w)*(q0 + dt*k3);

	Quaternionf dq = q0 + (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
	dq.normalize();

	this->q = dq * this->q;










}
