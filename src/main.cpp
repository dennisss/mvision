#include "msckf.h"
#include "fileio.h"

#include "MadgwickAHRS.h"


#include "testing.h"


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;


void print_usage(){
	cout << "Usage:" << endl
		 << "- Compute inertial attitude:" << endl
		 << "  ./mvision ahrs inertial.data output.txt" << endl
		 << "- Compute visual-inertial odometry:" << endl
		 << "  ./mvision msckf video.mp4 inertial.data output.txt" << endl;
}

int compute_ahrs(char *infile, char *outfile){

	vector<data_entry> data = read_data(infile);
	ofstream out(outfile);

	// Remove start and stop markers
	data.erase(data.begin());
	data.pop_back();

	for(data_entry &e : data){

		MadgwickAHRSupdateIMU(e.gyro[0], e.gyro[1], e.gyro[2], e.accel[0], e.accel[1], e.accel[2]);

		float quat[4];
		MadgwickAHRSgetquaternion(quat);
		out << quat[0] << " " << quat[1] << " " << quat[2] << " " << quat[3] << endl;
	}

	out.flush();

	return 0;
}

int compute_msckf(int64_t starttime /* time at which video starts recording */, char *videofile, char *imufile, char *outfile){
	VideoCapture cap(videofile);
	if(!cap.isOpened()){
		printf("Failed to open input video\n");
		return 1;
	}

	// Frame rate for synchronizing the video with the inertial data
	double fps = cap.get(CV_CAP_PROP_FPS);

	uint64_t frame_dt = 1000000000 / fps;

	int delay = 1000 / fps;

	vector<data_entry> data = read_data(imufile);

	ofstream out(outfile);

	MSCKF model;


	model.calib.sigma_ac = 0.0034; // 0.034;
	model.calib.sigma_gc = 0.0005; //0.005;
	model.calib.sigma_wac = 0.001; // 0.001; // TODO: Check the walk parameters
	model.calib.sigma_wgc = 0.0001; //0.0001;

	model.calib.sigma_img = 120; //1000; // 0.24;

	model.calib.Rbc << 0, -1, 0, // Body to camera
				 -1, 0, 0,
				 0, 0, -1;

	model.calib.CpB << 0.06, 0, 0;
	//model.calib.BpC << 0, 0.06, 0;

	model.calib.f_x = 1630.8706287301968;
	model.calib.f_y = 1626.9354701674479;
	model.calib.o_x = 982.51095058012686;
	model.calib.o_y = 679.82201075835394;
	model.calib.k1 = 0.26464632504837649;
	model.calib.k2 = -1.1306176294559058;
	model.calib.t1 = -0.0096476113378466070;
	model.calib.t2 = 0.0058735362455681027;

	model.aB_last = Vector3d(9.858704, 0.577621, 0.656265);
	model.wB_last = Vector3d(0.075638, 0.050049, -0.022354);

	model.time = 1447955692095000000;

	//model.state.segment<4>(0) = Vector4d(0, 0.707107, 0, 0.707107); //Vector4d(0.00331683, 0.0145046, -0.000166415, 0.998201);

	// Figure out initial orientation
	Vector3d avg_acc(0,0,0);
	for(int i = 0; i < 50; i++){
		Vector3d acc(data[i].accel[0], data[i].accel[1], data[i].accel[2]);
		avg_acc += acc;
	}
	avg_acc /= 50;

	cout << "~G: " << avg_acc.norm() << endl;

	model.calib.g = avg_acc.norm();

	Quaterniond orient = Quaterniond::FromTwoVectors(Vector3d(0,0,1), avg_acc);

	cout << orient.coeffs().transpose() << endl;

	model.state.segment<4>(0) = orient.coeffs();

	//Vector3d axis = Vector3d(0,1,0).cross()








	namedWindow("image", CV_WINDOW_NORMAL);


	uint64_t time = 1447955692095000000 ; // + 13271168;

	int i = 0;

	//int imgIdx = 0;


	//Mat K(3, 3, CV_64F);
	//K.at<double>(0,0) = 1.6308706287301968e+03;  K.at<double>(0,1) = 0.; K.at<double>(0,2) = 9.8251095058012686e+02;
	//K.at<double>(1,0) = 0.; K.at<double>(1,1) = 1.6269354701674479e+03; K.at<double>(1,2) = 6.7982201075835394e+02;
	//K.at<double>(2,0) = 0.; K.at<double>(2,1) = 0.; K.at<double>(2,2) = 1.;

	//Mat D(5, 1, CV_64F);
	//D.at<double>(0,0) = 2.6464632504837649e-01;
	//D.at<double>(0,1) = -1.1306176294559058e+00;
	//D.at<double>(0,2) = -9.6476113378466070e-03;
	//D.at<double>(0,3) = 5.8735362455681027e-03;
	//D.at<double>(0,4) = 1.9147234787371266e+00;


	Mat frame;
	while(true){

		cap >> frame;
		time += frame_dt;

		cout << "t:" << time << endl;


		if(frame.empty())
			break;


		// Make for VisualSFM
		//Mat undist;
		//undistort(frame, undist, K, D);
		//imwrite("data/imgs/" + to_string(imgIdx++) + ".jpg", frame);
		//continue;



		// Integrate
		while(data[i].timestamp < time){
			cout << "IMU: " << i << endl;
			//cout << "a: " << data[i].timestamp << endl;
			model.propagate_inertial(
				Vector3d(data[i].accel[0], data[i].accel[1], data[i].accel[2]),
				Vector3d(data[i].gyro[0], data[i].gyro[1], data[i].gyro[2]),
				data[i].timestamp
			);
			i++;
		}

		cout << "Image" << endl;
		model.update_image(frame);

		// Save [x y z w] [x y z]
		VectorXd pose = model.state.segment<7>(0);
		out << pose.transpose() << endl;



		// Draw
		for(int i = 0; i < model.tracks.size(); i++){
			vector<pair<int,Vector2d>> pts = model.tracks[i].extract(model.frames);

			for(int j = 1; j < pts.size(); j++){

				Point2f p1(pts[j-1].second.x(), pts[j-1].second.y()), p2(pts[j].second.x(), pts[j].second.y());

				line(frame, p1, p2, Scalar(255,0,0), 2);

				if(j == pts.size() - 1){
					circle(frame, p2, 4, Scalar(0,255,0));
				}
			}
		}



		imshow("image", frame);
		//waitKey(delay);
		waitKey(1);
	}



	out.flush();

}


int main(int argc, char *argv[]){

//	test_msckf();
//	return 0;


	if(argc < 2){
		print_usage();
		return 1;
	}

	string mode = argv[1];


	if(mode == "ahrs"){
		if(argc != 4){
			print_usage();
			return 1;
		}

		return compute_ahrs(argv[2], argv[3]);
	}
	else if(mode == "msckf"){
		if(argc != 5){
			print_usage();
			return 1;
		}

		// TODO: Take first few accelerometer values, assume steady-state and derive an initial orientation
		// Set initial position to 0,0,0


		return compute_msckf(0, argv[2], argv[3], argv[4]);
	}
	else{
		print_usage();
		return 1;
	}


	return 0;

}

