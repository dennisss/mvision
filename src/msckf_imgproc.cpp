// Code for the image processing part of the filter

//#include "vio.h"
#include "msckf.h"
#include "utils.h"
#include "fileio.h"
#include "stat.h"
#include "projection.h"

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>


#include <Eigen/Dense>
#include <Eigen/QR>



//int maxCorners = 100;

using namespace cv;
using namespace std;



static Mat last_descriptors;
static vector<KeyPoint> last_kps;


static int i = 0;

bool MSCKF::update_image(Mat &img){

/*
	Mat temp;
	resize(img, temp, Size(640, 480));
	img = temp;
*/


//    LOGI("IMAGE PROCESS");



	Mat imgGray;
    cvtColor(img, imgGray, CV_BGR2GRAY);


    vector<KeyPoint> kps;
	Mat descriptors;


	string featfile = "out/" + to_string(i) + ".feat";

	if(exists(featfile)){
		read_features(featfile, kps, descriptors);
	}
	else{
		featureType->detect(imgGray, kps);
		featureType->compute(imgGray, kps, descriptors);
		descriptors.convertTo(descriptors, CV_32F);

		write_features(featfile, kps, descriptors);
	}

	for(int i = 0; i < kps.size(); i++){
		circle(img, kps[i].pt, 4, Scalar(0,255,0));
	}

	// Matches believed to be correct
	vector<DMatch> matches;


	if(!last_kps.empty()){

		if(descriptors.empty())
			printf("First failed\n");

		if(last_descriptors.empty())
			printf("second failed\n");


		//descriptors.convertTo(descriptors, CV_32F);
		//last_descriptors.convertTo(last_descriptors, CV_32F);


		string matfile = "out/" + to_string(i) + ".mat";

		if(exists(matfile)){
			read_matches(matfile, matches);
		}
		else{

			vector<vector<DMatch> > all_matches;

			matcher->knnMatch(descriptors, last_descriptors, all_matches, 2);


			// Accept matches based on a ratio test
			for(int i = 0; i < descriptors.rows; i++){
				// small / large
				if(all_matches[i][0].distance / all_matches[i][1].distance > 0.15)
					continue;

				matches.push_back(all_matches[i][0]);
			}

			write_matches(matfile, matches);
		}


		// Remove inconsistent matches
		for(int i = 0; i < matches.size(); i++){

			bool inconsistent = false;
			for(int j = i+1; j < matches.size(); j++){

				if(matches[i].trainIdx == matches[j].trainIdx){
					inconsistent = true;
					matches.erase(matches.begin() + j);
					j--;
				}
			}


			if(inconsistent)
				matches.erase(matches.begin() + i);
		}



		printf("# good: %d\n", matches.size());


		for(int i = 0; i < matches.size(); i++){
			DMatch &m = matches[i];
			KeyPoint &kp1 = kps[m.queryIdx];
			KeyPoint &kp2 = last_kps[m.trainIdx];

			line(img, kp1.pt, kp2.pt, Scalar(255,0,0), 4);
		}

	}

//	imwrite("matches/" + to_string(i) + ".jpg", img);

	last_descriptors = descriptors;
	last_kps = kps;



	i++;

	//img = imgGray;
    // Scale back to original size
    //resize(temp, img, Size(1920, 1080));


	// TODO: Ensure that two features don't match to the same old feature
	// (inconsistent tracks)

	// Convert everything
	vector<Vector2d> fts(kps.size());
	vector<int> m(kps.size(), -1);

	for(int i = 0; i < matches.size(); i++){
		m[matches[i].queryIdx] = matches[i].trainIdx;
	}
	for(int i = 0; i < kps.size(); i++){
		fts[i] = Vector2d(kps[i].pt.x, kps[i].pt.y);
	}


	return this->update_image(fts, m);
}


/*
	Image processing will output a set of features for the current frame and either:
	- The index of the previous feature that it matches to
	- Or, the id of the 'global feature' that it corresponds to

	MSCKF needs to know, given the id of the new feature, which

*/


bool MSCKF::update_image(vector<Vector2d> features, vector<int> matches /* Which feature in the last image that each current feature corresponds to (or -1 if no match to previous) */){

	// TODO: Ensure position triangulated up to image center

	vector<int> trackmap;
	if(frames.size() > 0){
		trackmap.resize(frames.back().features.size()); // The track at which each of the latest features is located
		for(int i = 0; i < tracks.size(); i++){
			trackmap[tracks[i].indices.back()] = i;
		}
	}


	// Augment state
	this->augment_frames();

	// Update frames and tracks
	frames[frames.size() - 1].features = features;

//	Mat matmatch(1440, 1920, CV_8UC1, Scalar::all(0));

	for(int i = 0; i < features.size(); i++){
		int t;
		if(matches[i] == -1 || trackmap.size() == 0/* OR there were no previous matches. TODO: This should't be needed if i change the track format */){ // Doesn't match any existing track
			t = tracks.size();
			tracks.resize(t + 1);
			tracks[t].indices.push_back(-1);
		}
		else{
			t = trackmap[matches[i]];

			int q = tracks[t].indices[tracks[t].indices.size() - 2];
			assert(q == matches[i]);

			Frame &lastf = frames[frames.size() - 2];

//			Point2f p1(features[i].x(), features[i].y()), p2(lastf.features[q].x(), lastf.features[q].y());
//
//			circle(matmatch, p1, 4, Scalar::all(255));
//			circle(matmatch, p2, 4, Scalar::all(255));
//			line(matmatch, p1, p2, Scalar::all(255), 1);

		}

		tracks[t].indices.back() = i;
	}


	//imwrite("debug-match.jpg", matmatch);


//	Mat matmatch2(1440, 1920, CV_8UC1, Scalar::all(0));
//	for(int i = 0; i < tracks.size(); i++){
//		Track &t = tracks[i];
//		vector<pair<int, Vector2d>> ext = t.extract(frames);
//
//		if(ext.size() >= 2){
//
//			int l = ext.size() -1;
//
//			Point2f p1(ext[l].second.x(), ext[l].second.y()), p2(ext[l-1].second.x(), ext[l-1].second.y());
//
//			circle(matmatch2, p1, 4, Scalar::all(255));
//			circle(matmatch2, p2, 4, Scalar::all(255));
//			line(matmatch2, p1, p2, Scalar::all(255), 1);
//
//
//		}
//	}

	//imwrite("debug-match2.jpg", matmatch2);





	// Calculate residuals and update filter

	VectorXd r0; //(/*0*/);
	MatrixXd H0(0, covar.cols());

	Mat mat(1440, 1920, CV_8UC1, Scalar::all(0));

	for(int i = 0; i < tracks.size(); i++){

		vector<pair<int, Vector2d>> trk = tracks[i].extract(frames);
		int n = trk.size(); // t.indices.size(); // Number of measurements TODO: This is wrong (if the last one is -1)


		if(!(n >= WINDOW_SIZE || tracks[i].lost()))
			continue;


		if(n >= 3){ // Don't try to triangulate features without many observations

			// Compute camera positions
			vector<Vector2d> ms;
			vector<Quaterniond> qs;
			vector<Vector3d> ps;
			for(int j = 0; j < trk.size(); j++){
				int fIdx = trk[j].first;
				ms.push_back(trk[j].second);

				Quaterniond qi(state.segment<4>(16 + fIdx*10));
				Quaterniond qCi = Quaterniond(calib.Rbc) * qi; // Global camera rotation

				Vector3d pi = state.segment<3>(16 + fIdx*10 + 4);
				Vector3d pCi = pi - qCi.conjugate()._transformVector(calib.CpB); // Global camera position

				// TODO: I need to transform these to the pose of the camera
				qs.push_back(qCi);
				ps.push_back(pCi);
			}


			Vector3d p = triangulate(qs, ps, ms, calib);

			cout << "tri: " << p.transpose() << endl;

			for(int a = 0; a < ms.size() - 1; a++){
				Point2f p1(ms[a].x(), ms[a].y()), p2(ms[a+1].x(), ms[a+1].y());

				circle(mat, p1, 4, Scalar::all(255));
				circle(mat, p2, 4, Scalar::all(255));
				line(mat, p1, p2, Scalar::all(255), 1);
			}


			if(isfinite(p(0)) && isfinite(p(1)) && isfinite(p(2))){
				VectorXd r0i = VectorXd(2*n - 3);
				MatrixXd H0i = MatrixXd(2*n - 3, covar.cols());


				// Compute here
				this->marginalize(trk, p, r0i, H0i);


				if(is_inlier(r0i, H0i)){
					r0.conservativeResize(r0.rows() + r0i.rows(), NoChange);
					r0.bottomRows(r0i.rows()) = r0i;

					H0.conservativeResize(H0.rows() + H0i.rows(), NoChange);
					H0.bottomRows(H0i.rows()) = H0i;

				}
			}
		}


		if(tracks[i].lost()){
			// Remove track
			tracks.erase(tracks.begin() + i);
			i--;
		}
	}

//	imwrite("debug.jpg", mat);



	if(r0.rows() > 0){

		cout << "updating with observations" << endl;


		// Reduce dimensionality through QR decomposition
		HouseholderQR<MatrixXd> qr(H0);
		MatrixXd Q = qr.householderQ(), R = qr.matrixQR().triangularView<Upper>();

		MatrixXd Q1(H0.rows(), H0.cols()), TH(H0.cols(), H0.cols());
		int nz = 0;
		for(int i = 0; i < R.rows(); i++){
			bool allzero = true;
			for(int j = 0; j < R.cols(); j++){
				if(R(i,j) != 0.0)
					allzero = false;
			}

			if(!allzero){
				Q1.col(nz) = Q.col(i);
				TH.row(nz) = R.row(i);
				nz++;
			}
		}


		if(nz != H0.cols()){
//			cout << "Wrong rank? " << nz << " " << H0.cols() << endl;
			Q1.conservativeResize(NoChange, nz);
			TH.conservativeResize(nz, NoChange);
		}

//		// TODO: Check this
//		//MatrixXd Q1 = Q.block(0,0, H0.rows(), H0.cols()), TH = R.block(0,0, H0.cols(), H0.cols());

//		// New residual vector
//		VectorXd rq = Q1.transpose()*r0;

		// Measurement noise
		MatrixXd Rn = pow(calib.sigma_img, 2) * MatrixXd::Identity(r0.rows(), r0.rows());
//		MatrixXd Rq = pow(calib.sigma_img, 2) * MatrixXd::Identity(Q1.cols(), Q1.cols());

		MatrixXd K = this->covar*H0.transpose() * (H0*this->covar*H0.transpose() + Rn).inverse();
//		MatrixXd K = covar*TH.transpose()*(TH*covar*TH.transpose() + Rq).inverse();

		VectorXd dx = K*r0; // *rq;
		update_state(dx);


		// Update state


		// Update covariance

		// TODO: Check the dimensions of this identity matrix
		MatrixXd A = MatrixXd::Identity(covar.rows(), covar.cols()) - K * H0; // * TH; // * H0;
		this->covar = A*this->covar*A.transpose() + K*Rn*K.transpose(); // + K*Rq*K.transpose();

	}


	covar = (covar + covar.transpose()) / 2;




	// Cleanup old states (those that don't have tracked features)
	this->discard_frames();


	if(r0.rows() > 0){
		return false;
	}




	// Set current timestamp


	// Cleanup the buffer


	return true;

}


void MSCKF::marginalize(vector<pair<int, Vector2d>> track /* Pair of frame # and feature measurement */, Vector3d pF /* Estimated global position of feature */, VectorXd &r0, MatrixXd &H0){

	int n = track.size();

	VectorXd r(2*n);
	MatrixX3d Hf(2*n, 3);
	MatrixXd Hx = MatrixXd::Zero(2*n, covar.cols());



	for(int i = 0; i < n; i++){

		int fIdx = track[i].first;
		Vector2d z = track[i].second;



		int fi = (4+3+3+3+3) + fIdx*(4+3+3); // Start of frame in state
		Quaterniond q(state.segment<4>(fi));// Rotation of body in global
		Vector3d p(state.segment<3>(fi+4));


		Quaterniond qC = Quaterniond(calib.Rbc) * q; // Global orientation of camera
		Vector3d pC = p - qC.conjugate()._transformVector( calib.CpB ); // Global camera position

		// Position of feature in camera frame
		Vector3d CpF = qC._transformVector( pF - pC );


		r.segment<2>(2*i) = z - camera_project(CpF, calib);

		Hf.block<2,3>(2*i, 0) = JacobianH(CpF, calib) * qC.toRotationMatrix();

		Hx.block<2,9>(2*i, (3+3+3+3+3) + (3+3+3)*fIdx) <<
			Hf.block<2,3>(2*i, 0)*CrossMat(pF - p),
			-Hf.block<2,3>(2*i, 0),
			Matrix<double, 2, 3>::Zero();
	}


	cout << r << endl;




	// Start of marginalization trickery

	// Find left null-space
	JacobiSVD<MatrixXd> svd(Hf, Eigen::ComputeFullU);
	MatrixXd A = svd.matrixU().rightCols(2*n - 3).transpose();

	if(2*n - 3 != A.rows()){
		cout << "Error! Nullspace trick failed?" << endl;
	}

	// Marginalize
	r0 = A*r;
	H0 = A*Hx;
}


// Check if the Kalman model parameters for a camera feature are an inlier
bool MSCKF::is_inlier(VectorXd &ri, MatrixXd &Hi){
	// TODO: Check this
	MatrixXd gammaI = ri.transpose() * (Hi * covar * Hi.transpose()).inverse() * ri;
	return gammaI(0) <= chi2inv(ri.rows());
}


void MSCKF::update_state(VectorXd &dx){

	Quaterniond dq(1, dx(0) / 2.0, dx(1) / 2.0, dx(2) / 2.0);
	Quaterniond q = Quaterniond(state.segment<4>(0))*dq;
	q.normalize();
	state.segment<4>(0) = q.coeffs();

	// Update p, v, bG, bA
	state.segment<3+3+3+3>(4) += dx.segment<3+3+3+3>(3);

	int a = 4+3+3+3+3, b = 3+3+3+3+3;

	for(int i = 0; i < frames.size(); i++){

		Quaterniond qi = Quaterniond(state.segment<4>(a))*Quaterniond(1, dx(b) / 2.0, dx(b+1) / 2.0, dx(b+2) / 2.0);
		qi.normalize();
		state.segment<4>(a) = qi.coeffs();

		state.segment<3+3>(a + 4) += dx.segment<3+3>(b + 3);

		// Each frame has a rotation, position, and velocity
		a += 4+3+3;
		b += 3+3+3;
	}
}


void MSCKF::augment_frames(){

	int i = state.rows();

	state.conservativeResize(i + 10);
	state.segment<10>(i) = state.segment<10>(0);


	int j = covar.rows();
	covar.conservativeResize(j + 9, j + 9);
	covar.block<9,9>(j, j) = covar.block<9,9>(0,0);
	covar.block(0, j, j, 9) = covar.block(0, 0, j, 9);
	covar.block(j, 0, 9, j) = covar.block(0, 0, 9, j);

	frames.resize(frames.size() + 1);


	for(int i = 0; i < tracks.size(); i++){
		tracks[i].indices.push_back(-1);
	}
}


// This assumes that the tracks have been updated already
// Any lost tracks should have already been removed
void MSCKF::discard_frames(){

	int longest = 0;
	for(Track &t : tracks){

		cout << ":: ";
		for(int i = 0; i < t.indices.size(); i++){
			cout << t.indices[i] << " ";
		}
		cout << endl;

		if(t.indices.size() > WINDOW_SIZE){ // Clip tracks that are too long
			t.indices.erase(t.indices.begin(), t.indices.begin() + (t.indices.size() - WINDOW_SIZE));
		}

		if(t.lost())
			cout << "this should never happen" << endl;

		if(t.indices.size() > longest)
			longest = t.indices.size();
	}

	cout << longest << endl;


	int n = min(WINDOW_SIZE, (int)(frames.size() - longest)); // How many frames we can discard

	if(n == 0)
		return;

	cout << "erasing " << n << " of " << frames.size() <<  " frames" << endl;

	frames.erase(frames.begin(), frames.begin() + n);


	state.segment(16, 10*frames.size()) = state.segment(16 + 10*n, 10*frames.size());
	state.conservativeResize(state.rows() - 10*n);


	covar.block(15, 0, 9*frames.size(), covar.cols()) = covar.block(15+9*n, 0, 9*frames.size(), covar.cols()); // Push up
	covar.block(0, 15, covar.rows(), 9*frames.size()) = covar.block(0, 15+9*n, covar.rows(), 9*frames.size()); // Push left
	covar.conservativeResize(covar.rows() - 9*n, covar.cols() - 9*n);
}


bool Track::lost(){
	return indices.back() == -1;
}

vector<pair<int, Vector2d>> Track::extract(vector<Frame> &frames){
	vector<pair<int, Vector2d>> v;

	for(int i = 0; i < indices.size(); i++){
		int fIdx = (frames.size() - indices.size()) + i;

		if(indices[i] == -1)
			continue;

		v.push_back(pair<int, Vector2d>(fIdx, frames[fIdx].features[indices[i]]));
	}

	return v;
}
