#ifndef MSCKF_H_
#define MSCKF_H_


/*

	The state vector (should be [16 + 27 + 14 + 10m, 1] dimension)
	- Current estimate:  ...
	- IMU Parameters: ...
	-
	- Each body pose has the [position, quaternion, velocity]






*/

// TODO: Do a lot more pass-by-reference and inlining of functions to make it faster

/*

The camera calibration and distortion parameters.

The IMU shape matrices Tg and Ta as well as the g-sensitivity matrix Ts

The IMU-camera position offset, CpB

The IMU random walk bias values, ba and bg.
	-

The Camera to IMU time delay, td

The Camera rolling shutter read time, tr




*/




/*
 Based on a Kalman Filter with a sliding window of states (and potentially additional data such as IMU biases and online calibration values)

 A propagation phase is run when IMU readings are received


 An update phase is run when an image is available


 The base Kalman Filter needs to be preserved so that quadcopter control inputs can be easily used to make a 'predict' on the state

*/


#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <vector>

#include <Eigen/Dense>

using namespace Eigen;

using namespace std;

using namespace cv;


#define NANOPERSEC ((uint64_t)1000000000)


#define WINDOW_SIZE 30


Matrix3d CrossMat(Vector3d w);
Matrix4d OmegaMat(Vector3d w);


// Represents a single image stored as auxiliary data outside of the EKF vector
struct Frame {

    vector<Vector2d /*KeyPoint*/> features;
    //Mat descriptors;

	long long timestamp; // Nano-second timestamp at which image was taken

};


// Represents a single feature that is being tracked across many frames
struct Track{

	//int id;

	// The index of the feature in each frame
	// Note: These are valid in reverse order only (so indices[size - 1] is the index in the latest/last frame)
	// A value of -1 indicates no tracking
	vector<int> indices;


	vector<pair<int, Vector2d>> extract(vector<Frame> &frames);


	bool lost(); // Whether or not the track is lost in the most recent frame
};


//typedef struct {
//	long long timestamp;
//
//	double accel[3];
//	double rotat[3];
//} InertialData;


typedef struct {

	Matrix3d K; // Focal lengths and optical center
	vector<double> distortion;

} Camera;

typedef struct {

	double sigma_ac; // Accelerometer continous variance
	double sigma_gc; // Gyroscope continuous variance

	double sigma_wac; // Accelerometer random walk continous variance
	double sigma_wgc; // Gyroscope random walk continous variance


	double sigma_img; // Image pixel noise variance


	Matrix3d Rbc; // Rotation from body frame to camera frame

	Vector3d CpB; //BpC; // Position of camera in body frame
	//CpB;  // Position of body in camera frame


	double o_x, o_y, f_x, f_y;
	double k1, k2, t1, t2;

	double g;

} Calibration;




class MSCKF {

public:

	MSCKF();
	~MSCKF();


	// This updates the 'evolving body state'
	// Range-Kutta Integration of the angular velocity to propagate angle
	// Trapezoidal Integration for the velocity and position update
	// Also, the covariance needs to be updated
	void propagate_inertial(Vector3d acc, Vector3d gyro, uint64_t time);


	// Compute features/descriptors for the new frame, and match them with the current window of images
	// Initialize that camera position from the IMU state
	// Finally, optimize all the feature constraints and remove any old/unused frames
	bool update_image(Mat &img);
	bool update_image(vector<Vector2d> features, vector<int> matches);




	Calibration calib;

	//private:

	// TODO: Make private but expose some of the data
	// TODO: Move to the EKF class
	VectorXd state; // Size is 4+3 + 3+3 + 3 + 7*n = 16 + 7n
	MatrixXd covar;
	uint64_t time;



	void augment_frames(); // Add new frame from evolving body-state
	void discard_frames(); // Remove old frames that don't have tracked features or are outside the window

	void marginalize(vector<pair<int, Vector2d>> track, Vector3d pF, VectorXd &r0, MatrixXd &H0);
	void update_state(VectorXd &dx);
	bool is_inlier(VectorXd &ri, MatrixXd &Hi);




	//	EKF kf;

	vector<Frame> frames; // The frames in the same order they appear in the state vector
	vector<Track> tracks; // Sets of matched feature points in the current frames


	//vector<InertialData> buffer; // For integrating upto any time since the last image update


	//Matrix4d pose(); // Get current evolving body pose
	//Matrix4d pose(int i); // Get the pose of the i'th frame, (0 is the oldest and n is the newest)



	//Ptr<ORB> detector;
	//Ptr<DescriptorExtractor> extractor;

	/*
		What to use for feature detection and descriptor extraction
		Typically set to an instance of ORB
	*/
	Feature2D *featureType;

	/*
		For matching the generated feature descriptors
		Typically set to a FlannBasedMatcher
		TODO: The predicted orientation/trajectory needs to be incorporated into the matching to provide faster performance
	*/
	DescriptorMatcher *matcher;





	// IMU State: (one large column vector)
	// [ (quaternion rotation) (gyro bias)  (velocity)  (accel bias) (position)   ]
	// The above state is concatened with a column vector of [ ( rotation_1, position_1 ) ... (rotation_n, position_n) ] for all stored camera frames
	//
	//



	// Preset known values: rotation and translation between IMU and camera axes





	// Other stuff
	// TODO: Initialize these to the first reading
	Vector3d aB_last;
	Vector3d wB_last; // Last angular acceleration used.






};


#endif
