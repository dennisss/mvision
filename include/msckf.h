#ifndef MSCKF_H_
#define MSCKF_H_


/*

	The state vector (should be [16 + 27 + 14 + 10m, 1] dimension)
	- Current estimate:  ...
	- IMU Parameters: ...
	-
	- Each body pose has the [position, quaternion, velocity]






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


// Represents a single image stored as auxiliary data outside of the EKF vector
typedef struct {

    vector<KeyPoint> features;
    Mat descriptors;

} Frame;


typedef struct {

	// The index of the feature in the indices.size() latest Frames
	// a size < total # of frames means that the feature did not appear in at least one old frame
	vector<int> indices;

} Track;


class MSCKF {

public:

    MSCKF();
    ~MSCKF();


	// This updates the 'evolving body state'
    // Range-Kutta Integration of the angular velocity to propagate angle
	// Trapezoidal Integration for the velocity and position update
    // Also, the covariance needs to be updated
    void propagate_inertial(float *acc, float *gyro);


    // Compute features/descriptors for the new frame, and match them with the current window of images
    // Initialize that camera position from the IMU state
    // Finally, optimize all the feature constraints and remove any old/unused frames
    void update_image(Mat &img);



private:

//	EKF kf;

	// TODO: Move to the EKF class
    VectorXf state; // Size is 4+3 + 3+3 + 3 + 7*n = 16 + 7n
    MatrixXf covariance;


    vector<Frame> frames; // The frames in the same order they appear in the state vector
    vector<Track> tracks; // Sets of matched feature points in the current frames





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
	Vector3f w_last; // Last angular acceleration used. TODO: This should initially be null






};


#endif
