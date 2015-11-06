// Code for the image processing part of the filter

//#include "vio.h"
#include "msckf.h"


#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/persistence.hpp>
#include <iostream>





RNG rng(12345);


//int maxCorners = 100;

using namespace cv;
using namespace std;



MSCKF::MSCKF(){

	/*
	int nfeatures = 500;
	float scaleFactor = 1.2f;
	int nlevels = 4; // 8
	int edgeThreshold = 31; //15; // Changed default (31);
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

	//std::cout << "Found " << kp.size() << " Keypoints " << std::endl;

	//Ptr<DescriptorExtractor> extractor(detector);
	//this->extractor = extractor;

}


static Mat last_descriptors;
static vector<KeyPoint> last_kps;

void MSCKF::update_image(Mat &img){

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

	featureType->detect(imgGray, kps);
	featureType->compute(imgGray, kps, descriptors);



	for(int i = 0; i < kps.size(); i++){
		circle(img, kps[i].pt, 4, Scalar(0,255,0));
	}


	/*
	FileStorage fs("Keypoints.yml", FileStorage::WRITE);
	write(fs, "keypoints_1", keypoints_1);
	write(fs, "descriptors_1", tempDescriptors_1);
	fs.release();
	*/


	if(!last_kps.empty()){

		if(descriptors.empty())
			printf("First failed\n");

		if(last_descriptors.empty())
			printf("second failed\n");


		descriptors.convertTo(descriptors, CV_32F);
		last_descriptors.convertTo(last_descriptors, CV_32F);


		vector<vector<DMatch> > matches;

		matcher->knnMatch(descriptors, last_descriptors, matches, 2);


		// Matches believed to be correct
		vector<DMatch> good_matches;

		// Accept matchs based on a ratio test
		for(int i = 0; i < descriptors.rows; i++){
			if(matches[i][0].distance / matches[i][1].distance > 0.5)
				continue;

			good_matches.push_back(matches[i][0]);
		}


		printf("# good: %d\n", good_matches.size());


		for(int i = 0; i < good_matches.size(); i++){

			DMatch &m = good_matches[i];
			KeyPoint &kp1 = kps[m.queryIdx];
			KeyPoint &kp2 = last_kps[m.trainIdx];

			line(img, kp1.pt, kp2.pt, Scalar(255,0,0), 2);
		}

	}


	last_descriptors = descriptors;
	last_kps = kps;



	// http://docs.opencv.org/3.0-beta/doc/tutorials/features2d/feature_flann_matcher/feature_flann_matcher.html#feature-flann-matcher


/*
	vector<DMatch> matches;
	matcher.match(descriptors_1, descriptors_2, matches);

	last_descriptors = descriptors;
*/


	//drawKeypoints(imgGray, kps, imgGray, Scalar::all(255));


	//img = imgGray;
    // Scale back to original size
    //resize(temp, img, Size(1920, 1080));
}
