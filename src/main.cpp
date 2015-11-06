#include "msckf.h"

#include <stdio.h>


#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace cv;


int main(int argc, char *argv[]){

	if(argc != 3){
		printf("Usage: ./mvision video.mp4 inertial.data output.txt\n");
		printf("    Using the input files, the odometry will be estimated and saved to the output file\n");
		return 1;
	}




	VideoCapture cap(argv[1]);

	if(!cap.isOpened()){
		printf("Failed to open input video\n");
		return 1;
	}


	// Frame rate for synchronizing the video with the inertial data
	double fps = cap.get(CV_CAP_PROP_FPS);



	MSCKF model;

	namedWindow("image", CV_WINDOW_NORMAL);

	Mat frame;
	while(true){
		cap >> frame;

		if(frame.empty())
			break;



		model.update_image(frame);




		imshow("image", frame);
		waitKey(150);
	}






	printf("Hello World!");


}

