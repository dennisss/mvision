#ifndef FILEIO_H_
#define FILEIO_H_

#include <stdint.h>
#include <vector>
#include <string>


#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// Common functions for reading and writing the data files


// Note: The first and final data_entrys in each file have no data, but do have a precise start/end time
typedef struct {

	int64_t timestamp;

	float accel[3];
	float gyro[3];

} data_entry;





vector<data_entry> read_data(char *filename);






void write_features(string filename, vector<KeyPoint> &kps, Mat &descriptors);
void read_features(string filename, vector<KeyPoint> &kps, Mat &descriptors);


void write_matches(string filename, vector<DMatch> &matches);
void read_matches(string filename, vector<DMatch> &matches);





#endif
