#include "fileio.h"
#include "utils.h"

#include <fstream>
#include <string>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <zlib.h>

using namespace std;



vector<data_entry> read_data(char *filename){
	vector<data_entry> data;

	ifstream fin(filename);
	vector<string> params;
	while(!fin.eof()){

		fin >> params;

		data_entry e;

		e.timestamp = stoll(params[0]);

		e.accel[0] = stof(params[1]);
		e.accel[1] = stof(params[2]);
		e.accel[2] = stof(params[3]);

		e.gyro[0] = stof(params[4]);
		e.gyro[1] = stof(params[5]);
		e.gyro[2] = stof(params[6]);

		data.push_back(e);
	}

	return data;
}


void write_features(string filename, vector<KeyPoint> &kps, Mat &descriptors){

	gzFile f = gzopen(filename.c_str(), "wb");

	// Write keypoints
	int32_t n = kps.size();
	gzwrite(f, &n, sizeof(int32_t));
	gzwrite(f, &kps[0], sizeof(KeyPoint)*n);


	// Write descriptors
	assert(descriptors.type() == CV_32F);

	Size size = descriptors.size();
	int32_t height = size.height, width = size.width;

	gzwrite(f, &height, sizeof(int32_t));
	gzwrite(f, &width, sizeof(int32_t));
	gzwrite(f, descriptors.data, sizeof(float) * size.height * size.width);


	gzflush(f, Z_FINISH);
	gzclose(f);
}


void read_features(string filename, vector<KeyPoint> &kps, Mat &descriptors){

	gzFile f = gzopen(filename.c_str(), "rb");

	// Read keypoints
	int32_t n;
	gzread(f, &n, sizeof(int32_t));
	kps.resize(n);
	gzread(f, &kps[0], sizeof(KeyPoint) * n);


	// Read descriptors
	int32_t height, width;
	gzread(f, &height, sizeof(int32_t));
	gzread(f, &width, sizeof(int32_t));

	descriptors = Mat(height, width, CV_32F);
	gzread(f, descriptors.data, sizeof(float) * width * height);

	gzclose(f);
}


void write_matches(string filename, vector<DMatch> &matches){
	FILE *f = fopen(filename.c_str(), "wb");

	int32_t n = matches.size();
	fwrite(&n, sizeof(int32_t), 1, f);

	for(int i = 0; i < n; i++){
		int32_t m[2];
		m[0] = matches[i].queryIdx;
		m[1] = matches[i].trainIdx;
		fwrite(&m, sizeof(int32_t), 2, f);
	}

	fflush(f);
	fclose(f);
}

void read_matches(string filename, vector<DMatch> &matches){
	FILE *f = fopen(filename.c_str(), "rb");

	int32_t n;
	fread(&n, sizeof(int32_t), 1, f);
	matches.resize(n);

	for(int i = 0; i < n; i++){
		int32_t m[2];
		fread(&m, sizeof(int32_t), 2, f);
		matches[i] = DMatch(m[0], m[1], 0.0);
	}

	fclose(f);
}

