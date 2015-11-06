#ifndef VIDEO_ENCODE_H_
#define VIDEO_ENCODE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <libavutil/opt.h>
#include <libavcodec/avcodec.h>
//#include <libavutil/channel_layout.h>
//#include <libavutil/common.h>
#include <libavutil/imgutils.h>
//#include <libavutil/mathematics.h>
//#include <libavutil/samplefmt.h>

#ifdef __cplusplus
}
#endif

#include <stdio.h>




#include <opencv2/core/core.hpp>

using namespace cv;



class VideoEncoder {

public:

	VideoEncoder(char *filename);
	~VideoEncoder();


	void encode(Mat &img);


private:

	int nframes = 0;

	AVCodec *codec;
	AVCodecContext *c = NULL;
	FILE *f;
	AVFrame *frame;
};


#endif
