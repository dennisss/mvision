// Encodes OpenCV frames to a video

#include "vio.h"
#include "video_encode.h"

#include <opencv2/imgproc/imgproc.hpp>

#include <math.h>


VideoEncoder::VideoEncoder(char *filename){ // should be a .mpg file
	avcodec_register_all();


	AVCodecID codec_id = AV_CODEC_ID_MPEG2VIDEO; //AV_CODEC_ID_H264; //AV_CODEC_ID_MPEG1VIDEO;

	/* find the mpeg1 video encoder */
	codec = avcodec_find_encoder(codec_id);
	if(!codec){
		LOGI("Codec not found");
		exit(1);
	}

	c = avcodec_alloc_context3(codec);
	if(!c){
		LOGI("Could not allocate video codec context");
		exit(1);
	}


	/* put sample parameters */
	c->bit_rate = 400000;
	/* resolution must be a multiple of two */
	//c->width = 352;
	//c->height = 288;
	c->width = 1920;
	c->height = 1080;
	/* frames per second */
	c->time_base = (AVRational){1,30};
	/* emit one intra frame every ten frames
     * check frame pict_type before passing frame
     * to encoder, if frame->pict_type is AV_PICTURE_TYPE_I
     * then gop_size is ignored and the output of encoder
     * will always be I frame irrespective to gop_size
     */
	c->gop_size = 10;
	c->max_b_frames = 2;
	c->pix_fmt = AV_PIX_FMT_YUV420P; //AV_PIX_FMT_BGR24;

	if(codec_id == AV_CODEC_ID_H264)
		av_opt_set(c->priv_data, "preset", "veryfast", 0);

	/* open it */
	int res;
	if(avcodec_open2(c, codec, NULL) < 0){
		LOGE("Could not open codec");
		exit(1);
	}


	f = fopen(filename, "wb");
	if(!f) {
		LOGE("Could not open %s", filename);
		exit(1);
	}

	frame = av_frame_alloc();
	if(!frame){
		LOGE("Could not allocate video frame");
		exit(1);
	}
	frame->format = c->pix_fmt;
	frame->width  = c->width;
	frame->height = c->height;

	/* the image can be allocated by any means and av_image_alloc() is
     * just the most convenient way if av_malloc() is to be used */
	int ret = av_image_alloc(frame->data, frame->linesize, c->width, c->height,
						 c->pix_fmt, 32);
	if(ret < 0){
		LOGE("Could not allocate raw picture buffer");
		exit(1);
	}


}


VideoEncoder::~VideoEncoder(){

	AVPacket pkt;

	uint8_t endcode[] = { 0, 0, 1, 0xb7 };

	/* get the delayed frames */
	for(int got_output = 1; got_output; /*i++*/) {
		fflush(stdout);

		int ret = avcodec_encode_video2(c, &pkt, NULL, &got_output);
		if(ret < 0) {
			LOGE("Error encoding frame");
			exit(1);
		}

		if(got_output) {
			LOGI("Write frame %3d (size=%5d)", nframes++, pkt.size);
			fwrite(pkt.data, 1, pkt.size, f);
			av_free_packet(&pkt);
		}
	}

	/* add sequence end code to have a real mpeg file */
	fwrite(endcode, 1, sizeof(endcode), f);
	fclose(f);

	avcodec_close(c);
	av_free(c);
	av_freep(&frame->data[0]);
	av_frame_free(&frame);
	//printf("\n");

}


void VideoEncoder::encode(Mat &imgRaw){

	Mat img;

	cvtColor(imgRaw, img, CV_BGR2YCrCb);


	AVPacket pkt;

	LOGI("ENCODE THIS FRAME");

	av_init_packet(&pkt);
	pkt.data = NULL;    // packet data will be allocated by the encoder
	pkt.size = 0;

	//fflush(stdout);

	// Copy over the image
	for(int y = 0; y < c->height; y++) {
		for(int x = 0; x < c->width; x++) {
			Vec3b intensity = img.at<Vec3b>(y, x);

			frame->data[0][y * frame->linesize[0] + x] = intensity.val[0];
			frame->data[1][(y/2) * frame->linesize[1] + (x / 2)] = intensity.val[1];
			frame->data[2][(y/2) * frame->linesize[2] + (x / 2)] = intensity.val[2];
		}
	}

	frame->pts = nframes++;


	LOGI("COPIED THE FRAME");


	/* encode the image */
	int got_output;
	int ret = avcodec_encode_video2(c, &pkt, frame, &got_output);
	if(ret < 0) {
		LOGE("Error encoding frame");
		exit(1);
	}

	if(got_output) {
		LOGI("Write frame %3d (size=%5d)", nframes - 1, pkt.size);
		fwrite(pkt.data, 1, pkt.size, f);
		av_free_packet(&pkt);
	}

	LOGI("DONE ENCODING");

}

