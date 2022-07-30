#ifndef __LINEDETECT_H__
#define __LINEDETECT_H__
#include <iostream>
#include <queue>
#include <stdlib.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <string.h>
#include <linux/input.h>
#include <raspicam/raspicam_cv.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <unistd.h>
#include <poll.h>
#include <time.h>
#include <math.h>
#include <alsa/asoundlib.h>
#include <alsa/control.h>

#define IMG_H 480
#define IMG_W 640
#define THRSHLD_1 0.45f
#define THRSHLD_2 0.65f
#define THRSHLD_WARN 0.65f
#define LAST_N_ANGLE 2

namespace adas {
	class LaneDetector {
	private:
		// Frame's width and height
		size_t _image_width, _image_height;
		// Mat to save current frame; mask to filter out background image; Mat to save Canny-ed image
		cv::Mat _image, _mask, _imageCanny;//(IMG_H, IMG_W, CV_8UC3)
		//
		int _canny_threshold1, _canny_threshold2;
		// Points of the polygon on the mask
		std::vector<cv::Point> _maskPoints;
		// Points of the two ends of the estimated two lines
		std::vector<cv::Point> _linePoints;
		// The distance and angle from the top left corner of the image of the estimated two lines
		cv::Vec2f _leftLine, _rightLine;
		// Last n number of the angles of the estimated lines and their arithmetic mean
		int _left_ind = 0, _right_ind = 0;
		float _left_angle[LAST_N_ANGLE], _right_angle[LAST_N_ANGLE];
		float _mean_left_angle, _mean_right_angle;
		
		// Converts the two endpoints of a line in cartesian coordinate into polar coordinate
		cv::Vec2f _cart2polL(cv::Vec4i pts);
		cv::Vec2f _cart2polR(cv::Vec4i pts);

		// Ranges from -2 to 2. 0 means it's centered in the lane
		int _lane_rel_pos;
		// Angle threshold to decide lane departure level
		float _threshold1, _threshold2, _threshold_warn;

		// For framebuffer
		struct fb_t {
			uint16_t pix[8][8];
		};
		fb_t* _fb;
		struct fb_fix_screeninfo _finfo;
		int _fbfd;

		// For event device
		struct pollfd* _evpoll;
		char _name[256];
		
		// For speaker
		typedef struct wavFile_t {
			unsigned char riffID[4];
			unsigned long riffLen;
			unsigned char waveID[4];
			unsigned char fmtID[4];
			unsigned long fmtLen;
			unsigned short fmtTag;
			unsigned short nChannels;
			unsigned long sampleRate;
			unsigned long avgBytesPerSec;
			unsigned short nblockAlign;
			unsigned short bitsPerSample;
			unsigned char dataID[4];
			unsigned long dataLen;
		} WAVHEADER;
		int _fd, _rc, _buf_size, _dir, _channels, _format;
		long _count, _remain;
		unsigned int _val;
		char* _buffer;
		snd_pcm_t* _handle;
		snd_pcm_hw_params_t* _params;
		snd_pcm_uframes_t _frames;
		WAVHEADER _wavheader;
		int _should_warn;

			
	public:
		// Constructor
		LaneDetector(char* wavfile);
		// Takes frame's width and height directly
		LaneDetector(size_t image_width, size_t image_height, char* wavfile);
		// Takes frames' width and height using VideoCaptureProperties
		LaneDetector(cv::VideoCapture vc, char* wavfile);
		// Detects lines and estimate their relative position.
		void detect(cv::VideoCapture vc);
		// Detects and show the image
		void detectV(cv::VideoCapture vc);
		// Decides the level of the car being centered at the lane
		void decide();
		// Warns the user
		void warn();
		// Draws current status at the LED
		void render();
		// Reads events from the joystick and returns number
		int read_event();
	};
}

#endif