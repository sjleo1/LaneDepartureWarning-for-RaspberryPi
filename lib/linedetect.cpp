#include "linedetect.h"
#include "devaccs.h"

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

cv::Vec2f adas::LaneDetector::_cart2polL(cv::Vec4i pts) {
	cv::Vec2f pol;
	pol[0] = (pts[1] * (pts[2] - pts[0]) - pts[0] * (pts[3] - pts[1])) / sqrt(pow(pts[2] - pts[0], 2) + pow(pts[3] - pts[1], 2));
	pol[1] = (CV_PI / 2) - atan(fabs(pts[1] - pts[3]) / fabs(pts[0] - pts[2]));
	return pol;
}
cv::Vec2f adas::LaneDetector::_cart2polR(cv::Vec4i pts) {
	cv::Vec2f pol;
	pol[0] = (pts[1] * (pts[2] - pts[0]) - pts[0] * (pts[3] - pts[1])) / sqrt(pow(pts[2] - pts[0], 2) + pow(pts[3] - pts[1], 2));
	pol[1] = (CV_PI / 2) + atan(fabs(pts[1] - pts[3]) / fabs(pts[0] - pts[2]));
	return pol;
}
adas::LaneDetector::LaneDetector(cv::VideoCapture vc, char* wavfile) :
			_image_height(vc.get(cv::CAP_PROP_FRAME_HEIGHT)),
			_image_width(vc.get(cv::CAP_PROP_FRAME_WIDTH)),
			_maskPoints(4),
			_linePoints(4),
			_leftLine(0.0f, 0.0f),
			_rightLine(0.0f, 0.0f),
			_lane_rel_pos(0),
			_threshold1(THRSHLD_1),
			_threshold2(THRSHLD_2),
			_image(vc.get(cv::CAP_PROP_FRAME_HEIGHT), vc.get(cv::CAP_PROP_FRAME_WIDTH), CV_8UC3),
			_mask(vc.get(cv::CAP_PROP_FRAME_HEIGHT), vc.get(cv::CAP_PROP_FRAME_WIDTH), CV_8UC1, cv::Scalar(0)),
			_imageCanny(vc.get(cv::CAP_PROP_FRAME_HEIGHT), vc.get(cv::CAP_PROP_FRAME_WIDTH), CV_8UC1, cv::Scalar(0)) {
	_maskPoints[0].x = 7 * _image_width / 16;
	_maskPoints[0].y = _image_height / 2;
	_maskPoints[1].x = 0;
	_maskPoints[1].y = _image_height;
	_maskPoints[2].x = _image_width;
	_maskPoints[2].y = _image_height;
	_maskPoints[3].x = 9 * _image_width / 16;
	_maskPoints[3].y = _image_height / 2;
	cv::fillPoly(_mask, _maskPoints, cv::Scalar(255));


	for (int i = 0; i < LAST_N_ANGLE; i++) {
		_left_angle[i] = CV_PI / 2;
		_right_angle[i] = CV_PI / 2;
	}
	_threshold_warn = THRSHLD_WARN;
	_should_warn = 0;
	
	_fbfd = open("/dev/fb0", O_RDWR);
	if (_fbfd < 0)
		std::cerr << "Failed to open the framebuffer device." << std::endl;
	if (ioctl(_fbfd, FBIOGET_FSCREENINFO, &_finfo) < 0)
		std::cerr << "Error reading framebuffer device information." << std::endl;
	if (strcmp("RPi-Sense FB", _finfo.id) != 0)
		std::cerr << "Wrong device is opened." << std::endl;
	_fb = (fb_t*)mmap(NULL, 64 * sizeof(uint16_t), PROT_READ | PROT_WRITE, MAP_SHARED, _fbfd, 0);
	if ((int)_fb == -1)
		std::cerr << "Error while mmap() framebuffer to memory. " << std::endl;
	memset(_fb, 0, 128);

	_evpoll = (pollfd*)malloc(sizeof(pollfd));
	_evpoll->events = POLLIN;
	_evpoll->fd = open("/dev/input/event0", O_RDONLY);
	if (_evpoll->fd == -1)
		std::cerr << "Failed to open the joystick." << std::endl;
	ioctl(_evpoll->fd, EVIOCGNAME(sizeof(_name)), _name);
	if (strcmp("Raspberry Pi Sense HAT Joystick", _name) != 0)
		std::cerr << "Error reading variable information." << std::endl;
	
	_fd = open(wavfile, O_RDONLY);
    if (_fd == -1)
        std::cerr << "Could not open the specified wave file." << std::endl;
    _count = read(_fd, &_wavheader, sizeof(WAVHEADER));
    if (_count < 1)
        std::cerr << "Could not read wave data." << std::endl;
    _rc = snd_pcm_open(&_handle, "default", SND_PCM_STREAM_PLAYBACK, SND_PCM_ASYNC);
    if (_rc < 0)
        std::cerr << "Unable to open pcm device: " << snd_strerror(_rc) << std::endl;
    snd_pcm_hw_params_alloca(&_params);
    snd_pcm_hw_params_any(_handle, _params);
    _channels = _wavheader.nChannels;
    snd_pcm_hw_params_set_channels(_handle, _params, _channels);
    switch (_wavheader.nblockAlign) {
    case 1:
        _format = SND_PCM_FORMAT_U8;
        break;
    case 2:
        _format = (_channels == 1) ? SND_PCM_FORMAT_S16_LE : SND_PCM_FORMAT_U8;
        break;
    case 4:
        _format = SND_PCM_FORMAT_S16_LE;
        break;
    default:
        std::cerr << "Unknown byte rate for sound." << std::endl;
		break;
    }
    snd_pcm_hw_params_set_access(_handle, _params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(_handle, _params, (snd_pcm_format_t)_format);
    _val = _wavheader.sampleRate;
    snd_pcm_hw_params_set_rate_near(_handle, _params, &_val, &_dir);
    _frames = 32;
    snd_pcm_hw_params_set_period_size_near(_handle, _params, &_frames, &_dir);
    _rc = snd_pcm_hw_params(_handle, _params);
    if (_rc < 0)
        std::cerr << "Unable to set HW paramters: " << snd_strerror(_rc) << std::endl;
    snd_pcm_hw_params_get_period_size(_params, &_frames, &_dir);
    _buf_size = _frames * _channels * ((_format == SND_PCM_FORMAT_S16_LE) ? 2 : 1);
    _buffer = (char*)malloc(_buf_size);
    snd_pcm_hw_params_get_period_time(_params, &_val, &_dir);
}

void adas::LaneDetector::detect(cv::VideoCapture vc) {
	unsigned int _n_left_lines = 0, _n_right_lines = 0;
	double _cos, _sin, _x_0, _y_0;
	std::vector<cv::Vec4i> _lines;
	cv::Vec2f __leftLine(0.0f, 0.0f), __rightLine(0.0f, 0.0f);

	vc >> _image;

	// indoor: 20, 60
	// outdoor: 400, 800
	cv::Canny(_image, _imageCanny, 20, 60);
	cv::bitwise_and(_imageCanny, _mask, _imageCanny);
	cv::HoughLinesP(_imageCanny, _lines, 1, CV_PI / 180, 70, 20, 5);

	for (int i = 0; i < _lines.size(); i++) {
		if (fabs(_lines[i][1] - _lines[i][3]) * 1.9f < fabs(_lines[i][0] - _lines[i][2])) {
			continue;
		}
		else {
			if (_lines[i][0] > _lines[i][2]) {
				if (_lines[i][1] < _lines[i][3]) {
					_n_left_lines++;
					__leftLine += _cart2polL(_lines[i]);
				}
				else if (_lines[i][1] > _lines[i][3]) {
					_n_right_lines++;
					__rightLine += _cart2polR(_lines[i]);
				}
			}
			else if (_lines[i][0] < _lines[i][2]) {
				if (_lines[i][1] < _lines[i][3]) {
					_n_right_lines++;
					__rightLine += _cart2polR(_lines[i]);
				}
				else if (_lines[i][1] > _lines[i][3]) {
					_n_left_lines++;
					__leftLine += _cart2polL(_lines[i]);
				}
			}
		}
	}

	__leftLine[0] /= _n_left_lines;
	__leftLine[1] /= _n_left_lines;
	__rightLine[0] /= _n_right_lines;
	__rightLine[1] /= _n_right_lines;

	if (_n_left_lines != 0) {
		_leftLine = __leftLine;
		_cos = cos(_leftLine[1]);
		_sin = sin(_leftLine[1]);
		_x_0 = _cos * _leftLine[0];
		_y_0 = _sin * _leftLine[0];
		_linePoints[0].x = cvRound(_x_0 + 1000 * (-_sin));
		_linePoints[0].y = cvRound(_y_0 + 1000 * (_cos));
		_linePoints[1].x = cvRound(_x_0 - 1000 * (-_sin));
		_linePoints[1].y = cvRound(_y_0 - 1000 * (_cos));

		_left_angle[_left_ind] = _leftLine[1];
		_left_ind = ++_left_ind % LAST_N_ANGLE;
		_mean_left_angle = 0.0f;
		for (int i = 0; i < LAST_N_ANGLE; i++)
			_mean_left_angle += _left_angle[i];
		_mean_left_angle /= LAST_N_ANGLE;
	}
	if (_n_right_lines != 0) {
		_rightLine = __rightLine;
		_cos = cos(_rightLine[1]);
		_sin = sin(_rightLine[1]);
		_x_0 = _cos * _rightLine[0];
		_y_0 = _sin * _rightLine[0];
		_linePoints[2].x = cvRound(_x_0 + 1000 * (-_sin));
		_linePoints[2].y = cvRound(_y_0 + 1000 * (_cos));
		_linePoints[3].x = cvRound(_x_0 - 1000 * (-_sin));
		_linePoints[3].y = cvRound(_y_0 - 1000 * (_cos));

		_right_angle[_right_ind] = _rightLine[1];
		_right_ind = ++_right_ind % LAST_N_ANGLE;
		_mean_right_angle = 0.0f;
		for (int i = 0; i < LAST_N_ANGLE; i++)
			_mean_right_angle += _right_angle[i];
		_mean_right_angle /= LAST_N_ANGLE;
	}

	return;
}
void adas::LaneDetector::detectV(cv::VideoCapture vc) {
	unsigned int _n_left_lines = 0, _n_right_lines = 0;
	double _cos, _sin, _x_0, _y_0;
	std::vector<cv::Vec4i> _lines;
	cv::Vec2f __leftLine(0.0f, 0.0f), __rightLine(0.0f, 0.0f);

	vc >> _image;

	// indoor: 20, 60
	// outdoor: 400, 800
	cv::imshow("original", _image);
	cv::Canny(_image, _imageCanny, 20, 60);
	cv::imshow("canny", _imageCanny);
	cv::bitwise_and(_imageCanny, _mask, _imageCanny);
	cv::imshow("canny bitwise and", _imageCanny);
	cv::HoughLinesP(_imageCanny, _lines, 1, CV_PI / 180, 70, 20, 5);

	for (int i = 0; i < _lines.size(); i++) {
		if (fabs(_lines[i][1] - _lines[i][3]) * 1.9f < fabs(_lines[i][0] - _lines[i][2])) {
			cv::line(_image, cv::Point(_lines[i][0], _lines[i][1]), cv::Point(_lines[i][2], _lines[i][3]), cv::Scalar(0, 255, 0), 2, 8);
			continue;
		}
		else {
			cv::line(_image, cv::Point(_lines[i][0], _lines[i][1]), cv::Point(_lines[i][2], _lines[i][3]), cv::Scalar(0, 0, 255), 2, 8);
			if (_lines[i][0] > _lines[i][2]) {
				if (_lines[i][1] < _lines[i][3]) {
					_n_left_lines++;
					__leftLine += _cart2polL(_lines[i]);
				}
				else if (_lines[i][1] > _lines[i][3]) {
					_n_right_lines++;
					__rightLine += _cart2polR(_lines[i]);
				}
			}
			else if (_lines[i][0] < _lines[i][2]) {
				if (_lines[i][1] < _lines[i][3]) {
					_n_right_lines++;
					__rightLine += _cart2polR(_lines[i]);
				}
				else if (_lines[i][1] > _lines[i][3]) {
					_n_left_lines++;
					__leftLine += _cart2polL(_lines[i]);
				}
			}
		}
	}

	__leftLine[0] /= _n_left_lines;
	__leftLine[1] /= _n_left_lines;
	__rightLine[0] /= _n_right_lines;
	__rightLine[1] /= _n_right_lines;
	std::cout << _leftLine << "\t" << _rightLine << std::endl;

	if (_n_left_lines != 0) {
		_leftLine = __leftLine;
		_cos = cos(_leftLine[1]);
		_sin = sin(_leftLine[1]);
		_x_0 = _cos * _leftLine[0];
		_y_0 = _sin * _leftLine[0];
		_linePoints[0].x = cvRound(_x_0 + 1000 * (-_sin));
		_linePoints[0].y = cvRound(_y_0 + 1000 * (_cos));
		_linePoints[1].x = cvRound(_x_0 - 1000 * (-_sin));
		_linePoints[1].y = cvRound(_y_0 - 1000 * (_cos));

		_left_angle[_left_ind] = _leftLine[1];
		_left_ind = ++_left_ind % LAST_N_ANGLE;
		_mean_left_angle = 0.0f;
		for (int i = 0; i < LAST_N_ANGLE; i++)
			_mean_left_angle += _left_angle[i];
		_mean_left_angle /= LAST_N_ANGLE;
	}
	if (_n_right_lines != 0) {
		_rightLine = __rightLine;
		_cos = cos(_rightLine[1]);
		_sin = sin(_rightLine[1]);
		_x_0 = _cos * _rightLine[0];
		_y_0 = _sin * _rightLine[0];
		_linePoints[2].x = cvRound(_x_0 + 1000 * (-_sin));
		_linePoints[2].y = cvRound(_y_0 + 1000 * (_cos));
		_linePoints[3].x = cvRound(_x_0 - 1000 * (-_sin));
		_linePoints[3].y = cvRound(_y_0 - 1000 * (_cos));

		_right_angle[_right_ind] = _rightLine[1];
		_right_ind = ++_right_ind % LAST_N_ANGLE;
		_mean_right_angle = 0.0f;
		for (int i = 0; i < LAST_N_ANGLE; i++)
			_mean_right_angle += _right_angle[i];
		_mean_right_angle /= LAST_N_ANGLE;
	}

	cv::line(_image, _linePoints[0], _linePoints[1], cv::Scalar(255, 255, 0), 2, 8);
	cv::line(_image, _linePoints[2], _linePoints[3], cv::Scalar(255, 0, 255), 2, 8);

	cv::line(_image, _maskPoints[0], _maskPoints[1], cv::Scalar(255, 255, 255));
	cv::line(_image, _maskPoints[2], _maskPoints[1], cv::Scalar(255, 255, 255));
	cv::line(_image, _maskPoints[2], _maskPoints[3], cv::Scalar(255, 255, 255));
	cv::line(_image, _maskPoints[0], _maskPoints[3], cv::Scalar(255, 255, 255));

	cv::imshow("original detected", _image);

	return;
}
void adas::LaneDetector::decide() {
	if (_rightLine[1] > CV_PI - _threshold2) {
		if (_rightLine[1] > CV_PI - _threshold1)
			_lane_rel_pos = 2;
		else
			_lane_rel_pos = 1;
	}
	else if (_leftLine[1] < _threshold2) {
		if (_leftLine[1] < _threshold1)
			_lane_rel_pos = -2;
		else
			_lane_rel_pos = -1;
	}
	else {
		_lane_rel_pos = 0;
		if ((_mean_left_angle >= _threshold_warn) && (_mean_right_angle <= CV_PI - _threshold_warn))
			_should_warn = 1;
	}
}
void adas::LaneDetector::warn() {
	if (_should_warn) {
		if ((_mean_left_angle < _threshold_warn) || (_mean_right_angle > CV_PI - _threshold_warn)) {
			lseek(_fd, sizeof(WAVHEADER), SEEK_SET);
			_remain = _wavheader.dataLen;
			_buf_size = _frames * _channels * ((_format == SND_PCM_FORMAT_S16_LE) ? 2 : 1);
			do {
				_buf_size = (_remain > _buf_size) ? _buf_size : _remain;
				if ((_count = read(_fd, _buffer, _buf_size)) <= 0)
					break;
				_remain -= _count;
				_rc = snd_pcm_writei(_handle, _buffer, _frames);
				if (_rc == -EPIPE) {
					//std::cerr << "Underrun occurred." << std::endl;
					snd_pcm_prepare(_handle);
				}
				else if (_rc < 0)
					std::cerr << "Error from write: " << snd_strerror(_rc) << std::endl;
				else if (_rc != (int)_frames)
					std::cerr << "Short write, write " << _rc << " frames." << std::endl;
			} while (_count == _buf_size);
			_should_warn = 0;
		}
	}
	else
		return;
}
void adas::LaneDetector::render() {
	memset(_fb, 0, 128);

	_fb->pix[1][3] = 0x1F;
	_fb->pix[1][4] = 0x1F;
	_fb->pix[2][3] = 0x1F;
	_fb->pix[2][4] = 0x1F;
	_fb->pix[3][3] = 0x1F;
	_fb->pix[3][4] = 0x1F;

	if (_lane_rel_pos == 0) {
		_fb->pix[0][1] = 0xFFFF;
		_fb->pix[1][1] = 0xFFFF;
		_fb->pix[2][1] = 0xFFFF;
		_fb->pix[3][1] = 0xFFFF;
		_fb->pix[4][1] = 0xFFFF;
		_fb->pix[5][1] = 0xFFFF;
		_fb->pix[6][1] = 0xFFFF;
		_fb->pix[7][1] = 0xFFFF;

		_fb->pix[0][6] = 0xFFFF;
		_fb->pix[1][6] = 0xFFFF;
		_fb->pix[2][6] = 0xFFFF;
		_fb->pix[3][6] = 0xFFFF;
		_fb->pix[4][6] = 0xFFFF;
		_fb->pix[5][6] = 0xFFFF;
		_fb->pix[6][6] = 0xFFFF;
		_fb->pix[7][6] = 0xFFFF;
		return;
	}
	else if (_lane_rel_pos == -1) {
		_fb->pix[0][0] = 0xFFFF;
		_fb->pix[1][0] = 0xFFFF;
		_fb->pix[2][0] = 0xFFFF;
		_fb->pix[3][0] = 0xFFFF;
		_fb->pix[4][0] = 0xFFFF;
		_fb->pix[5][0] = 0xFFFF;
		_fb->pix[6][0] = 0xFFFF;
		_fb->pix[7][0] = 0xFFFF;

		_fb->pix[0][5] = 0xFBE0;
		_fb->pix[1][5] = 0xFBE0;
		_fb->pix[2][5] = 0xFBE0;
		_fb->pix[3][5] = 0xFBE0;
		_fb->pix[4][5] = 0xFBE0;
		_fb->pix[5][5] = 0xFBE0;
		_fb->pix[6][5] = 0xFBE0;
		_fb->pix[7][5] = 0xFBE0;
		return;
	}
	else if (_lane_rel_pos == 1) {
		_fb->pix[0][2] = 0xFBE0;
		_fb->pix[1][2] = 0xFBE0;
		_fb->pix[2][2] = 0xFBE0;
		_fb->pix[3][2] = 0xFBE0;
		_fb->pix[4][2] = 0xFBE0;
		_fb->pix[5][2] = 0xFBE0;
		_fb->pix[6][2] = 0xFBE0;
		_fb->pix[7][2] = 0xFBE0;

		_fb->pix[0][7] = 0xFFFF;
		_fb->pix[1][7] = 0xFFFF;
		_fb->pix[2][7] = 0xFFFF;
		_fb->pix[3][7] = 0xFFFF;
		_fb->pix[4][7] = 0xFFFF;
		_fb->pix[5][7] = 0xFFFF;
		_fb->pix[6][7] = 0xFFFF;
		_fb->pix[7][7] = 0xFFFF;
		return;
	}
	else if (_lane_rel_pos == -2) {
		_fb->pix[0][4] = 0xF800;
		///////////////////////
		///////////////////////
		///////////////////////
		_fb->pix[4][4] = 0xF800;
		_fb->pix[5][4] = 0xF800;
		_fb->pix[6][4] = 0xF800;
		_fb->pix[7][4] = 0xF800;
		return;
	}
	else if (_lane_rel_pos == 2) {
		_fb->pix[0][3] = 0xF800;
		///////////////////////
		///////////////////////
		///////////////////////
		_fb->pix[4][3] = 0xF800;
		_fb->pix[5][3] = 0xF800;
		_fb->pix[6][3] = 0xF800;
		_fb->pix[7][3] = 0xF800;
		return;
	}

	return;

}
int adas::LaneDetector::read_event() {
	struct input_event ev[64];

	int rd = read(_evpoll->fd, ev, sizeof(struct input_event) * 64);
	if (rd < (int)sizeof(struct input_event)) {
		std::cerr << "Expected " << (int)sizeof(struct input_event) << " bytes, got " << rd << "bytes." << std::endl;
		return 1;
	}

	for (int i = 0; i < rd / sizeof(struct input_event); i++) {
		if (ev->type != EV_KEY)
			continue;
		if (ev->value != 1)
			continue;
		switch (ev->code) {
		case KEY_ENTER:
			return 0;
		default:
			break;
		}
	}

	return 1;
}