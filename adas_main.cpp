#include <iostream>
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
#include "devaccs.h"
#include "linedetect.h"

int runADAS() {
	int running = 1;
    struct timeval start, end;
    unsigned long long frames = 0;

    dev::EventDevice ed;
    dev::WavPlayer wp("beep1.wav");

    cv::VideoCapture vc(0);
    if (!vc.isOpened()) {
        std::cerr << "Error opening the camera." << std::endl;
		return -1;
    }
    vc.set(cv::CAP_PROP_FORMAT, CV_8UC3);
	vc.set(cv::CAP_PROP_FRAME_WIDTH, IMG_W);
	vc.set(cv::CAP_PROP_FRAME_HEIGHT, IMG_H);

    adas::LaneDetector ld(vc, "beep1.wav");


    gettimeofday(&start, NULL);
	while (running) {
		frames++;
		
		ld.detect(vc);
		ld.decide();
		ld.warn();
		ld.render();

		running = ed.read_event();
	}
	gettimeofday(&end, NULL);
	std::cout << frames / (double)(end.tv_sec - start.tv_sec) << " fps" << std::endl;

    return 0;
}

int runADAS_Visual() {
	int running = 1;
    struct timeval start, end;
    unsigned long long frames = 0;

    dev::EventDevice ed;
    dev::WavPlayer wp("beep1.wav");

    cv::VideoCapture vc(0);
    if (!vc.isOpened()) {
        std::cerr << "Error opening the camera." << std::endl;
		return -1;
    }
    vc.set(cv::CAP_PROP_FORMAT, CV_8UC3);
	vc.set(cv::CAP_PROP_FRAME_WIDTH, IMG_W);
	vc.set(cv::CAP_PROP_FRAME_HEIGHT, IMG_H);

    adas::LaneDetector ld(vc, "beep1.wav");


    gettimeofday(&start, NULL);
	while (running) {
		frames++;
		
		ld.detectV(vc);
		ld.decide();
		ld.warn();
		ld.render();

		running = ed.read_event();
		if (cv::waitKey(20) == 27)
			running = 0;
	}
	gettimeofday(&end, NULL);
	std::cout << frames / (double)(end.tv_sec - start.tv_sec) << " fps" << std::endl;

    return 0;
}

int camSet() {
	cv::VideoCapture vc(0);
    if (!vc.isOpened()) {
        std::cerr << "Error opening the camera." << std::endl;
		return -1;
    }
    vc.set(cv::CAP_PROP_FORMAT, CV_8UC3);
	vc.set(cv::CAP_PROP_FRAME_WIDTH, IMG_W);
	vc.set(cv::CAP_PROP_FRAME_HEIGHT, IMG_H);

	cv::Mat image(IMG_H, IMG_W, CV_8UC3);
	std::vector<cv::Point> maskPoints(4);
	maskPoints[0].x = 7 * IMG_W / 16;
	maskPoints[0].y = IMG_H / 2;
	maskPoints[1].x = 0;
	maskPoints[1].y = IMG_H;
	maskPoints[2].x = IMG_W;
	maskPoints[2].y = IMG_H;
	maskPoints[3].x = 9 * IMG_W / 16;
	maskPoints[3].y = IMG_H / 2;

	while (1) {
		vc >> image;

		cv::line(image, maskPoints[0], maskPoints[1], cv::Scalar(255, 0, 0));
		cv::line(image, maskPoints[2], maskPoints[1], cv::Scalar(255, 0, 0));
		cv::line(image, maskPoints[2], maskPoints[3], cv::Scalar(255, 0, 0));
		cv::line(image, maskPoints[0], maskPoints[3], cv::Scalar(255, 0, 0));

		cv::imshow("cam setup", image);

		if (cv::waitKey(20) == 27)
			break;
	}

	return 0;
}

int main(int argc, char* argv[]) {
	if (argc == 1) {
		return runADAS();
	}
	else if (argc == 2) {
		if (strcmp(argv[1], "setup") == 0) {
			return camSet();
		}
		else if (strcmp(argv[1], "visual") == 0) {
			return runADAS_Visual();
		}
		else {
			std::cerr << argv[1] << ": command not found." << std::endl;
			return -1;
		}
	}
	else {
		std::cerr << "Number of commands: " << (argc - 1) << std::endl;
		return -1;
	}

	return 0;
}