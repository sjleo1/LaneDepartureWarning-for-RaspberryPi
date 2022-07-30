#ifndef __DEVACCS_H__
#define __DEVACCS_H__
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/input.h>
#include <poll.h>
#include <cstring>
#include <dirent.h>
#include <opencv2/highgui/highgui.hpp>
#include <raspicam/raspicam_cv.h>
#include <alsa/asoundlib.h>
#include <alsa/control.h>

namespace dev {
	class FrameBuffer {
	private:
		struct fb_t {
			uint16_t pix[8][8];
		};
		struct fb_fix_screeninfo _finfo;
		int _fbfd;

	public:
		fb_t* _fb;
		FrameBuffer();
	};

	class EventDevice {
	private:
		struct pollfd* _evpoll;
		char _name[256];

	public:
		EventDevice();
		int read_event();
	};

	class WavPlayer {
	private:
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

	public:
		WavPlayer(char* wavfile); // only one of two?
		~WavPlayer();
		void play();
	};
}

#endif