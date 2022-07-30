#include "devaccs.h"

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

dev::FrameBuffer::FrameBuffer() {
	int _fbfd = open("/dev/fb0", O_RDWR);
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
}

dev::EventDevice::EventDevice() {
	_evpoll = (pollfd*)malloc(sizeof(pollfd));
	_evpoll->events = POLLIN;
	_evpoll->fd = open("/dev/input/event0", O_RDONLY);
	if (_evpoll->fd == -1)
		std::cerr << "Failed to open the joystick." << std::endl;

	ioctl(_evpoll->fd, EVIOCGNAME(sizeof(_name)), _name);
	if (strcmp("Raspberry Pi Sense HAT Joystick", _name) != 0)
		std::cerr << "Error reading variable information." << std::endl;
}
int dev::EventDevice::read_event() {
	int ret = 1;
	
	while (poll(_evpoll, 1, 0) > 0) {
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
				ret = 0;
				continue;
			default:
				break;
			}
		}
	}

	return ret;
}


dev::WavPlayer::WavPlayer(char* wavfile) {
    _fd = open(wavfile, O_RDONLY);
    if (_fd == -1)
        std::cerr << "Could not open the specified wave file." << std::endl;

    _count = read(_fd, &_wavheader, sizeof(WAVHEADER));
    if (_count < 1)
        std::cerr << "Could not read wave data." << std::endl;

    _rc = snd_pcm_open(&_handle, "default", SND_PCM_STREAM_PLAYBACK, 0);
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
    _remain = _wavheader.dataLen;
}
dev::WavPlayer::~WavPlayer() {
    close(_fd);
    sleep(1);
    snd_pcm_drain(_handle);
    snd_pcm_close(_handle);
    free(_buffer);
}
void dev::WavPlayer::play() {
    do {
        _buf_size = (_remain > _buf_size) ? _buf_size : _remain;
        if ((_count = read(_fd, _buffer, _buf_size)) <= 0)
            break;
        _rc = snd_pcm_writei(_handle, _buffer, _frames);
        if (_rc == -EPIPE) {
            std::cerr << "Underrun occurred." << std::endl;
            snd_pcm_prepare(_handle);
        }
        else if (_rc < 0)
            std::cerr << "Error from write: " << snd_strerror(_rc) << std::endl;
        else if (_rc != (int)_frames)
            std::cerr << "Short write, write " << _rc << " frames." << std::endl;
    } while (_count == _buf_size);
}