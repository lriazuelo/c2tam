/*
	This file is part of the CVD Library.

	Copyright (C) 2005 The Authors

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <cvd/Linux/v4lbuffer.h>
#include <cvd/timer.h>

#include <vector>
#include <iomanip>
using namespace std;

namespace CVD { // CVD

Exceptions::V4LBuffer::DeviceOpen::DeviceOpen(string device)
{
    what = "V4LBuffer: failed to open \""+device+ "\": " + strerror(errno);
}

Exceptions::V4LBuffer::DeviceSetup::DeviceSetup(string device, string action)
{
    what = "V4LBuffer: \""+action + "\" ioctl failed on " + device + ": " +strerror(errno);
}

Exceptions::V4LBuffer::PutFrame::PutFrame(string device, string msg)
{
    what = "V4LBuffer: PutFrame on " + device + " failed: " + msg;
}

Exceptions::V4LBuffer::GetFrame::GetFrame(string device, string msg)
{
    what = "V4LBuffer: GetFrame on " + device + " failed: " + msg;
}


class VPrint_
{
	public:
		VPrint_(bool _p)
		:p(_p)
		{}

		template<class C> VPrint_& operator<<(const C& c)
		{
			if(p)
				cerr << c;
			
			return *this;
		}
	
	bool p;
};

class VPrint: public VPrint_
{
	public:
	VPrint(bool _b)
	:VPrint_(_b){}

		template<class C> VPrint_& operator<<(const C& c)
		{
			if(p)
				cerr << "V4L2Client: " << c;
			
			return *this;
		}

};

string unfourcc(unsigned long c)
{
	string ret;
	ret.resize(4);

	ret[0] = c & 0xff;
	ret[1] = (c>>8) & 0xff;
	ret[2] = (c>>16) & 0xff;
	ret[3] = (c>>24) & 0xff;
	return ret;
}

typedef const char fourcc_string[5];

unsigned int fourcc(const fourcc_string& s)
{
	return v4l2_fourcc(s[0], s[1], s[2], s[3]);
}

void print_v4l2_framerates(int fd, unsigned int fmt, unsigned int width, unsigned int height, VPrint & log){
    struct v4l2_frmivalenum fr;
    fr.pixel_format = fmt;
    fr.width = width;
    fr.height = height;
    for(fr.index = 0; ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &fr) == 0; ++fr.index){
        switch(fr.type){
        case V4L2_FRMIVAL_TYPE_DISCRETE:
            log << "\t\t      rate discrete\t" << fr.discrete.numerator << "/" << fr.discrete.denominator << "\n";
            break;
        case V4L2_FRMIVAL_TYPE_CONTINUOUS:
            log << "\t\t      rate cont\t" << fr.stepwise.min.numerator << "/" << fr.stepwise.min.denominator << " - " << fr.stepwise.step.numerator << "/" << fr.stepwise.step.denominator << " - " << fr.stepwise.max.numerator << "/" << fr.stepwise.max.denominator << "\n";
            break;
        case V4L2_FRMIVAL_TYPE_STEPWISE:
            log << "\t\t      rate step\t" << fr.stepwise.min.numerator << "/" << fr.stepwise.min.denominator << " - " << fr.stepwise.step.numerator << "/" << fr.stepwise.step.denominator << " - " << fr.stepwise.max.numerator << "/" << fr.stepwise.max.denominator << "\n";
            break;
        default: assert(false);
        }
    }
}

namespace V4L { // V4L

    struct V4L2Client::State {
	struct Frame {
	    void* data;
	    size_t length;
	};
	int fd;
	v4l2_format format;
	vector<Frame> frames;
	double frameRate;
	v4l2_buffer refbuf;
    };

  V4L2Client::V4L2Client(int fd, unsigned int fmt, ImageRef size, int input, bool fields, int frames_per_second, bool verbose)
	{
	state = 0;

	VPrint log(verbose);

	log << "verbose operation. Initializing\n";
	log << "Querying capabilities.\n";
	struct v4l2_capability caps;
	if (0 != ioctl(fd, VIDIOC_QUERYCAP, &caps))
	{
	    throw string("VIDIOC_QUERYCAP failed");
	}

	log << "   driver: " << caps.driver << "\n";
	log << "   card: " << caps.card << "\n";
	log << "   bus_info: " << caps.bus_info << "\n";
	log << "   version: " << caps.version  << "\n";
	log << "   capabilities: 0x" << hex << caps.capabilities << dec << "\n";
	log << "                V4L2_CAP_VIDEO_CAPTURE        = " << !!(caps.capabilities & V4L2_CAP_VIDEO_CAPTURE) << "\n";
	log << "                V4L2_CAP_VIDEO_OUTPUT         = " << !!(caps.capabilities & V4L2_CAP_VIDEO_OUTPUT) << "\n";
	log << "                V4L2_CAP_VIDEO_OVERLAY        = " << !!(caps.capabilities & V4L2_CAP_VIDEO_OVERLAY) << "\n";
	log << "                V4L2_CAP_VBI_CAPTURE          = " << !!(caps.capabilities & V4L2_CAP_VBI_CAPTURE) << "\n";
	log << "                V4L2_CAP_VBI_OUTPUT           = " << !!(caps.capabilities & V4L2_CAP_VBI_OUTPUT) << "\n";
	log << "                V4L2_CAP_SLICED_VBI_CAPTURE   = " << !!(caps.capabilities & V4L2_CAP_SLICED_VBI_CAPTURE) << "\n";
	log << "                V4L2_CAP_SLICED_VBI_OUTPUT    = " << !!(caps.capabilities & V4L2_CAP_VIDEO_CAPTURE) << "\n";
	log << "                V4L2_CAP_RDS_CAPTURE          = " << !!(caps.capabilities & V4L2_CAP_SLICED_VBI_OUTPUT) << "\n";
	#ifdef V4L2_CAP_VIDEO_OUTPUT_OVERLAY
		log << "                V4L2_CAP_VIDEO_OUTPUT_OVERLAY = " << !!(caps.capabilities & V4L2_CAP_VIDEO_OUTPUT_OVERLAY) << "\n";
	#endif
	log << "                V4L2_CAP_TUNER                = " << !!(caps.capabilities & V4L2_CAP_VIDEO_CAPTURE) << "\n";
	log << "                V4L2_CAP_AUDIO                = " << !!(caps.capabilities & V4L2_CAP_TUNER) << "\n";
	log << "                V4L2_CAP_RADIO                = " << !!(caps.capabilities & V4L2_CAP_RADIO) << "\n";
	log << "                V4L2_CAP_READWRITE            = " << !!(caps.capabilities & V4L2_CAP_READWRITE) << "\n";
	log << "                V4L2_CAP_ASYNCIO              = " << !!(caps.capabilities & V4L2_CAP_ASYNCIO) << "\n";
	log << "                V4L2_CAP_STREAMING            = " << !!(caps.capabilities & V4L2_CAP_STREAMING) << "\n";


	log << "Requested format: " << unfourcc(fmt) << "\n";
	unsigned int actual_fmt=fourcc("None");

	log << "Enumerating formats.\n";

	struct v4l2_fmtdesc f;
	f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	log   << "   Index  FourCC     Flags    Description\n";
	for(f.index=0; ioctl(fd, VIDIOC_ENUM_FMT, &f) == 0; f.index++)
	{
		log << "     " << setw(3) << setfill(' ') << f.index << "   "<< unfourcc(f.pixelformat) 
				<< "   0x" << hex <<  setfill('0')  << setw(8) << f.flags << dec << "  " 
				<< f.description << "\n";

		if(fmt == f.pixelformat)
			actual_fmt = fmt;

		//Hack. Some cameras won't send GREY, but the planar formats
		//are close enough, since the first chunk is the same as GREY
		if(fmt == V4L2_PIX_FMT_GREY && actual_fmt == fourcc("None") && (
		   f.pixelformat == fourcc("422P") ||
		   f.pixelformat == fourcc("YU12") ||
		   f.pixelformat == fourcc("YV12") ||
		   f.pixelformat == fourcc("411P") ||
		   f.pixelformat == fourcc("YUV9") ||
		   f.pixelformat == fourcc("YVU9")))
		{
			actual_fmt = f.pixelformat;
		}
        if(verbose){
            struct v4l2_frmsizeenum fs;
            fs.pixel_format = f.pixelformat;
            for(fs.index = 0; ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &fs) == 0; ++fs.index){
                switch(fs.type){
                case V4L2_FRMSIZE_TYPE_DISCRETE:
                    log << "\t\tframe size discrete\t" << fs.discrete.width << "x" << fs.discrete.height << "\n";
                    print_v4l2_framerates(fd, fs.pixel_format, fs.discrete.width, fs.discrete.height, log);
                    break;
                case V4L2_FRMSIZE_TYPE_CONTINUOUS:
                    log << "\t\tframe size cont\t" << fs.stepwise.min_width << "x" << fs.stepwise.min_height << " - " << fs.stepwise.step_width << "x" << fs.stepwise.step_height << " - " << fs.stepwise.max_width << "x" << fs.stepwise.max_height <<  "\n";
                    print_v4l2_framerates(fd, fs.pixel_format, fs.stepwise.max_width, fs.stepwise.max_height, log);
                    break;
                case V4L2_FRMSIZE_TYPE_STEPWISE: {
                    log << "\t\tframe size step\t" << fs.stepwise.min_width << "x" << fs.stepwise.min_height << " - " << fs.stepwise.step_width << "x" << fs.stepwise.step_height << " - " << fs.stepwise.max_width << "x" << fs.stepwise.max_height <<  "\n";
                    print_v4l2_framerates(fd, fs.pixel_format, fs.stepwise.max_width, fs.stepwise.max_height, log);
                } break;
                default: assert(false);
                }
            }
        }
    }

	if(errno != EINVAL)
		throw string("VIDIOC_ENUM_FMT");
	
	fmt = actual_fmt;
	log << "Selected format: " << unfourcc(fmt) << "\n";


	if (strcmp((const char*)caps.driver,"bttv") == 0)
	{
	    v4l2_std_id stdId=V4L2_STD_PAL;
	    if(ioctl(fd, VIDIOC_S_STD, &stdId ))
				throw string("VIDIOC_S_STD");
	}
	
	if (input != -1) {
	    struct v4l2_input v4l2Input;
	    v4l2Input.index=input; 
	    if (0 != ioctl(fd, VIDIOC_S_INPUT, &v4l2Input))
				throw string("VIDIOC_S_INPUT");
	}

	// Get / Set capture format.
	struct v4l2_format format;
	format.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;

	log << "Getting format (VIDIOC_G_FMT)\n";
	
	if (0 != ioctl(fd, VIDIOC_G_FMT, &format))
	    throw string("VIDIOC_G_FMT");

	log << "   size: " << format.fmt.pix.width  << "x" << 	format.fmt.pix.height << "\n";
	log << "   format: " << unfourcc(format.fmt.pix.pixelformat) << "\n";
	log << "   field flag: " << format.fmt.pix.field << "\n";
	log << "   bytes per line: " << format.fmt.pix.bytesperline << "\n";
	log << "   image size: " << format.fmt.pix.sizeimage << "\n";
	log << "   colourspace: " << format.fmt.pix.colorspace << "\n";

	format.fmt.pix.width = size.x;
	format.fmt.pix.height = size.y;
	format.fmt.pix.pixelformat = fmt;
	format.fmt.pix.field = fields ? V4L2_FIELD_ALTERNATE : V4L2_FIELD_ANY;
	
	log << "Setting format (VIDIOC_S_FMT)\n";
	log << "   size: " << format.fmt.pix.width  << "x" << 	format.fmt.pix.height << "\n";
	log << "   format: " << unfourcc(format.fmt.pix.pixelformat) << "\n";
	log << "   field flag: " << format.fmt.pix.field << "\n";
	log << "   bytes per line: " << format.fmt.pix.bytesperline << "\n";
	log << "   image size: " << format.fmt.pix.sizeimage << "\n";
	log << "   colourspace: " << format.fmt.pix.colorspace << "\n";

	if (0 != ioctl(fd, VIDIOC_S_FMT, &format))
	    throw string("VIDIOC_S_FMT");

	log << "Getting format (VIDIOC_G_FMT)\n";
	if (0 != ioctl(fd, VIDIOC_G_FMT, &format))
	    throw string("VIDIOC_G_FMT");

	log << "   size: " << format.fmt.pix.width  << "x" << 	format.fmt.pix.height << "\n";
	log << "   format: " << unfourcc(format.fmt.pix.pixelformat) << "\n";
	log << "   field flag: " << format.fmt.pix.field << "\n";
	log << "   bytes per line: " << format.fmt.pix.bytesperline << "\n";
	log << "   image size: " << format.fmt.pix.sizeimage << "\n";
	log << "   colourspace: " << format.fmt.pix.colorspace << "\n";

	
	if (fmt != format.fmt.pix.pixelformat)
	    throw string("Requested format not supported");

	struct v4l2_requestbuffers reqbufs;
	reqbufs.count = 10;
	reqbufs.memory = V4L2_MEMORY_MMAP;
	reqbufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	log << "Issuing VIDIOC_REQBUFS ioctl.\n";
	int ret = ioctl(fd,VIDIOC_REQBUFS,&reqbufs);
	log << "   Return code: " << ret << "\n";
	
	//WARNING!!!!!
	//The documentation says -1 for error, 0 otherwise.
	//The BTTV driver returns num_bufs on success!
	//So the test against 0 fails.
	if (ioctl(fd,VIDIOC_REQBUFS,&reqbufs) == -1)
	    throw string("VIDIOC_REQBUFS");

	num_bufs = reqbufs.count;

	log << "Number of buffers: " << num_bufs << "\n";
	
	if (reqbufs.count < 2)
	    throw string("Insufficient buffers available");
	vector<State::Frame> frames(reqbufs.count);
	struct v4l2_buffer refbuf;
	for (size_t i=0; i<frames.size(); i++) {
	    struct v4l2_buffer buffer;
	    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	    buffer.memory = V4L2_MEMORY_MMAP;
	    buffer.index = i;
	    if (0 != ioctl(fd, VIDIOC_QUERYBUF, &buffer))
		throw string("VIDIOC_QUERYBUF");
	    if (i == 0)
		refbuf = buffer;
	    frames[i].data = mmap(0,buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buffer.m.offset);
	    if (frames[i].data == MAP_FAILED) {
		frames[i].data = mmap(0,buffer.length, PROT_READ, MAP_SHARED, fd, buffer.m.offset);
		if (frames[i].data == MAP_FAILED)
		    throw string("mmap failed");
	    }
	    frames[i].length = buffer.length;
	    if (0 != ioctl(fd, VIDIOC_QBUF, &buffer))
		throw string("VIDIOC_QBUF");
	}

	// Do we want to manually set FPS?
	if(frames_per_second != 0) 
	  {
	    v4l2_streamparm streamparams;
	    streamparams.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	    if (0 != ioctl(fd, VIDIOC_G_PARM, &streamparams))
	      throw string("VIDIOC_G_PARM");
	    
	    // Check if the device has the capability to set a frame-rate.
	    if(streamparams.parm.capture.capability & V4L2_CAP_TIMEPERFRAME)
	      {
		streamparams.parm.capture.timeperframe.denominator = frames_per_second;
		streamparams.parm.capture.timeperframe.numerator = 1;
		if (0 != ioctl(fd, VIDIOC_S_PARM, &streamparams))
		  throw string("VIDIOC_S_PARM");
	      }
	  }

	if (0 != ioctl(fd, VIDIOC_STREAMON, &reqbufs.type))
	    throw string("STREAMON");
	state = new State;
	state->fd = fd;
	state->format = format;
	state->frames = frames;	
	state->frameRate = 0;
	state->refbuf = refbuf;
    }
    V4L2Client::~V4L2Client() {
	if (state == 0)
	    return;
	if(0 != ioctl(state->fd, VIDIOC_STREAMOFF, &state->refbuf.type))
	  throw string("streamoff failed");
	for (size_t i=0; i<state->frames.size(); i++) {
	    if (0 != munmap(state->frames[i].data, state->frames[i].length))
		throw string("munmap failed");
	}
	close(state->fd);
	delete state;
    }

    ImageRef V4L2Client::getSize()
    {
	return ImageRef(state->format.fmt.pix.width, state->format.fmt.pix.height);
    }
    
    V4L2Client::Buffer V4L2Client::getFrame()
    {
	struct v4l2_buffer buffer = state->refbuf;
	buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buffer.memory = V4L2_MEMORY_MMAP;
	int err = ioctl(state->fd, VIDIOC_DQBUF, &buffer);
	while (err != 0 && errno == EAGAIN) {
	    usleep(10);
	    err = ioctl(state->fd, VIDIOC_DQBUF, &buffer);
	}
	if (err != 0)
	    throw string("VIDIOC_DQBUF");

	Buffer ret;
	ret.id = buffer.index;
	ret.data = static_cast<unsigned char*>(state->frames[buffer.index].data);
	ret.when = timer.conv_ntime(buffer.timestamp);
	return ret;
    }
    
    void V4L2Client::releaseFrame(int id)
    {
	struct v4l2_buffer buffer = state->refbuf;
	buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buffer.memory = V4L2_MEMORY_MMAP;
	buffer.index = id;
	
	if (0 != ioctl(state->fd, VIDIOC_QBUF, &buffer))
	    throw string("VIDIOC_QBUF");
    }

    double V4L2Client::getRate() {
	return state->frameRate;
    }

    bool V4L2Client::pendingFrame() {
	fd_set fdsetRead;
	fd_set fdsetOther;
	struct timeval tv;
 
        FD_ZERO(&fdsetRead);
	FD_SET(state->fd,&fdsetRead);
	FD_ZERO(&fdsetOther);
	tv.tv_sec=0;
	tv.tv_usec=0;
	if (select(state->fd+1,&fdsetRead,&fdsetOther,&fdsetOther,&tv)>0)
		return true;
	return false;
    }

};

};
