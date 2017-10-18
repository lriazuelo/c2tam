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
// -*- c++ -*-

#include <cvd/Linux/dvbuffer3.h>
#include <cvd/byte.h>
#include <cvd/Linux/dvbuffer.h>
#include <vector>
#include <algorithm>

using namespace CVD;
using namespace DV3;
using CVD::Exceptions::DVBuffer3::All;

namespace CVD 
{

  namespace DV3
  {
    
    struct LibDCParams 
    {
      DC::RawDCVideo *pRawDCV;
    };
    
    static unsigned int DC_from_DV3_Feature(DV3Feature f)
    {
      switch(f)
	{
	case BRIGHTNESS:    return FEATURE_BRIGHTNESS;
	case EXPOSURE  :    return FEATURE_EXPOSURE;
	case SHARPNESS:     return FEATURE_SHARPNESS;
	case WHITE_BALANCE: return FEATURE_WHITE_BALANCE;
	case HUE:           return FEATURE_HUE;
	case SATURATION:    return FEATURE_SATURATION;
	case GAMMA:         return FEATURE_GAMMA;
	case SHUTTER:       return FEATURE_SHUTTER;
	case GAIN:          return FEATURE_GAIN;
	case IRIS:          return FEATURE_IRIS;
	case FOCUS:         return FEATURE_FOCUS;
	case ZOOM:          return FEATURE_ZOOM;
	case PAN:           return FEATURE_PAN;
	case TILT:          return FEATURE_TILT;
	}
    }

    RawDVBuffer3::RawDVBuffer3(DV3ColourSpace colourspace,
			       unsigned int nCamNumber,
			       ImageRef irSize,
			       float fFrameRate,
			       ImageRef irOffset)
    {
      
      int mode;
      double fps;
      // Mode and FPS selection are VERY HACKY
      // Basically this does what DVBuffer 2 would do
      // And in doing so completely ignores the requested values
      if(colourspace == YUV411)
	{
	  mode = MODE_640x480_YUV411;
	  fps = 30.0;
	}
      else if(colourspace == RGB8)
	{
	  mode = MODE_640x480_RGB;
	  fps = 15.0;
	}
      else if(colourspace == MONO8)
	{
	  mode = MODE_640x480_MONO;
	  fps = 30.0;
	}
      else
	throw CVD::Exceptions::DVBuffer3::All("Non-implemented colourspace for DVBuffer2's RawDCVideo.");
      
      if(fFrameRate > 0 && fFrameRate != fps)
	std::cout << "! Warning: DVBuffer3/libdc1394v1: Ignoring requested frame-rate, you're getting " << fps << " instead." << std::endl;
      
      mpLDCP = new LibDCParams;
      mpLDCP->pRawDCV = NULL;
      mpLDCP->pRawDCV = new DC::RawDCVideo(nCamNumber,
					   4,
					   -1, -1, 
					   mode, fps);
      
      mirSize = mpLDCP->pRawDCV->size();
      mdFramerate = mpLDCP->pRawDCV->frame_rate();
    }
    
    RawDVBuffer3::~RawDVBuffer3()
    {
      if(mpLDCP->pRawDCV)
	delete(mpLDCP->pRawDCV);
      delete mpLDCP;
    }
    
    bool RawDVBuffer3::frame_pending()
    {
      return mpLDCP->pRawDCV->frame_pending();
    }
    
    VideoFrame<byte>* RawDVBuffer3::get_frame()
    {
      return mpLDCP->pRawDCV->get_frame();
    }

    void RawDVBuffer3::put_frame(VideoFrame<byte> *pVidFrame)
    {
      mpLDCP->pRawDCV->put_frame(pVidFrame);
    }
    
    unsigned int  RawDVBuffer3::get_feature_value(DV3Feature nFeature)
    {
      return mpLDCP->pRawDCV->get_feature_value(DC_from_DV3_Feature(nFeature));
    }
    
    void RawDVBuffer3::set_feature_value(DV3Feature nFeature, unsigned int nValue)
    {
      mpLDCP->pRawDCV->set_feature_value(DC_from_DV3_Feature(nFeature), nValue);
    }
    
    std::pair<unsigned int,unsigned int> RawDVBuffer3::get_feature_min_max(DV3Feature nFeature)
    {
      return mpLDCP->pRawDCV->get_feature_min_max(DC_from_DV3_Feature(nFeature));
    }
    
    void RawDVBuffer3::auto_on_off(DV3Feature nFeature, bool bValue)
    {
      mpLDCP->pRawDCV->auto_on_off(DC_from_DV3_Feature(nFeature), bValue);
    }
   
    void RawDVBuffer3::power_on_off(DV3Feature nFeature, bool bValue)
    {
        // not implemented
        std::cout << "! Warning: DVBuffer3/libdc1394v1 power_on_off is only implemented for V2!" << std::endl;
    }
  }
  
}


