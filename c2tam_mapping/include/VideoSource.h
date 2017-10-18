#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>

struct VideoSourceData;

class VideoSource
{
 public:
 #ifdef _SEQUENCE_
  VideoSource(std::string path, double fps = 25.0 );
 #else
  VideoSource();
 #endif
  void GetAndFillFrameBWandRGB(CVD::Image<CVD::byte> &imBW, CVD::Image<CVD::Rgb<CVD::byte> > &imRGB);
  CVD::ImageRef Size();



 private:
  void *mptr;
  CVD::ImageRef mirSize;
};
