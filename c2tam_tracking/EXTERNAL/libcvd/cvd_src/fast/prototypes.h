#include <cvd/fast_corner.h>
#include <vector>
namespace CVD
{
	using namespace std;
	void fast_corner_detect_plain_7(const SubImage<byte>& i, std::vector<ImageRef>& corners, int b);
	void fast_corner_detect_plain_8(const SubImage<byte>& i, std::vector<ImageRef>& corners, int b);
	void fast_corner_detect_plain_9(const SubImage<byte>& i, std::vector<ImageRef>& corners, int b);
    void fast_corner_detect_plain_10(const SubImage<byte>& i, std::vector<ImageRef>& corners, int b);
    void fast_corner_detect_plain_11(const SubImage<byte>& i, std::vector<ImageRef>& corners, int b);
    void fast_corner_detect_plain_12(const SubImage<byte>& i, std::vector<ImageRef>& corners, int b);
}

