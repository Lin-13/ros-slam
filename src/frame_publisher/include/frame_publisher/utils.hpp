#include<mutex>
#include <opencv2/core.hpp>
#include<string>
class ImgBuffer{
public:
    ImgBuffer():_empty(1){}
    void write(cv::Mat& dst){
        std::lock_guard<std::mutex> guard(_img_mutex);
        _img_single_buffer = dst.clone();
        _empty = false;
    }
    void read(cv::Mat& dst){
        std::lock_guard<std::mutex> guard(_img_mutex);
        dst = _img_single_buffer.clone();
        _empty = true;
    }
    bool isEmpty(){
        return _empty;
    }
private:
    std::mutex _img_mutex;
    cv::Mat _img_single_buffer;
    bool _empty;
};
int64_t  getCUrrentTimeStamp(){
    std::chrono::time_point<std::chrono::system_clock,std::chrono::milliseconds> tp =
            std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    auto tmp = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
    return tmp.count();
}
std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}