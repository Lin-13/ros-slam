#include<mutex>
#include <opencv2/core.hpp>
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