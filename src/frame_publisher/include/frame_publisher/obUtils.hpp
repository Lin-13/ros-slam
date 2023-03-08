#include <libobsensor/ObSensor.hpp>
// #include "libobsensor/hpp/Pipeline.hpp"
// #include "libobsensor/hpp/Error.hpp"
#include <vector>
std::vector<cv::Mat> processFrames(std::vector<std::shared_ptr<ob::Frame>> frames) {
    std::vector<cv::Mat> mats;
    for(auto frame: frames) {
        if(frame == nullptr || frame->dataSize() < 1024) {
            continue;
            //return mats;
        }
        auto videoFrame = frame->as<ob::VideoFrame>();
        cv::Mat rstMat;

        if(videoFrame->type() == OB_FRAME_COLOR && videoFrame->format() == OB_FORMAT_MJPG) {
            cv::Mat rawMat(1, videoFrame->dataSize(), CV_8UC1, videoFrame->data());
            rstMat = cv::imdecode(rawMat, 1);
        }
        else if(videoFrame->type() == OB_FRAME_COLOR && videoFrame->format() == OB_FORMAT_NV21) {
            cv::Mat rawMat(videoFrame->height() * 3 / 2, videoFrame->width(), CV_8UC1, videoFrame->data());
            cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_NV21);
        }
        else if(videoFrame->type() == OB_FRAME_COLOR && (videoFrame->format() == OB_FORMAT_YUYV || videoFrame->format() == OB_FORMAT_YUY2)) {
            cv::Mat rawMat(videoFrame->height(), videoFrame->width(), CV_8UC2, videoFrame->data());
            cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_YUY2);
        }
        else if(videoFrame->type() == OB_FRAME_COLOR && videoFrame->format() == OB_FORMAT_RGB888) {
            cv::Mat rawMat(videoFrame->height(), videoFrame->width(), CV_8UC3, videoFrame->data());
            cv::cvtColor(rawMat, rstMat, cv::COLOR_RGB2BGR);
        }else if(videoFrame->type() == OB_FRAME_COLOR && videoFrame->format() == OB_FORMAT_UYVY) {
            cv::Mat rawMat(videoFrame->height(), videoFrame->width(), CV_8UC2, videoFrame->data());
            cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_UYVY);
        }
        else if(videoFrame->format() == OB_FORMAT_Y16 || videoFrame->format() == OB_FORMAT_YUYV || videoFrame->format() == OB_FORMAT_YUY2) {
            // IR or Depth Frame
            cv::Mat cvtMat;
            cv::Mat rawMat = cv::Mat(videoFrame->height(), videoFrame->width(), CV_16UC1, videoFrame->data());
            float   scale;
            if(videoFrame->type() == OB_FRAME_DEPTH) {
                scale = 1.0f / pow(2, videoFrame->pixelAvailableBitSize() - 10);
            }
            else {
                scale = 1.0f / pow(2, videoFrame->pixelAvailableBitSize() - 8);
            }
            cv::convertScaleAbs(rawMat, cvtMat, scale);
            cv::cvtColor(cvtMat, rstMat, cv::COLOR_GRAY2RGB);
        }
        else if(videoFrame->type() == OB_FRAME_IR && videoFrame->format() == OB_FORMAT_Y8) {
            cv::Mat rawMat = cv::Mat(videoFrame->height(), videoFrame->width(), CV_8UC1, videoFrame->data());

            cv::cvtColor(rawMat, rstMat, cv::COLOR_GRAY2RGB);
        }
        else if(videoFrame->type() == OB_FRAME_IR && videoFrame->format() == OB_FORMAT_MJPG) {
            cv::Mat rawMat(1, videoFrame->dataSize(), CV_8UC1, videoFrame->data());
            rstMat = cv::imdecode(rawMat, 1);
        }
        mats.push_back(rstMat);
    }
    return mats;
}
void StartObdevice(std::shared_ptr<ob::Pipeline> pipe,bool color_enable = true, bool depth_enable = false){
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

    try{
        if(color_enable){
            auto colorProfileList = pipe ->getStreamProfileList(OB_SENSOR_COLOR);

            std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
            try{
                colorProfile = colorProfileList->getVideoStreamProfile(640,0,OB_FORMAT_RGB888,30);
            }catch(ob::Error &e){
                colorProfile = colorProfileList->getVideoStreamProfile(640,0,OB_FORMAT_UNKNOWN,30);
            }
            config->enableStream(colorProfile);
        }
        if(depth_enable){
            auto depthProfileList = pipe ->getStreamProfileList(OB_SENSOR_DEPTH);
            std::shared_ptr<ob::VideoStreamProfile> depthProfile = nullptr;
            try{
                //根据指定的格式查找对应的Profile,优先查找Y16格式
                depthProfile = depthProfileList->getVideoStreamProfile(640,0,OB_FORMAT_Y16,30);
            }catch(ob::Error &e){
                //没找到Y16格式后不匹配格式查找对应的Profile进行开流
                depthProfile = depthProfileList->getVideoStreamProfile(640,0,OB_FORMAT_UNKNOWN,30);
            }
            config->enableStream(depthProfile);
        }
        
    }catch(ob::Error &e){
        std::cerr<<"Current device is not support color sensor!"<<std::endl;   
        exit(EXIT_FAILURE);
    }

    pipe -> start(config);
    //获取镜像属性是否有可写的权限
    if(pipe->getDevice()->isPropertySupported(OB_PROP_COLOR_MIRROR_BOOL, OB_PERMISSION_WRITE)) {
        //设置镜像
        pipe->getDevice()->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, true);
    }
}
inline bool readObdeviceColor(std::shared_ptr<ob::Pipeline> pipe,cv::Mat& img,uint time_out_ms = 10){
    auto frameSet = pipe -> waitForFrames(time_out_ms);
    if(frameSet == nullptr){
        return 0;
    }
    std::vector<cv::Mat> imgs;
    imgs = processFrames({frameSet->colorFrame()});
    if(imgs.size()>0){
        cv::Mat src = imgs[0];
        img = src.clone();
        return 1;
    }else{
        return 0;
    }
}
inline bool readObdeviceDepth(std::shared_ptr<ob::Pipeline> pipe,cv::Mat& img,uint time_out_ms = 10){
    auto frameSet = pipe -> waitForFrames(time_out_ms);
    if(frameSet == nullptr){
        return 0;
    }
    std::vector<cv::Mat> imgs;
    imgs = processFrames({frameSet->depthFrame()});
    if(imgs.size()>0){
        cv::Mat src = imgs[0];
        img = src.clone();
        return 1;
    }else{
        return 0;
    }
}
int ob_read_device(std::shared_ptr<bool> ob_enable_read, uint time_cap_ms = 30) try {
    ob::Pipeline pipe;
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

    int windowsWidth = 0;
    int windowsHeight = 0;
    try{
        auto profiles = pipe.getStreamProfileList(OB_SENSOR_COLOR);

        std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
        try{
            colorProfile = profiles->getVideoStreamProfile(640,0,OB_FORMAT_RGB888,30);
        }catch(ob::Error &e){
            colorProfile = profiles->getVideoStreamProfile(640,0,OB_FORMAT_UNKNOWN,30);
        }
        windowsWidth = colorProfile->width();
        windowsHeight = colorProfile->height();
        config->enableStream(colorProfile);
    }catch(ob::Error &e){
        std::cerr<<"Current device is not support color sensor!"<<std::endl;   
        exit(EXIT_FAILURE);
    }

    pipe.start(config);
    //获取镜像属性是否有可写的权限
    if(pipe.getDevice()->isPropertySupported(OB_PROP_COLOR_MIRROR_BOOL, OB_PERMISSION_WRITE)) {
        //设置镜像
        pipe.getDevice()->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, true);
    }

    while(*ob_enable_read) {
        auto frameSet = pipe.waitForFrames(100);
        if(frameSet == nullptr) {
            continue;
        }
    }
    pipe.stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}