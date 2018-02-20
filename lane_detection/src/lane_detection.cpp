//warpAffine 
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <preprocessing/preLaneDetect.hpp>
//#include <lanedetection/coordinateMsg.h>
//#include <lanedetection/vecMsg.h>
//#include <lanedetection/dataSize.h>
static const std::string OPENCV_WINDOW_VF = "Image by videofile";
static const std::string OPENCV_WINDOW_WC = "Image by webcam";
static const bool DEBUG_SW = true;
using namespace lane_detect_algo;

//for WebCam
//for VideoFile
class InitImgObjectforROS{
    public:
        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        image_transport::Subscriber sub_img;
        
        InitImgObjectforROS():it(nh){
            if(DEBUG_SW){//'DEBUG_SW == TURE' means subscribing videofile image
                sub_img = it.subscribe("/videofile/image_raw",1,&InitImgObjectforROS::imgCb,this);
            //this topic is publish by s'video_stream_opencv' package please run video_file.launch of 
                cv::namedWindow(OPENCV_WINDOW_VF);
            }
            else{//'DEBUG_SW == FALE' means subscribing webcam image
                sub_img = it.subscribe("/cam0/raw_image",1,&InitImgObjectforROS::imgCb,this);
                cv::namedWindow(OPENCV_WINDOW_WC);
            }
        }
        ~InitImgObjectforROS(){
            if(DEBUG_SW){//'DEBUG_SW == TURE' means subscribing videofile image
                cv::destroyWindow(OPENCV_WINDOW_VF);
            }
            else{//'DEBUG_SW == FALE' means subscribing webcam image
                cv::destroyWindow(OPENCV_WINDOW_WC);
            }
        }
        void imgCb(const sensor_msgs::ImageConstPtr& img_msg){
            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::BGR8);
            }
            catch(cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception : %s", e.what());
            }
        }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "lane_detection");
    InitImgObjectforROS img_obj;
    ros::Rate loop_rate(5);
    while(img_obj.nh.ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("program killed!\n");
    return 0;
}