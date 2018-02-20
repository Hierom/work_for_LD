#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
//using namespace std;

namespace lane_detect_algo
{
    
    
    typedef std::vector<cv::Vec2i> vec2i_t;
    typedef std::vector<cv::Vec4i> vec4i_t;
    typedef std::vector<cv::Mat> vec_mat_t;
    typedef std::vector<cv::Point> vec_p_t;
    
    class CalLane {
            public:
            struct sPoint{
                double x,y;
            };
            struct sLine{
                double mx, my;
                double sx, sy;
            };
            void printVec4i(vec4i_t print_vec_4i);
            void printVec2i(vec2i_t print_vec_2i);
            void printMat(cv::Mat& print_mat);
            void sortLaneVec(vec_p_t &lane_p_vec);
            void storePointVec(cv::Point &lane_point, vec_p_t &lane_p_vec);
            void storeMatVec(cv::Mat &lane_mat, vec_mat_t &lane_m_vec);
            void scoreBoard(cv::Mat &b_score_board, cv::Mat b_img);
            cv::Mat mulMat(cv::Mat mul_mat);
            cv::Mat mulMatVec(vec_mat_t mul_mat_vec);
            cv::Mat cannyToBImg(cv::Mat canny_img);
            cv::Mat outputRANSAC(cv::Mat score_board);
            //////********http://blog.daum.net/pg365/62*****//////
            /////*********RANSAC algorithm 출처 *************//////
            void outputPCA(cv::Mat lane_point_mat, sLine &model);
            bool find_in_samples(cv::Mat sample, cv::Mat data);
            //////********http://blog.daum.net/pg365/62*****//////
            /////*********RANSAC algorithm 출처 *************//////
    };
    class DetectLane {
        public:
            DetectLane();
            ~DetectLane();
            vec2i_t laneLinearEquation(int x_0, int y_0, int x_1, int y_1);
            vec4i_t houghTransform(cv::Mat& b_img);
            cv::Mat myCanny(cv::Mat& raw_img);
            void storeFrameForVideoFile(cv::Mat& frame);
            void storeFrameForWebCam(cv::Mat& frame);
            void checkFrame(vec_mat_t frame_vec);
            cv::Mat makeLanePoint(cv::Mat lane_img);
            void drawLane(vec2i_t coordi_vec);
    }; 
    
}
