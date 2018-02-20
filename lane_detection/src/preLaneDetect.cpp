#include <preprocessing/preLaneDetect.hpp>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
using namespace std;//다하고 지워
using namespace cv;//다하고 지워
static const bool DEBUG_SW = true;

//http://blog.daum.net/pg365/62 --> RANSAC algorithm 참고.
namespace lane_detect_algo{
    
            
            void CalLane::printVec4i(vec4i_t print_vec_4i){
                for(auto &vec_4i : print_vec_4i){
                    if(DEBUG_SW) {//print vec_4i info when DEBUG_SW is true.
                        std::cout<<vec_4i<<std::endl;
                    }
                }   
            }
            void CalLane::printVec2i(vec2i_t print_vec_2i){
                for(auto &vec_2i : print_vec_2i){
                    if(DEBUG_SW) {//print vec_2i info when DEBUG_SW is true.
                        std::cout<<vec_2i<<std::endl;
                    }
                }
            }
            void CalLane::printMat(cv::Mat &print_mat){
                if(DEBUG_SW){//print mat info when DEBUG_SW is true.
                    std::cout<<print_mat<<std::endl;
                }
            }

            void CalLane::sortLaneVec(vec_p_t &lane_p_vec){
                auto temp_y = lane_p_vec.front().y;
                for(int i =0; i<lane_p_vec.size(); ++i){//데이터 정렬되어있을 가능성이 높으므로 삽입정렬
                    for(int j=i+1; j<lane_p_vec.size(); ++j){
                        if(lane_p_vec[j].y > lane_p_vec[i].y){
                            temp_y = lane_p_vec[i].y;
                            lane_p_vec[i].y = lane_p_vec[j].y;
                            lane_p_vec[j].y = temp_y;
                        }
                    }
                }
                
            }

            void CalLane::storePointVec(cv::Point &lane_point, vec_p_t &lane_p_vec){
                
     //**********//   vec2i_t a;
     //**********//   std::vector<vec2i_t> tt;
     //**********//   tt.push_back(a);//point말고 벡터를 넣은 벡터..이렇게하는거가 더 좋을지 생각좀...
                try{
                    lane_p_vec.push_back(lane_point);
                }
                catch(cv::Exception e){
                    ROS_INFO("%s\n storeLaneVec is fail!\n",e.what());
                }
               
            }

            void storeMatVec(cv::Mat &lane_mat, vec_mat_t &lane_m_vec){
                try{
                    lane_m_vec.push_back(lane_mat);
                }
                catch(cv::Exception e){
                    ROS_INFO("%s\n storeMatVec is fail!\n",e.what());
                }
            }

            void CalLane::scoreBoard(cv::Mat &b_score_board, cv::Mat b_img){
                if(b_score_board.size() == b_img.size()){
                    
                }
                else{//b_sroce_bore.size() != b_img
                    std::cout<<"the score_board and the b_img do not match the size!"<<std::endl;
                }
                
            }

            cv::Mat CalLane::mulMat(cv::Mat mul_mat){
                return mul_mat;
            }
            cv::Mat CalLane::mulMatVec(vec_mat_t mul_mat_vec){
                cv::Mat mul_mat;
                return mul_mat;
            }
            cv::Mat CalLane::cannyToBImg(cv::Mat canny_img){
                auto it = canny_img.begin<int>();
                auto itend = canny_img.end<int>();
                if(canny_img.channels()==1){
                   for(;it!=itend; ++it){
                        if(*it > 0){
                            *it = 1;
                        }
                    }
                }else{//canny_img.channels() != 1
                    std::cout<<"please input grayImg for cannyToBImg"<<std::endl;
                }
                return canny_img;
            }
            
            cv::Mat CalLane::outputRANSAC(cv::Mat score_board){
                return score_board;
            }
            void outputPCA(cv::Mat lane_point_mat, CalLane::sLine &model){
                int i=0, j=0;
                double sum_Xi=0, sum_Yi=0, sum_Xi_expo=0, sum_Yi_expo=0, sum_XixYi=0, N=0;
                double mean_x, mean_y, s_dev, cov_xx, cov_yy, cov_xy;
                auto it = lane_point_mat.begin<int>();
                auto itend = lane_point_mat.end<int>();
                for(; i<lane_point_mat.rows; ++i){
                    for(; j<lane_point_mat.cols; ++j,++it){
                        if(*it == 1){
                            sum_Xi += i;
                            sum_Yi += j;
                            sum_Xi_expo += i*i;
                            sum_Yi_expo += j*j;
                            sum_XixYi = i*j;
                            ++N;
                            *it = 2;
                        }
                       // if(*it == itend){//itend+1이랑 같아지면 끝내야하나?.. 이조건 없어도 되나?
                         //   break;
                        }
                    }
                cov_xx = (sum_Xi_expo - sum_Xi*sum_Xi/N)/N;
                cov_xy = (sum_XixYi - sum_Xi*sum_Yi/N)/N;
                cov_yy = (sum_Yi_expo - sum_Yi*sum_Yi/N)/N;
                
                double theta = std::atan2(2*cov_xy, cov_xx-cov_yy)/2;
                model.mx = std::cos(theta);
                model.my = std::sin(theta);
                model.sx = sum_Xi/N;
                model.sy = sum_Yi/N;
                /***///cov_xx == cos(theta)^2, cov_yy = sin(theta)^2
                /***///cov_xx - cov_yy == cos(theta)^2 - sin(theta)^2
                /***/// == cos(2*theta)
                /***///2*cov_xy == 2*cos(theta)*sin(theta)
                /***/// == sin(2*theta)
                /***///atan2(y,x)--> atan2(sin(2theta),cos(2*theta)) 로 적용시키기 위해 위와 같이 쓴것임.
                /***///2로 나누는 이유는 2*cov_xy를 넣어줬으므로..
                /***///******************************//
                /***///** Xm == sigma(Xi)/N ** (vice versa for Y)
                /***///** Xm is means of Xi **
                /***///N*s_dev_x(standard devision for X)^2 == sigma((Xi - Xm)^2) 
                /***///== sigma(Xi^2) - 2*Xm(sigma(Xi)) + sigma(Xm^2)
                /***///== sigma(Xi^2) - 2*Xm*Xm*N + N*Xm*Xm
                /***///== sigma(Xi^2) - N*(Xm^2)
                /***///******************************//
                /***///N*cov(this likes s_dev_xy)^2 == sigma((Xi-Xm)(Yi-Ym))
                /***///== sigma(Xi*Yi) - sigma(Xm*Yi) - sigma(Xi*Ym) + sigma(Xm*Ym)
                /***///== sigma(Xi*Yi) - Xm*sigma(Yi) - Ym*sigma(Xi) + N*Xm*Ym
                /***///== sigma(Xi*Yi) - Xm*N*Ym - Ym*N*Xm + N*Xm*Ym
                /***///== sigma(Xi*Yi) - N*Xm*Ym
                /***///== sigma(Xi*Yi) - N*(sgima(Xi)/N)*(sigma(Yi)/N)
                /***///== sigma(Xi*Yi) - sigma(Xi)*sigma(Yi)/N
                
            }
   
            DetectLane::DetectLane(){

             }
            DetectLane::~DetectLane(){

            }
            vec2i_t DetectLane::laneLinearEquation(int x_0, int y_0, int x_1, int y_1){
                vec2i_t lane_coordi;
                return lane_coordi;
            }
            vec4i_t DetectLane::houghTransform(cv::Mat& b_img){
                vec4i_t hough_vec;
                return hough_vec;
            }
            cv::Mat DetectLane::myCanny(cv::Mat& raw_img){  
                return raw_img;
            }
            void DetectLane::storeFrameForVideoFile(cv::Mat& frame){
                
            }
            void DetectLane::storeFrameForWebCam(cv::Mat& frame){

            }
            void DetectLane::checkFrame(vec_mat_t frame_vec){

            }
            cv::Mat DetectLane::makeLanePoint(cv::Mat lane_img){
                return lane_img;
            }
            void DetectLane::drawLane(vec2i_t coordi_vec){

            }
    
}

/*
//포인터 o
std::vector<int*> a;
int* pA = NULL;
a.reserve(3);//예약 걸어둘수있는건 다 걸어두자
pA= new int;
a.push_back(pA);
pA=new int;
a.push_back(pA);
이런 데이터일 경우

std::vector<int*>::iterator iter;
for(iter = a.begin(); iter!=a.end(); ++iter)
{
    delete (*iter);
}a.clear();

//포인터 x
std::vector vclear;
a.swap(vclear);

vclear.clear();
a.clear();
*/