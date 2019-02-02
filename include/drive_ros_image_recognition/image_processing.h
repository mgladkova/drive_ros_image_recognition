#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <opencv2/opencv.hpp>
#include <drive_ros_msgs/DrivingLine.h>
#include <drive_ros_image_recognition/common_image_operations.h>

namespace drive_ros_image_recognition {

class ImageProcessing
{
public:
  ImageProcessing(ros::NodeHandle nh, ros::NodeHandle pnh, bool nodelet=false);
  bool init();
  bool detectLaneLines(cv::Mat image, cv::Vec4i& l1, cv::Vec4i& l2);
  void extractRelevantRegion(cv::Mat img_in, cv::Mat& mask, cv::Mat& img_out, const cv::Vec4i l1, const cv::Vec4i l2);
  bool detectStartLine(cv::Mat img_out, std::vector<cv::Point2f> corners, cv::Vec4i l1, cv::Vec4i l2);
  bool findStartLineRANSAC(std::vector<cv::Point2f> corners,
                           cv::Vec4i l1, cv::Vec4i l2,
                           cv::Vec4i& startline, float& error);
private:
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  void laneCallback(const drive_ros_msgs::DrivingLinePtr &msg);
  void homograpImageCallback(const sensor_msgs::ImageConstPtr &msg);
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ImageOperator image_operator_;
  image_transport::ImageTransport imageTransport_;
  image_transport::Subscriber img_sub_;
  image_transport::Subscriber homImg_sub_;
  ros::Subscriber lane_sub_;
  bool nodelet_;
};

class ImageProcessingNodelet : public nodelet::Nodelet {
public:
    virtual void onInit();
private:
    std::unique_ptr<ImageProcessing> img_proc_;
};

} // namespace drive_ros_image_recognition

#endif //IMAGE_PROCESSING_H
