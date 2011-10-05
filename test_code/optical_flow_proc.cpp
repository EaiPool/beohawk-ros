#include <ros/ros.h>
#include <opencv2/video/video.hpp>

using namespace std;

class OpticalFlowProc
{
  // Parameters.
  static int max_corners = 100;
  static double quality_level = 0.01;
  static double min_distance = 10;
  static int width = 640, height = 480;
  /// Everything else.
  ros::Nodehandle nh;
  ros::Subscriber sub_image;

  cv::Mat image_old, image_new;
  vector<cv::Point> features_old, features_new,
  vector<char> status, err;

  int counter;

  public OpticalFlowProc (ros::NodeHandle& _nh): nh(_nh), counter(0)
  {
    cv::namedWindow("haha");
    sub_image = nh.subscribe("/usb_cam/image_raw", 10,
                             &OpticalFlowProc::cb_sub_image, this);
  }

  void cb_sub_iamge (const sensor_msgs::Image::ConstPtr msg)
  {
    cv::Mat image_temp(height, width, CV_8UC3, (void*)&msg->data[0]);
    cv::cvtColor(image_temp, image_new, CV_BRG2GRAY); // really BGR?

    if (counter == 0) {} else
    {
      cv::goodFeaturesToTrack(image_old, features_old, max_corners
                              quality_level, min_distance);
      cv::calcOpticalFlowPyrLK(img_old, img_new, features_old,
                               features_new, status, err);
    }

    cv::Mat image_output(height, width, CV_8UC1, cv::Scalar(0));

    for (int i = 0; i < status.size(); i++)
      if (status[i] == 1)
        cv::line(image_output, feature_old[i], feature_new[i],
                 cv::Scalar(255));
            
    cv::imshow("haha", image_output);
    cv::waitKey(2);

    image_old = image_new;
    ++ counter;
  }
  
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "art_vision_node");
	ros::NodeHandle nh;

  OpticalFlowProc ofp(nh);
  ros::spin();
  return 0;
}


