#include <ros/ros.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <iostream>
#include <std_msgs/Int32MultiArray.h>
using namespace std;


class Yolov7Converter
{
public:
  Yolov7Converter()
  {
    std::printf("TEST2");

    // Initialize ROS node, publishers/subscribers
    nh_ = ros::NodeHandle("~");
    sub_detection_ = nh_.subscribe("/choose_object/Filtered_detection", 10, &Yolov7Converter::detectionCallback, this);
    pub_bboxArray = nh_.advertise<std_msgs::Int32MultiArray>("bboxArray", 1);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_bboxArray; 
  ros::Subscriber sub_detection_;

  
 
  void detectionCallback(const vision_msgs::Detection2DArray::ConstPtr& detection_msg)
  {

    // Iterate through detections in the message
    for (const auto& detection : detection_msg->detections)
    {

      std_msgs::Int32MultiArray array;

      //clear array
      array.data.clear();

      double x_min = detection.bbox.center.x - detection.bbox.size_x / 2;
      double y_min = detection.bbox.center.y - detection.bbox.size_y / 2;
      double x_max = detection.bbox.center.x + detection.bbox.size_x / 2;
      double y_max = detection.bbox.center.y + detection.bbox.size_y / 2;
    //   cout << "x_min = " << x_min << ", y_min = " << y_min << ", x_max = " << x_max << ", y_max = " << y_max << "\n";
      
      array.data.push_back(x_min);
      array.data.push_back(y_min);
      array.data.push_back(x_max);
      array.data.push_back(y_max);
      ROS_INFO("ADDED X Y MIN MAX");
      pub_bboxArray.publish(array);

      
  
    }
  }

  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yolov7_converter");
  Yolov7Converter node;
  
  ros::spin();
}

