#include <ros/ros.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <iostream>
#include <std_msgs/Int64MultiArray.h>
#include <ros_3d_bb/Detection2DMeta.h>
using namespace std;


class Yolov7Converter
{
public:
  Yolov7Converter()
  {
    std::printf("TEST2");

    // Initialize ROS node, publishers/subscribers
    nh_ = ros::NodeHandle("~");
    sub_detection_ = nh_.subscribe("/yolov7/yolov7", 10, &Yolov7Converter::detectionCallback, this);
    pub_converted_data_ = nh_.advertise<ros_3d_bb::Detection2DMeta>("converted2Ddetection", 1);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_converted_data_; 
  ros::Subscriber sub_detection_;

  
 
  void detectionCallback(const vision_msgs::Detection2DArray::ConstPtr& detection_msg)
  {

    ros_3d_bb::Detection2DMeta data;

    // Iterate through detections in the message
    for (const auto& detection : detection_msg->detections)
    {

     
     
      //clear array
      // array.data.clear();
      double x_min = detection.bbox.center.x - detection.bbox.size_x / 2;
      double y_min = detection.bbox.center.y - detection.bbox.size_y / 2;
      double x_max = detection.bbox.center.x + detection.bbox.size_x / 2;
      double y_max = detection.bbox.center.y + detection.bbox.size_y / 2;
      // cout << "x_min = " << x_min << ", y_min = " << y_min << ", x_max = " << x_max << ", y_max = " << y_max << "\n";
      // cout << "detection.bbox.size_x " << detection.bbox.size_x << " detection.bbox.size_y " << detection.bbox.size_y << "\n";


        //This keeps the bounding boxes within the bounds of the camera. Otherwise large (usually false) detections cause in index error in ros_3d_bb
        if (x_max >= 640) {
          x_max = 639;
        }
        if (y_max >= 480) {
          y_max = 479;
        }

      data.data.push_back(x_min);
      data.data.push_back(y_min);
      data.data.push_back(x_max);
      data.data.push_back(y_max);
      // ROS_INFO("ADDED X Y MIN MAX");
      // pub_bboxArray.publish(array);
      data.classes.push_back(detection.results[0].id);
      data.probabilities.push_back(int(detection.results[0].score * 100));

  
    }
    pub_converted_data_.publish(data);
  }

  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yolov7_converter");
  Yolov7Converter node;
  
  ros::spin();
}

