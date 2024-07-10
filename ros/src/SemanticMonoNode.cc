#include "SemanticMonoNode.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SemanticMono");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport (node_handle);

    SemanticMonoNode node(ORB_SLAM2::System::SemanticMONOCULAR, node_handle, image_transport);

    node.Init();

    ros::spin();

    ros::shutdown();

    return 0;
}


SemanticMonoNode::SemanticMonoNode (ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : Node (sensor, node_handle, image_transport) {
  // image_subscriber = image_transport.subscribe ("/tesse/left_cam/mono/image_raw", 1, &SemanticMonoNode::ImageCallback, this);
  camera_info_topic_ = "/camera/camera_info";
  // semantic_subscriber = image_transport.subscribe("/tesse/seg_cam/rgb/image_raw", 1, &SemanticMonoNode::SemanticCallback, this);
  left_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/tesse/left_cam/mono/image_raw", 1);
  semantic_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/tesse/seg_cam/rgb/image_raw", 1);

  sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *left_sub_, *semantic_sub_);
  sync_->registerCallback(boost::bind(&SemanticMonoNode::ImageCallback, this, _1, _2));
}


SemanticMonoNode::~SemanticMonoNode () {
  delete left_sub_;
  delete semantic_sub_;
  delete sync_;
}


// void SemanticMonoNode::ImageCallback (const sensor_msgs::ImageConstPtr& msg) {
//   cv_bridge::CvImageConstPtr cv_in_ptr;
//   try {
//       cv_in_ptr = cv_bridge::toCvShare(msg);
//   } catch (cv_bridge::Exception& e) {
//       ROS_ERROR("cv_bridge exception: %s", e.what());
//       return;
//   }

//   current_frame_time_ = msg->header.stamp;

//   orb_slam_->TrackMonocular(cv_in_ptr->image,cv_in_ptr->header.stamp.toSec());

//   Update ();
// }

void SemanticMonoNode::ImageCallback(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& left_semantic_image) {
  cv_bridge::CvImageConstPtr cv_ptrLeft;
  try {
      cv_ptrLeft = cv_bridge::toCvShare(left_image);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrSemanticLeft;
  try {
      cv_ptrSemanticLeft = cv_bridge::toCvShare(left_semantic_image);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = left_image->header.stamp;

  // orb_slam_->TrackMonocular(cv_ptrLeft->image,cv_ptrLeft->header.stamp.toSec());
  orb_slam_->TrackSemanticMonocular(cv_ptrLeft->image, cv_ptrSemanticLeft->image, cv_ptrLeft->header.stamp.toSec());
  // -> System

  Update ();
}