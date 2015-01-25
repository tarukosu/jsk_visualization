#include <ros/ros.h>

#include <std_msgs/Empty.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

class InteractiveMarkerManager {
 public:
  InteractiveMarkerManager();
  void publishFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void addMarkerCallback( const visualization_msgs::InteractiveMarkerConstPtr &feedback );
  void clearMarkerCallback( const std_msgs::EmptyConstPtr &empty_ptr );
 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  ros::Subscriber sub_interactive_marker_;
  ros::Subscriber sub_clear_marker_;
  ros::Publisher pub_feedback_;

};
