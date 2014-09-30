#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/tools.h>
#include <interactive_markers/menu_handler.h>

#include <geometry_msgs/PoseStamped.h>

class InteractiveMarkerCreater {
public:
InteractiveMarkerCreater() {
  ros::NodeHandle nh, pnh("~");
  //map<string, double> initial_pose_map_;
  pose_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("pose", 1);
  create_sub_ = pnh.subscribe<visualization_msgs::Marker>("create", 1, &InteractiveMarkerCreater::createCB, this);
  server_.reset( new interactive_markers::InteractiveMarkerServer(ros::this_node::getName()));
}
  virtual ~InteractiveMarkerCreater() {};
  
protected:
  void moveMarkerCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    pose_pub_.publish(msg);
    server_->setPose("marker", msg->pose, msg->header);
    server_->applyChanges();
  }
  
  void createCB(const visualization_msgs::Marker::ConstPtr& marker) {
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = marker->header.frame_id;
    int_marker.name = marker->id;
    int_marker.pose.orientation.w = 1.0;
    
    /*
    visualization_msgs::Marker sphere_marker;
    sphere_marker.type = visualization_msgs::Marker::SPHERE;
    sphere_marker.scale.x = 0.1;
    sphere_marker.scale.y = 0.1;
    sphere_marker.scale.z = 0.1;
    sphere_marker.color.r = 0.0;
    sphere_marker.color.g = 1.0;
    sphere_marker.color.b = 0.0;
    sphere_marker.color.a = 1.0;
    sphere_marker.pose.orientation.w = 1.0;
    
    visualization_msgs::InteractiveMarkerControl sphere_marker_control;
    sphere_marker_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    sphere_marker_control.always_visible = true;
    sphere_marker_control.markers.push_back(sphere_marker);
    int_marker.controls.push_back(sphere_marker_control);
    */
  
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(*marker);
  
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  
    server_->insert(int_marker,
                    boost::bind(&InteractiveMarkerCreater::processFeedbackCB, this, _1));
    server_->applyChanges();
  }
  
  void processFeedbackCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    geometry_msgs::PoseStamped pose;
    pose.header = feedback->header;
    pose.pose = feedback->pose;
    pose_pub_.publish(pose);
  }
  
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  ros::Subscriber create_sub_;
  ros::Publisher pose_pub_;
  std::string frame_id_;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "interactive_marker_creater");
  InteractiveMarkerCreater marker;
  ros::spin();
  return 0;
}
