#include <jsk_interactive_marker/interactive_marker_manager.h>
using namespace std;

void InteractiveMarkerManager::publishFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  std::cout <<"feedback" << std::endl;
  //pub_feedback_.publish(*feedback);
}

void InteractiveMarkerManager::addMarkerCallback( const visualization_msgs::InteractiveMarkerConstPtr &im_ptr ){
  std::cout <<"aa" << std::endl;
  server_->insert(*im_ptr, boost::bind( &InteractiveMarkerManager::publishFeedback, this, _1));
  server_->applyChanges();
}

void InteractiveMarkerManager::clearMarkerCallback( const std_msgs::EmptyConstPtr &empty_ptr ){
  server_->clear();
}


InteractiveMarkerManager::InteractiveMarkerManager() : nh_(), pnh_("~"){
  //server_ = 
  string server_name;
  pnh_.param("server_name", server_name, std::string ("") );
  if ( server_name == "" ) {
    server_name = ros::this_node::getName();
  }
  server_.reset( new interactive_markers::InteractiveMarkerServer(server_name, "", false) );
  sub_interactive_marker_ = pnh_.subscribe<visualization_msgs::InteractiveMarker> ("add_marker", 100, boost::bind( &InteractiveMarkerManager::addMarkerCallback, this, _1));
  sub_clear_marker_ = pnh_.subscribe<std_msgs::Empty> ("clear_marker", 100, boost::bind( &InteractiveMarkerManager::clearMarkerCallback, this, _1));
  pub_feedback_ = pnh_.advertise<visualization_msgs::InteractiveMarkerFeedback> ("feedback", 100);


}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_marker_manager");
  //ros::NodeHandle n;
  //ros::NodeHandle pnh_("~");


  //InteractiveMarkerManager imm = new InteractiveMarkerManager();// imm;
  new InteractiveMarkerManager();// imm;
  ros::spin();
  return 0;
}
