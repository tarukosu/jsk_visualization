#include <jsk_interactive_marker/transformable_interactive_server.h>

using namespace jsk_interactive_marker;

TransformableInteractiveServer::TransformableInteractiveServer():n_(new ros::NodeHandle("~")){
  n_->param("display_interactive_manipulator", display_interactive_manipulator_, true);
  n_->param("torus_udiv", torus_udiv_, 20);
  n_->param("torus_vdiv", torus_vdiv_, 20);
  tf_listener_.reset(new tf::TransformListener);
  setpose_sub_ = n_->subscribe("set_pose", 1, &TransformableInteractiveServer::setPose, this);
  setcolor_sub_ = n_->subscribe("set_color", 1, &TransformableInteractiveServer::setColor, this);

  set_r_sub_ = n_->subscribe("set_radius", 1, &TransformableInteractiveServer::setRadius, this);
  set_sm_r_sub_ = n_->subscribe("set_small_radius", 1, &TransformableInteractiveServer::setSmallRadius, this);
  set_x_sub_ = n_->subscribe("set_x", 1, &TransformableInteractiveServer::setX, this);
  set_y_sub_ = n_->subscribe("set_y", 1, &TransformableInteractiveServer::setY, this);
  set_z_sub_ = n_->subscribe("set_z", 1, &TransformableInteractiveServer::setZ, this);

  addpose_sub_ = n_->subscribe("add_pose", 1, &TransformableInteractiveServer::addPose, this);
  addpose_relative_sub_ = n_->subscribe("add_pose_relative", 1, &TransformableInteractiveServer::addPoseRelative, this);

  setrad_sub_ = n_->subscribe("set_radius", 1, &TransformableInteractiveServer::setRadius, this);

  focus_text_pub_ = n_->advertise<jsk_rviz_plugins::OverlayText>("focus_marker_name", 1);
  focus_pose_pub_ = n_->advertise<jsk_rviz_plugins::OverlayText>("focus_marker_pose", 1);

  get_pose_srv_ = n_->advertiseService("get_pose", &TransformableInteractiveServer::getPoseService, this);
  set_pose_srv_ = n_->advertiseService("set_pose", &TransformableInteractiveServer::setPoseService, this);
  get_color_srv_ = n_->advertiseService("get_color", &TransformableInteractiveServer::getColorService, this);
  set_color_srv_ = n_->advertiseService("set_color", &TransformableInteractiveServer::setColorService, this);
  get_focus_srv_ = n_->advertiseService("get_focus", &TransformableInteractiveServer::getFocusService, this);
  set_focus_srv_ = n_->advertiseService("set_focus", &TransformableInteractiveServer::setFocusService, this);
  get_type_srv_ = n_->advertiseService("get_type", &TransformableInteractiveServer::getTypeService, this);
  get_exist_srv_ = n_->advertiseService("get_existence", &TransformableInteractiveServer::getExistenceService, this);
  set_dimensions_srv =  n_->advertiseService("set_dimensions", &TransformableInteractiveServer::setDimensionsService, this);
  get_dimensions_srv =  n_->advertiseService("get_dimensions", &TransformableInteractiveServer::getDimensionsService, this);
  marker_dimensions_pub_ = n_->advertise<jsk_interactive_marker::MarkerDimensions>("marker_dimensions", 1);
  request_marker_operate_srv_ = n_->advertiseService("request_marker_operate", &TransformableInteractiveServer::requestMarkerOperateService, this);

  config_srv_ = boost::make_shared <dynamic_reconfigure::Server<InteractiveSettingConfig> > (*n_);
  dynamic_reconfigure::Server<InteractiveSettingConfig>::CallbackType f =
    boost::bind (&TransformableInteractiveServer::configCallback, this, _1, _2);
  config_srv_->setCallback (f);

  tf_timer = n_->createTimer(ros::Duration(0.05), &TransformableInteractiveServer::tfTimerCallback, this);

  server_ = new interactive_markers::InteractiveMarkerServer("simple_marker");
}

TransformableInteractiveServer::~TransformableInteractiveServer()
{
  for (std::map<string, TransformableObject* >::iterator itpairstri = transformable_objects_map_.begin(); itpairstri != transformable_objects_map_.end(); itpairstri++) {
    delete itpairstri->second;
  }
  transformable_objects_map_.clear();
}

void TransformableInteractiveServer::configCallback(InteractiveSettingConfig &config, uint32_t level)
{
    boost::mutex::scoped_lock lock(mutex_);
    display_interactive_manipulator_ = config.display_interactive_manipulator;
    for (std::map<string, TransformableObject* >::iterator itpairstri = transformable_objects_map_.begin(); itpairstri != transformable_objects_map_.end(); itpairstri++) {
      TransformableObject* tobject = itpairstri->second;
      tobject->setInteractiveMarkerSetting(config);
      updateTransformableObject(tobject);
    }
  }


void TransformableInteractiveServer::processFeedback(
                                                     const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  switch ( feedback->event_type )
    {
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      focus_object_marker_name_ = feedback->marker_name;
      focusTextPublish();
      focusPosePublish();
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      TransformableObject* tobject = transformable_objects_map_[feedback->marker_name.c_str()];
      if(tobject){
        geometry_msgs::PoseStamped input_pose_stamped;
        input_pose_stamped.header = feedback->header;
        input_pose_stamped.pose = feedback->pose;
        setPoseWithTfTransformation(tobject, input_pose_stamped);
      }else
        ROS_ERROR("Invalid ObjectId Request Received %s", feedback->marker_name.c_str());
      break;
    }
}

void TransformableInteractiveServer::assocMenuFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback , std::string name){
  ROS_ERROR("assoc %s to %s", feedback->marker_name.c_str(), name.c_str());

  TransformableObject* referenceObject  = transformable_objects_map_[name.c_str()];
  unsigned int slash_pos = name.find_last_of("/");
  string reference_object_root_name = name.erase(slash_pos) + "/root";
  string reference_object_control_name = name.erase(slash_pos) + "/control";
  TransformableObject* referenceObjectRoot = transformable_objects_map_[reference_object_root_name.c_str()];
  TransformableObject* referenceObjectControl = transformable_objects_map_[reference_object_control_name.c_str()];

  string target_object_name = feedback->marker_name;
  TransformableObject* targetObject  = transformable_objects_map_[target_object_name.c_str()];
  slash_pos = target_object_name.find_last_of("/");
  string target_object_root_name = target_object_name.erase(slash_pos) + "/root";
  string target_object_control_name = target_object_name.erase(slash_pos) + "/control";

  TransformableObject* targetObjectRoot = transformable_objects_map_[target_object_root_name.c_str()];
  TransformableObject* targetObjectControl = transformable_objects_map_[target_object_control_name.c_str()];


  float target_x, target_y, target_z, target_r, target_sr;
  float reference_x, reference_y, reference_z, reference_r, reference_sr;

  switch(referenceObject->getType()){
  case AXIS:
    {
      referenceObject->getRZ(reference_r, reference_z);
    }
  }

  switch(targetObject->getType()){
  case AXIS:
    {
      targetObject->getRZ(target_r, target_z);
    }
  }



  switch(targetObject->getType()){
  case AXIS:
    switch(referenceObject->getType()){
    case AXIS:
      {
	geometry_msgs::Pose reference_pose = referenceObject->getPose();
	Eigen::Affine3d eigen_pose;
	tf::poseMsgToEigen(reference_pose, eigen_pose);
	eigen_pose = eigen_pose * Eigen::Translation3d(0.0, 0.0, (target_z + reference_z) / 2);
	tf::poseEigenToMsg(eigen_pose, reference_pose);

	geometry_msgs::PoseStamped reference_ps;
	reference_ps.header.frame_id = referenceObject->getFrameId();
	reference_ps.pose = reference_pose;
	//setPose(reference_ps, target_object_root_name);
	setPoseWithTfTransformation(targetObjectControl, reference_ps);
	//targetObjectRoot->enableControls(false, false, true, false, false, true);
	targetObjectControl->enableControls(false, false, true, false, false, true);
	updateTransformableObject(targetObjectControl);
	//targetObjectRoot->setPose(referenceObjectRoot->getPose());
	ROS_ERROR("AXIS, AXIS");
      }
    }
  }

  server_->applyChanges();
}


void TransformableInteractiveServer::setColor(std_msgs::ColorRGBA msg)
{
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  tobject->setRGBA(msg.r, msg.g, msg.b, msg.a);
  updateTransformableObject(tobject);
}

void TransformableInteractiveServer::setRadius(std_msgs::Float32 msg)
{
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  if(tobject->setRadius(msg)){
    updateTransformableObject(tobject);
    publishMarkerDimensions();
  }
}

void TransformableInteractiveServer::setSmallRadius(std_msgs::Float32 msg)
{
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  if(tobject->setSmallRadius(msg)){
    updateTransformableObject(tobject);
    publishMarkerDimensions();
  }
}

void TransformableInteractiveServer::setX(std_msgs::Float32 msg)
{
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  if(tobject->setX(msg)){
    updateTransformableObject(tobject);
    publishMarkerDimensions();
  }
}

void TransformableInteractiveServer::setY(std_msgs::Float32 msg)
{
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  if(tobject->setY(msg)){
    updateTransformableObject(tobject);
    publishMarkerDimensions();
  }
}

void TransformableInteractiveServer::setZ(std_msgs::Float32 msg)
{
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  if(tobject->setZ(msg)){
    updateTransformableObject(tobject);
    publishMarkerDimensions();
  }
}

void TransformableInteractiveServer::updateTransformableObject(TransformableObject* tobject)
{
  visualization_msgs::InteractiveMarker int_marker = tobject->getInteractiveMarker();
  server_->insert(int_marker, boost::bind( &TransformableInteractiveServer::processFeedback,this, _1));
  server_->applyChanges();
}

void TransformableInteractiveServer::setPose(geometry_msgs::PoseStamped msg){
  setPose(msg, focus_object_marker_name_);
}

void TransformableInteractiveServer::setPose(geometry_msgs::PoseStamped msg, string object_marker_name){
  if (transformable_objects_map_.find(object_marker_name) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject =  transformable_objects_map_[focus_object_marker_name_];
  setPoseWithTfTransformation(tobject, msg);
  server_->setPose(focus_object_marker_name_, msg.pose, msg.header);
  server_->applyChanges();
}

bool TransformableInteractiveServer::getPoseService(jsk_interactive_marker::GetTransformableMarkerPose::Request &req,jsk_interactive_marker::GetTransformableMarkerPose::Response &res)
{
  TransformableObject* tobject;
  geometry_msgs::PoseStamped transformed_pose_stamped;
  if(req.target_name.compare(std::string("")) == 0){
    if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[focus_object_marker_name_];
  }else{
    if (transformable_objects_map_.find(req.target_name) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[req.target_name];
  }
  transformed_pose_stamped.header.stamp = ros::Time::now();
  transformed_pose_stamped.header.frame_id = tobject->frame_id_;
  transformed_pose_stamped.pose = tobject->getPose();
  res.pose_stamped = transformed_pose_stamped;
  return true;
}

bool TransformableInteractiveServer::setPoseService(jsk_interactive_marker::SetTransformableMarkerPose::Request &req,jsk_interactive_marker::SetTransformableMarkerPose::Response &res)
{
  TransformableObject* tobject;
  geometry_msgs::PoseStamped transformed_pose_stamped;
  if(req.target_name.compare(std::string("")) == 0){
    if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[focus_object_marker_name_];
  }else{
    if (transformable_objects_map_.find(req.target_name) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[req.target_name];
  }
  if(setPoseWithTfTransformation(tobject, req.pose_stamped)){
    server_->setPose(focus_object_marker_name_, req.pose_stamped.pose, req.pose_stamped.header);
    server_->applyChanges();
  }

  return true;
}

bool TransformableInteractiveServer::getColorService(jsk_interactive_marker::GetTransformableMarkerColor::Request &req,jsk_interactive_marker::GetTransformableMarkerColor::Response &res)
{
  TransformableObject* tobject;
  if(req.target_name.compare(std::string("")) == 0){
    if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[focus_object_marker_name_];
  }else{
    if (transformable_objects_map_.find(req.target_name) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[req.target_name];
  }
  tobject->getRGBA(res.color.r, res.color.g, res.color.b, res.color.a);
  return true;
}

bool TransformableInteractiveServer::setColorService(jsk_interactive_marker::SetTransformableMarkerColor::Request &req,jsk_interactive_marker::SetTransformableMarkerColor::Response &res)
{
  TransformableObject* tobject;
  if(req.target_name.compare(std::string("")) == 0){
    if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[focus_object_marker_name_];
  }else{
    if (transformable_objects_map_.find(req.target_name) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[req.target_name];
  }
  tobject->setRGBA(req.color.r, req.color.g, req.color.b, req.color.a);
  updateTransformableObject(tobject);
  return true;
}

bool TransformableInteractiveServer::getFocusService(jsk_interactive_marker::GetTransformableMarkerFocus::Request &req,jsk_interactive_marker::GetTransformableMarkerFocus::Response &res)
{
  res.target_name = focus_object_marker_name_;
  return true;
}

bool TransformableInteractiveServer::setFocusService(jsk_interactive_marker::SetTransformableMarkerFocus::Request &req,jsk_interactive_marker::SetTransformableMarkerFocus::Response &res)
{
  focus_object_marker_name_ = req.target_name;
  focusTextPublish();
  focusPosePublish();
  return true;
}

bool TransformableInteractiveServer::getTypeService(jsk_interactive_marker::GetType::Request &req,jsk_interactive_marker::GetType::Response &res)
{
  TransformableObject* tobject;
  if(req.target_name.compare(std::string("")) == 0){
    if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[focus_object_marker_name_];
    res.type = tobject->type_;
  }else{
    if (transformable_objects_map_.find(req.target_name) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[req.target_name];
    res.type = tobject->type_;
  }
  return true;
}

bool TransformableInteractiveServer::getExistenceService(jsk_interactive_marker::GetTransformableMarkerExistence::Request &req,jsk_interactive_marker::GetTransformableMarkerExistence::Response &res)
{
  if (transformable_objects_map_.find(req.target_name) == transformable_objects_map_.end()) {
    res.existence = false;
  } else {
    res.existence = true;
  }
  return true;
}

bool TransformableInteractiveServer::setDimensionsService(jsk_interactive_marker::SetMarkerDimensions::Request &req,jsk_interactive_marker::SetMarkerDimensions::Response &res)
{
  TransformableObject* tobject;
  if(req.target_name.compare(std::string("")) == 0){
    if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[focus_object_marker_name_];
  }else{
    if (transformable_objects_map_.find(req.target_name) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[req.target_name];
  }
  if (tobject) {
    if (tobject->getType() == jsk_rviz_plugins::TransformableMarkerOperate::BOX) {
      tobject->setXYZ(req.dimensions.x, req.dimensions.y, req.dimensions.z);
    } else if (tobject->getType() == jsk_rviz_plugins::TransformableMarkerOperate::CYLINDER) {
      tobject->setRZ(req.dimensions.radius, req.dimensions.z);
    } else if (tobject->getType() == jsk_rviz_plugins::TransformableMarkerOperate::TORUS) {
      tobject->setRSR(req.dimensions.radius, req.dimensions.small_radius);
    }
    publishMarkerDimensions();
    updateTransformableObject(tobject);
  }
  return true;
}

bool TransformableInteractiveServer::getDimensionsService(jsk_interactive_marker::GetMarkerDimensions::Request &req,jsk_interactive_marker::GetMarkerDimensions::Response &res)
{
  TransformableObject* tobject;
  if(req.target_name.compare(std::string("")) == 0){
    if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[focus_object_marker_name_];
  }else{
    if (transformable_objects_map_.find(req.target_name) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[req.target_name];
  }
  if (tobject) {
    if (tobject->getType() == jsk_rviz_plugins::TransformableMarkerOperate::BOX) {
      tobject->getXYZ(res.dimensions.x, res.dimensions.y, res.dimensions.z);
    } else if (tobject->getType() == jsk_rviz_plugins::TransformableMarkerOperate::CYLINDER) {
      tobject->getRZ(res.dimensions.radius, res.dimensions.z);
    } else if (tobject->getType() == jsk_rviz_plugins::TransformableMarkerOperate::TORUS) {
      tobject->getRSR(res.dimensions.radius, res.dimensions.small_radius);
    }
  }
  return true;
}

void TransformableInteractiveServer::publishMarkerDimensions()
{
  TransformableObject* tobject;
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  tobject = transformable_objects_map_[focus_object_marker_name_];
  if (tobject) {
    jsk_interactive_marker::MarkerDimensions marker_dimensions;
    if (tobject->getType() == jsk_rviz_plugins::TransformableMarkerOperate::BOX) {
      tobject->getXYZ(marker_dimensions.x, marker_dimensions.y, marker_dimensions.z);
    } else if (tobject->getType() == jsk_rviz_plugins::TransformableMarkerOperate::CYLINDER) {
      tobject->getRZ(marker_dimensions.radius, marker_dimensions.z);
    } else if (tobject->getType() == jsk_rviz_plugins::TransformableMarkerOperate::TORUS) {
      tobject->getRSR(marker_dimensions.radius, marker_dimensions.small_radius);
    }
    marker_dimensions.type = tobject->type_;
    marker_dimensions_pub_.publish(marker_dimensions);
  }
}
  
bool TransformableInteractiveServer::requestMarkerOperateService(jsk_rviz_plugins::RequestMarkerOperate::Request &req,jsk_rviz_plugins::RequestMarkerOperate::Response &res)
{
  switch(req.operate.action){
  case jsk_rviz_plugins::TransformableMarkerOperate::INSERT:
    if (req.operate.type == jsk_rviz_plugins::TransformableMarkerOperate::BOX) {
      insertNewBox(req.operate.frame_id, req.operate.name, req.operate.description);
    } else if (req.operate.type == jsk_rviz_plugins::TransformableMarkerOperate::CYLINDER) {
      insertNewCylinder(req.operate.frame_id, req.operate.name, req.operate.description);
    } else if (req.operate.type == jsk_rviz_plugins::TransformableMarkerOperate::TORUS) {
      insertNewTorus(req.operate.frame_id, req.operate.name, req.operate.description);
    }
    return true;
    break;
  case jsk_rviz_plugins::TransformableMarkerOperate::ERASE:
    eraseObject(req.operate.name);
    return true;
    break;
  case jsk_rviz_plugins::TransformableMarkerOperate::ERASEALL:
    eraseAllObject();
    return true;
    break;
  case jsk_rviz_plugins::TransformableMarkerOperate::ERASEFOCUS:
    eraseFocusObject();
    return true;
    break;
  case jsk_rviz_plugins::TransformableMarkerOperate::COPY:
    if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return true; }
    TransformableObject *tobject = transformable_objects_map_[focus_object_marker_name_], *new_tobject;
    if (tobject->type_ == jsk_rviz_plugins::TransformableMarkerOperate::BOX) {
      float x, y, z;
      tobject->getXYZ(x, y, z);
      insertNewBox(tobject->frame_id_, req.operate.name, req.operate.description);
      new_tobject = transformable_objects_map_[req.operate.name];
      new_tobject->setXYZ(x, y, z);
      new_tobject->setPose(tobject->getPose());
    } else if (tobject->type_ == jsk_rviz_plugins::TransformableMarkerOperate::CYLINDER) {
      float r, z;
      tobject->getRZ(r, z);
      insertNewCylinder(tobject->frame_id_, req.operate.name, req.operate.description);
      new_tobject = transformable_objects_map_[req.operate.name];
      new_tobject->setRZ(r, z);
      new_tobject->setPose(tobject->getPose());
    } else if (tobject->type_ == jsk_rviz_plugins::TransformableMarkerOperate::TORUS) {
      float r, sr;
      tobject->getRSR(r, sr);
      insertNewTorus(tobject->frame_id_, req.operate.name, req.operate.description);
      new_tobject = transformable_objects_map_[req.operate.name];
      new_tobject->setRSR(r, sr);
      new_tobject->setPose(tobject->getPose());
    }
    float r, g, b, a;
    tobject->getRGBA(r, g, b, a);
    new_tobject->setRGBA(r, g, b, a);
    return true;
    break;
  };
  return false;
}

void TransformableInteractiveServer::addPose(geometry_msgs::Pose msg){
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  tobject->addPose(msg,false);
  updateTransformableObject(tobject);
}

void TransformableInteractiveServer::addPoseRelative(geometry_msgs::Pose msg){
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  tobject->addPose(msg,true);
  updateTransformableObject(tobject);
}

void TransformableInteractiveServer::focusTextPublish(){
  jsk_rviz_plugins::OverlayText focus_text;
  focus_text.text = focus_object_marker_name_;
  focus_text.top = 0;
  focus_text.left = 0;
  focus_text.width = 300;
  focus_text.height = 50;
  focus_text.bg_color.r = 0.9;
  focus_text.bg_color.b = 0.9;
  focus_text.bg_color.g = 0.9;
  focus_text.bg_color.a = 0.1;
  focus_text.fg_color.r = 0.3;
  focus_text.fg_color.g = 0.3;
  focus_text.fg_color.b = 0.8;
  focus_text.fg_color.a = 1;
  focus_text.line_width = 1;
  focus_text.text_size = 30;
  focus_text_pub_.publish(focus_text);
}

void TransformableInteractiveServer::focusPosePublish(){
  geometry_msgs::Pose target_pose;
  std::stringstream ss;
  if (transformable_objects_map_.find(focus_object_marker_name_) != transformable_objects_map_.end()) {
    target_pose = transformable_objects_map_[focus_object_marker_name_]->getPose();
    ss << "Pos x: " << target_pose.position.x  << " y: " << target_pose.position.y << " z: " << target_pose.position.z
       << std::endl
       << "Ori x: " << target_pose.orientation.x << " y: " << target_pose.orientation.y << " z: " << target_pose.orientation.z << " w: " << target_pose.orientation.w;
  }

  jsk_rviz_plugins::OverlayText focus_pose;
  focus_pose.text = ss.str();
  focus_pose.top = 50;
  focus_pose.left = 0;
  focus_pose.width = 500;
  focus_pose.height = 50;
  focus_pose.bg_color.r = 0.9;
  focus_pose.bg_color.b = 0.9;
  focus_pose.bg_color.g = 0.9;
  focus_pose.bg_color.a = 0.1;
  focus_pose.fg_color.r = 0.8;
  focus_pose.fg_color.g = 0.3;
  focus_pose.fg_color.b = 0.3;
  focus_pose.fg_color.a = 1;
  focus_pose.line_width = 1;
  focus_pose.text_size = 15;
  focus_pose_pub_.publish(focus_pose);
}

void TransformableInteractiveServer::insertNewFace(std::string frame_id, std::string name, std::string description, geometry_msgs::Vector3 scale, geometry_msgs::Pose pose){
  //name = frame_id + "/" + name;
  double face_x = 0.1;
  double face_y = scale.y;
  double face_z = scale.z;
  
  double face_r = 0.1;
  double face_g = 0.1;
  double face_b = 0.6;
  double face_a = 0.2;
  
  TransformableBox* transformable_box = new TransformableBox(face_x, face_y, face_z, face_r, face_g, face_b, face_a, frame_id, name, description);
  transformable_box->setType(FACE);
  //display_interactive_manipulator_ = false;
  //insertNewObject(transformable_box, name);
  insertNewObject(transformable_box, name, true, visualization_msgs::InteractiveMarkerControl::BUTTON);
  geometry_msgs::PoseStamped input_pose_stamped;
  input_pose_stamped.header.frame_id = frame_id;

  //translate face in x directions
  Eigen::Affine3d eigen_pose;
  tf::poseMsgToEigen(pose, eigen_pose);
  eigen_pose = eigen_pose * Eigen::Translation3d(face_x/2, 0.0, 0.0);
  tf::poseEigenToMsg(eigen_pose, pose);

  input_pose_stamped.pose = pose;
  setPose(input_pose_stamped);
}

void TransformableInteractiveServer::insertNewAxis(std::string frame_id, std::string name, std::string description, geometry_msgs::Vector3 scale, geometry_msgs::Pose pose){
  //name = frame_id + "/" + name;
  double axis_radius = scale.x;
  double axis_z = scale.z;
  
  double axis_r = 0.1;
  double axis_g = 0.6;
  double axis_b = 0.1;
  double axis_a = 0.2;
  
  //TransformableBox* transformable_box = new TransformableBox(face_x, face_y, face_z, face_r, face_g, face_b, face_a, frame_id, name, description);
  TransformableCylinder* transformable_cylinder = new TransformableCylinder(axis_radius, axis_z, axis_r, axis_g, axis_b, axis_a, frame_id, name, description);

  transformable_cylinder->setType(AXIS);
  //insertNewObject(transformable_cylinder, name, true, visualization_msgs::InteractiveMarkerControl::MENU);
  insertNewObject(transformable_cylinder, name, true, visualization_msgs::InteractiveMarkerControl::BUTTON);

  boost::shared_ptr<interactive_markers::MenuHandler> menu_handler;
  menu_handler.reset(new interactive_markers::MenuHandler);
  menu_handlers_map_[name] = menu_handler;
  
  for (std::map<string, TransformableObject* >::iterator itpairstri = transformable_objects_map_.begin(); itpairstri != transformable_objects_map_.end(); itpairstri++) {
    menu_handler->insert( itpairstri->first, boost::bind(&TransformableInteractiveServer::assocMenuFeedback, this, _1, itpairstri->first));
  }
  menu_handler->apply(*server_, name);
  

  geometry_msgs::PoseStamped input_pose_stamped;
  input_pose_stamped.header.frame_id = frame_id;

  //translate face in x directions
  /*
  Eigen::Affine3d eigen_pose;
  tf::poseMsgToEigen(pose, eigen_pose);
  eigen_pose = eigen_pose * Eigen::Translation3d(face_x/2, 0.0, 0.0);
  tf::poseEigenToMsg(eigen_pose, pose);
  */
  input_pose_stamped.pose = pose;
  setPose(input_pose_stamped);
}


void TransformableInteractiveServer::insertNewBox(std::string frame_id, std::string name, std::string description)
{
  double x = 0.25;
  double y = 0.45;
  double z = 0.15;
  TransformableBox* transformable_box2 = new TransformableControl(x, y, z, 0.5, 0.5, 0.5, 1.0, frame_id, name + "/control", description);
  insertNewObject(transformable_box2, name + "/control");

  bool display_interactive_manipulator_tmp = display_interactive_manipulator_;
  display_interactive_manipulator_ = false;
  
  TransformableBox* transformable_box = new TransformableBox(x, y, z, 0.5, 0.5, 0.5, 1.0, name + "/control" , name + "/root", description);
  insertNewObject(transformable_box, name + "/root");
  
  //faces
  {
    geometry_msgs::Vector3 scale;
    scale.x = x;
    scale.y = y;
    scale.z = z;
    geometry_msgs::Pose pose;
    pose.position.x = x/2;
    pose.orientation.w = 1.0;
    insertNewFace(name + "/root", name + "/face1", description, scale, pose);

    pose.position.x = -x/2;
    pose.orientation.w = 0.0;
    pose.orientation.z = 1.0;
    insertNewFace(name + "/root", name + "/face2", description, scale, pose);
  }

  //axes
  {
    geometry_msgs::Vector3 scale;
    scale.x = 0.1;
    scale.y = 0.0;
    scale.z = z * 2.0;
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    insertNewAxis(name + "/root", name + "/axis1", description, scale, pose);
  }
  display_interactive_manipulator_ = display_interactive_manipulator_tmp;
}

void TransformableInteractiveServer::insertNewCylinder( std::string frame_id, std::string name, std::string description)
{
  TransformableCylinder* transformable_cylinder = new TransformableCylinder(0.45, 0.45, 0.5, 0.5, 0.5, 1.0, frame_id, name, description);
  insertNewObject(transformable_cylinder, name);
}

void TransformableInteractiveServer::insertNewTorus( std::string frame_id, std::string name, std::string description)
{
  TransformableTorus* transformable_torus = new TransformableTorus(0.45, 0.2, torus_udiv_, torus_vdiv_, 0.5, 0.5, 0.5, 1.0, frame_id, name, description);
  insertNewObject(transformable_torus, name);
}

void TransformableInteractiveServer::insertNewObject( TransformableObject* tobject , std::string name, bool always_visible, unsigned int interaction_mode )
{
  SetInitialInteractiveMarkerConfig(tobject);
  visualization_msgs::InteractiveMarker int_marker = tobject->getInteractiveMarker(always_visible, interaction_mode);
  transformable_objects_map_[name] = tobject;
  server_->insert(int_marker, boost::bind( &TransformableInteractiveServer::processFeedback,this, _1));
  server_->applyChanges();

  focus_object_marker_name_ = name;
  focusTextPublish();
  focusPosePublish();
  //tobject->publishTF();
}

void TransformableInteractiveServer::SetInitialInteractiveMarkerConfig( TransformableObject* tobject )
{
  InteractiveSettingConfig config;
  config.display_interactive_manipulator = display_interactive_manipulator_;
  tobject->setInteractiveMarkerSetting(config);
}

void TransformableInteractiveServer::eraseObject( std::string name )
{
  server_->erase(name);
  server_->applyChanges();
  if (focus_object_marker_name_.compare(name) == 0) {
    focus_object_marker_name_ = "";
    focusTextPublish();
    focusPosePublish();
  }
  delete transformable_objects_map_[name];
  transformable_objects_map_.erase(name);
}

void TransformableInteractiveServer::eraseAllObject()
{
  for (std::map<string, TransformableObject* >::iterator itpairstri = transformable_objects_map_.begin(); itpairstri != transformable_objects_map_.end(); itpairstri++) {
    eraseObject(itpairstri->first);
  }
}

void TransformableInteractiveServer::eraseFocusObject()
{
  eraseObject(focus_object_marker_name_);
}

void TransformableInteractiveServer::tfTimerCallback(const ros::TimerEvent&)
{
  /*
    if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
    TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
    tobject->publishTF();
  */
  for (std::map<string, TransformableObject* >::iterator itpairstri = transformable_objects_map_.begin(); itpairstri != transformable_objects_map_.end(); itpairstri++) {
    itpairstri->second->publishTF();
  }

}

bool TransformableInteractiveServer::setPoseWithTfTransformation(TransformableObject* tobject, geometry_msgs::PoseStamped pose_stamped)
{
  try {
    geometry_msgs::PoseStamped transformed_pose_stamped;
    if (tf_listener_->waitForTransform(tobject->getFrameId(),
                                       pose_stamped.header.frame_id, pose_stamped.header.stamp, ros::Duration(1.0))) {
      tf_listener_->transformPose(tobject->getFrameId(), pose_stamped, transformed_pose_stamped);
      tobject->setPose(transformed_pose_stamped.pose);
    }
    else {
      ROS_ERROR("failed to lookup transform %s -> %s", tobject->getFrameId().c_str(), 
                pose_stamped.header.frame_id.c_str());
      return false;
    }
  }
  catch (tf2::ConnectivityException &e)
  {
    ROS_ERROR("Transform error: %s", e.what());
    return false;
  }
  catch (tf2::InvalidArgumentException &e)
  {
    ROS_ERROR("Transform error: %s", e.what());
    return false;
  }
  return true;
}

void TransformableInteractiveServer::run(){
  ros::spin();
}
