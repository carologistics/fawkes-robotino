#include "tag_position_list.h"

TagPositionList::TagPositionList(fawkes::BlackBoard *blackboard, u_int32_t max_markers, std::string frame, std::string thread_name, fawkes::Logger *logger)
{
  // store parameters
  this->blackboard_ = blackboard;
  this->max_markers_ = max_markers;
  this->thread_name_ = thread_name;
  this->logger_ = logger;

  // create blackboard interfaces
  for(size_t i=0; i < this->max_markers_; i++)
  {
    // create a name for the new interface
    std::string interface_name = std::string("/tag-vision/") + std::to_string(i);
    // create the interface and store
    try
    {
      // get an interface from the blackboard
      auto interface = this->blackboard_->open_for_writing<fawkes::Position3DInterface>(interface_name.c_str());
      // set the frame of the interface
      interface->set_frame(frame.c_str());
      // generate a helper class and push it into this vector
      this->push_back(new TagPositionInterface(interface, i));
    }
    catch (std::exception &e)
    {
      this->logger_->log_error(thread_name.c_str(),(std::string("Could not open the blackboard: ") + std::string(e.what())).c_str());
      throw(e);
    }
  }

  // initialize tag info interface
  try
  {
    this->tag_vision_interface_ = blackboard->open_for_writing<fawkes::TagVisionInterface>("/tag-vision/info");
  }
  catch (std::exception &e)
  {
    this->logger_->log_error(thread_name.c_str(),(std::string("Could not open the blackboard: ") + std::string(e.what())).c_str());
    throw(e);
  }
}

TagPositionList::~TagPositionList()
{
  // close all blackboards
  for(size_t i=0; i < this->max_markers_; i++)
  {
    this->blackboard_->close(this->at(i)->interface());
  }


  // close tag vision interface
  this->blackboard_->close(this->tag_vision_interface_);

  // delete this interfaces
  for(auto &&interface: *this)
  {
    this->blackboard_->close(interface->interface());
    delete interface;
  }
}

void TagPositionList::update_blackboard(std::vector<alvar::MarkerData> *marker_list)
{
  for(alvar::MarkerData& marker: *marker_list)
  {
    // skip the marker, if the pose is directly on the camera (error)
    alvar::Pose tmp_pose = marker.pose;
    if(tmp_pose.translation[0]<1 && tmp_pose.translation[1]<1 && tmp_pose.translation[2]<1){
        continue;
    }
    // get the id of the marker
    u_int32_t marker_id=marker.GetId();

    // find an interface with this marker assigned or an empty interface
    TagPositionInterface *marker_interface = NULL;
    TagPositionInterface *empty_interface = NULL;
    for(TagPositionInterface *interface: *this)
    {
      // assign marker_interface
      if(interface->marker_id() == marker_id){
        marker_interface = interface;
      }
      // assign empty interface
      if(empty_interface == NULL && interface->marker_id() == EMPTY_INTERFACE_MARKER_ID)
      {
        empty_interface = interface;
      }
    }
    // if no marker interface is found, assign the empty interface and assign the marker id
    if(marker_interface == NULL)
    {
      marker_interface = empty_interface;
      marker_interface->set_marker_id(marker_id);
    }
    // continue with the marker interface
    marker_interface->set_pose(tmp_pose);
  }
  // update blackboard with interfaces
  u_int32_t visible_markers = 0;
  for(TagPositionInterface *interface: *this)
  {
    this->tag_vision_interface_->set_tag_id(interface->vector_position(), interface->marker_id());
    if(interface->marker_id() != EMPTY_INTERFACE_MARKER_ID)
    {
      visible_markers++;
    }
    interface->write();
  }
  this->tag_vision_interface_->set_tags_visible(visible_markers);
  this->tag_vision_interface_->write();
}
