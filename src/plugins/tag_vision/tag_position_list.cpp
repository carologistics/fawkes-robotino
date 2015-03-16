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
      auto interface = this->blackboard_->open_for_writing<fawkes::Position3DInterface>(interface_name.c_str());
      interface->set_frame(frame.c_str());
      interface->set_visibility_history(0);
      this->push_back(interface);
    }
    catch (std::exception &e)
    {
      this->logger_->log_error(thread_name.c_str(),(std::string("Could not open the blackboard: ") + std::string(e.what())).c_str());
      throw(e);
    }
  }

  // initilize touched marker
  this->touched_.assign(this->max_markers_, false);

  // initialize the id positions
  this->ids_ = new int32_t[max_markers];

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
    this->blackboard_->close(this->at(i));
  }

  // free the ids
  delete [] this->ids_;

  // close tag vision interface
  this->blackboard_->close(this->tag_vision_interface_);
}

void TagPositionList::update_blackboard(std::vector<alvar::MarkerData> marker_list)
{
  for(alvar::MarkerData& marker: marker_list)
  {
    // skip the marker, if the pose is directly on the camera (error)
    alvar::Pose tmp_pose = marker.pose;
    if(tmp_pose.translation[0]<1 && tmp_pose.translation[1]<1 && tmp_pose.translation[2]<1){
        continue;
    }
    // get the id of the marker
    int32_t marker_id=marker.GetId();
    // if the marker was not seen recently add it
    if(this->mapping_.find(marker_id) == this->mapping_.end())
    {
      // find free interface
      int position;
      for(position = 0; position < (int)this->size() ;position++)
      {
        fawkes::Position3DInterface *interface = this->at(position);
        if(interface->visibility_history() < 0)
        {
          // reinit the interface by setting the visibillity history to 0
          interface->set_visibility_history(0);
          break;
        }
      }
      this->mapping_[marker_id]=position;
      // get new position
    }
    // a position for the marker exists here, so set the interface information
    fawkes::Position3DInterface *interface_to_set = this->at((this->mapping_)[marker_id]);
    //temp mat to get cv data
    CvMat mat;
    //angles in heading attitude bank
    double rot[3];
    //create the mat
    cvInitMatHeader(&mat, 3, 1, CV_64F, rot);
    //get the quaternion of this pose into the mat
    marker.pose.GetEuler(&mat);
    //get the temporary quaternion in wxyz order to calculate angles
    rot[0] = CV_MAT_ELEM(mat, double, 0, 0);
    rot[1] = CV_MAT_ELEM(mat, double, 1, 0);
    rot[2] = CV_MAT_ELEM(mat, double, 2, 0);
    //publish the angles
    interface_to_set->set_rotation(ROT::X,rot[ROT::X]*M_PI/180);
    interface_to_set->set_rotation(ROT::Y,rot[ROT::Y]*M_PI/180);
    interface_to_set->set_rotation(ROT::Z,rot[ROT::Z]*M_PI/180);
    interface_to_set->set_rotation(ROT::W,0);
    //publish the translation
    interface_to_set->set_translation(1,marker.pose.translation[0]/1000);
    interface_to_set->set_translation(2,marker.pose.translation[1]/1000);
    interface_to_set->set_translation(0,marker.pose.translation[2]/1000);

    //interface_to_set->set_frame(fv_cam_info.frame.c_str());
    interface_to_set->set_visibility_history(interface_to_set->visibility_history() + 1);

    // write interface
    interface_to_set->write();
    // update position
    this->touched_[this->mapping_[marker_id]]=true;

    this->tag_vision_interface_->set_tag_id(this->mapping_[marker_id],marker_id);
  }
  // update remaining interfaces
  for (size_t i=0; i < this->size(); i++)
  {
    if(!this->touched_[i])
    {
      fawkes::Position3DInterface *interface=this->at(i);
      int32_t new_history = interface->visibility_history() - 1;
      interface->set_visibility_history(new_history);
      if(new_history < 0){
        this->tag_vision_interface_->set_tag_id(i,0);
      }
    }
  }
  this->tag_vision_interface_->write();
  // reset touched
  this->touched_.assign(this->max_markers_, false);
}
