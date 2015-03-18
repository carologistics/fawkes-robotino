#include "tag_position_interface.h"

TagPositionIntreface::TagPositionIntreface(fawkes::Position3DInterface *position_interface)
{
  this->interface_ = position_interface;
  this->visibility_history_ = 0;
  this->marker_id_ = 0;
  this->touched_ = false;
}

TagPositionIntreface::~TagPositionIntreface()
{

}

void TagPositionIntreface::set_pose(alvar::Pose new_pose)
{
  //temp mat to get cv data
  CvMat mat;
  //angles in heading attitude bank
  double rot[3];
  //create the mat
  cvInitMatHeader(&mat, 3, 1, CV_64F, rot);
  //get the quaternion of this pose into the mat
  new_pose.GetEuler(&mat);
  //get the temporary quaternion in wxyz order to calculate angles
  rot[0] = CV_MAT_ELEM(mat, double, 0, 0);
  rot[1] = CV_MAT_ELEM(mat, double, 1, 0);
  rot[2] = CV_MAT_ELEM(mat, double, 2, 0);
  //publish the angles
  this->interface_->set_rotation(ROT::X,rot[ROT::X]*M_PI/180);
  this->interface_->set_rotation(ROT::Y,rot[ROT::Y]*M_PI/180);
  this->interface_->set_rotation(ROT::Z,rot[ROT::Z]*M_PI/180);
  this->interface_->set_rotation(ROT::W,0);
  //publish the translation
  this->interface_->set_translation(1,new_pose.translation[0]/1000);
  this->interface_->set_translation(2,new_pose.translation[1]/1000);
  this->interface_->set_translation(0,new_pose.translation[2]/1000);

  this->touched_=true;
}

void TagPositionIntreface::set_marker_id(u_int32_t new_id)
{
  // apply the new id
  this->marker_id_ = new_id;
  // reset the interface, the visibility history is 0
  this->visibility_history_ = 0;
}

void TagPositionIntreface::write()
{
  // update the visibility history according to the marker, weather this interface got a new pose
  if(this->touched_){
    this->visibility_history_++;
  }
  else
  {
    this->visibility_history_--;
  }
  // empty marker id if the tag is no longer visible
  if(this->marker_id_ != EMPTY_INTERFACE_MARKER_ID && this->visibility_history_ < 0)
  {
    this->marker_id_ = EMPTY_INTERFACE_MARKER_ID;
  }
  // set the new visibility history
  this->interface_->set_visibility_history(this->visibility_history_);
  // write out the interface
  this->interface_->write();
  // reset the update marker
  this->touched_ = false;
}
