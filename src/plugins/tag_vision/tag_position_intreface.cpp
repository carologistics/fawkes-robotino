#include "tag_position_intreface.h"

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

void TagPositionIntreface::set_position(alvar::Pose)
{

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
  // set the new visibility history
  this->interface_->set_visibility_history(this->visibility_history_);
  // write out the interface
  this->interface_->write();
  // reset the update marker
  this->touched_ = false;
}
