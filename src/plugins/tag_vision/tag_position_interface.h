#ifndef TAG_POSITION_INTREFACE_H
#define TAG_POSITION_INTREFACE_H

#include <string>

#include <interfaces/Position3DInterface.h>

#include <alvar/Pose.h>

#define EMPTY_INTERFACE_MARKER_ID 0

class TagPositionIntreface
{
  enum ROT{
      X=0,
      Y=1,
      Z=2,
      W=3
  };

public:
  // constructor
  TagPositionIntreface(fawkes::Position3DInterface *position_interface, u_int32_t vector_position_);
  // destructor
  ~TagPositionIntreface();

  // update the position of the interface
  void set_pose(alvar::Pose new_pose);

  // write the interface on the blackboard
  void write();

  // get the marker id
  u_int32_t marker_id() {return this->marker_id_;}

  // set the marker id
  void set_marker_id(u_int32_t new_id);

  // get the interface
  fawkes::Position3DInterface *interface() { return this->interface_; }

  // get the vector position
  u_int32_t vector_position(){ return this->vector_position_; }

private:
  // the interface to handle
  fawkes::Position3DInterface *interface_;

  // the visibility history for the interface to handle
  int32_t visibility_history_;

  // marker weather the interface was updated since last write
  bool touched_;

  // the id of the marker the interface represents
  u_int32_t marker_id_;

  // the position of the interface in the vector
  u_int32_t vector_position_;
};

#endif // TAG_POSITION_INTREFACE_H
