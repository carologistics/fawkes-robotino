#ifndef TAG_POSITION_INTREFACE_H
#define TAG_POSITION_INTREFACE_H

#include <string>

#include <interfaces/Position3DInterface.h>

#include <alvar/Pose.h>

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
  TagPositionIntreface(fawkes::Position3DInterface *position_interface);
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

private:
  // the interface to handle
  fawkes::Position3DInterface *interface_;

  // the visibility history for the interface to handle
  int32_t visibility_history_;

  // marker weather the interface was updated since last write
  bool touched_;

  // the id of the marker the interface represents
  u_int32_t marker_id_;
};

#endif // TAG_POSITION_INTREFACE_H
