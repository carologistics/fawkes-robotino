#ifndef TAG_POSITION_LIST_H
#define TAG_POSITION_LIST_H

#include <vector>
#include <string>
#include <interfaces/TagVisionInterface.h>
#include <blackboard/blackboard.h>
#include <blackboard/exceptions.h>
#include <alvar/Marker.h>
#include <logging/logger.h>

#include "tag_position_interface.h"


enum ROT{
    X=0,
    Y=1,
    Z=2,
    W=3
};

class TagPositionList : public std::vector<TagPositionIntreface*>
{
public:
  /// Constructor
  TagPositionList(fawkes::BlackBoard *blackboard, u_int32_t max_markers, std::string frame, std::string thread_name, fawkes::Logger *logger_);
  /// Destructor
  ~TagPositionList();
  /// Update the blackboard with the stored data
  void update_blackboard(std::vector<alvar::MarkerData> *marker_list);

private:
  /// how many markers can be detected at the same time
  u_int32_t max_markers_;
  /// Tahe blackboard to publish on
  fawkes::BlackBoard *blackboard_;
  /// tag vision inforamtion interface
  fawkes::TagVisionInterface *tag_vision_interface_;
  /// Name of the calling thread
  std::string thread_name_;
  /// Logger for logging
  fawkes::Logger *logger_;
};

#endif // TAG_POSITION_LIST_H
