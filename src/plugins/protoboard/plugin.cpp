#include "plugin.h"

#include "blackboard_manager.h"
#include "protobuf_thread.h"

ProtoboardPlugin::ProtoboardPlugin(fawkes::Configuration *config)
  : Plugin(config)
{
  protoboard::ProtobufThead *msg_handler = new protoboard::ProtobufThead();
  protoboard::BlackboardManager *bb_mgr = new protoboard::BlackboardManager(msg_handler);
  msg_handler->set_bb_manager(bb_mgr);
  thread_list.push_back(bb_mgr);
  thread_list.push_back(msg_handler);
}
