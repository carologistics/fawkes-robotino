#ifndef PROTOBOARD_PLUGIN_H
#define PROTOBOARD_PLUGIN_H

#include <core/plugin.h>


class ProtoboardPlugin : public fawkes::Plugin {
public:
  ProtoboardPlugin(fawkes::Configuration *config);
};

PLUGIN_DESCRIPTION("Publish LLSF protobuf messages to blackboard")
EXPORT_PLUGIN(ProtoboardPlugin)

#endif // PROTOBOARD_PLUGIN_H
