#include <boost/fusion/include/pair.hpp>

#include "interface_definitions.h"

#include <interfaces/OrderInterface.h>
#include <interfaces/RCLLGameStateInterface.h>
#include <interfaces/RecvBeaconInterface.h>



namespace protoboard {


using namespace fawkes;
using namespace boost::fusion;


std::vector<std::string> proto_dirs()
{ return { BASEDIR "/src/libs/llsf_msgs" }; }



/*
 * Sending interfaces: ID is used as-is
 */

template<>
std::string iface_id_for_type<SendBeaconInterface>()
{ return "/protoboard/send_beacon"; }

template<>
std::string iface_id_for_type<PrepareMachineInterface>()
{ return "/protoboard/prepare_machine"; }



/*
 * Receiving interfaces: ID ends with slash before index is appended
 */

template<>
std::string iface_id_for_type<OrderInterface>()
{ return "/protoboard/order/"; }

template<>
std::string iface_id_for_type<RCLLGameStateInterface>()
{ return "/protoboard/game_state/"; }

template<>
std::string iface_id_for_type<RecvBeaconInterface>()
{ return "/protoboard/beacon/"; }


} // namespace protoboard
