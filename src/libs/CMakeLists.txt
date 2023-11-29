# *****************************************************************************
# CMake Build System for Fawkes
# -------------------
# Copyright (C) 2023 by Tarik Viehmann and Daniel Swoboda
#
# *****************************************************************************
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program.  If not, see <http://www.gnu.org/licenses/>.
#
# *****************************************************************************

add_subdirectory(interfaces)
find_package(Protobuf REQUIRED)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS rcll-protobuf-msgs/AgentTask.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
                      rcll-protobuf-msgs/AttentionMessage.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
                      rcll-protobuf-msgs/BeaconSignal.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
                      rcll-protobuf-msgs/ConveyorVisionResult.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
                      rcll-protobuf-msgs/ExplorationInfo.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS rcll-protobuf-msgs/GameInfo.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS rcll-protobuf-msgs/GameState.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
                      rcll-protobuf-msgs/LightSignals.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
                      rcll-protobuf-msgs/LockMasterAnnounce.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
                      rcll-protobuf-msgs/LockMessage.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
                      rcll-protobuf-msgs/MachineCommands.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
                      rcll-protobuf-msgs/MachineDescription.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
                      rcll-protobuf-msgs/MachineInfo.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
                      rcll-protobuf-msgs/MachineInstructions.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
                      rcll-protobuf-msgs/MachineReport.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
                      rcll-protobuf-msgs/NavigationChallenge.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS rcll-protobuf-msgs/OrderInfo.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS rcll-protobuf-msgs/Pose2D.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS rcll-protobuf-msgs/Pose3D.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
                      rcll-protobuf-msgs/PreparedForProduction.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
                      rcll-protobuf-msgs/ProductColor.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
                      rcll-protobuf-msgs/PuckDetectionResult.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS rcll-protobuf-msgs/PuckInfo.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS rcll-protobuf-msgs/RingInfo.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
                      rcll-protobuf-msgs/RobotCommands.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS rcll-protobuf-msgs/RobotInfo.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
                      rcll-protobuf-msgs/SimTimeSync.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS rcll-protobuf-msgs/Sync.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS rcll-protobuf-msgs/Team.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS rcll-protobuf-msgs/Time.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
                      rcll-protobuf-msgs/VersionInfo.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS rcll-protobuf-msgs/Worldmodel.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS rcll-protobuf-msgs/Zone.proto)
