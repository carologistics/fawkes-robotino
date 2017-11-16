/***************************************************************************
 *  configure_shelf.cpp - Set a goal shelf configuration for the Hackathon
 *
 *  Created: Thu 16 Nov 2017 10:42:13 CET 10:42
 *  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <core/exception.h>
#include <utils/system/argparser.h>
#include <netcomm/fawkes/client.h>
#include <blackboard/remote.h>
#include <interfaces/ShelfConfigurationInterface.h>

#include <stdio.h>
#include <unistd.h>

#include <algorithm>
#include <string>

using namespace fawkes;
using namespace std;

class
WrongArgumentException : public Exception
{
 public:
  WrongArgumentException(const char *arg) : Exception()
  {
    append("Wrong argument: %s", arg);
  }
};

void
print_usage(const char *program_name)
{
  printf("Usage: %s [-h] [-r host[:port]] <LEFT> <CENTER> <RIGHT>\n"
      " -h                 This help message\n"
      " -r host[:port]     Remote host (and optionally port) to connect to\n"
      " LEFT|CENTER|RIGHT  One of 'RED' and 'BLACK'\n",
      program_name);
}

ShelfConfigurationInterface::SlotColor
string_to_slot_color(string arg)
{
  transform(arg.begin(), arg.end(), arg.begin(), ::tolower);
  if (arg == "red") {
    return ShelfConfigurationInterface::RED;
  } else if (arg == "black") {
    return ShelfConfigurationInterface::BLACK;
  } else {
    throw WrongArgumentException(arg.c_str());
  }
}


int
main(int argc, char **argv)
{
  ArgumentParser arg_parser(argc, argv, "hr:");
  if (arg_parser.has_arg("h")) {
    print_usage(argv[0]);
    return 0;
  }
  FawkesNetworkClient *client = NULL;
  BlackBoard *blackboard = NULL;
  string host = "localhost";
  unsigned short int port = 1921;
  if (arg_parser.has_arg("r")) {
    arg_parser.parse_hostport("r", host, port);
  }
  if (argv[optind] == NULL || argv[optind+1] == NULL || argv[optind+2] == NULL)
  {
    printf("Missing argument\n");
    print_usage(argv[0]);
    return 1;
  }
  ShelfConfigurationInterface::SlotColor left_color, center_color, right_color;
  try {
    left_color = string_to_slot_color(argv[optind]);
    center_color = string_to_slot_color(argv[optind+1]);
    right_color = string_to_slot_color(argv[optind+2]);
  } catch (WrongArgumentException &e) {
    printf("%s\n", e.what_no_backtrace());
    print_usage(argv[0]);
    return 1;
  }

  try {
    client = new FawkesNetworkClient(host.c_str(), port);
    client->connect();
    blackboard = new RemoteBlackBoard(client);
  } catch (Exception &e) {
    printf("Failed to connect to remote host at %s:%u\n", host.c_str(), port);
    return 1;
  }
  ShelfConfigurationInterface *shelf = NULL;
  const char *shelf_id = "shelf";
  try {
    shelf = blackboard->open_for_writing<ShelfConfigurationInterface>(shelf_id);
  } catch (Exception &e) {
    printf("Failed to open blackboard interface %s\n", shelf_id);
    return 1;
  }
  shelf->set_left(left_color);
  shelf->set_center(center_color);
  shelf->set_right(right_color);
  shelf->set_ready(true);
  shelf->write();
  blackboard->close(shelf);
  client->disconnect();
}
