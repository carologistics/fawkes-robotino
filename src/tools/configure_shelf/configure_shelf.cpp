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

#include <netcomm/fawkes/client.h>
#include <blackboard/remote.h>
#include <interfaces/ShelfConfigurationInterface.h>

#include <stdio.h>
#include <unistd.h>

using namespace fawkes;

int
main(int argc, char **argv)
{
  FawkesNetworkClient *client = NULL;
  BlackBoard *blackboard = NULL;
  const char * host = "localhost";
  const uint port = 1921;
  try {
    client = new FawkesNetworkClient(host, port);
    client->connect();
    blackboard = new RemoteBlackBoard(client);
  } catch (Exception &e) {
    printf("Failed to connect to remote host at %s:%u\n", host, port);
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
  shelf->set_left(ShelfConfigurationInterface::BLACK);
  shelf->set_center(ShelfConfigurationInterface::RED);
  shelf->set_right(ShelfConfigurationInterface::BLACK);
  shelf->write();
  blackboard->close(shelf);
  client->disconnect();
}
