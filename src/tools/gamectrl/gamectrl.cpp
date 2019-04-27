
/***************************************************************************
 *  main.cpp - Robotino GameCtrl
 *
 *  Created: Tue Jun 19 20:39:42 2021
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
 *
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

#include <blackboard/remote.h>
#include <core/threading/thread.h>
#include <interfaces/SwitchInterface.h>
#include <utils/system/argparser.h>
#include <utils/system/signal.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unistd.h>

using namespace fawkes;

static void
print_usage(const char *progname)
{
	printf("Usage: %s -r remote start|stop\n", progname);
}

int
main(int argc, char **argv)
{
	ArgumentParser argp(argc, argv, "hr:");

	if (argp.has_arg("h") || argp.num_items() == 0) {
		print_usage(argp.program_name());
		exit(0);
	}

	Thread::init_main();

	std::string        host = "localhost";
	unsigned short int port = 1910;
	if (argp.has_arg("r")) {
		argp.parse_hostport("r", host, port);
	}

	BlackBoard *     remote_bb   = new RemoteBlackBoard(host.c_str(), port);
	SwitchInterface *gamectrl_if = remote_bb->open_for_writing<SwitchInterface>("Clips Agent Start");

	if (strcmp(argp.items()[0], "start") == 0) {
		printf("Sending START to robot %s\n", host.c_str());
		gamectrl_if->set_enabled(true);
	} else if (strcmp(argp.items()[0], "start") == 0) {
		printf("Sending STOP to robot %s\n", host.c_str());
		gamectrl_if->set_enabled(false);
	} else {
		printf("Unknown command %s\n", argp.items()[0]);
	}
	gamectrl_if->write();
	usleep(1000000);
	remote_bb->close(gamectrl_if);
	delete remote_bb;

	Thread::destroy_main();

	return 0;
}
