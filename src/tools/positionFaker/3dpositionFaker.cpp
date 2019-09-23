
/***************************************************************************
 *  3dpositionFaker.cpp - fakes 3dPositionInterface for Testing
 *
 *  Created: Sat Mar 15 13:57:22 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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
#include <interfaces/Position3DInterface.h>
#include <netcomm/fawkes/client.h>
#include <netcomm/fawkes/client_handler.h>
#include <readline/history.h>
#include <readline/readline.h>
#include <utils/system/argparser.h>
#include <utils/system/signal.h>

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>

using namespace fawkes;

void
print_usage(const char *program_name)
{
	printf("Usage: %s [-h] [-r host[:port]]\n"
	       " -h              This help message\n"
	       " -r host[:port]  Remote host (and optionally port) to connect to\n",
	       program_name);
}

static int
event_hook()
{
	return 0;
}

/** Flite shell thread.
 * This thread opens a network connection to a host and uses a RemoteBlackBoard
 * connection to send commands to the flite-plugin
 * @author Tim Niemueller, Frederik Zwilling
 */
class PositionFakerThread : public Thread, public FawkesNetworkClientHandler
{
public:
	/** Constructor.
   * @param argp argument parser
   */
	PositionFakerThread(ArgumentParser *argp) : Thread("FliteShellThread", Thread::OPMODE_CONTINUOUS)
	{
		this->argp               = argp;
		prompt                   = "-# ";
		just_connected           = true;
		connection_died_recently = false;

		sif = NULL;
		using_history();
		// this is needed to get rl_done working
		rl_event_hook = event_hook;

		continuous = false;

		char *             host      = (char *)"localhost";
		unsigned short int port      = 1910;
		bool               free_host = argp->parse_hostport("r", &host, &port);

		c = new FawkesNetworkClient(host, port);

		if (free_host)
			free(host);
		c->register_handler(this, FAWKES_CID_SKILLER_PLUGIN);
		c->connect();
	}

	/** Destructor. */
	~PositionFakerThread()
	{
		printf("Finalizing\n");
		usleep(500000);

		rbb->close(sif);
		delete rbb;
		rbb = NULL;
		c->deregister_handler(FAWKES_CID_SKILLER_PLUGIN);
		c->disconnect();
		delete c;
	}

	virtual void
	loop()
	{
		if (c->connected()) {
			if (just_connected) {
				just_connected = false;
				try {
					rbb           = new RemoteBlackBoard(c);
					sif           = rbb->open_for_writing<Position3DInterface>("Light");
					poseInterface = rbb->open_for_writing<Position3DInterface>("Pose");
					usleep(100000);
				} catch (Exception &e) {
					e.print_trace();
					return;
				}
			}

			if (argp->num_items() > 0) {
				std::string                      sks   = "";
				const std::vector<const char *> &items = argp->items();

				std::vector<const char *>::const_iterator i = items.begin();
				sks                                         = *i;
				++i;
				for (; i != items.end(); ++i) {
					sks += " ";
					sks += *i;
				}

				usleep(100000);
				exit();
			} else {
				char *lineX    = NULL;
				char *lineY    = NULL;
				char *lineXPos = NULL;
				char *lineYPos = NULL;

				printf("Enter Light x: ");
				lineX = readline(prompt);
				printf("Enter Light y: ");
				lineY = readline(prompt);
				printf("Enter Pose x: ");
				lineXPos = readline(prompt);
				printf("Enter Pose y: ");
				lineYPos = readline(prompt);
				if (lineX && lineY) {
					if (strcmp(lineX, "") != 0) {
						std::stringstream sstr1;
						std::stringstream sstr2;
						std::stringstream sstr3;
						std::stringstream sstr4;
						double            x, y, xPos, yPos;
						sstr1 << lineX;
						sstr1 >> x;
						sstr2 << lineY;
						sstr2 >> y;
						sstr3 << lineXPos;
						sstr3 >> xPos;
						sstr4 << lineYPos;
						sstr4 >> yPos;
						printf("Light: %f, %f \n", x, y);
						printf("Pos: %f, %f \n", xPos, yPos);

						double translation[3];
						translation[0] = x;
						translation[1] = y;
						translation[2] = 0.0;
						double translationPos[3];
						translationPos[0] = xPos;
						translationPos[1] = yPos;
						translationPos[2] = 0.0;
						sif->set_translation(translation);
						poseInterface->set_translation(translationPos);
						sif->write();
						poseInterface->write();
					}
				} else {
					if (!connection_died_recently) {
						exit();
					}
				}
			}
		} else {
			if (connection_died_recently) {
				connection_died_recently = false;
				printf("Connection died\n");
				c->disconnect();
			}
			try {
				c->connect();
			} catch (Exception &e) {
				printf(".");
				fflush(stdout);
				sleep(1);
			}
		}
	}

	virtual void
	deregistered(unsigned int id) throw()
	{
	}

	virtual void
	inbound_received(FawkesNetworkMessage *m, unsigned int id) throw()
	{
	}

	virtual void
	connection_died(unsigned int id) throw()
	{
		prompt = "-# ";

		rbb->close(sif);
		delete rbb;
		rbb = NULL;
		sif = NULL;

		connection_died_recently = true;

		// fprintf(stdin, "\n");
		// kill(SIGINT);
		rl_done = 1;
	}

	virtual void
	connection_established(unsigned int id) throw()
	{
		printf("Connection established\n");
		just_connected = true;
		prompt         = "+# ";
	}

private:
	ArgumentParser *     argp;
	FawkesNetworkClient *c;
	BlackBoard *         rbb;
	Position3DInterface *sif, *poseInterface;
	const char *         prompt;
	bool                 just_connected;
	bool                 connection_died_recently;

	bool continuous;
};

/** Config tool main.
 * @param argc argument count
 * @param argv arguments
 */
int
main(int argc, char **argv)
{
	ArgumentParser argp(argc, argv, "hr:");

	if (argp.has_arg("h")) {
		print_usage(argv[0]);
		exit(0);
	}

	PositionFakerThread sst(&argp);
	sst.start();
	sst.join();

	return 0;
}
