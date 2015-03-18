
/***************************************************************************
 *  webview-refbox-processor.cpp - RCLL refbox webview
 *
 *  Created: Tue Mar 17 18:00:29 2015
 *  Copyright  2015  Tim Niemueller [www.niemueller.de]
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

#include "webview-refbox-processor.h"

#include <core/exception.h>
#include <core/threading/mutex_locker.h>
#include <webview/page_reply.h>
#include <webview/error_reply.h>
#include <webview/redirect_reply.h>

#include <mongo/client/dbclient.h>

#include <cstring>
#include <memory>

using namespace fawkes;
using namespace mongo;

/** @class WebviewRCLLRefBoxRequestProcessor "webview-refbox-processor.h"
 * Pan/tilt/zoom camera request processor.
 * @author Tim Niemueller
 */


/** Constructor.
 * @param base_url base URL of the webview PTZ cam web request processor.
 * @param image_id Shared memory image buffer ID for viewing
 * @param pantilt_id PanTiltInterface ID
 * @param camctrl_id CameraControlInterface ID
 * @param power_id SwitchInterface ID for powering PTU
 * @param camera_id SwitchInterface ID for enabling/disabling image retrieval
 * @param pan_increment value by which to increment pan value on request
 * @param tilt_increment value by which to increment tilt value on request
 * @param zoom_increment value by which to increment zoom value on request
 * @param post_powerup_time time in seconds by which to delay reponse when
 * turning on PTU and camera after inactivity
 * @param presets pan/tilt preset values
 * @param blackboard blackboard to open interfaces 
 * @param logger logger to report problems
 */
WebviewRCLLRefBoxRequestProcessor::WebviewRCLLRefBoxRequestProcessor
  (std::string base_url, mongo::DBClientBase *mongodb_client)
{
  mongodb_ = mongodb_client;
}


/** Destructor. */
WebviewRCLLRefBoxRequestProcessor::~WebviewRCLLRefBoxRequestProcessor()
{
}


WebReply *
WebviewRCLLRefBoxRequestProcessor::process_request(const fawkes::WebRequest *request)
{
  if ( request->url().find(baseurl_) == 0 ) {
    std::string subpath = request->url().substr(baseurl_.length());

    WebPageReply *r = new WebPageReply("RCLL RefBox");

    *r += "<h2>RCLL RefBox Database Report</h2>\n";

    time_t query_time = time(0);
    query_time -= 24 * 60;

    BSONObj query = BSON("start-time" << BSONObjBuilder().appendTimeT("$gt", query_time).obj());

    // check if there is a running game
    std::shared_ptr<DBClientCursor> cursor =
      mongodb_->query("llsfrb.game_report", Query(query).sort("start-time", -1), 1);

    if (cursor->more()) {
      BSONObj doc = cursor->next();
      std::vector<BSONElement> teams  = doc["teams"].Array();
      std::vector<BSONElement> points = doc["total-points"].Array();
      r->append_body("<h3>Active Game %s vs. %s</h3>\n",
		     teams[0].String().c_str(), teams[1].String().c_str());

      *r += "<table>\n";
      r->append_body("<tr><td><b>Started at:</b></td>\n"
		     "<td>%s</td></tr>\n", doc["start-time"].Date().toString().c_str());
      r->append_body("<tr><td><b>Points:</b></td>\n"
		     "<td style=\"background-color: #aaaaaa\"><span style=\"color: #00ffff\">%u</span> "
		     ": <span style=\"color: #ff00ff\">%u</span></td></tr>\n",
		     points[0].Long(), points[1].Long());

      r->append_body("<tr><td><b>Points Cyan:</b></td>\n"
		     "<td><i>Exploration:</i> %ld</td></tr>\n",
		     doc["phase-points-cyan"]["EXPLORATION"].Long());
      r->append_body("<tr><td></td>\n"
		     "<td><i>Production:</i> %ld</td></tr>\n",
		     doc["phase-points-cyan"]["PRODUCTION"].Long());

      r->append_body("<tr><td><b>Points Magenta:</b></td>\n"
		     "<td><i>Exploration:</i> %ld</td></tr>\n",
		     doc["phase-points-magenta"]["EXPLORATION"].Long());
      r->append_body("<tr><td></td>\n"
		     "<td><i>Production:</i> %ld</td></tr>\n",
		     doc["phase-points-magenta"]["PRODUCTION"].Long());

      *r += "</table>\n";
    }

    *r += "<h3>Previous Games</h3>\n";

    // check if there is a running game
    cursor =
      mongodb_->query("llsfrb.game_report", QUERY("end-time" << BSON("$exists" << true)).sort("start-time"));

    *r +=
      "<table>\n"
      "<tr><th>Teams</th><th>Points</th><th colspan=\"2\">Points Cyan</th>"
      "<th colspan=\"2\">Points Magenta</th></tr>\n";

    std::map<std::string, unsigned int> games_won;

    while (cursor->more()) {
      BSONObj doc = cursor->next();
      std::vector<BSONElement> teams  = doc["teams"].Array();
      std::vector<BSONElement> points = doc["total-points"].Array();

      int winner_team = -1;
      if (points[0].Long() > points[1].Long()) {
	winner_team = 0;
      } else if (points[1].Long() > points[0].Long()) {
	winner_team = 1;
      } // else draw

      if (winner_team >= 0) {
	if (games_won.find(teams[winner_team].String()) != games_won.end()) {
	  games_won[teams[winner_team].String()] += 1;
	} else {
	  games_won[teams[winner_team].String()]  = 1;
	}
      }

      r->append_body("<tr><td><b>%s vs. %s</b></td>\n",
		     teams[0].String().c_str(), teams[1].String().c_str());

      r->append_body("<td style=\"background-color: #aaaaaa\"><span style=\"color: #00ffff\">%u</span> "
		     ": <span style=\"color: #ff00ff\">%u</span></td>\n",
		     points[0].Long(), points[1].Long());

      r->append_body("<td><i>Exploration:</i> %ld</td>\n",
		     doc["phase-points-cyan"]["EXPLORATION"].Long());
      r->append_body("<td><i>Production:</i> %ld</td>\n",
		     doc["phase-points-cyan"]["PRODUCTION"].Long());

      r->append_body("<td><i>Exploration:</i> %ld</td>\n",
		     doc["phase-points-magenta"]["EXPLORATION"].Long());
      r->append_body("<td><i>Production:</i> %ld</td>\n",
		     doc["phase-points-magenta"]["PRODUCTION"].Long());

      *r += "</tr>\n";

    }

    *r += "</table>\n";

    *r +=
      "<h4>Tournament Statistics</h4>\n"
      "<table>\n"
      "<tr><th>Team</th><th>Wins</th></tr>\n";
    for (const auto &g : games_won) {
      r->append_body("<tr><td><i>%s</i></td><td>%u</td></tr>\n",
		     g.first.c_str(), g.second);
    }
    *r += "</table>\n";

    return r;
    /*
    } else {
      return new WebErrorPageReply(WebReply::HTTP_NOT_FOUND, "Unknown request");
    }
    */
  } else {
    return NULL;
  }
}
