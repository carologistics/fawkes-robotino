
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
#include <mongo/client/dbclient.h>
#include <webview/error_reply.h>
#include <webview/page_reply.h>
#include <webview/redirect_reply.h>

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
WebviewRCLLRefBoxRequestProcessor::WebviewRCLLRefBoxRequestProcessor(
  std::string          base_url,
  mongo::DBClientBase *mongodb_client)
{
	mongodb_ = mongodb_client;
}

/** Destructor. */
WebviewRCLLRefBoxRequestProcessor::~WebviewRCLLRefBoxRequestProcessor()
{
}

std::string
WebviewRCLLRefBoxRequestProcessor::gen_orders_table(const mongo::BSONObj *doc)
{
	std::string rv = "<table>"
	                 "<tr><th>ID</th><th>Product</th>"
	                 "<th>Requested</th><th>Delivered</th><th "
	                 "colspan=\"2\">Period</th></tr>\n";

	std::vector<BSONElement> orders = (*doc)["orders"].Array();
	for (const BSONElement &o : orders) {
		std::vector<BSONElement> del_period  = o["delivery-period"].Array();
		std::vector<BSONElement> quant_deliv = o["quantity-delivered"].Array();
		std::string              base_color  = o["base-color"].String().substr(5);
		std::transform(base_color.begin(), base_color.end(), base_color.begin(), ::tolower);
		std::string cap_color = o["cap-color"].String().substr(4);
		std::transform(cap_color.begin(), cap_color.end(), cap_color.begin(), ::tolower);
		std::string prod_string = std::string("<div>")
		                          + "<div style=\"float: left; width: 8px; "
		                            "height:12px; background-color: "
		                          + base_color + "\"></div>";

		if (o.Obj().hasField("ring-colors")) {
			std::vector<BSONElement> rings = o["ring-colors"].Array();
			for (const BSONElement &r : rings) {
				const std::string ring_color = r.String().substr(5);
				prod_string += "<div style=\"float: left; margin-left: 1px; "
				               "width: 6px; height:12px; background-color: "
				               + ring_color + "\"></div>" + "</div>";
			}
		}

		prod_string += "<div style=\"float: left; margin-left: 1px; "
		               "width: 4px; height:12px; background-color: "
		               + cap_color + "\"></div>" + "</div>";

		rv += "<tr><td>" + std::to_string(o["id"].Long()) + "</td><td>" + prod_string + "</td>" + "<td>"
		      + std::to_string(o["quantity-requested"].Long()) + "</td>"
		      + "<td><span style=\"color: #88ffff; weight: bold;\">"
		      + std::to_string(quant_deliv[0].Long())
		      + "</span> / "
		        "<span style=\"color: #ff88ff; weight: bold;\">"
		      + std::to_string(quant_deliv[1].Long()) + "</span></td>" + "<td>"
		      + std::to_string(del_period[0].Long()) + "-" + std::to_string(del_period[1].Long())
		      + "</td>" + "<td>(Act. " + std::to_string(o["activate-at"].Long()) + ")</td></tr>";
	}
	rv += "</table>";
	return rv;
}

WebReply *
WebviewRCLLRefBoxRequestProcessor::process_request(const fawkes::WebRequest *request)
{
	if (request->url().find(baseurl_) == 0) {
		std::string subpath = request->url().substr(baseurl_.length());

		WebPageReply *r = new WebPageReply("RCLL RefBox");
		r->set_html_header("  <link type=\"text/css\" href=\"/static/css/jqtheme/"
		                   "jquery-ui.custom.css\" rel=\"stylesheet\" />\n"
		                   "  <script type=\"text/javascript\" src=\"/static/js/"
		                   "jquery.min.js\"></script>\n"
		                   "  <script type=\"text/javascript\" src=\"/static/js/"
		                   "jquery-ui.custom.min.js\"></script>\n");
		*r += "<style type=\"text/css\">\n"
		      "  .game-link { color: black; }\n"
		      "</style>";

		*r += "<h2>RCLL RefBox Database Report</h2>\n";

		time_t query_time = time(0);
		query_time -= 24 * 3600;

		BSONObj query = BSON("start-time" << BSONObjBuilder().appendTimeT("$gt", query_time).obj()
		                                  << "end-time" << BSON("$exists" << false));

		// check if there is a running game
		std::shared_ptr<DBClientCursor> cursor =
		  mongodb_->query("llsfrb.game_report", Query(query).sort("start-time", -1), 1);

		if (cursor->more()) {
			BSONObj                  doc    = cursor->next();
			std::vector<BSONElement> teams  = doc["teams"].Array();
			std::vector<BSONElement> points = doc["total-points"].Array();
			r->append_body("<h3>Active Game %s vs. %s</h3>\n",
			               teams[0].String().c_str(),
			               teams[1].String().c_str());

			*r += "<table>\n";
			r->append_body("<tr><td><b>Started at:</b></td>\n"
			               "<td>%s</td></tr>\n",
			               doc["start-time"].Date().toString().c_str());
			r->append_body("<tr><td><b>Points:</b></td>\n"
			               "<td><span style=\"background-color: #88ffff\">%u</span> "
			               ": <span style=\"background-color: #ff88ff\">%u</span></td></tr>\n",
			               points[0].Long(),
			               points[1].Long());

			r->append_body("<tr><td><b>Points Cyan:</b></td>\n"
			               "<td><i>Exploration:</i> %ld</td></tr>\n",
			               doc["phase-points-cyan"]["EXPLORATION"].numberLong());
			r->append_body("<tr><td></td>\n"
			               "<td><i>Production:</i> %ld</td></tr>\n",
			               doc["phase-points-cyan"]["PRODUCTION"].numberLong());

			r->append_body("<tr><td><b>Points Magenta:</b></td>\n"
			               "<td><i>Exploration:</i> %ld</td></tr>\n",
			               doc["phase-points-magenta"]["EXPLORATION"].numberLong());
			r->append_body("<tr><td></td>\n"
			               "<td><i>Production:</i> %ld</td></tr>\n",
			               doc["phase-points-magenta"]["PRODUCTION"].numberLong());

			r->append_body("<tr><td><b>Orders:</b></td>\n"
			               "<td>%s</td></tr>\n",
			               gen_orders_table(&doc).c_str());

			*r += "</table>\n";
		}

		*r += "<h3>Previous Games</h3>\n";

		// check if there is a running game
		cursor = mongodb_->query("llsfrb.game_report",
		                         QUERY("end-time" << BSON("$exists" << true)).sort("start-time"));

		*r += "<table>\n"
		      "<tr><th>Teams</th><th colspan=\"2\">Points</th><th "
		      "colspan=\"2\">Points Cyan</th>"
		      "<th colspan=\"2\">Points Magenta</th></tr>\n";

		std::list<std::string> reports;

		while (cursor->more()) {
			BSONObj                  doc    = cursor->next();
			std::vector<BSONElement> teams  = doc["teams"].Array();
			std::vector<BSONElement> points = doc["total-points"].Array();

			int winner_team = -1;
			if (points[0].numberLong() > points[1].numberLong()) {
				winner_team = 0;
			} else if (points[1].numberLong() > points[0].numberLong()) {
				winner_team = 1;
			} // else draw

			reports.push_back(doc["_id"].OID().str());

			r->append_body("<tr><td><a id=\"game-%s\" href=\"#\" class=\"game-link\">"
			               "<img id=\"game-%s-icon\" class=\"game-icon\" "
			               "src=\"/static/images/icon-triangle-e.png\" />"
			               " %s%s%s vs. %s%s%s</a></td>\n",
			               doc["_id"].OID().str().c_str(),
			               doc["_id"].OID().str().c_str(),
			               winner_team == 0 ? "<b>" : "",
			               teams[0].String().c_str(),
			               winner_team == 0 ? "</b>" : "",
			               winner_team == 1 ? "<b>" : "",
			               teams[1].String().c_str(),
			               winner_team == 1 ? "</b>" : "");

			r->append_body("<td style=\"background-color: #88ffff; text-align: center;\">%u</td>"
			               "<td style=\"background-color: #ff88ff; text-align: "
			               "center;\">%u</td>\n",
			               points[0].numberLong(),
			               points[1].numberLong());

			r->append_body("<td><i>Exploration:</i> %ld</td>\n",
			               doc["phase-points-cyan"]["EXPLORATION"].numberLong());
			r->append_body("<td><i>Production:</i> %ld</td>\n",
			               doc["phase-points-cyan"]["PRODUCTION"].numberLong());

			r->append_body("<td><i>Exploration:</i> %ld</td>\n",
			               doc["phase-points-magenta"]["EXPLORATION"].numberLong());
			r->append_body("<td><i>Production:</i> %ld</td>\n",
			               doc["phase-points-magenta"]["PRODUCTION"].numberLong());

			*r += "</tr>\n";

			r->append_body("<tr id=\"game-%s-time\">"
			               "<td valign=\"top\">Game Time</td>"
			               "<td colspan=\"6\">"
			               "%s to %s</td></tr>\n",
			               doc["_id"].OID().str().c_str(),
			               doc["start-time"].Date().toString().c_str(),
			               doc["end-time"].Date().toString().c_str());

			r->append_body("<tr id=\"game-%s-desc\">"
			               "<td valign=\"top\"><div style=\"margin-top: 12px\">Events</div></td>"
			               "<td colspan=\"6\"><table>"
			               "<tr><th>Team</th><th>Points</th><th>Game "
			               "Time</th><th>Reason</th></tr>\n",
			               doc["_id"].OID().str().c_str());
			std::vector<BSONElement> pointinfo = doc["points"].Array();
			for (const BSONElement &pd : pointinfo) {
				r->append_body("<tr><td style=\"background-color: "
				               "#%s\">%s</td><td>%lu</td><td>%.2f</td><td>%s</td></tr>\n",
				               pd["team"].String() == "CYAN" ? "88ffff" : "ff88ff",
				               pd["team"].String().c_str(),
				               pd["points"].numberLong(),
				               pd["game-time"].Double(),
				               pd["reason"].String().c_str());
			}
			*r += "</table></td></tr>\n";

			if (doc.hasField("orders")) {
				r->append_body("<tr id=\"game-%s-orders\">"
				               "<td valign=\"top\"><div style=\"margin-top: "
				               "12px\">Orders</div></td>"
				               "<td colspan=\"6\">%s</td></tr>\n",
				               doc["_id"].OID().str().c_str(),
				               gen_orders_table(&doc).c_str());
			}

			if (doc.hasField("machines")) {
				r->append_body("<tr id=\"game-%s-machines\">"
				               "<td valign=\"top\"><div style=\"margin-top: "
				               "12px\">Machines</div></td>"
				               "<td colspan=\"6\"><table>"
				               "<tr><th>Name</th><th>Type</th><th>Team</th>"
				               "<th>Productions</th><th>Proc. Time</th></tr>\n",
				               doc["_id"].OID().str().c_str());
				std::vector<BSONElement> machines = doc["machines"].Array();
				for (const BSONElement &m : machines) {
					r->append_body("<tr><td>%s</td><td>%s</td><td>%s</td>"
					               "<td>%lu</td><td>%lu</td></tr>",
					               m["name"].String().c_str(),
					               m["type"].String().c_str(),
					               m["team"].String().c_str(),
					               m["productions"].Long(),
					               m.Obj().hasField("proc-time") ? m["proc-time"].Long() : 0);
				}
				*r += "</table></td></tr>\n";
			}
		}
		*r += "</table>\n";

		*r += "<script type=\"text/javascript\">\n"
		      "  $(function(){\n";
		for (const std::string &gr : reports) {
			r->append_body("    $(\"#game-%s\").click(function(){\n"
			               "      if ( $(\"#game-%s-desc\").is(\":visible\") ) {\n"
			               "        $(\"#game-%s-time\").hide();\n"
			               "        $(\"#game-%s-desc\").hide();\n"
			               "        $(\"#game-%s-orders\").hide();\n"
			               "        $(\"#game-%s-machines\").hide();\n"
			               "        $(\"#game-%s-icon\").attr(\"src\", "
			               "\"/static/images/icon-triangle-e.png\");\n"
			               "      } else {\n"
			               "        $(\"#game-%s-time\").show();\n"
			               "        $(\"#game-%s-desc\").show();\n"
			               "        $(\"#game-%s-orders\").show();\n"
			               "        $(\"#game-%s-machines\").show();\n"
			               "        $(\"#game-%s-icon\").attr(\"src\", "
			               "\"/static/images/icon-triangle-s.png\");\n"
			               "      }\n"
			               "    });\n"
			               "    $(\"#game-%s-time\").hide();\n"
			               "    $(\"#game-%s-desc\").hide();\n"
			               "    $(\"#game-%s-orders\").hide();\n"
			               "    $(\"#game-%s-machines\").hide();\n",
			               gr.c_str(),
			               gr.c_str(),
			               gr.c_str(),
			               gr.c_str(),
			               gr.c_str(),
			               gr.c_str(),
			               gr.c_str(),
			               gr.c_str(),
			               gr.c_str(),
			               gr.c_str(),
			               gr.c_str(),
			               gr.c_str(),
			               gr.c_str(),
			               gr.c_str(),
			               gr.c_str(),
			               gr.c_str());
		}
		*r += "  });\n"
		      "</script>\n";

		// This set of function reduces to a collection with
		// a separate document per team
		const char *map_func = "function () {\n"
		                       "  if (this.teams[0] != \"\") {\n"
		                       "    emit(this.teams[0], {points: this[\"total-points\"][0],\n"
		                       "			        opp_points: "
		                       "this[\"total-points\"][1]});\n"
		                       "  }\n"
		                       "  if (this.teams[1] != \"\") {\n"
		                       "    emit(this.teams[1], {points: this[\"total-points\"][1],\n"
		                       "			 	opp_points: "
		                       "this[\"total-points\"][0]});\n"
		                       "	 }\n"
		                       "}";
		const char *red_func = "function(key, values) {\n"
		                       "	 rv = { games: 0, wins: 0, points: 0, opp_points: 0 };\n"
		                       "	 for (var i = 0; i < values.length; ++i) {\n"
		                       "	   rv.games += (\"games\" in values[i]) ? values[i].games : "
		                       "1;\n"
		                       "	   if (\"wins\" in values[i]) {\n"
		                       "	     rv.wins += values[i].wins;\n"
		                       "	   } else if (values[i].points > values[i].opp_points) {\n"
		                       "	     rv.wins += 1;\n"
		                       "	   }\n"
		                       "	   rv.points += values[i].points;\n"
		                       "	   rv.opp_points += values[i].opp_points;\n"
		                       "	 }\n"
		                       "	 return rv;\n"
		                       "}\n";

		/*
    // these functions reduce into a single document for all teams
    const char *map_func =
      "function () {\n"
      "  if (this.teams[0] != \"\") {\n"
      "	   value = {teams: [this.teams[0]]}\n"
      "	   value[this.teams[0]] = {points: this[\"total-points\"][0],\n"
      "	 		           opp_points: this[\"total-points\"][1]}\n"
      "    emit(\"stats\", value);\n"
      "	 }\n"
      "  if (this.teams[1] != \"\") {\n"
      "	   value = {teams: [this.teams[1]]}\n"
      "	   value[this.teams[1]] = {points: this[\"total-points\"][1],\n"
      "	                           opp_points: this[\"total-points\"][0]}\n"
      "    emit(\"stats\", value);\n"
      "	 }\n"
      "}\n";

    const char *red_func =
      "function(key, values) {\n"
      "  rv = { teams: [] };\n"
      "  for (var i = 0; i < values.length; ++i) {\n"
      "	   for (var t = 0; t < values[i].teams.length; ++t) {\n"
      "	     if (rv.teams.indexOf(values[i].teams[t]) == -1) {\n"
      "	       rv.teams.push(values[i].teams[t])\n"
      "	     }\n"
      "	     if (! (values[i].teams[t] in rv)) {\n"
      "	       rv[values[i].teams[t]] = { games: 0, wins: 0, points: 0,
    opp_points: 0 };\n" "	     }\n" " rv[values[i].teams[t]].games +=\n"
      "	       (\"games\" in values[i][values[i].teams[t]]) ?
    values[i][values[i].teams[t]].games : 1;\n" "	     if (\"wins\" in
    values[i][values[i].teams[t]]) {\n" " rv[values[i].teams[t]].wins +=
    values[i][values[i].teams[t]].wins;\n" "	     } else if (values[i].points
    > values[i][values[i].teams[t]].opp_points) {\n" "
    rv[values[i].teams[t]].wins += 1;\n" "	     }\n" "
    rv[values[i].teams[t]].points += values[i][values[i].teams[t]].points;\n" "
    rv[values[i].teams[t]].opp_points +=
    values[i][values[i].teams[t]].opp_points;\n" "	    }\n" "	 }\n" "
    return rv;\n"
      "}\n";
    */

		BSONObj out = mongodb_->mapreduce("llsfrb.game_report",
		                                  map_func,
		                                  red_func,
		                                  BSON("end-time" << BSON("$exists" << true)));

		*r += "<h4>Tournament Statistics</h4>\n"
		      "<table>\n"
		      "<tr><th>Team</th><th>Games</th><th>Wins</th><th>Points</th></tr>\n";
		std::vector<BSONElement> teams = out["results"].Array();
		for (const BSONElement &t : teams) {
			BSONObj tdoc = t["value"].Obj();
			r->append_body("<tr><td><i>%s</i></td><td>%u</td><td>%u</td><td>%d:%d "
			               "(%d)</td></tr>\n",
			               t["_id"].String().c_str(),
			               tdoc["games"].numberInt(),
			               tdoc["wins"].numberInt(),
			               tdoc["points"].numberInt(),
			               tdoc["opp_points"].numberInt(),
			               tdoc["points"].numberInt() - tdoc["opp_points"].numberInt());
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
