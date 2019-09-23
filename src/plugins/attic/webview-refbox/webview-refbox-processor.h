
/***************************************************************************
 *  webview-refbox-processor.h - RCLL refbox webview
 *
 *  Created: Tue Mar 17 17:56:18 2015
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

#ifndef __PLUGINS_WEBVIEW_REFBOX_WEBVIEW_REFBOX_PROCESSOR_H_
#define __PLUGINS_WEBVIEW_REFBOX_WEBVIEW_REFBOX_PROCESSOR_H_

#include <webview/request_processor.h>

#include <map>
#include <string>
#include <tuple>

namespace mongo {
class DBClientBase;
class BSONObj;
} // namespace mongo

class WebviewRCLLRefBoxRequestProcessor : public fawkes::WebRequestProcessor
{
public:
	WebviewRCLLRefBoxRequestProcessor(std::string base_url, mongo::DBClientBase *mongodb_client);
	virtual ~WebviewRCLLRefBoxRequestProcessor();

	virtual fawkes::WebReply *process_request(const fawkes::WebRequest *request);

private:
	std::string gen_orders_table(const mongo::BSONObj *doc);

private:
	mongo::DBClientBase *mongodb_;

	std::string baseurl_;
};

#endif
