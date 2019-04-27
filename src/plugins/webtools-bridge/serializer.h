/***************************************************************************
 *  serliazer.h - Serialize to RosBridge Protocol  JSON Messages.
 *  Created: 2016
 *  Copyright  2016 Mostafa Gomaa
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

#ifndef _UTIL_SERIALIZER_
#define _UTIL_SERIALIZER_

#include <clipsmm.h>
#include <string>

class Serializer
{
public:
	static std::string op_subscribe(std::string  topic_name,
	                                std::string  id,
	                                std::string  type,
	                                std::string  compression,
	                                unsigned int throttle_rate,
	                                unsigned int queue_length,
	                                unsigned int fragment_size);

	static std::string op_unsubscribe(std::string topic_name, std::string id);

	static std::string op_advertise(std::string topic_name, std::string id, std::string type);
	static std::string op_unadvertise(std::string topic_name, std::string id);
	static std::string
	op_publish(std::string topic_name, std::string id, bool latch, std::string msg_in_json);

	static std::string serialize(CLIPS::Fact::pointer fact);
};

#endif