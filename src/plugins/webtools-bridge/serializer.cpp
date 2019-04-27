/***************************************************************************
 *  serliazer.cpp - Serialize to RosBridge Protocol JSON Messages.
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

#include "serializer.h"

#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

using namespace rapidjson;

/** @class Serializer "serializer.h"
 * This class is used to serilze the differnt JSON messages of
 * the Rosbridge protocol
 */

/** Create an subscribe message
 * This creates a RosBridge "subscribe" JSON message (Ex,
 * message formate:
 * @code
 * { "op": "subscribe",
 *  (optional) "id": <string>,
 *  "topic": <string>,
 *  (optional) "type": <string>,
 *  (optional) "throttle_rate": <int>,
 *  (optional) "queue_length": <int>,
 *  (optional) "fragment_size": <int>,
 *  (optional) "compression": <string>
 * }
 * @endcode
 * @param prefixed_topic_name
 * @param id provided in the JSON message of the rosbridge protocol
 * @param type provided in the JSON message of the rosbridge protocol
 * @param compression provided in the JSON message of the rosbridge protocol
 * @param throttle_rate provided in the JSON message of the rosbridge protocol
 * @param queue_length provided in the JSON message of the rosbridge protocol
 * @param fragment_size provided in the JSON message of the rosbridge protocol
 * @return serialized json string corresponding to rosbridge protocol message
 */
std::string
Serializer::op_subscribe(std::string  prefixed_topic_name,
                         std::string  id,
                         std::string  type,
                         std::string  compression,
                         unsigned int throttle_rate,
                         unsigned int queue_length,
                         unsigned int fragment_size)
{
	std::string op = "subscribe";

	StringBuffer         s;
	Writer<StringBuffer> writer(s);
	writer.StartObject();

	writer.String("op");
	writer.String(op.c_str(), (SizeType)op.length());

	writer.String("id");
	writer.String(id.c_str(), (SizeType)id.length());

	writer.String("type");
	writer.String(type.c_str(), (SizeType)type.length());

	writer.String("topic");
	writer.String(prefixed_topic_name.c_str(), (SizeType)prefixed_topic_name.length());

	writer.String("throttle_rate");
	writer.Uint(throttle_rate);

	writer.String("queue_length");
	if (queue_length < 1)
		queue_length = 1;
	writer.Uint(queue_length);

	if (fragment_size != 0) {
		writer.String("fragment_size");
		writer.Uint(fragment_size);
	}

	writer.String("compression");
	writer.String(compression.c_str(), (SizeType)compression.length());

	writer.EndObject(); // End of complete Json_msg

	////std::cout << s.GetString() << std::endl;

	return s.GetString();
}

/** Create an unsubscribe message
 * This creates a RosBridge "unsubscribe" JSON message (Ex,
 * message formate:
 * @code
 * { "op": "unsubscribe",
 * (optional) "id": <string>,
 *  "topic": <string>
 * }
 * @endcode
 * @param prefixed_topic_name
 * @param id
 * @return serialized json string corresponding to rosbridge protocol message
 */
std::string
Serializer::op_unsubscribe(std::string prefixed_topic_name, std::string id)
{
	std::string op = "unsubscribe";

	StringBuffer         s;
	Writer<StringBuffer> writer(s);
	writer.StartObject();

	writer.String("op");
	writer.String(op.c_str(), (SizeType)op.length());

	writer.String("id");
	writer.String(id.c_str(), (SizeType)id.length());

	writer.String("topic");
	writer.String(prefixed_topic_name.c_str(), (SizeType)prefixed_topic_name.length());

	writer.EndObject(); // End of complete Json_msg

	////std::cout << s.GetString() << std::endl;

	return s.GetString();
}

/** Create an advertise message
 * This creates a RosBridge "advertise" JSON message (Ex,
 * message formate:
 * @code
 * { "op": "advertise",
 *  (optional) "id": <string>,
 *  "topic": <string>,
 *  "type": <string>
 * }
 * @endcode
 * @param prefixed_topic_name the session wishes to advertise
 * @param id specified in the JSON message
 * @param type  specified in the  JSON message
 * @return serialized json string corresponding to rosbridge protocol message
 */
std::string
Serializer::op_advertise(std::string prefixed_topic_name, std::string id, std::string type)
{
	std::string op = "advertise";

	StringBuffer         s;
	Writer<StringBuffer> writer(s);
	writer.StartObject();

	writer.String("op");
	writer.String(op.c_str(), (SizeType)op.length());

	writer.String("id");
	writer.String(id.c_str(), (SizeType)id.length());

	writer.String("topic");
	writer.String(prefixed_topic_name.c_str(), (SizeType)prefixed_topic_name.length());

	writer.String("type");
	writer.String(type.c_str(), (SizeType)type.length());

	writer.String("latch");
	writer.Bool(false);

	writer.String("queue_size");
	writer.Int(100);

	writer.EndObject(); // End of complete Json_msg

	////std::cout << s.GetString() << std::endl;

	return s.GetString();
}

/** Create an unadvertise message
 * This creates a JSON message with a RosBridge "unadvertise"  opcode
 * message formate:
 * @code
 * { "op": "unadvertise",
 *  (optional) "id": <string>,
 *  "topic": <string>
 * }
 * @endcode
 * @param prefixed_topic_name
 * @param id
 * @return serialized json string corresponding to rosbridge protocol message
 */
std::string
Serializer::op_unadvertise(std::string prefixed_topic_name, std::string id)
{
	std::string op = "unadvertise";

	StringBuffer         s;
	Writer<StringBuffer> writer(s);
	writer.StartObject();

	writer.String("op");
	writer.String(op.c_str(), (SizeType)op.length());

	writer.String("id");
	writer.String(id.c_str(), (SizeType)id.length());

	writer.String("topic");
	writer.String(prefixed_topic_name.c_str(), (SizeType)prefixed_topic_name.length());

	writer.EndObject(); // End of complete Json_msg

	////std::cout << s.GetString() << std::endl;

	return s.GetString();
}

/** Create an publish message
 * This creates a JSON message with a RosBridge "publish"  opcode
 * message formate:
 * @code
 *{ "op": "publish",
 *  (optional) "id": <string>,
 *  "topic": <string>,
 *  "msg": <json>
 *}
 * @endcode
 * @param prefixed_topic_name
 * @param id
 * @param latch
 * @param msg_in_json The topic data in JSON
 * @return serialized json string corresponding to rosbridge protocol message
 */
std::string
Serializer::op_publish(std::string prefixed_topic_name,
                       std::string id,
                       bool        latch,
                       std::string msg_in_json)
{
	std::string op = "publish";

	StringBuffer         s;
	Writer<StringBuffer> writer(s);
	writer.StartObject();

	writer.String("op");
	writer.String(op.c_str(), (SizeType)op.length());

	writer.String("id");
	writer.String(id.c_str(), (SizeType)id.length());

	writer.String("topic");
	writer.String(prefixed_topic_name.c_str(), (SizeType)prefixed_topic_name.length());

	writer.String("latch");
	writer.Bool(latch);

	writer.String("msg");
	Document d;

	if (d.Parse(msg_in_json.c_str()).HasParseError()) {
		////std::cout<< GetParseError_En(d.GetParseError());
		return "";
		// throw
	}
	d.Parse(msg_in_json.c_str());
	d.Accept(writer);

	writer.EndObject(); // End of complete Json_msg

	////std::cout << s.GetString() << std::endl;

	return s.GetString();
}

// Serialier::append_json(	Writer<StringBuffer> &writer, std::string
// json_str)
// {
// 	Document  d;

// 	if (d.Parse(json_str.c_str()).HasParseError())
// 	{
// 		////std::cout<< GetParseError_En(d.GetParseError());
// 		return "";
// 		//throw
// 	}
// 	d.Parse(json_str.c_str());
// 	d.Accept(writer);
// }

/** Serialize a Clips Fact as a JSON msg
 * @param fact clips fact to serialize
 * @return serialized json string corresponding to rosbridge protocol message
 */
std::string
Serializer::serialize(CLIPS::Fact::pointer fact)
{
	StringBuffer         s;
	Writer<StringBuffer> writer(s);
	writer.StartObject();

	std::string name = fact->get_template()->name();
	writer.String("name"); // key for fact_name
	writer.String(name.c_str(), (SizeType)name.length());

	std::vector<std::string> slot_names = fact->slot_names();
	if (slot_names.size() > 0) // is it a non-ordered fact
	{
		for (std::vector<std::string>::iterator it = slot_names.begin();
		     slot_names.size() > 0 && it != slot_names.end();
		     it++) {
			writer.String(it->c_str(),
			              (SizeType)it->length()); // write slot name as a key for JOSN pair

			CLIPS::Values values     = fact->slot_value(*it);
			bool          multifield = values.size() > 0;
			if (multifield)
				writer.StartArray(); // write values of slot as a json array only if
				                     // slot is multi field
			for (CLIPS::Values::iterator it2 = values.begin(); it2 != values.end(); it2++) {
				switch (it2->type()) {
				case CLIPS::TYPE_INTEGER: writer.Int(it2->as_integer());
				case CLIPS::TYPE_FLOAT: writer.Double(it2->as_float());
				default: writer.String(it2->as_string().c_str(), (SizeType)it2->as_string().length());
				}
			}
			if (multifield)
				writer.EndArray();
		}
	} else {                   // ordered fact
		writer.String("fields"); // write the json pair key that will be used to
		                         // reference the fields of the fact
		CLIPS::Values values = fact->slot_value("");
		writer.StartArray(); // field values will be stored in a json array
		for (CLIPS::Values::iterator it = values.begin(); it != values.end(); it++) {
			switch (it->type()) {
			case CLIPS::TYPE_INTEGER: writer.Int(it->as_integer());
			case CLIPS::TYPE_FLOAT: writer.Double(it->as_float());
			default: writer.String(it->as_string().c_str(), (SizeType)it->as_string().length());
			}
		}
		writer.EndArray();
	}

	writer.EndObject();

	return s.GetString();
}
