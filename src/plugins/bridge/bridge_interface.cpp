
/***************************************************************************
 *  BridgeInterface.cpp - Interface the FawkesBridge API to python
 *
 *  Copyright  2015 Mostafa Gomaa
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


#include <boost/python.hpp>

#include "bridge_interface.h"
//#include "bridge_processor.h"


using namespace boost::python;

	World::World(std::string msg): msg(msg) {}
    void World::set(std::string msg) { this->msg = msg; }
    std::string World::greet() { return msg; }

   // int getTestInt() { 
   // 	int d=BridgeProcessor::TestInt();
   // 	d++;ls
   // 	return d;}


BOOST_PYTHON_MODULE(bridge_interface)
{
      
      class_<World>("World", init<std::string>())
        .def("greet", &World::greet)
        .def("set", &World::set)
    ;
}