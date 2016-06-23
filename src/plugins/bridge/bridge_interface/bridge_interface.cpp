
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
#include "bridge_processor.h"




char const* greet()
{

       return "hello, worlfdgsdfgsdfgsdfgd";
}
 

char const* makecall()
{
	
       return BridgeProcessor::TestString();
}


  
BOOST_PYTHON_MODULE(bridge_interface)
{
      using namespace boost::python;
          def("greet", greet);
          def("makecall", make);
}
