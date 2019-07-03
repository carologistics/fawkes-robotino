/***************************************************************************
 *  outputter.cpp - General outputter of data
 *
 *  Created: Wed Jul 3 10:23:50 2019
 *  Copyright  2019 Morian Sonnet
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

#include "outputter.h"

TF_Plugin_Outputter::TF_Plugin_Outputter(std::string name, fawkes::Logger *logger) : name_(name), logger_(logger) {}

TF_Plugin_Outputter::~TF_Plugin_Outputter() {}
