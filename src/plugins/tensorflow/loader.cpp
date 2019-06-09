/***************************************************************************
 *  loader.cpp - General loader
 *
 *  Created: Thu May 5 10:23:50 2019
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

#include "loader.h"

TF_Plugin_Loader::TF_Plugin_Loader(std::string name, fawkes::Logger *logger)
    : name_(name), logger_(logger) {
  this->logger_->log_debug(name_.c_str(), "New TF_Plugin_Loader loaded");
}

TF_Plugin_Loader::~TF_Plugin_Loader() {
  this->logger_->log_debug(name_.c_str(), "TF_Plugin_Loader destroyed");
}
