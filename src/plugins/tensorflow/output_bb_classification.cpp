/***************************************************************************
 *  outputter_bb_classification.cpp - General outputter of data
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

#include "output_bb_classification.h"
#include <fstream>
#include <limits>

TF_Plugin_Outputter_BB_Classification::TF_Plugin_Outputter_BB_Classification(std::string name, fawkes::Logger *logger, fawkes::BlackBoard *blackboard, std::string classification_table_file) : TF_Plugin_Outputter(name, logger), blackboard_(blackboard)
{
  // read classification file
  std::ifstream classification_table_file_handler(classification_table_file.c_str());
  while(!classification_table_file_handler.eof()) {
    classification_table_.push_back("");
    std::getline(classification_table_file_handler,classification_table_[classification_table_.size()-1]);
  }

  classification_if_ = blackboard_->open_for_writing<fawkes::ClassificationResultInterface>("TensorflowResult");

}

TF_Plugin_Outputter_BB_Classification::~TF_Plugin_Outputter_BB_Classification() {
  blackboard_->close(classification_if_);
}

void TF_Plugin_Outputter_BB_Classification::write(const float *data) {
  float max_prob = std::numeric_limits<float>::lowest();
  size_t index_max_prob = -1;
  for(size_t i=0;i<classification_table_.size();++i){
    if(data[i]>=max_prob){
      max_prob=data[i];
      index_max_prob = i;
    }
  }
  classification_if_->set_probability(max_prob);
  classification_if_->set_result(classification_table_[index_max_prob].c_str());
  classification_if_->write();
}

