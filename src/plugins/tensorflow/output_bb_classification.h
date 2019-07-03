/***************************************************************************
 *  outputter_bb_classification.h - General outputter of data
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

#ifndef OUTPUT_BB_CLASSIFICATION_H
#define OUTPUT_BB_CLASSIFICATION_H

#include "outputter.h"
#include <logging/logger.h>
#include <string>
#include <vector>
#include <blackboard/blackboard.h>
#include <interfaces/ClassificationResultInterface.h>

class TF_Plugin_Outputter_BB_Classification : public TF_Plugin_Outputter {
  public: 
    TF_Plugin_Outputter_BB_Classification(std::string name, fawkes::Logger *logger, fawkes::BlackBoard *blackboard, std::string classification_table_file);

    ~TF_Plugin_Outputter_BB_Classification();

    virtual void write(const float *data) override;

  private:
    fawkes::BlackBoard *blackboard_;
    std::vector<std::string> classification_table_;
    fawkes::ClassificationResultInterface *classification_if_;

};


#endif
