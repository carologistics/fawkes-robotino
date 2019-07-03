/***************************************************************************
 *  outputter.h - General outputter of data
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

#ifndef OUTPUTTER_H
#define OUTPUTTER_H

#include <logging/logger.h>
#include <string>

/** Class for general outputting of data from the output of the graph
 */
class TF_Plugin_Outputter {
  public:
  /** Constructor
   * @param name The name of the calling thread, used for logging purposes
   * @param logger Logger of the calling thread, used for logging purposes
   */
    TF_Plugin_Outputter(std::string name, fawkes::Logger *logger);
  /** Destructor
   */
    virtual ~TF_Plugin_Outputter();

    /** Function to output the data in some way 
     * @param data A pointer to the data which shall be output (should get deep copied)
     */
    virtual void write(const float *data){};
    /** Function to output the data in some way 
     * @param data A pointer to the data which shall be output (should get deep copied)
     */
    virtual void write(const int *data){};
    // TODO add more dtypes

  protected:
    std::string name_;
    fawkes::Logger *logger_;
};

#endif
