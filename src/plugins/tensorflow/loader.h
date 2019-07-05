/***************************************************************************
 *  loader.h - General loader
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

#ifndef LOADER_H
#define LOADER_H

#include <logging/logger.h>
#include <string>

/** Class for general loading of data into the input of the graph
 */
class TF_Plugin_Loader {
public:
  /** Constructor
   * @param name The name of the calling thread, used for logging purposes
   * @param logger Logger of the calling thread, used for logging purposes
   */
  TF_Plugin_Loader(std::string name, fawkes::Logger *logger);
  /** Destructor
   */
  virtual ~TF_Plugin_Loader();

  /** Function to read the data
   * @return Pointer to the data buffer
   */
  virtual const void *read() = 0;
  /** Function to clean up after the buffer returned by read() was used
   */
  virtual void post_read() = 0;
  /** Function to verify the objects integrity
   * This function should be called exactly once after creating a new Loader
   * @return true if object is ok, false if not
   */
  virtual bool verify() { return true; }

protected:
  /** Store the name of the calling thread */
  std::string name_;
  /** Logger of the calling thread */
  fawkes::Logger *logger_;

private:
};

#endif
