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
