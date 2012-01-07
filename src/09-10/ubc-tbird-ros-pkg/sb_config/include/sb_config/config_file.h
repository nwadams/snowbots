#ifndef SB_CONFIG_CONFIG_FILE_H
#define SB_CONFIG_CONFIG_FILE_H

#include <string>

namespace sb_config
{
  /** Searches for a config file in the command-line arguments,
   * environment, and finally in a default location.  WARNING:
   * if no valid config file is found, the value of the result
   * parameter is unspecified!  Always make sure to check the return
   * value!
   * @param argc Argument count
   * @param argv Argument list
   * @param result std::string reference to store the path, if any
   * @return true if a config file is found, false otherwise.
   */
  bool findConfigFile(int argc, char** argv, std::string& result);

} // end namespace sb_config


# endif // SB_CONFIG_CONFIG_FILE_H

