/** @file
 * Defines the findConfigFile() function, which finds the robot's configuration
 * file.
 * @author Ian Phillips <ianfp@freeshell.org>
 */
 
 
#include <sb_config/config_file.h>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/program_options.hpp>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace bfs = boost::filesystem;
namespace bpo = boost::program_options;
using std::string;

namespace sb_config
{
  /** The environment variable that can be used to specify the config file. */
  const string CFG_ENV_VAR = "SB_CONFIG_FILE";

  /** The name of this package. */
  const string THIS_PACKAGE = "sb_config";

  /** The name of the default config file. */
  const string DEFAULT_CFG_FILE = "robot.cfg";

  /** Check the command-line arguments for a --config-file (or -c) switch.
   * If it is there, see if the file specified after is a valid file.  If
   * so, that's our config file. */
  static bool checkArgs(int argc, char** argv, string& result)
  {
    // Describe command-line args we are looking for.
    bpo::options_description desc;
    desc.add_options()
      ("config-file,c", bpo::value<string>() );

    // Load args into memory.
    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);
    bpo::notify(vm);

    // If the arg we want is specified...
    if ( vm.count("config-file") )
    {
      bfs::path cfgfile( vm["config-file"].as<string>() );
      // ... and if it is a valid file...
      if ( bfs::exists(cfgfile) )
      {
        // ... then it's our config file.
        result = cfgfile.string();
        return true;
      }
    }
    return false;
  }

  /** Check the environment for a config file. */
  static bool checkEnv(string& result)
  {
    // See if our environment variable exists.
    char* env_var = getenv(CFG_ENV_VAR.c_str());
    if (env_var) {
      bfs::path cfgfile(env_var);
      // If it does, make sure the file path is valid.
      if ( bfs::exists(cfgfile) ) {
        result = cfgfile.string();
        return true;
      }
    }
    return false;
  }

  /** Look for the existence of a default config file at
   * sb_config/cfg/robot.cfg. */
  static bool checkDefault(string& result)
  {
    // First, we have to find the sb_config package, because there is
    // no API for rospack.
    if (! getenv("ROS_PACKAGE_PATH") ) {
      return false;
    }
    string package_path = getenv("ROS_PACKAGE_PATH");
    
    // Split the ROS_PACKAGE_PATH into a vector of stack names.
    std::vector<string> stacks;
    boost::split(stacks, package_path, boost::is_any_of(":"));

    // For each stack in ROS_PACKAGE_PATH
    for ( std::vector<string>::iterator dirp = stacks.begin();
          dirp != stacks.end();
          ++dirp )
    {
      bfs::path stack(*dirp);
      // If the stack directory exists...
      if ( bfs::exists(stack) ) {
        // ... loop through directory contents
        bfs::directory_iterator end;
        for ( bfs::directory_iterator itr(stack); itr != end; ++itr ) {
          bfs::path package(*itr);
          // If the sb_config package is found...
          if ( bfs::is_directory(package) && 
               THIS_PACKAGE.compare( package.leaf() ) == 0 ) 
          {
            // ... see if it contains the default config file.
            bfs::path filename = package / "cfg" / DEFAULT_CFG_FILE;
            if ( bfs::exists(filename) ) {
              result = filename.string();
              return true;
            }
          }
        } // end package loop
      }
    } // end stack loop
    return false;
  }


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
  bool findConfigFile(int argc, char** argv, string& result)
  {
    if      ( checkArgs(argc, argv, result) ) return true;
    else if ( checkEnv(result) ) return true;
    else if ( checkDefault(result) ) return true;
    else return false;
  }

} // end namespace sb_config

