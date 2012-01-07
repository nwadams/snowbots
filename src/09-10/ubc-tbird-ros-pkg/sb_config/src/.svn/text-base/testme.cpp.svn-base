#include <sb_config/config_file.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
  string answer;
  if (sb_config::findConfigFile(argc, argv, answer)) {
    cout << "I found it in " << answer << endl;
  }
  else {
    cout << "I don't know where it is!" << endl;
  }
  return 0;
}
