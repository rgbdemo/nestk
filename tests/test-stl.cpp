
#include <ntk/core.h>
#include <ntk/utils/stl.h>
#include <map>
#include <iostream>
#include <string>

using namespace ntk;

int main()
{
  std::map<std::string,std::string> m;
  m["hello"] = "bonjour";
  m["bye"] = "au revoir";
  
  {
    key_iterator< std::map<std::string,std::string>::const_iterator > it (m.begin());
    for (; it != m.end(); ++it)
      std::cout << it->c_str() << std::endl;
  }
  
  {
    value_iterator< std::map<std::string,std::string>::const_iterator > it (m.begin());
    for (; it != m.end(); ++it)
      std::cout << (*it).c_str() << std::endl;
  }
  
  return 0;
}

