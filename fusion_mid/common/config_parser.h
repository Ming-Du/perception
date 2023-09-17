#ifndef _CONFIG_H
#define _CONFIG_H

#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>
#include <string>
namespace perception {
namespace mid_common {

class ConfigParser {
 private:
  boost::property_tree::ptree configPT;

 public:
  ConfigParser(char* filePath) { boost::property_tree::read_ini(filePath, configPT); }

  ConfigParser(std::string& filePath) { boost::property_tree::read_ini(filePath, configPT); }

  ~ConfigParser() {}

  std::string getString(std::string& path) { return configPT.get<std::string>(path); }

  int getInt(std::string& path) { return configPT.get<int>(path); }

  double getDouble(std::string& path) { return configPT.get<double>(path); }

  std::string getString(const char* path) { return configPT.get<std::string>(path); }

  int getInt(const char* path) { return configPT.get<int>(path); }

  double getDouble(const char* path) { return configPT.get<double>(path); }

  int getBool(const char* path) { return configPT.get<bool>(path); }
};

typedef boost::shared_ptr<ConfigParser> ConfigParserPtr;

}  // namespace mid_common
}  // namespace perception
#endif
