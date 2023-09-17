#ifndef _CONFIG_H
#define _CONFIG_H

/*
 * @brief:读取配置文件参数
 * 输入:配置文件路径,对应参数索引
 * 输出:对应参数数值
 * 过程:无
 */

#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>
#include <string>

namespace calib_common {

class ConfigParser {
 private:
  boost::property_tree::ptree configPT;

 public:
  /* @brief: 构造函数
   * @param [in]: filePath-配置文件路径及文件名,char[]型
   * @param [out]: NONE
   * @return NONE
   */
  ConfigParser(char* filePath) { boost::property_tree::read_ini(filePath, configPT); }

  /* @brief: 构造函数
   * @param [in]: filePath-配置文件路径及文件名,string型
   * @param [out]: NONE
   * @return NONE
   */
  ConfigParser(std::string& filePath) { boost::property_tree::read_ini(filePath, configPT); }

  /* @brief:析构函数
   * @param [in]: NONE
   * @param [out]: NONE
   * @return NONE
   */
  ~ConfigParser() {}

  /* @brief:获取string型变量
   * @param [in]: path-变量名
   * @param [out]: NONE
   * @return 变量参数值
   */
  std::string getString(std::string& path) { return configPT.get<std::string>(path); }

  /* @brief:获取int型变量
   * @param [in]: path-变量名,string型
   * @param [out]: NONE
   * @return 变量参数值
   */
  int getInt(std::string& path) { return configPT.get<int>(path); }

  /* @brief:获取double型变量
   * @param [in]: path-变量名,string型
   * @param [out]: NONE
   * @return 变量参数值
   */
  double getDouble(std::string& path) { return configPT.get<double>(path); }

  /* @brief:获取string型变量
   * @param [in]: path-变量名,char[]型
   * @param [out]: NONE
   * @return 变量参数值
   */
  std::string getString(const char* path) { return configPT.get<std::string>(path); }

  /* @brief:获取int型变量
   * @param [in]: path-变量名,char[]型
   * @param [out]: NONE
   * @return 变量参数值
   */
  int getInt(const char* path) { return configPT.get<int>(path); }

  /* @brief:获取double型变量
   * @param [in]: path-变量名,char[]型
   * @param [out]: NONE
   * @return 变量参数值
   */
  double getDouble(const char* path) { return configPT.get<double>(path); }

  /* @brief:获取bool型变量
   * @param [in]: path-变量名,char[]型
   * @param [out]: NONE
   * @return 变量参数值
   */
  int getBool(const char* path) { return configPT.get<bool>(path); }
};

typedef boost::shared_ptr<ConfigParser> ConfigParserPtr;

}  // namespace calib_common
#endif
