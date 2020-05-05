#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

class Utils{
 public:

  Utils() = default;
 
  ~Utils() = default;
 
  const std::vector<std::vector<int>>& GetMap(const std::string& map_name);

  const std::vector<std::vector<int>>& GetMap(){return map_;};

 private:
  const std::string GetAbsolutePath(const std::string& map_name) const;
  // convert relative path to absolute path in the system

  void ParseMap(const std::string & path_to_map);

  std::vector<std::vector<int>> map_ = {};

};

#endif