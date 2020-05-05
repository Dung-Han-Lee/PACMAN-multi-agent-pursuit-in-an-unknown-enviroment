#include "map_reader.h"

const std::string Utils::GetAbsolutePath(const std::string& map_name) const
{
  std::string current_path = __FILE__;
  std::size_t pos = current_path.find("src");
  return current_path.substr(0, pos) + "data/" + map_name;
}

void Utils::ParseMap(const std::string & path_to_map)
{
  // Read map file
  std::ifstream file;
  file.open(path_to_map); 
  if (!file) 
  {
    std::cerr << "Unable to open file: "<<path_to_map<<"\n";
    std::exit(1);
  }

  // Parse dimension of the map file
  int dim[2] = {0 ,0}; //h, w
  std::string line, num;
  file.seekg(0);
  std::getline(file, line);  
  std::istringstream ss(line);
  
  int n = 0;
  while( n < 2 && ss >> num) dim[n++] = std::stoi(num);

  // Allocate memeory for map
  map_.resize(dim[0]);
  for(int i = 0; i < dim[0]; i++) map_[i].resize(dim[1]);

  // Fill in numbers for map
  for(int row = 0; row < dim[0] && std::getline(file, line); row++)
  {
    std::istringstream ss(line); 
    for(int col = 0; col < dim[1]; col++)
    {
      ss >> num;
      map_[row][col] = std::stoi(num);
    }
  }
}

const std::vector<std::vector<int>>& Utils::GetMap(const std::string& map_name)
{
  std::string path_to_map = GetAbsolutePath(map_name);
  ParseMap(path_to_map);
  return map_;
}

