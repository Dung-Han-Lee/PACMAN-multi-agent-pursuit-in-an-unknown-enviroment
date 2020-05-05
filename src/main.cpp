#include <time.h>
#include <string>
#include "map_reader.h"
#include "hunter.h"
#include "prey.h"
#include "visualizer.h"

void ReSet( std::vector<std::vector<int>>& map,
    world_state& world)
{
  for(size_t r = 0; r < map.size(); r++)
  {
    for(size_t c = 0; c < map[0].size(); c++)
    {
      world[r][c].occupancy = map[r][c] ? WALL : FREE;
      world[r][c].entropy = 255;
      world[r][c].known = false;
    }
  }
}

void IncrementEntropy(world_state& world)
{
  for(size_t r = 0; r < world.size(); r++)
  {
    for(size_t c = 0; c < world[0].size(); c++)
    {
      if(world[r][c].entropy <= 245) 
        world[r][c].entropy++;
    }
  }
}

const int square_size = 16, num_test = 100, num_iter = 500;
std::vector<std::string> maps = { "dense_blocks.txt", "rooms.txt",
  "sparse_blocks.txt", "maze.txt", "openspace.txt"};


int main(int argc, char** argv)
{
  bool manual_control = 1;
  if(argc > 1 ) manual_control = 0;
  Utils utils;
  for(auto& m : maps)
  {
    // Read in map enviroment and initialize world
    auto map = utils.GetMap(m);
    world_state world;
    world.resize(map.size());
    for(size_t i = 0; i < world.size(); i++) world[i].resize(map[0].size());
    ReSet(map, world);

    auto seed = time(NULL);
    srand ( seed);

    HunterMaster hunters(world, 4);
    Prey pacman(world);

    // Run game
    std::cout<<"press any key to start\n";
    int time = 0;
    VisualizeHunt(world, hunters, pacman, square_size, 1);
    for(; time < num_iter; time++)
    {
      IncrementEntropy(world);
      if(!manual_control) pacman.Move(world);  
      int control = VisualizeHunt(world, hunters, pacman, square_size, manual_control);
      if(manual_control) pacman.Move(world, control);
      hunters.Move(world, pacman.GetPos());
      VisualizeHunt(world, hunters, pacman, square_size);
      if(hunters.GetTarget(pacman.GetPos())) break;
    }
    VisualizeHunt(world, hunters, pacman, square_size, true);
    std::cout<<"successfully survived "<<time<<" iteration\n";
  }
}

