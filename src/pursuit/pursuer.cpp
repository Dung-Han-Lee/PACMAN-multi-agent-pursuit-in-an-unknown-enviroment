#include "pursuer.h"

void print_states(const std::unordered_map<int, State>& closed, const std::string mode, int h, int w)
{
	std::vector<std::vector<int>> show(h, std::vector<int>(w, -1));
	std::cout<<"printing "<<mode<<"\n";
	for(auto it = closed.begin(); it!= closed.end(); it++)
	{
		int r = it->second.row;
		int c = it->second.col;
		if (mode == "gval") show[r][c] = it->second.g_val;
		if (mode == "fval") show[r][c] = it->second.f_val;
		if (mode == "hval") show[r][c] = (it->second.f_val - it->second.g_val);
	}
	for(int i = 0; i < h; i++)
	{
		for(int j = 0; j < w; j++)
			std::cout<<std::setw(3)<<show[i][j]<<" ";
		std::cout<<"\n";
	}
	std::cout<<"\n";
}

Pursuer::Pursuer(world_state& world): BaseAgent(0, 0)
{
  curr_pos_ = RandomFreeSpot(world); 
};

bool Pursuer::Move4Target(world_state& world, position target)
{

  if(world.size() <= 0 || world[0].size() <= 0)
  {
    std::cerr<<"Invalid input world dimension\n";
    return false;
  }

  int tr = target.first;	
  int tc = target.second;
  int cr = curr_pos_.first;		
  int cc = curr_pos_.second;
  int h = world.size();   
  int w = world[0].size();

  std::unordered_map<int, State> closed = Search("hunt", world, curr_pos_, target);

  bool success = false;
  if(!closed.count((tr * w + tc)) ) return success;

	State* ptr = &closed[tr * w + tc];
  while(ptr->prev != NULL && ptr->prev != &closed[cr * w + cc])
  {
    ptr = ptr->prev;
  }

  world[cr][cc].occupancy = FREE;
  curr_pos_.first  = cr = ptr->row;
  curr_pos_.second = cc = ptr->col;
  world[cr][cc].occupancy = GHOST;
  Perceive(world);

  return success = true;
}
