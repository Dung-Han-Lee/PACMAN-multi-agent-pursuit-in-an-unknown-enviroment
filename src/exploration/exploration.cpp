#include "exploration.h"

Explorer::Explorer(world_state & world, 
    priority_queue<node, std::vector<node>, CompareCost>* open_list, 
    unordered_map<position,bool,boost::hash<position>>* visited): 
        BaseAgent(0, 0)
{
  _init = true;
  _open_list = open_list;
  _visited = visited;
  curr_pos_ = RandomFreeSpot(world); 
  world[curr_pos_.first][curr_pos_.second].occupancy = GHOST;
}

bool Explorer::Frontier_Move(world_state& world)
{

  if(_plan.size() < 1) return false;
  
  position new_pose = _plan.front();
  int cr = curr_pos_.first;		
  int cc = curr_pos_.second;
  int h = world.size();   
  int w = world[0].size();
  int nr = new_pose.first;
  int nc = new_pose.second;
  world[cr][cc].occupancy = FREE;
  int dr[8] = {-1, -1, -1,  0,  0,  1, 1, 1};
  int dc[8] = {-1,  0,  1, -1,  1, -1, 0, 1};

  if(world[nr][nc].occupancy != WALL)
  {
    curr_pos_.first  = cr = nr;
    curr_pos_.second = cc = nc;
    world[cr][cc].occupancy = GHOST;
    
    // Suppress nearby frontiers
    if(!_open_list->empty())
    {
		node top = _open_list->top();
		if(proximity(curr_pos_,top.pos)<3)
			_open_list->pop();
	  }

    if(_plan.size() > 0)
      _plan.pop_front();
    else
      return false;

    Perceive(world);

    // Update if a cell is visited
    (*_visited)[make_pair(nr,nc)] = true;
    for(int i =0; i < 8; i++)
    {
      (*_visited)[make_pair(nr+dr[i],nc+dc[i])] = true;
    }
    return true;
  }
  else
    return false;
}

int Explorer::proximity(position pos1, position pos2)
{
  int dist = pow(pow(pos1.first-pos2.first,2) + pow(pos1.second-pos2.second,2),0.5);
  return dist;
}

bool Explorer::Frontier_Plan(world_state& world,position target)
{
  _plan.clear();
  std::unordered_map<int, State> closed = Search("hunt", world, curr_pos_, target);
  int cr = curr_pos_.first;   
  int cc = curr_pos_.second;
  int h = world.size();   
  int w = world[0].size();
  int tr = target.first;
  int tc = target.second;
  int dist_thresh = 3;
  if(closed.count((tr * w + tc)))
  {
    State* ptr = &closed[tr * w + tc];
    while(ptr->prev != NULL)
    {
      position candidate_pos = make_pair(ptr->row,ptr->col);
      _plan.push_front(candidate_pos);
      if(!_open_list->empty())
      {
	      node top = _open_list->top();
	      if(proximity(candidate_pos,top.pos) < dist_thresh)
	      {
	        _open_list->pop();
	        top = _open_list->top();
	      }
  	  }
      ptr = ptr->prev;
      if(ptr ==  &closed[cr * w + cc]) break;
    }
    return true;
  }
  else
    return false;
}


void Explorer::initialize(world_state& world)
{
  int cr = curr_pos_.first;   
  int cc = curr_pos_.second;
  world[cr][cc].occupancy = GHOST;
  _target = Find_Best_Point(world);
  _init = false;
}

position Explorer::Find_Best_Point(world_state& world)
{
  vector<position> frontier_points = Perceive(world);
  int max_gain = 0.0;
  position max_fp;
  priority_queue<node, std::vector<node>, CompareCost> local_list;
  for(const auto &fp: frontier_points)
  {
    int info_gain = information(fp,world);
    _open_list->emplace(fp,info_gain);
    local_list.emplace(fp,info_gain);
  }

  if(!local_list.empty())
  {
	  node local_top = local_list.top();
	while(!local_list.empty())
	 {
	  	if(Frontier_Plan(world,local_top.pos))
	  	{
	  		max_fp = local_top.pos;
	  		max_gain = local_top.reward;
	  		break;
	  	}
	  	else
	  	{
	  		local_list.pop();
	  		local_top = local_list.top();
	  	}
	  }
  }

  if(!_open_list->empty())
  {
	  node first = _open_list->top();
	  int global_reward = information(first.pos,world)-600*proximity(curr_pos_,first.pos);
	  int local_reward = max_gain-600*proximity(curr_pos_,max_fp);

	  if(global_reward>local_reward)
	  {
	    if(Frontier_Plan(world,first.pos))
	    {
	      max_gain = first.reward;
	      max_fp = first.pos;
	      _open_list->pop();
	    }
	  }
	  else
	  	_open_list->pop();
  }
  return max_fp;
}

int Explorer::information(position fp, world_state&world)
{
  float incr = 0.5;
  int kNumofBeams = 180;
  float kSenseLimit = 10.0; 
  int sum_entropy = 0;
  if(world[fp.first][fp.second].known == false)
    sum_entropy += 255*kNumofBeams*20;
  if(_visited->find(make_pair(fp.first,fp.second)) == _visited->end())
    sum_entropy += 255*kNumofBeams*10;
  for(int i = 0; i < kNumofBeams; i++)
  {
    double angle = i * 2 * M_PI/kNumofBeams;
    for(double l = incr; l < kSenseLimit; l+= incr)
    {
      int row = fp.first  + l * cos(angle);
      int col = fp.second + l * sin(angle);
      if(!InsideBoundary(world, row, col)) break;
      if(row ==  fp.first && col == fp.second) continue;
      sum_entropy += world[row][col].entropy;
    }
  }
  return sum_entropy;
}


void Explorer::Frontier_Exec(world_state& world)
{
  bool plan_success = false;
  if(_init == true)
    initialize(world);

  if(Frontier_Move(world) == false)
  {
    if(Frontier_Plan(world,_target) == false)
    {
      initialize(world);
    }
  }
  if(curr_pos_ == _target)
  {
    initialize(world);
  }
}

position Explorer::GetTarget()
{
  return _target;
}
