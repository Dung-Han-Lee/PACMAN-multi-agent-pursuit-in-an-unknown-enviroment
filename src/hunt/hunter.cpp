#include "hunter.h"
// TODO: assign pursuers using hungarian algorithm

Hunter::Hunter(
    world_state& world,
    priority_queue<node, std::vector<node>, 
    CompareCost>* open_list, 
    unordered_map<position,bool,boost::hash<position>>* visited): 
      Explorer(world,open_list,visited), 
      Pursuer(world), 
      last_exec_mode_(EXPLORE) 
{ 
  curr_pos_ = this->Pursuer::curr_pos_ = this->Explorer::curr_pos_;
}

void Hunter::UpdatePos()
{
  if(last_exec_mode_ == EXPLORE)
  {
    curr_pos_ = this->Pursuer::curr_pos_ = this->Explorer::curr_pos_; 
  }
  else if(last_exec_mode_ == PURSUE)
  {
    curr_pos_ = this->Explorer::curr_pos_ = this->Pursuer::curr_pos_; 
  }
}

void Hunter::SetMode(Mode m)
{
  if(m != last_exec_mode_)
    this->Explorer::_init = true;
  last_exec_mode_ = m; 
};

void Hunter::Dijkstra(world_state& world)
{
  const int h = world.size(), w = world[0].size();
  min_moves_ = std::vector<std::vector<int>>(h, std::vector<int>(w, 1e9));
  auto states = this->Pursuer::Search(
      "dijkstra", world, curr_pos_, {-1, -1});
  for(auto it = states.begin(); it != states.end(); it++)
  {
    int r = it->second.row, c = it->second.col;
    min_moves_[r][c] = it->second.g_val;
  }
}

bool Hunter::HasPath(const position& target)
{
  int r = target.first, c = target.second; 
  return min_moves_[r][c] < 1e9;
}

/////////////////////////////////////////////////////////
//  Centralized controller for the overall behavior    //
/////////////////////////////////////////////////////////

HunterMaster::HunterMaster(world_state& world, int num_pursuers)
{
  for(int i = 0; i < num_pursuers; i++)
    hunters_.emplace_back(world,&_open_list,&_visited);
}

bool HunterMaster::Move(world_state& world, const position target)
{
  bool see_prey = false;
  for(auto& hunter : hunters_)
  {
    hunter.Explorer::test_curr_scan_.clear();
    if(hunter.Explorer::SeePrey()) see_prey = true;
  }
  test_predict_paths_.clear();
  test_assignments_.clear();

  const int n = hunters_.size();
  std::vector<int> explorer_ids = {}, pursuer_ids = {};
  for(int id = 0; id < n; id++)
  {
    explorer_ids.push_back(id);

    if(!see_prey) continue;

    hunters_[id].Dijkstra(world);
    if(!hunters_[id].HasPath(target)) continue;
    
    explorer_ids.pop_back();
    pursuer_ids.push_back(id);
  }
  MoveExplorers(explorer_ids, world);
  MovePursuers(pursuer_ids, world, target);
  for(auto & hunter : hunters_)
    hunter.UpdatePos();
  return see_prey;
}

void HunterMaster::MoveExplorers(const std::vector<int>& explorer_ids, 
    world_state& world)
{
  const int num_explorers = explorer_ids.size();
  if(num_explorers <= 0)return;
  for(int id : explorer_ids)
  {
    hunters_[id].SetMode(EXPLORE);
    hunters_[id].Explorer::Frontier_Exec(world);
  }
}

void HunterMaster::MovePursuers(const std::vector<int>& pursuer_ids, 
    world_state& world, const position& target)
{
  const int num_pursuers = pursuer_ids.size();
  if(num_pursuers <= 0) return;

  std::vector<std::list<position>> paths = {};
  PredictEscapePaths(world, target, paths, num_pursuers);
  auto assignments = AssignBlockingPos(world, target, paths, pursuer_ids);
  for(const auto& assignment : assignments)
  {
    int id = assignment.first;
    auto goal = assignment.second;

    // Go straight for target if close enough
    auto p = hunters_[id].Pursuer::GetPos();
    int dist = hunters_[0].Pursuer::ManhattanDist(p, target);
    if(std::rand() % 10 > dist) goal = target;

    // Stand still to break repetitive behavior
    if(MoreThanOne(world, target) && std::rand() % 10 > 7) goal = p;

    hunters_[id].SetMode(PURSUE);
    hunters_[id].Move4Target(world, goal);
  }
}

bool HunterMaster::MoreThanOne(world_state& world, const position& target)
{
  int dr[8] = {0, 0, 1,-1, 1, 1,-1,-1};
  int dc[8] = {1,-1, 0, 0, 1,-1, 1,-1};
  int num_ghost = 0;
  for(int i = 0; i < 8; i++)
  {
    int nr = target.first  + dr[i];
    int nc = target.second + dc[i];
    if(!hunters_[0].Pursuer::InsideBoundary(world, nr, nc)) continue;
    if(world[nr][nc].occupancy == GHOST) num_ghost++;
  }
  return num_ghost >= 2;
}

bool HunterMaster::GetTarget(position target)
{
  for(auto& h : hunters_)
    if(h.GetPos() == target) return true;
  return false;
}

void HunterMaster::PredictEscapePaths(
  world_state& world, 
  const position& target, 
  std::vector<std::list<position>>& paths,
  int num_pursuers)
{
  const int n = num_pursuers -1; //exlude the nearest
  const int width = world[0].size();
  paths.clear();
  paths.resize(n);

  for(int i = 0; i < n; i++)
  {
    auto goal = hunters_[0].Pursuer::RandomFreeSpot(world);
    auto target_path  = hunters_[0].Pursuer::Search(\
        "predict", world, target, goal);
    int gr = goal.first; int gc = goal.second;
    State* ptr = &target_path[gr * width + gc];
    while(ptr->prev != NULL)
    {
      paths[i].emplace_front(ptr->row, ptr->col);
      ptr = ptr->prev; 
    }
  }
  test_predict_paths_ = paths;
}

int HunterMaster::NearestPursuerID(const std::vector<int>& pursuer_ids,
    const position& target)
{
  const int r = target.first, c = target.second;
  int closest_id = -1;
  int min = 1e9;
  for(int id : pursuer_ids)
  {
    const auto& moves = hunters_[id].GetMinMoves();
    int move = moves[r][c];
    closest_id = move < min ? id : closest_id;
    min = std::min(move, min);
  }
  return closest_id;
}

std::vector<std::vector<int>> HunterMaster::Permute(std::vector<int>& num) 
{
  std::vector<std::vector<int>> res;
  std::sort(num.begin(), num.end());
  res.push_back(num);
  while (std::next_permutation(num.begin(), num.end())) {
      res.push_back(num);
  }
  return res;
}

position HunterMaster::GetBlockingPos(
    const std::list<position>& path,
    const std::unordered_map<int, State>& target_states, 
    int width, 
    int id)
{
  for(auto it = path.begin(); it != path.end(); it++)
  {
    int r = it->first, c = it->second;
    int target_min_move  = target_states.at(r * width + c).g_val;
    if( target_min_move >= hunters_[id].GetMinMoves()[r][c])
      return {r, c};
  }
  return {-1, -1};
}

std::vector<assignment> HunterMaster::AssignBlockingPos(
  world_state& world, 
  const position& target, 
  const std::vector<std::list<position>>& paths,
  const std::vector<int>& pursuer_ids)
{
  // Copy ids of pursuerer except for the one nearest to the target 
  int nearest_id = NearestPursuerID(pursuer_ids, target);
  std::vector<int> ids = {};
  for(int id : pursuer_ids)
    if(id != nearest_id) ids.push_back(id);

  //brute force O(N!), replace with hungarian algorithm
  auto permutations = Permute(ids); 

  auto target_states = hunters_[0].Pursuer::Search(\
      "dijkstra", world, target, {-1, -1});
  int min = 1e9, width = world[0].size();
  position no_solution(-1, -1);
  std::vector<assignment> res, tmp;

  for(const auto& permutation : permutations)
  {
    int cost = 0, m = permutation.size();
    tmp.clear();
    for(int i = 0; i < m; i++)
    {
      int id = permutation[i];
      auto p = GetBlockingPos(paths[i], target_states, width, id);
      if(p == no_solution)
      {
        tmp.emplace_back(id, target);
        cost += 1e8;
      }
      else
      {
        tmp.emplace_back(id, p);
        cost += hunters_[id].GetMinMoves()[p.first][p.second];
      }
    } 
    res = cost < min ? tmp : res;
    min = std::min(min, cost);
  }
  res.emplace_back(nearest_id, target);
  test_assignments_ = res;
  return res;
}


