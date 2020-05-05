#include "prey.h"

Prey::Prey(world_state& world): BaseAgent(0, 0), orient_(0)
{
  curr_pos_ = goal_ = SafeFreeSpot(world); 
  world[curr_pos_.first][curr_pos_.second].occupancy = PACMAN;
};

position Prey::SafeFreeSpot(world_state& world)
{
  const int h = world.size();
  const int w = world[0].size();
  bool found = false;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> randh(0, h-1);
  std::uniform_int_distribution<> randw(0, w-1);

  int count = 0;
  while(!found && count++ < 100)
  {
    int r = randh(gen);
    int c = randw(gen);
    if(world[r][c].occupancy != FREE ) continue;
    if(Close2Predator(world, {r,c}, 10)) continue;
    return {r, c};
  }
}

void Prey::Move(world_state& world, int control)
{
  int r = curr_pos_.first;
  int c = curr_pos_.second;
  if(control == 81) c -= 1;
  else if(control == 82) r-= 1;
  else if(control == 83) c+= 1;
  else if(control == 84) r+= 1;
  if(!InsideBoundary(world, r, c)) return;
  if(world[r][c].occupancy != FREE) return;
  UpdateStates(world, r, c);
}

void Prey::Move(world_state& world)
{
  const int r = curr_pos_.first;
  const int c = curr_pos_.second;
  if(r == goal_.first && c== goal_.second)
    goal_ = RandomFreeSpot(world);

  if(Close2Predator(world, curr_pos_, 2))
  {
    Escape(world);
  }
  else
  {
    PursueGoal(world, goal_);
  }
}

// Look for predactors within dist blocks
bool Prey::Close2Predator(world_state& world, position pos, int dist)
{
  for(int i = -dist; i <= dist; i++)
  {
    for(int j = -dist; j <= dist; j++)
    {
      int r = pos.first  + i;
      int c = pos.second + j;
      if(!InsideBoundary(world, r, c)) continue;
      if(world[r][c].occupancy == GHOST) return true;
    }
  }
  return false;
}

// Escape with preference to stick with walls
// break tie randomly
void Prey::Escape(world_state& world)
{
  int dr[8] = {0, 0, 1,-1, 1, 1,-1,-1};
  int dc[8] = {1,-1, 0, 0, 1,-1, 1,-1};

  // Check if one-step-away from predactor
  auto next_to_predactor = [&](int r, int c)
  {
    for(int i = 0; i < 4; i++)
    {
      int nr = r + dr[i];
      int nc = c + dc[i];
      if(!InsideBoundary(world, nr, nc)) continue;
      if(world[nr][nc].occupancy == GHOST)
        return true;
    }
    return false;
  };

  std::vector<position> valid_moves;
  for(int i = 0; i < 4; i++)
  {
    int nr = curr_pos_.first  + dr[i];
    int nc = curr_pos_.second + dc[i];
    if(!InsideBoundary(world, nr, nc)) continue;
    if(world[nr][nc].occupancy != FREE ) continue;
    if(next_to_predactor(nr, nc)) continue;
    valid_moves.emplace_back(nr, nc);
  }

  const int n = valid_moves.size();
  if(n <= 0) return;

  // Check if next step is close to wall
  std::vector<position> close_wall;
  int idx = -1;
  for(int i = 0; i < n ; i++)
  {
    for(int j = 0; j < 8; j++)
    {
      int tr = valid_moves[i].first  + dr[j];
      int tc = valid_moves[i].second + dc[j];
      if(!InsideBoundary(world, tr, tc)) continue; 
      if(world[tr][tc].occupancy == WALL)
        close_wall.push_back(valid_moves[i]);
    }
  }

  // Give preference to stick with walls
  position next_pos;
  if(close_wall.empty()) next_pos = valid_moves[std::rand() % n];
  else next_pos = close_wall[std::rand() % close_wall.size()];
  UpdateStates(world, next_pos.first, next_pos.second);
}

void Prey::PursueGoal(world_state& world, position goal)
{
  const int cr = curr_pos_.first;		
  const int cc = curr_pos_.second;
  const int w = world[0].size();

  std::unordered_map<int, State> closed;
  while(true)
  {
    int gr = goal.first;
    int gc = goal.second;
    closed = Search("goal", world, curr_pos_, goal, true);
    if(closed.count(gr * w + gc)) break;
    goal_ = goal = RandomFreeSpot(world);
  }

	State* ptr = &closed[goal_.first * w + goal_.second];
  while(ptr->prev != NULL && ptr->prev != &closed[cr * w + cc])
    ptr = ptr->prev;
    
  UpdateStates(world, ptr->row, ptr->col);
}

void Prey::UpdateStates(world_state& world, int nr, int nc)
{
  int cr = curr_pos_.first;
  int cc = curr_pos_.second;
  int dr = cr - nr;
  int dc = cc - nc;
  if(dr != 0) orient_ = dr > 0 ? 3 : 1;
  if(dc != 0) orient_ = dc > 0 ? 2 : 0;

  world[cr][cc].occupancy = FREE;
  world[nr][nc].occupancy = PACMAN;
  curr_pos_.first  = nr;
  curr_pos_.second = nc;
}
