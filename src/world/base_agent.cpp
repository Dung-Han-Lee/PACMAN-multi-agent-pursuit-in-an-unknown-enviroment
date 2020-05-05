#include "base_agent.h"
// TODO: add boolean is_prey in search. prey has full knownledge of the world

BaseAgent::BaseAgent(int r, int c): kNumofBeams(180), kSenseLimit(5.0), 
    curr_pos_(r, c), prey_in_sight_(false) {};

bool BaseAgent::InsideBoundary(const world_state& world, int row, int col) const
{
	return (row >= 0 && row < (int)world.size() &&\
			    col >= 0 && col < (int)world[0].size());
}

std::vector<position> BaseAgent::Perceive(world_state& world) 
{
  prey_in_sight_ = false;
  std::vector<std::pair<int,int>> vec_frontier;
  float incr = 0.5;
	for(int i = 0; i < kNumofBeams; i++)
	{
		double angle = i * 2 * M_PI/kNumofBeams;
		for(double l = incr; l < kSenseLimit; l+= incr)
		{
			int row = curr_pos_.first  + l * cos(angle);
			int col = curr_pos_.second + l * sin(angle);
			if(!InsideBoundary(world, row, col)) break;
			auto& cell = world[row][col];
      if(cell.occupancy == PACMAN) prey_in_sight_ = true;
			if(cell.occupancy == WALL) break;
      // std::cout<<"beam "<<i<<":"<<l<<"vs"<<kSenseLimit<<std::endl;
      if(l == kSenseLimit-incr) 
      {
        // std::cout<<"Success add\n";
        vec_frontier.push_back(std::make_pair(row,col));
      }
			cell.known = true;
			cell.entropy = 0;
		}
	}
  // std::cout<<"size:"<<vec_frontier.size()<<std::endl;
  test_curr_scan_ = vec_frontier;
  return vec_frontier;
}

int BaseAgent::ManhattanDist(int ar, int ac, int br, int bc) const
{
  return (std::abs(ar - br) + std::abs(ac - bc));
};

int BaseAgent::ManhattanDist(position a, position b) const
{
  return ManhattanDist(a.first, a.second, b.first, b.second);
}

std::unordered_map<int, State> BaseAgent::Search(
    const std::string& mode,
    const world_state& world, 
    const position& curr, 
    const position& target,
    bool is_prey) const
{

	const int tr = target.first;	const int tc = target.second;
	const int cr = curr.first;		const int cc = curr.second;
  const int h = world.size();   const int w = world[0].size();

  int dr[4] = {0, 0, 1,-1};
  int dc[4] = {1,-1, 0, 0};

  std::vector<std::vector<double>> min_cost(h, std::vector<double>(w, 1e9));
  std::unordered_map<int, State> closed;
  std::priority_queue<State, std::vector<State>, cmp> open;
  double hval = mode == "dijkstra" ? 0. : ManhattanDist(cr, cc, tr, tc);
  open.emplace(cr, cc, 0, hval);
  while(!open.empty() && !closed.count((tr * w + tc)))
  {
    auto cur = open.top(); open.pop();
    closed[cur.row * w + cur.col] = cur;
    for(int i = 0; i < 4; i++)
    {
      int nr = cur.row + dr[i];
      int nc = cur.col + dc[i];
      if(!InsideBoundary(world, nr, nc)) continue;
			if(closed.count((nr * w + nc))) continue;
			if(world[nr][nc].occupancy == WALL) continue;
      if(world[nr][nc].occupancy == GHOST) continue;
      if(!is_prey && !world[nr][nc].known) continue;
      int ng = cur.g_val + 1;
      if(ng >= min_cost[nr][nc]) continue;
      min_cost[nr][nc] = ng;
      double nh = mode == "dijkstra" ? 0. : ManhattanDist(cr, cc, tr, tc);
      open.emplace(nr, nc, ng, nh, &closed[cur.row * w + cur.col]);
    }
  }
  return closed;
}

position BaseAgent::RandomFreeSpot(world_state& world)
{
  const int h = world.size();
  const int w = world[0].size();
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> randh(0, h-1);
  std::uniform_int_distribution<> randw(0, w-1);

  bool found = false;
  while(!found)
  {
    int r = randh(gen);
    int c = randw(gen);
    if(world[r][c].occupancy != FREE ) continue;
    return {r, c};
  }
}