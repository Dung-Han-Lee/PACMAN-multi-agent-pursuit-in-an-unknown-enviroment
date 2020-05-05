/*
This file defines basic class that would be inherited and shared by 
derived agents: pursuer and explorer. Some central behavior like 
perceive, getpos(),boundary check, A* expansion are all defined 
in this class.
*/

#ifndef BASE_H
#define BASE_H

#include <queue>
#include <vector>
#include <utility>
#include <cmath>
#include <iostream>
#include <random>
#include <unordered_map>
#include "world.h"
#include "math.h"

class BaseAgent
{
 public:
	BaseAgent(int r, int c);

	~BaseAgent() = default;

	std::vector<position> Perceive(world_state& world);
	// Ray casting to find frontier points, decrease entropy and
	// update the known world

	const position& GetPos() {return curr_pos_;};

	bool InsideBoundary(const world_state& world, int row, int col) const;
  // Boundary check

  int ManhattanDist(int ar, int ac, int br, int bc) const;

  int ManhattanDist(position a, position b) const;

	std::unordered_map<int, State> Search(\
      const std::string& mode, 
	  const world_state& world, 
      const position& curr, 
	  const position& target,
	  bool is_prey = false) const;
  // A* search, by setting mode = Dijkstra, this function can also perform
  // dijkstra expansion

  position RandomFreeSpot(world_state& world);
  // Select a freespot (not occupied by wall or other agents) in the wolrd

  const bool& SeePrey() const {return prey_in_sight_;};

  std::vector<position> test_curr_scan_;

 protected:

	position curr_pos_;

	bool prey_in_sight_;

	const int kNumofBeams;

	const double kSenseLimit;

};

#endif