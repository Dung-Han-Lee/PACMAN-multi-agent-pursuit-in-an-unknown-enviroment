/*
This class defines the movement for the prey to escape from
pursuers. The prey would perform A* walk between randomly
selected goals if pursuers(ghost) are not close, otherwise
it will escape into random direction with a preference stay-
ing close to walls. Such heuristic allow the prey to perform
more optimal escape behaviors. In a closed environment, the
prey is gauranteed not to be captured by a single pursuers.
With more than 1 pursuer, though, its more and more likely
to be corned and captured. 
*/

#ifndef PREY_H
#define PREY_H

#include <unordered_map>
#include <utility>
#include <iostream>
#include <random>
#include <iomanip>
#include <queue>
#include <algorithm>
#include <functional>
#include "base_agent.h"
#include "world.h"

class Prey : public BaseAgent
{
 public:

  Prey(world_state& world);
   
  ~Prey() = default;

  void Move(world_state& world);
  // Auto control

  void Move(world_state& world, int control);
  // Manual control

  int GetOrient(){return orient_;};
  // For drawing function

 protected:

  bool Close2Predator(world_state& world, position pos, int dist);

  void Escape(world_state& world);

  void PursueGoal(world_state& world, position goal);
  // A* search and walk toward goal

  void UpdateStates(world_state& world, int nr, int nc);

  position SafeFreeSpot(world_state& world);
  // Init prey outside the perception field of hunters

  int orient_;

  position goal_;
};

#endif