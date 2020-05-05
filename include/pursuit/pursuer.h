#ifndef PURSUER_H
#define PURSUER_H

#include <iomanip>
#include <cmath>
#include <algorithm>
#include <list>
#include "base_agent.h"
#include "world.h"

class Pursuer : public BaseAgent
{
 public:
  
  Pursuer(world_state& world);

  ~Pursuer() = default;

  bool Move4Target(world_state& world, position target);
  // A* search and walk toward target
};

#endif