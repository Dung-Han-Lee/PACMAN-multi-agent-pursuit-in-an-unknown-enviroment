/*
This class is derived from base agent. The explorers would perform ray-casting
to identify frontier cells, which would be shared between explorers as well. 
At each move, the explorer would identify the best frontier cell by taking into
account the information gain and travel distance, then perform A* searh & walk.
*/
#ifndef EXPLORATION_H
#define EXPLORATION_H

#include <iomanip>
#include <list>
#include "base_agent.h"
#include <boost/functional/hash.hpp>
#include "world.h"
#include <queue> 


using namespace std;
struct node { 
  position pos;
  float reward; 
  node(position id1, float rew1)
  {
    pos = id1;
    reward = rew1;
  } 
};

struct CompareCost { 
  bool operator()(node const& node1, node const& node2) 
  { 
    return node1.reward < node2.reward; 
  } 
};  

class Explorer : public BaseAgent
{
 private:
	list<position> _plan;
	position _target;
	priority_queue<node, std::vector<node>, CompareCost>* _open_list;
	unordered_map<position,bool,boost::hash<position>>* _visited;
 public:
  bool _init;

  Explorer(world_state & world, 
      priority_queue<node, std::vector<node>, CompareCost>* open_list, 
      unordered_map<position,bool,boost::hash<position>>* visited);

  ~Explorer() = default;

  bool Frontier_Move(world_state& world);

  bool Frontier_Plan(world_state& world, position target);

  position Find_Best_Point(world_state &world);

  int information(position fp, world_state& world);
  // Apply sensor model on frontier cell to compute info gain by summing
  // up the entropy

  void Frontier_Exec(world_state& world);

  void initialize(world_state& world);

  int proximity(position p1, position p2);

  position GetTarget();
};

#endif