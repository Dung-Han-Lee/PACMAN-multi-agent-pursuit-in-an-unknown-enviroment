/*
Hunter class is a diamond inherited class: both explorer and pursuer are deri-
ved from the base class. During execution, hunter would switch between explor-
er and pursuer mode. 

The pursuer cooperation strategy is defined in huntermaster class, where the 
hunter would perdict escape paths of target and attempt to block these paths 
with optimal(minimize total travel distance) assignment of hunters.
*/

#ifndef HUNTER_H
#define HUNTER_H

#include "world.h"
#include "exploration.h"
#include "pursuer.h"

typedef std::pair<int, position> assignment;

enum Mode {EXPLORE, PURSUE};

class Hunter: public Explorer, public Pursuer
{
 public:
  Hunter(world_state& world, 
        priority_queue<node, std::vector<node>, CompareCost>* open_list,
        unordered_map<position,bool, boost::hash<position>>* visited);

  virtual ~Hunter() = default;

  void SetMode(Mode m);

  void UpdatePos();

  void Dijkstra(world_state& world);
  // fill in minimum move to a cell accroding to dijkstra

  bool HasPath(const position& target);

  position GetPos(){ return curr_pos_; }

  std::vector<std::vector<int>>& GetMinMoves(){return min_moves_;};

 protected:

  std::vector<std::vector<int>> min_moves_;

  position curr_pos_;

  Mode last_exec_mode_;
};


class HunterMaster
{
 public:

  HunterMaster(world_state& world, int num_of_hunters);

  ~HunterMaster() = default;

  bool Move(world_state& world, position target);
  // master function that triggers all the process

  bool MoreThanOne(world_state& world, const position& target);
  // more than 1 hunter in the 8-connected grid of target

  std::vector<Hunter>& Members() {return hunters_;};

  bool GetTarget(position target);

  std::vector<assignment> test_assignments_;

  std::vector<std::list<position>> test_predict_paths_;

 private:
  unordered_map<position,bool,boost::hash<position>> _visited;

  priority_queue<node, std::vector<node>, CompareCost> _open_list;

  void MoveExplorers(const std::vector<int>& explorer_ids, 
      world_state& world);
  // master function for explorer behaviors

  void MovePursuers(const std::vector<int>& pursuer_ids, 
      world_state& world, const position& target);
  // master function for pursuer behaviors

  int NearestPursuerID(const std::vector<int>& pursuer_ids,
      const position& target); 
  // return index of the hunter which is nearest to the target

  std::vector<Hunter> hunters_;

  std::vector<std::vector<std::vector<int>>> min_moves_hunters_;

  void PredictEscapePaths(  
      world_state& world, 
      const position& target, 
      std::vector<std::list<position>>& paths,
      int num_pursuers);
  // predict by randomly selecting goal points and planning paths
  // towards it

  std::vector<std::vector<int>> Permute(std::vector<int>& num);
  // return all possible permutation of assignment

  position GetBlockingPos(
      const std::list<position>& path,
      const std::unordered_map<int, State>& target_states, 
      int width,
      int id);
  // return blocking position of a predicted path

  std::vector<assignment> AssignBlockingPos(
      world_state& world, 
      const position& target, 
      const std::vector<std::list<position>>& paths,
      const std::vector<int>& pursuer_ids);
  // assign hunters to blocking points by minimizing the total travel
  // distance of all hunters

};

#endif