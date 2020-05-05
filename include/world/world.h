#ifndef DATA_STRUCTURE_H
#define DATA_STRUCTURE_H

#include <vector>

enum Occupancy {FREE, WALL, GHOST, PACMAN};

// World Representation
struct Cell
{
	Occupancy occupancy;
	int entropy;
	bool known;
	Cell(): occupancy(FREE), entropy(255), known(false) {};
};

typedef std::vector<std::vector<Cell>> world_state;

typedef std::pair<int, int> position;

// State for A* search
struct State
{
	int row, col;
	double g_val, f_val;
	State* prev;
	State() = default;
	State(int r, int c, double gval, double hval, State* prev = nullptr):
    	row(r), col(c), g_val(gval), f_val(gval + hval), prev(prev) {};
};

// Comparator for A* search
struct cmp
{
  bool operator()(const State& a, const State& b)
  {
    return a.f_val > b.f_val;
  }
};

#endif