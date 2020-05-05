#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <vector>
#include <utility>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "world.h"

static std::vector<cv::Scalar> colors = {cv::Scalar(255, 255, 0), 
    cv::Scalar(220, 120, 255), cv::Scalar(0, 0, 255), cv::Scalar(50, 150, 255)};

// Fill in a h x w matrix by the return value of function F
template <typename T, typename F> inline cv::Mat Convert2Mat(\
    const std::vector<std::vector<T>>& input,F fucntion)
{
  if ( input.size() < 1 || input[0].size() < 1)
  {
    std::cerr<<"error: visualizer received empty input!\n";
  }

  // Allocate memory and fill in for mat
  cv::Mat visual_map( cv::Size(input.size(), input[0].size()),  CV_8UC1);  
  for(size_t i = 0; i < input.size(); i++)
    for(size_t j  =0; j < input[0].size(); j++)
      visual_map.at<uchar>(i, j) = fucntion(input, i, j);

  return visual_map;
}

// Return key to allow manual control
inline int Visualize(const std::string& name, const cv::Mat& viz, int time = 0)
{
  cv::namedWindow(name, cv::WINDOW_NORMAL );
  cv::imshow(name, viz);
  int key = cv::waitKey(time);
  return key;
}

// Walls are 255 otherwise 0
template <typename T> 
inline cv::Mat GetMatMap(const std::vector<std::vector<T>>& map)
{
  auto is_wall = [](const std::vector<std::vector<T>>& map, int row, int col)
  {
    return map[row][col] == 1 ? 255 : 0;
  };
  return Convert2Mat(map, is_wall);
}

template <typename T> 
inline cv::Mat GetMatKnown(const std::vector<std::vector<T>>& world)
{
  auto is_known = [](\
      const std::vector<std::vector<T>>& world, int row, int col)
  {
    return world[row][col].known ? 255 : 0;
  };
  return Convert2Mat(world, is_known);
}

template <typename T> 
inline cv::Mat GetMatEntropy(const std::vector<std::vector<T>>& world)
{
  auto entropy = [](\
      const std::vector<std::vector<T>>& world, int row, int col)
  {
    return world[row][col].entropy;
  };
  return Convert2Mat(world, entropy);
}

template <typename T> 
inline void VisualizeMap(const std::vector<std::vector<T>>& map)
{
  Visualize("map", GetMatMap(map));
}

template <typename T> 
inline void VisualizeFree(const std::vector<std::vector<T>>& world)
{
  Visualize("Free Space", GetMatFree(world));
}

template <typename T> 
inline void VisualizeKnown(const std::vector<std::vector<T>>& world)
{
  Visualize("Known World", GetMatKnown(world));
}

template <typename T> 
inline void VisualizeEntropy(const std::vector<std::vector<T>>& world, int square_size=32)
{
  const int r = world.size();
  const int c = world[0].size();
  int h = r * square_size;
  int w = c * square_size;
  cv::Mat entropy = GetMatEntropy(world);
  cv::Mat src(r, c,  CV_8UC3, cv::Scalar(0, 0, 0));  

  for(int i = 0; i < r; i++)
  {
    for(int j  =0; j < c; j++)
    {
      int e = entropy.at<uchar>(i, j);
      src.at<cv::Vec3b>(i, j) = cv::Vec3b(e, e, e);
      if (world[i][j].occupancy == 1) src.at<cv::Vec3b>(i, j) =\
          cv::Vec3b(255, 0, 50);
    }
  }
}

inline void DrawPacman(cv::Mat src, position pos,\
    float rotation, int square_size){

  int pos_x = pos.second;
  int pos_y = pos.first;
	int x, y;

  const int dx = pos_x*square_size + square_size/2;
  const int dy = pos_y*square_size + square_size/2;
  const double width = square_size;

	for (int k = 0; k < width; k++){
		for (int i = 40; i < 320; i++){
      double a = i * M_PI/180.;
			x = k / 2.0 * cos((i + 90 * rotation) * M_PI / 180.0) + dx;
			y = k / 2.0* sin((i + 90 * rotation) * M_PI / 180.0) + dy;
      src.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 255);
    }
	}
}

inline void DrawGhost(cv::Mat src, position pos, int square_size, int id = 0){
  int pos_x = pos.second;
  int pos_y = pos.first;
	int x, y;
  auto color = colors[id];
  const int dx = pos_x*square_size + square_size/2;
  const int dy = pos_y*square_size + square_size/2;
  const int width = square_size;

	//draw the head
	for (int k = 0; k < width; k++){
		for (int i = 180; i <= 360; i++){
			x = k / 2.0 * cos(i * M_PI / 180.0) + dx;
			y = k / 2.0* sin(i * M_PI / 180.0) + dy;
      src.at<cv::Vec3b>(y, x) = cv::Vec3b(color[0], color[1], color[2]);
		}
	}

  auto draw_rect = [&](cv::Mat& img, int r1, int c1, int r2, int c2)
  {
    auto p1 = cv::Point(r1, c1);
    auto p2 = cv::Point(r2, c2);
    cv::rectangle(src, p1, p2, color, -1);
  };

  if(width < 8) return;
  draw_rect(src, dx-4*width/8, dy, dx+4*width/8-1, dy+2*width/8);  // draw body
  draw_rect(src, dx-5*width/10, dy + 2*width/8, dx-3*width/10-1, dy+3*width/8); // draw leg
  draw_rect(src, dx-1*width/10, dy + 2*width/8, dx+1*width/10-1, dy+3*width/8); // draw leg
  draw_rect(src, dx+3*width/10, dy + 2*width/8, dx+5*width/10-1, dy+3*width/8); // draw leg
}

template <typename T, typename G, typename P> 
inline int VisualizePursuit(const std::vector<std::vector<T>>& world,\
    G& predactor_master, P& prey, int square_size, bool halt = false)
{
  const int r = world.size();
  const int c = world[0].size();
  int h = r * square_size;
  int w = c * square_size;

  cv::Mat src(r, c,  CV_8UC3, cv::Scalar(0, 0, 0));  
  for(int i = 0; i < r; i++)
    for(int j  =0; j < c; j++)
      if (world[i][j].occupancy == 1) src.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 50);

  cv::resize(src, src, cv::Size(h, w), 0, 0, cv::INTER_NEAREST);

  DrawPacman(src, prey.GetPos(), prey.GetOrient(), square_size); 

  auto& predactors = predactor_master.Members();
  const int n = predactors.size();
  for(int i = 0; i < n; i++)
    DrawGhost(src, predactors[i].GetPos(), square_size, i);
  
  int time = halt ? 0 : 10;
  return Visualize("Game", src, time);
}


template <typename T, typename G> 
inline void VisualizeExplore(const std::vector<std::vector<T>>& world,\
    G& predactor, int square_size, bool halt = false)
{
  const int r = world.size();
  const int c = world[0].size();
  int h = r * square_size;
  int w = c * square_size;
  cv::Mat entropy = GetMatEntropy(world);
  cv::Mat src(r, c,  CV_8UC3, cv::Scalar(0, 0, 0));  

  for(int i = 0; i < r; i++)
  {
    for(int j  =0; j < c; j++)
    {
      int e = entropy.at<uchar>(i, j);
      src.at<cv::Vec3b>(i, j) = cv::Vec3b(e, e, e);
      if (world[i][j].occupancy == 1) src.at<cv::Vec3b>(i, j) =\
          cv::Vec3b(255, 0, 50);
    }
  }

  cv::resize(src, src, cv::Size(h, w), 0, 0, cv::INTER_NEAREST);
  DrawGhost(src, predactor.Explorer::GetPos(), square_size);

  for(const auto& p : predactor.Explorer::test_curr_scan_)
  {
    int x = p.second * square_size + square_size/2;
    int y = p.first  * square_size + square_size/2;
    auto pt = cv::Point(x, y);
    cv::circle(src, pt, 3, cv::Scalar(0, 255, 255), -1);
  }
  Visualize("Game", src);
}

template <typename T, typename H, typename P> 
inline int VisualizeHunt(const std::vector<std::vector<T>>& world,\
    H& hunter_master, P& prey, int square_size, bool halt = false)
{
  const int r = world.size();
  const int c = world[0].size();
  int h = r * square_size;
  int w = c * square_size;
  cv::Mat entropy = GetMatEntropy(world);
  cv::Mat known = GetMatKnown(world);
  cv::Mat src(r, c,  CV_8UC3, cv::Scalar(0, 0, 0));  

  for(int i = 0; i < r; i++)
  {
    for(int j  =0; j < c; j++)
    {
      int e = entropy.at<uchar>(i, j);
      src.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0.3 * e);
      
      if(known.at<uchar>(i,j) == 0)
        src.at<cv::Vec3b>(i, j) = cv::Vec3b(75, 0, 75);

      if (world[i][j].occupancy == 1) 
        src.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 50);
    }
  }

  cv::resize(src, src, cv::Size(h, w), 0, 0, cv::INTER_NEAREST);

  // Draw predicted paths of target
  auto paths = hunter_master.test_predict_paths_;
  for(const auto& path : paths)
  {
    for(auto it = path.begin(); it != path.end(); it++)
    {
      int r = it->first * square_size + square_size/2;
      int c = it->second * square_size + square_size/2;
      auto p1 = cv::Point(c, r);
      auto color = cv::Scalar(0, 255, 255);
      cv::circle(src, p1, 3, color, -1);
    }
  }

  // Draw blocking point
  auto assignments = hunter_master.test_assignments_;
  const int m = assignments.size();
  for(int i = 0; i < m; i++)
  {
    int id = assignments[i].first;
    auto pos = assignments[i].second;
    int br = pos.first * square_size + square_size/2;
    int bc = pos.second * square_size + square_size/2;
    auto p1 = cv::Point(bc, br);

    auto color = colors[id];
    cv::circle(src, p1, 10, color, -1);
  }

  DrawPacman(src, prey.GetPos(), prey.GetOrient(), square_size); 

  auto& hunters = hunter_master.Members();
  const int n = hunters.size();
  for(int i = 0; i < n; i++)
  {
    DrawGhost(src, hunters[i].GetPos(), square_size, i);
    // Draw perceptiob boundary of ghost
    for(const auto& p : hunters[i].Explorer::test_curr_scan_)
    {
      int x = p.second * square_size + square_size/2;
      int y = p.first  * square_size + square_size/2;
      auto pt = cv::Point(x, y);
      cv::circle(src, pt, 3, colors[i], -1);
    }
  }
  int time = halt ? 0 : 10;
  return Visualize("Game", src, time);
}



#endif