#include <navigator_vision_lib/active_contours.hpp>

using namespace std;
using namespace cv;

namespace nav
{

namespace Perturbations
{

void initPerturbationCache()
{
  std::unordered_map<uint8_t, uint8_t> seq_to_mod_border_idxs;
  seq_to_mod_border_idxs[0] = 0;
  seq_to_mod_border_idxs[1] = 1;
  seq_to_mod_border_idxs[2] = 2;
  seq_to_mod_border_idxs[3] = 3;
  seq_to_mod_border_idxs[4] = 4;
  seq_to_mod_border_idxs[5] = 12;
  seq_to_mod_border_idxs[6] = 20;
  seq_to_mod_border_idxs[7] = 28;
  seq_to_mod_border_idxs[8] = 36;
  seq_to_mod_border_idxs[9] = 35;
  seq_to_mod_border_idxs[10] = 34;
  seq_to_mod_border_idxs[11] = 33;
  seq_to_mod_border_idxs[12] = 32;
  seq_to_mod_border_idxs[13] = 24;
  seq_to_mod_border_idxs[14] = 16;
  seq_to_mod_border_idxs[15] = 8;
  for(uint8_t entry = 0; entry < 25; entry++)
  {
    for(uint8_t exit = 0; exit < 25; exit++)
    {
      if(entry > exit || exit - entry == 1 || (exit + 16) - entry == 1)
        continue;
      vector<uint8_t> perturbation_list;
      vector<uint8_t> occupied_squares;
      auto in = seq_to_mod_border_idxs[entry];
      auto out = seq_to_mod_border_idxs[exit];
      growRoute(perturbation_list, occupied_squares, in, out);  // Appends all possible paths from entry point to
    }                                                           // exit point at the corresponding cache position 
  }
}


vector<vector<uint8_t>> getPerturbations(uint8_t entry, uint8_t exit)
{
  auto key = entry <= exit? make_pair(entry, exit) : make_pair(exit, entry);
  auto perturbations = perturbation_cache[key];
  if(exit < entry)
    for(auto& route : perturbations)
      reverse(route.begin(), route.end());
  return perturbations;
}

uint8_t getIdxFromPoint(Point2i hood_point)
{
  uint8_t idx = hood_point.y * 8;
  idx += hood_point.x;
  return idx;
}

Point2i getPointFromIdx(uint8_t idx)
{
  auto x = idx % 8;
  auto y = idx / 8;
  return std::move(Point2i(x, y));
}

vector<Point2i> getPointList(vector<uint8_t>& idx_list)
{
  vector<Point2i> point_list;
  for(auto& idx : idx_list)
    point_list.push_back(getPointFromIdx(idx));

  for(auto& pt : point_list)
  {
    pt.x -= 2;
    pt.y -= 2;
  }

  return point_list;
}

vector<uint8_t> getHoodIdxs(uint8_t idx, bool include_border)
{
  vector<uint8_t> hood;
  hood.push_back(idx - 8 - 1);
  hood.push_back(idx - 8);
  hood.push_back(idx - 8 + 1);
  hood.push_back(idx - 1);
  hood.push_back(idx + 1);
  hood.push_back(idx + 8 - 1);
  hood.push_back(idx + 8);
  hood.push_back(idx + 8 + 1);

  if(include_border)
    hood = vector<uint8_t>(hood.begin(), remove_if(hood.begin(), hood.end(),
      [](uint8_t x){return x < 0 || x >= 39 || x%8 > 4; }));
  else
    hood = vector<uint8_t>(hood.begin(), remove_if(hood.begin(), hood.end(), 
      [](uint8_t x){return x < 0 || x >= 39 || x/8 == 0 || x/8 == 4 || x%8 == 0 || x%8 > 3; }));
  
  return hood;
}

bool isNeighbor(uint8_t idx1, uint8_t idx2)
{
  auto hood = getHoodIdxs(idx1, true);
  return find(hood.begin(), hood.end(), idx2) != hood.end();
}

bool isNeighborPoint(const Point2i &pt1, const Point2i &pt2)
{
 return abs(pt1.x - pt2.x) == 1 && abs(pt1.y - pt2.y) == 1; 
}

void growRoute(const vector<uint8_t>& partial, const vector<uint8_t>& occupied, uint8_t entry, uint8_t exit)
{
  uint8_t tail = (partial.size() == 0)? entry : partial.back();
  auto candidates = getHoodIdxs(tail, false);

  // remove candidates that were neighbors of prev tail
  candidates = vector<uint8_t>(candidates.begin(), remove_if(candidates.begin(), candidates.end(),
    [&occupied](uint8_t x){return find(occupied.begin(), occupied.end(), x) != occupied.end(); }));

  auto next_occupied = occupied;  // neighbors of current tailed should be blacklisted for next route growth
  for(auto cand : candidates)
    next_occupied.push_back(cand);

  for(auto new_elem : candidates)
  {
    auto next_partial = partial;
    next_partial.push_back(new_elem);
    
    auto next_tails = getHoodIdxs(new_elem, true);  // true --> include border
    auto find_exit_itr = find(next_tails.begin(), next_tails.end(), exit);
    if(find_exit_itr != next_tails.end() && *find_exit_itr != tail)  // add to cache if exit is a possible next tail
    {
      pair<uint8_t, uint8_t> key;

      if(entry <= exit)
        key = make_pair(entry, exit);
      else
      {
        key = make_pair(exit, entry);
        reverse(next_partial.begin(), next_partial.end());
      }
      perturbation_cache[key].push_back(next_partial);
    }
    else
      growRoute(next_partial, next_occupied, entry, exit);
  }
  
  return;  // No viable growth candidates
}

vector<Point2i> perturb(const vector<Point2i>& src_curve, vector<uint8_t> perturbation,int idx)
{
  auto pt = src_curve[idx];
  auto pt_list = getPointList(perturbation);
  for(auto& pt_elem : pt_list)
  {
    pt_elem.x += pt.x;
    pt_elem.y += pt.y;
  }

  vector<Point2i> dest(src_curve.begin(), src_curve.begin() + idx - 1);
  for(auto& pert : pt_list)
    dest.push_back(pert);
  auto it = src_curve.begin() + idx + 2;
  while(it != src_curve.end())
  {
    dest.push_back(*it);
    it++;
  }

  return dest;
}

}  // namespace Perturbations

ClosedCurve::ClosedCurve(vector<Point2i> points)
{
  _curve_points = points;
}

void ClosedCurve::applyPerturbation(const vector<uint8_t>& perturbation, int idx)
{
  _curve_points = Perturbations::perturb(_curve_points, perturbation, idx);
}

ClosedCurve ClosedCurve::perturb(const std::vector<uint8_t>& perturbation, int idx)
{
  return ClosedCurve(Perturbations::perturb(_curve_points, perturbation, idx));
}

bool ClosedCurve::validateCurve(std::vector<cv::Point2i>& curve)
{
  // Make sure that all consecutive points are 8-connected
  auto eight_connected =  [](Point2i a, Point2i b){ return abs(a.x - b.x) <= 1 && abs(a.y - b.y) <= 1; };
  if(!eight_connected(curve[0], curve.back()))
    return false;
  cout << "Checking for unconnected consecutive pts" << endl;
  for(size_t i = 1; i < curve.size(); i++)
    if(!eight_connected(curve[i - 1], curve[i]))
    {
      cout << "failure pts: " << curve[i - 1] << "\t" << curve[i] << endl;
      return false;
    }

  // Make sure that points that are not adjacent are never 8-connected
  vector<Point2i> forbidden_neighbors;
  forbidden_neighbors.push_back(curve.back());
  cout << "Checking for non-adjacent neighbors" << endl;
  for(size_t i = 1; i < curve.size(); i++)
  {
    auto& pt = curve[i];
    auto count = count_if(forbidden_neighbors.begin(), forbidden_neighbors.end(), [pt](const Point2i &test_pt)
      { return Perturbations::isNeighborPoint(pt, test_pt); });
    forbidden_neighbors.push_back(curve[i - 1]);
    
    if(i > curve.size() - 2) // Should be neighbor with first one added only
    {
      if(count > 1)
      {
        auto conflict_pt = find_if(forbidden_neighbors.begin(), forbidden_neighbors.end(),
          [pt](Point2i test_pt){ return nav::Perturbations::isNeighborPoint(pt, test_pt); });
        cout << "failure pts: " << curve[1] << "\t" << (conflict_pt != forbidden_neighbors.end()? Point2i(0,0) : *conflict_pt) << endl;
        return false;
      }
    }
    else
    {
      if(count > 0)
      {
        auto conflict_pt = find_if(forbidden_neighbors.begin(), forbidden_neighbors.end(),
          [pt](Point2i test_pt){ return nav::Perturbations::isNeighborPoint(pt, test_pt); });
        cout << "failure pts: " << curve[1] << "\t" << (conflict_pt != forbidden_neighbors.end()? Point2i(0,0) : *conflict_pt) << endl;
        return false;
      }
    }
  }
  return true; // No failures! 
}

vector<float> calcCosts(const Mat& img, vector<ClosedCurve::Perturbation> candidate_perturbs, function<float(const Mat&, ClosedCurve::Perturbation)> cb)
{

}

}  // namespace nav
