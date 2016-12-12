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

}  // namespace Perturbations

}  // namespace nav
