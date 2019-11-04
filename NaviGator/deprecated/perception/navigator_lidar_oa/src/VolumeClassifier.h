////////////////////////////////////////////////////////////
//
// Volume Classifier
//
////////////////////////////////////////////////////////////
#ifndef VOLUMECLASSIFIER_H
#define VOLUMECLASSIFIER_H

#include <algorithm>
#include <string>
#include <vector>
#include "ConnectedComponents.h"
#include "lidarParams.h"
#include "navigator_msgs/PerceptionObject.h"

void VolumeClassifier(objectMessage &object)
{
  // Seperate variables we care about
  auto h = object.maxHeightFromLidar;
  auto x = object.scale.x;
  auto y = object.scale.y;
  auto z = object.scale.z;

  // Skip classification if the object is locked, set as a start_gate, or if it isn't real
  if (object.locked || object.name == navigator_msgs::PerceptionObject::START_GATE_BUOY || !object.real)
  {
    return;
  }

  // Possible classifications
  std::vector<std::string> names = { "dock", "shooter", "scan_the_code", "totem", "buoy" };

  // Update confidence for each type of object
  std::vector<std::tuple<int, int>> options;
  for (auto ii = 0; ii < names.size(); ++ii)
  {
    if (h >= volumes[ii][0] && h <= volumes[ii][1] &&
        ((x >= volumes[ii][2] && x <= volumes[ii][3]) || (y >= volumes[ii][4] && y <= volumes[ii][5])) &&
        z >= volumes[ii][6] && z <= volumes[ii][7])
    {
      if (object.confidence[ii] < size_t(-1))
      {
        ++object.confidence[ii];
      }
    }
    if (object.confidence[ii] >= MIN_HITS_FOR_VOLUME)
    {
      options.push_back(std::make_tuple(object.confidence[ii], ii));
    }
  }

  // Confidence has to meet minimum threshold before being selected
  object.name = "unknown";
  object.bestConfidence = 0;
  for (auto ii = 0; ii < names.size(); ++ii)
  {
    if (ii < names.size() - 1 && object.confidence[ii] >= MIN_HITS_FOR_VOLUME &&
        object.confidence[ii] > object.confidence[ii + 1] / 2)
    {
      object.name = names[ii];
      object.bestConfidence = (double)object.confidence[ii] /
                              std::accumulate(object.confidence.begin(), object.confidence.end(), 0.0) * 255;
      break;
    }
    else if (ii == names.size() - 1 && object.confidence[ii] >= MIN_HITS_FOR_VOLUME)
    {
      object.name = names[ii];
      object.bestConfidence = (double)object.confidence[ii] /
                              std::accumulate(object.confidence.begin(), object.confidence.end(), 0.0) * 255;
      break;
    }
  }
}
#endif
