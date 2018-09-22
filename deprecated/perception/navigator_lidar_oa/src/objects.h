////////////////////////////////////////////////////////////
//
// Object Database
//
////////////////////////////////////////////////////////////
#ifndef OBJECTS_H
#define OBJECTS_H

#include <ros/console.h>
#include <Eigen/Dense>
#include <boost/assert.hpp>
#include <iostream>
#include <tuple>
#include <vector>
#include "ConnectedComponents.h"
#include "VolumeClassifier.h"
#include "lidarParams.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class ObjectTracker
{
private:
  float diff_thresh;
  int curr_id = 0;
  bool foundGates = false;
  std::vector<geometry_msgs::Point> gatePositions;
  geometry_msgs::Vector3 gateNormal;
  enum
  {
    Gate_1 = 6,
    Gate_2,
    Gate_3
  };

public:
  // This breaks encapsulation, blah blah blah....
  std::vector<objectMessage> saved_objects;

  ////////////////////////////////////////////////////////////
  /// \brief ?
  ///
  /// \param ?
  /// \param ?
  ////////////////////////////////////////////////////////////
  ObjectTracker(float diff_thresh = 4)
  {
    this->diff_thresh = diff_thresh;
  }

  ////////////////////////////////////////////////////////////
  /// \brief ?
  ///
  /// \param ?
  /// \param ?
  ////////////////////////////////////////////////////////////
  void reset()
  {
    curr_id = 10;
    foundGates = false;
    gatePositions.clear();
    saved_objects.erase(saved_objects.begin() + 10, saved_objects.end());
  }

  ////////////////////////////////////////////////////////////
  /// \brief ?
  ///
  /// \param ?
  /// \param ?
  ////////////////////////////////////////////////////////////
  std::vector<objectMessage> add_objects(std::vector<objectMessage> objects, sensor_msgs::PointCloud &rosCloud,
                                         const geometry_msgs::Pose &boatPose_enu)
  {
    // Reset all saved objects to not seen
    for (auto &s_obj : saved_objects)
    {
      s_obj.current = false;
    }

    // Does each incoming object match one of the saved objects in the database?
    for (auto &obj : objects)
    {
      // Only process real objects from the lidar, not ROI
      if (!obj.real)
      {
        continue;
      }

      // What is the closest saved object to this one?
      float min_dist = diff_thresh;
      objectMessage *min_obj = nullptr;
      bool badPersist = false;
      for (auto &s_obj : saved_objects)
      {
        auto xyDistance = sqrt(pow(obj.position.x - s_obj.position.x, 2) + pow(obj.position.y - s_obj.position.y, 2));
        auto persistMax = 1.0 + std::max(obj.strikesPersist.size(), s_obj.strikesPersist.size());
        auto persistMin = 1.0 + std::min(obj.strikesPersist.size(), s_obj.strikesPersist.size());
        if (xyDistance < min_dist)
        {
          if (persistMin / persistMax >= 0.20)
          {
            min_dist = xyDistance;
            min_obj = &s_obj;
          }
          else
          {
            badPersist = true;
          }
        }
      }

      // If the saved object was within in the minimum threshold, update the database. Otherwise, create a new object
      if (min_obj != nullptr && min_dist < diff_thresh)
      {
        ROS_INFO_STREAM("LIDAR : Updating " << min_obj->name << " with " << obj.strikesPersist.size() << " vs the old "
                                            << min_obj->strikesPersist.size());
        obj.name = min_obj->name;
        obj.id = min_obj->id;
        obj.normal = min_obj->normal;
        obj.pclInliers = min_obj->pclInliers;
        obj.color = min_obj->color;
        obj.current = true;
        obj.locked = min_obj->locked;
        obj.real = min_obj->real;
        obj.confidence = min_obj->confidence;
        obj.bestConfidence = min_obj->bestConfidence;
        *min_obj = obj;
      }
      else if (badPersist == false)
      {
        obj.id = curr_id++;
        obj.current = true;
        saved_objects.push_back(obj);
      }
    }

    // After updating database, process each object to updates its info
    auto cnt = -1;
    std::vector<std::tuple<std::string, int, int, double, unsigned> > duplicates = {
      std::make_tuple("shooter", 0, -1, -1, 4), std::make_tuple("scan_the_code", 0, -1, -1, 5)
    };
    for (auto &s_obj : saved_objects)
    {
      ++cnt;
      if (!s_obj.real)
      {
        continue;
      }
      // Classify volume
      VolumeClassifier(s_obj);
      // Look for duplicates of the shooter or scan_the_code
      for (auto &dups : duplicates)
      {
        if (s_obj.name == std::get<0>(dups))
        {
          ++std::get<1>(dups);
          auto xyDistance = sqrt(pow(s_obj.position.x - saved_objects[std::get<4>(dups)].position.x, 2) +
                                 pow(s_obj.position.y - saved_objects[std::get<4>(dups)].position.y, 2));
          if (std::get<1>(dups) == 1)
          {
            std::get<2>(dups) = cnt;
            std::get<3>(dups) = xyDistance;
          }
          else
          {
            if (xyDistance < std::get<3>(dups))
            {
              saved_objects.at(std::get<2>(dups)).name = "unknown";
              std::get<2>(dups) = cnt;
              std::get<3>(dups) = xyDistance;
            }
            else
            {
              s_obj.name = "unknown";
            }
            // BOOST_ASSERT_MSG(false," Duplicate object error");
          }
        }
      }
    }
    return saved_objects;
  }

  ////////////////////////////////////////////////////////////
  /// \brief ?
  ///
  /// \param ?
  /// \param ?
  ////////////////////////////////////////////////////////////
  void addROI(std::string name)
  {
    objectMessage obj;
    obj.name = name;
    obj.real = false;
    obj.id = curr_id++;
    saved_objects.push_back(obj);
  }

  ////////////////////////////////////////////////////////////
  /// \brief ?
  ///
  /// \param ?
  /// \param ?
  ////////////////////////////////////////////////////////////
  void lock(std::string name, geometry_msgs::Point p, bool status = true)
  {
    for (auto &s_obj : saved_objects)
    {
      if (s_obj.name == name)
      {
        s_obj.locked = status;
        s_obj.position = p;
      }
    }
  }

  ////////////////////////////////////////////////////////////
  /// \brief ?
  ///
  /// \param ?
  /// \param ?
  ////////////////////////////////////////////////////////////
  bool lookUpByName(std::string name, std::vector<navigator_msgs::PerceptionObject> &objects)
  {
    // Name may also be the numeric id of the object
    int id;
    try
    {
      id = stod(name);
    }
    catch (...)
    {
      id = -1;
    }

    for (const auto &s_obj : saved_objects)
    {
      if ((name == "allAll") || (name == "all" && s_obj.real) || (name == s_obj.name || id == s_obj.id) ||
          (name == "All" && !s_obj.real))
      {
        navigator_msgs::PerceptionObject thisOne;
        thisOne.header.stamp - s_obj.age;
        thisOne.name = s_obj.name;
        thisOne.position = s_obj.position;
        thisOne.id = s_obj.id;
        thisOne.confidence = 0;
        std_msgs::Header header = std_msgs::Header();
        header.stamp = ros::Time::now();
        thisOne.header = header;
        thisOne.size.z = s_obj.scale.z;
        thisOne.size.x = s_obj.scale.x;
        thisOne.size.y = s_obj.scale.y;
        thisOne.points = s_obj.strikesFrame;
        thisOne.intensity = s_obj.intensityFrame;
        thisOne.pclInliers = s_obj.pclInliers;
        thisOne.normal = s_obj.normal;
        thisOne.color = s_obj.color;
        thisOne.confidence = s_obj.bestConfidence;
        objects.push_back(thisOne);
      }
    }
    return objects.size() > 0;
  }

  ////////////////////////////////////////////////////////////
  /// \brief ?
  ///
  /// \param ?
  /// \param ?
  ////////////////////////////////////////////////////////////
  std::tuple<std::vector<geometry_msgs::Point>, geometry_msgs::Vector3> FindThreeGates()
  {
    // There should only be 1 set of gates per course, don't keep searching after finding it
    if (foundGates)
    {
      return make_tuple(gatePositions, gateNormal);
    }

    // Create a list of all the totems in the database
    std::vector<int> totems;
    for (auto &obj : saved_objects)
    {
      if (obj.name == navigator_msgs::PerceptionObject::TOTEM || obj.name == "start_gate")
      {
        totems.push_back(obj.id);
      }
    }

    // Using permutations, find the combinations (order doesn't matter) of n choose 4
    auto combos = combinations(totems.size());

    // Run through all combinations
    for (auto &permute : combos)
    {
      // Average the xy position of all 4 totems to find the center
      geometry_msgs::Point center;
      for (auto id : permute)
      {
        center.x += saved_objects[totems[id]].position.x / 4.;
        center.y += saved_objects[totems[id]].position.y / 4.;
        center.z += 0;  // saved_objects[totems[id]].position.z/4.;
      }

      // Sort the totems based on distance from center to find the two outer totems
      std::vector<std::tuple<double, int> > order;
      for (auto id : permute)
      {
        auto distance = sqrt(pow(saved_objects[totems[id]].position.x - center.x, 2) +
                             pow(saved_objects[totems[id]].position.y - center.y, 2));
        order.push_back(std::make_tuple(distance, totems[id]));
      }
      std::sort(order.begin(), order.end());

      // The last 2 in sorted list are the outside edges
      auto edge1 = std::get<1>(order[2]);
      auto edge2 = std::get<1>(order[3]);

      // Check seperation between outside buoys
      auto distance = sqrt(pow(saved_objects[edge1].position.x - saved_objects[edge2].position.x, 2) +
                           pow(saved_objects[edge1].position.y - saved_objects[edge2].position.y, 2));

      // Calculate line equation: m,b
      auto m = (saved_objects[edge1].position.y - saved_objects[edge2].position.y) /
               (saved_objects[edge1].position.x - saved_objects[edge2].position.x);
      auto b = saved_objects[edge1].position.y - m * saved_objects[edge1].position.x;

      // Find the residual error for all totems on line
      auto error = 0.;
      for (auto id : permute)
      {
        error += fabs(saved_objects[totems[id]].position.y - m * saved_objects[totems[id]].position.x - b);
      }

      // Did we find the gates?
      // ROS_INFO_STREAM("LIDAR | FOUND 4 TOTEMS: " << totems[permute[0]] << "," << totems[permute[1]] << "," <<
      // totems[permute[2]] << "," << totems[permute[3]] << " with edge distance of " << distance << " between " <<
      // edge1 << " and " << edge2 << " and line error of " << error);
      if (distance >= MIN_GATE_SEPERATION && distance <= MAX_GATE_SEPERATION && error <= MAX_GATE_ERROR_METRIC)
      {
        // Save center gate
        gatePositions.push_back(center);

        // Find gate 1
        Eigen::Vector2d dir1(center.x - saved_objects[edge1].position.x, center.y - saved_objects[edge1].position.y);
        geometry_msgs::Point p;
        p.x = saved_objects[edge1].position.x + dir1(0) / 3.0;
        p.y = saved_objects[edge1].position.y + dir1(1) / 3.0;
        gatePositions.push_back(p);

        // Find gate 3
        Eigen::Vector2d dir2(center.x - saved_objects[edge2].position.x, center.y - saved_objects[edge2].position.y);
        p.x = saved_objects[edge2].position.x + dir2(0) / 3.0;
        p.y = saved_objects[edge2].position.y + dir2(1) / 3.0;
        gatePositions.push_back(p);

        // Don't search for gates anymore
        foundGates = true;

        // Calculate normal to gate (direction is arbitrary)
        dir1.normalize();
        gateNormal.x = -dir1(1);
        gateNormal.y = dir1(0);

        // Update the ROI in the database
        saved_objects[Gate_2].position = gatePositions[0];
        saved_objects[Gate_2].normal = gateNormal;
        saved_objects[Gate_2].locked = true;
        saved_objects[Gate_1].position = gatePositions[1];
        saved_objects[Gate_1].normal = gateNormal;
        saved_objects[Gate_1].locked = true;
        saved_objects[Gate_3].position = gatePositions[2];
        saved_objects[Gate_3].normal = gateNormal;
        saved_objects[Gate_3].locked = true;

        // Done!
        return std::make_pair(gatePositions, gateNormal);
      }
    }
    return std::make_pair(gatePositions, gateNormal);
  }

  ////////////////////////////////////////////////////////////
  /// \brief ?
  ///
  /// \param ?
  /// \param ?
  ////////////////////////////////////////////////////////////
  std::vector<std::vector<int> > combinations(int n, int r = 4)
  {
    // Check that the desired combination is possible
    std::vector<std::vector<int> > combos;
    if (n < r)
    {
      return combos;
    }

    // Nifty online code for using permutations to find combinations
    std::string bits(r, 1);
    bits.resize(n, 0);
    do
    {
      std::vector<int> permute;
      for (auto ii = 0; ii < n; ++ii)
      {
        if (bits[ii])
        {
          permute.push_back(ii);
        }
      }
      combos.push_back(permute);
    } while (std::prev_permutation(bits.begin(), bits.end()));
    return combos;
  }
};
#endif

// Verify that the none of the saved objects are too close together.
/*#ifdef DEBUG
  cnt = 0;
  for (auto &s_obj : saved_objects) {
    ++cnt;
    if (!s_obj.real) {continue;}
    for (auto jj = saved_objects.begin()+cnt; jj != saved_objects.end(); ++jj) {
      auto xyDistance = sqrt( pow(s_obj.position.x - jj->position.x, 2) + pow(s_obj.position.y - jj->position.y, 2)  );
      std::stringstream ss;
      ss << "Database object position failure: " << s_obj.position.x << "," << s_obj.position.y << " vs " <<
jj->position.x << "," << jj->position.y;
      BOOST_ASSERT_MSG(xyDistance > diff_thresh, ss.str().c_str());
    }
  }
#endif*/
