#!/bin/bash
# POI
poisave()
{
  rosservice call /poi_server/save_to_param
  rosparam dump $1 /poi_server
}
poiadd()
{
  rosservice call /poi_server/add "name: '$1'"
}
poidelete()
{
  rosservice call /poi_server/delete "name: '$1'"
}
