#pragma once
#include <ros/ros.h>
#include <mil_petri_nets/petri_net.hpp>
namespace petri_net::tests
{
void timing(PetriNet& pn)
{
  pn.AddPlace("Start", { in_type<Empty>() });
  pn.SetPlaceCallback("Start", [](const Token _t) -> Token { return _t; });
  pn.AddTransition("Start_Transition");
  pn.ConnectPlaceTransition("Start", "Start_Transition", typeid(Empty));
  for (int i = 0; i < 3; ++i)
  {
    pn.AddPlace("Wait " + std::to_string(i), { in_type<Empty>() });
    pn.ConnectTransitionPlace(
        "Start_Transition", "Wait " + std::to_string(i),
        [](const PlaceTypeTokenVec& _in) -> Token { return _in.at("Start").at(hash_code<Empty>()).at(0); });
    std::string name = "Wait " + std::to_string(i);
    pn.SetPlaceCallback(name, [=](const Token _in) -> Token {
      ros::Time::init();
      ros::Duration(i).sleep();
      return make_token(ros::Time::now());
    });
    pn.ConnectPlaceTransition(name, "Return/Transition", typeid(ros::Time));
  }
}
}
